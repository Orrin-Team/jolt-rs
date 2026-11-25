//! Height field terrain that can be read and edited after creation, e.g. for craters or digging.
//!
//! ```
//! # use jolt::*;
//! # fn demo() -> Result<()> {
//! let n = 16;
//! let flat = vec![0.0; n * n];
//! // Leave room to raise or lower the terrain by 5 m later.
//! let settings = HeightFieldSettings { height_range: Some((-5.0, 5.0)), ..HeightFieldSettings::new(&flat, n as u32) };
//! let mut terrain = Shape::height_field_with_settings(&settings)?;
//!
//! let mut temp = TempAllocator::default();
//! let mut edit = terrain.edit_height_field()?;
//! edit.set_heights(4, 4, 4, 4, &[2.0; 16], &mut temp)?; // a 4 x 4 bump; edits are aligned to the block size
//! # Ok(()) }
//! ```

use std::ffi::c_void;
use std::marker::PhantomData;
use std::panic::{AssertUnwindSafe, catch_unwind, resume_unwind};
use std::ptr::NonNull;

use jolt_sys as sys;

use crate::{BodyId, BodyInterface, Error, PhysicsMaterial, Result, Shape, ShapeSubType, TempAllocator, Vec3};

/// Everything a height field is made from, for [`Shape::height_field_with_settings`].
#[derive(Clone, Debug)]
pub struct HeightFieldSettings<'a> {
    /// `sample_count * sample_count` heights, row by row. `f32::MAX` makes a hole.
    pub samples: &'a [f32],
    pub sample_count: u32,
    pub offset: Vec3,
    pub scale: Vec3,
    /// Range the heights can be edited within later; `None` takes the range of the samples.
    pub height_range: Option<(f32, f32)>,
    /// 2 to 8. Samples are compressed and culled per block, and edits work on whole blocks.
    pub block_size: u32,
    /// 1 to 16.
    pub bits_per_sample: u32,
    /// `(sample_count - 1)^2` material indices, one per cell, into the materials (at most 256).
    pub materials: Option<(&'a [u8], &'a [PhysicsMaterial])>,
}

impl<'a> HeightFieldSettings<'a> {
    pub fn new(samples: &'a [f32], sample_count: u32) -> Self {
        let mut d = sys::JPC_HeightFieldSettings::default();
        unsafe { sys::JPC_HeightFieldSettings_SetDefault(&mut d) };
        Self {
            samples,
            sample_count,
            offset: d.offset.into(),
            scale: d.scale.into(),
            height_range: None,
            block_size: d.block_size,
            bits_per_sample: d.bits_per_sample,
            materials: None,
        }
    }
}

/// Layout of a height field shape.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HeightFieldInfo {
    /// Rounded up to a multiple of the block size.
    pub sample_count: u32,
    pub block_size: u32,
    /// Edited heights are clamped to this range.
    pub min_height: f32,
    pub max_height: f32,
    pub num_materials: u32,
    /// False for a height field created entirely of holes, which stores no samples.
    pub editable: bool,
}

impl Shape {
    /// A height field with control over compression, block size and the range it can be edited within.
    pub fn height_field_with_settings(settings: &HeightFieldSettings<'_>) -> Result<Self> {
        crate::ensure_init();
        let n = settings.sample_count as u64;
        if settings.samples.len() as u64 != n * n {
            return Err(Error::new("height field needs sample_count * sample_count samples"));
        }
        let (indices, materials) = settings.materials.unwrap_or((&[], &[]));
        if settings.materials.is_some() && indices.len() as u64 != n.saturating_sub(1).pow(2) {
            return Err(Error::new("height field needs (sample_count - 1)^2 material indices"));
        }
        let raw_materials: Vec<*const sys::JPC_PhysicsMaterial> = materials.iter().map(|m| m.as_raw()).collect();
        let (min, max) = settings.height_range.unwrap_or((1.0, -1.0));
        let raw = sys::JPC_HeightFieldSettings {
            samples: settings.samples.as_ptr(),
            sample_count: settings.sample_count,
            offset: settings.offset.into(),
            scale: settings.scale.into(),
            min_height_value: min,
            max_height_value: max,
            block_size: settings.block_size,
            bits_per_sample: settings.bits_per_sample,
            material_indices: if indices.is_empty() { std::ptr::null() } else { indices.as_ptr() },
            materials: raw_materials.as_ptr(),
            num_materials: u32::try_from(raw_materials.len()).map_err(|_| Error::new("too many materials"))?,
        };
        let raw = unsafe { sys::JPC_HeightFieldShape_CreateWithSettings(&raw) };
        // SAFETY: Create hands out a new reference.
        NonNull::new(raw).map(|raw| unsafe { Shape::from_raw(raw) }).ok_or_else(Error::last)
    }

    /// `None` if this is not a height field.
    pub fn height_field_info(&self) -> Option<HeightFieldInfo> {
        height_field_info(self.as_raw())
    }

    /// Local position of sample `(x, y)` and whether it is a hole.
    pub fn height_field_position(&self, x: u32, y: u32) -> Option<(Vec3, bool)> {
        height_field_position(self.as_raw(), x, y)
    }

    /// `size_x * size_y` heights from sample `(x, y)`, row by row (`f32::MAX` for holes). The region must be aligned to
    /// the block size.
    pub fn height_field_heights(&self, x: u32, y: u32, size_x: u32, size_y: u32) -> Result<Vec<f32>> {
        heights(self.as_raw(), x, y, size_x, size_y)
    }

    /// `size_x * size_y` material indices of the cells from cell `(x, y)`, row by row.
    pub fn height_field_material_indices(&self, x: u32, y: u32, size_x: u32, size_y: u32) -> Result<Vec<u8>> {
        material_indices(self.as_raw(), x, y, size_x, size_y)
    }

    /// Edits a height field in place. Fails if the shape is another kind or is shared with another handle or body;
    /// to edit the terrain of a body use [`BodyInterface::modify_height_field`].
    pub fn edit_height_field(&mut self) -> Result<HeightFieldEditor<'_>> {
        if self.sub_type() != ShapeSubType::HeightField {
            return Err(Error::new("shape is not a height field"));
        }
        if unsafe { sys::JPC_Shape_GetRefCount(self.as_raw()) } != 1 {
            return Err(Error::new("shape is shared; modify it before cloning it or using it in a body"));
        }
        // SAFETY: unique handle, borrowed mutably for the editor's lifetime.
        Ok(unsafe { HeightFieldEditor::from_raw(NonNull::new_unchecked(self.as_raw().cast_mut())) })
    }
}

fn height_field_info(raw: *const sys::JPC_Shape) -> Option<HeightFieldInfo> {
    let mut i = sys::JPC_HeightFieldInfo::default();
    unsafe { sys::JPC_HeightFieldShape_GetInfo(raw, &mut i) }.then_some(HeightFieldInfo {
        sample_count: i.sample_count,
        block_size: i.block_size,
        min_height: i.min_height_value,
        max_height: i.max_height_value,
        num_materials: i.num_materials,
        editable: i.editable,
    })
}

fn height_field_position(raw: *const sys::JPC_Shape, x: u32, y: u32) -> Option<(Vec3, bool)> {
    let mut position = sys::JPC_Vec3::default();
    let mut hole = false;
    unsafe { sys::JPC_HeightFieldShape_GetPosition(raw, x, y, &mut position, &mut hole) }.then(|| (position.into(), hole))
}

fn region_len(size_x: u32, size_y: u32) -> Result<usize> {
    usize::try_from(size_x as u64 * size_y as u64).map_err(|_| Error::new("region too large"))
}

fn heights(raw: *const sys::JPC_Shape, x: u32, y: u32, size_x: u32, size_y: u32) -> Result<Vec<f32>> {
    let mut out = vec![0.0; region_len(size_x, size_y)?];
    if unsafe { sys::JPC_HeightFieldShape_GetHeights(raw, x, y, size_x, size_y, out.as_mut_ptr()) } { Ok(out) } else { Err(Error::last()) }
}

fn material_indices(raw: *const sys::JPC_Shape, x: u32, y: u32, size_x: u32, size_y: u32) -> Result<Vec<u8>> {
    let mut out = vec![0; region_len(size_x, size_y)?];
    if unsafe { sys::JPC_HeightFieldShape_GetMaterialIndices(raw, x, y, size_x, size_y, out.as_mut_ptr()) } { Ok(out) } else { Err(Error::last()) }
}

/// Edits a height field, from [`Shape::edit_height_field`] or [`BodyInterface::modify_height_field`].
pub struct HeightFieldEditor<'a> {
    raw: NonNull<sys::JPC_Shape>,
    _shape: PhantomData<&'a mut Shape>,
}

impl HeightFieldEditor<'_> {
    /// # Safety
    /// `raw` must be a height field that nothing else uses for `'a`.
    unsafe fn from_raw<'a>(raw: NonNull<sys::JPC_Shape>) -> HeightFieldEditor<'a> {
        HeightFieldEditor { raw, _shape: PhantomData }
    }

    pub fn info(&self) -> HeightFieldInfo {
        height_field_info(self.raw.as_ptr()).expect("editor always holds a height field")
    }

    pub fn heights(&self, x: u32, y: u32, size_x: u32, size_y: u32) -> Result<Vec<f32>> {
        heights(self.raw.as_ptr(), x, y, size_x, size_y)
    }

    pub fn material_indices(&self, x: u32, y: u32, size_x: u32, size_y: u32) -> Result<Vec<u8>> {
        material_indices(self.raw.as_ptr(), x, y, size_x, size_y)
    }

    /// Replaces `size_x * size_y` heights from sample `(x, y)`, row by row (`f32::MAX` for holes, others clamped to
    /// the height range). The region must be aligned to the block size.
    pub fn set_heights(&mut self, x: u32, y: u32, size_x: u32, size_y: u32, heights: &[f32], temp: &mut TempAllocator) -> Result<()> {
        if heights.len() != region_len(size_x, size_y)? {
            return Err(Error::new("need size_x * size_y heights"));
        }
        let ok = unsafe { sys::JPC_HeightFieldShape_SetHeights(self.raw.as_ptr(), x, y, size_x, size_y, heights.as_ptr(), temp.raw()) };
        if ok { Ok(()) } else { Err(Error::last()) }
    }

    /// Replaces material indices of cells from cell `(x, y)`; they must refer to the height field's materials.
    pub fn set_material_indices(&mut self, x: u32, y: u32, size_x: u32, size_y: u32, indices: &[u8], temp: &mut TempAllocator) -> Result<()> {
        if indices.len() != region_len(size_x, size_y)? {
            return Err(Error::new("need size_x * size_y material indices"));
        }
        let ok = unsafe { sys::JPC_HeightFieldShape_SetMaterialIndices(self.raw.as_ptr(), x, y, size_x, size_y, indices.as_ptr(), temp.raw()) };
        if ok { Ok(()) } else { Err(Error::last()) }
    }
}

impl BodyInterface<'_> {
    /// Edits the height field shape of a body in place, then updates its bounds and contact cache. If the shape is also
    /// held elsewhere the body first gets its own copy. Bodies resting on the terrain are not woken; follow up with
    /// [`activate_bodies_in_box`](Self::activate_bodies_in_box) over the edited area.
    ///
    /// `edit` runs with the body locked: using this body interface from inside it deadlocks.
    pub fn modify_height_field<R, F: FnOnce(&mut HeightFieldEditor<'_>) -> R>(&self, id: BodyId, edit: F) -> Result<R> {
        struct Context<F, R> {
            edit: Option<F>,
            result: Option<std::thread::Result<R>>,
        }

        unsafe extern "C" fn trampoline<F: FnOnce(&mut HeightFieldEditor<'_>) -> R, R>(user_data: *mut c_void, shape: *mut sys::JPC_Shape) {
            // SAFETY: user_data is the Context below; the shape is the body's own, locked, for this call.
            let context = unsafe { &mut *user_data.cast::<Context<F, R>>() };
            let mut editor = unsafe { HeightFieldEditor::from_raw(NonNull::new_unchecked(shape)) };
            if let Some(edit) = context.edit.take() {
                context.result = Some(catch_unwind(AssertUnwindSafe(|| edit(&mut editor))));
            }
        }

        let mut context: Context<F, R> = Context { edit: Some(edit), result: None };
        let ok = unsafe {
            sys::JPC_BodyInterface_ModifyHeightField(self.as_raw(), id.0, Some(trampoline::<F, R>), (&mut context as *mut Context<F, R>).cast())
        };
        if !ok {
            return Err(Error::last());
        }
        match context.result {
            Some(Ok(value)) => Ok(value),
            Some(Err(payload)) => resume_unwind(payload),
            None => Err(Error::new("height field edit did not run")),
        }
    }
}
