use std::fmt;
use std::ptr::NonNull;

use jolt_sys as sys;

use crate::{AABox, Error, Mat44, PhysicsMaterial, Quat, Result, SubShapeId, Vec3};

/// Default convex radius Jolt uses to round off the corners of boxes and cylinders.
pub const DEFAULT_CONVEX_RADIUS: f32 = sys::JPC_DEFAULT_CONVEX_RADIUS as f32;

/// A reference counted, immutable collision shape. Cloning is cheap and shares the shape.
pub struct Shape {
    raw: NonNull<sys::JPC_Shape>,
}

// SAFETY: Jolt shapes are immutable after creation (the only mutators here require unique ownership)
// and their reference count is atomic.
unsafe impl Send for Shape {}
unsafe impl Sync for Shape {}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShapeType {
    Convex,
    Compound,
    Decorated,
    Mesh,
    HeightField,
    SoftBody,
    User(u8),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShapeSubType {
    Sphere,
    Box,
    Triangle,
    Capsule,
    TaperedCapsule,
    Cylinder,
    ConvexHull,
    StaticCompound,
    MutableCompound,
    RotatedTranslated,
    Scaled,
    OffsetCenterOfMass,
    Mesh,
    HeightField,
    SoftBody,
    Plane,
    TaperedCylinder,
    Empty,
    User(u8),
}

/// Mass and inertia tensor (about the center of mass).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MassProperties {
    pub mass: f32,
    pub inertia: Mat44,
}

impl From<sys::JPC_MassProperties> for MassProperties {
    fn from(m: sys::JPC_MassProperties) -> Self {
        Self { mass: m.mass, inertia: m.inertia.into() }
    }
}

impl From<MassProperties> for sys::JPC_MassProperties {
    fn from(m: MassProperties) -> Self {
        Self { mass: m.mass, inertia: m.inertia.into() }
    }
}

/// A child of a [`Shape::static_compound`].
#[derive(Clone, Debug)]
pub struct CompoundSubShape {
    pub position: Vec3,
    pub rotation: Quat,
    pub shape: Shape,
    pub user_data: u32,
}

impl CompoundSubShape {
    pub fn new(position: Vec3, rotation: Quat, shape: Shape) -> Self {
        Self { position, rotation, shape, user_data: 0 }
    }
}

impl Shape {
    fn from_raw_or_error(raw: *mut sys::JPC_Shape) -> Result<Self> {
        NonNull::new(raw).map(|raw| Self { raw }).ok_or_else(Error::last)
    }

    /// Takes ownership of one reference.
    ///
    /// # Safety
    /// `raw` must point to a live Jolt shape and the caller must own the reference being transferred.
    pub unsafe fn from_raw(raw: NonNull<sys::JPC_Shape>) -> Self {
        Self { raw }
    }

    pub fn as_raw(&self) -> *const sys::JPC_Shape {
        self.raw.as_ptr()
    }

    pub fn sphere(radius: f32) -> Result<Self> {
        crate::ensure_init();
        Self::from_raw_or_error(unsafe { sys::JPC_SphereShape_Create(radius) })
    }

    /// A box with the default convex radius.
    pub fn cuboid(half_extent: Vec3) -> Result<Self> {
        let radius = DEFAULT_CONVEX_RADIUS.min(half_extent.x).min(half_extent.y).min(half_extent.z);
        Self::cuboid_with_convex_radius(half_extent, radius)
    }

    pub fn cuboid_with_convex_radius(half_extent: Vec3, convex_radius: f32) -> Result<Self> {
        crate::ensure_init();
        Self::from_raw_or_error(unsafe { sys::JPC_BoxShape_Create(half_extent.into(), convex_radius) })
    }

    /// A capsule along the Y axis. `half_height` excludes the hemispherical caps.
    pub fn capsule(half_height: f32, radius: f32) -> Result<Self> {
        crate::ensure_init();
        Self::from_raw_or_error(unsafe { sys::JPC_CapsuleShape_Create(half_height, radius) })
    }

    pub fn tapered_capsule(half_height: f32, top_radius: f32, bottom_radius: f32) -> Result<Self> {
        crate::ensure_init();
        Self::from_raw_or_error(unsafe { sys::JPC_TaperedCapsuleShape_Create(half_height, top_radius, bottom_radius) })
    }

    /// A cylinder along the Y axis with the default convex radius.
    pub fn cylinder(half_height: f32, radius: f32) -> Result<Self> {
        let convex_radius = DEFAULT_CONVEX_RADIUS.min(half_height).min(radius);
        Self::cylinder_with_convex_radius(half_height, radius, convex_radius)
    }

    pub fn cylinder_with_convex_radius(half_height: f32, radius: f32, convex_radius: f32) -> Result<Self> {
        crate::ensure_init();
        Self::from_raw_or_error(unsafe { sys::JPC_CylinderShape_Create(half_height, radius, convex_radius) })
    }

    pub fn tapered_cylinder(half_height: f32, top_radius: f32, bottom_radius: f32, convex_radius: f32) -> Result<Self> {
        crate::ensure_init();
        Self::from_raw_or_error(unsafe {
            sys::JPC_TaperedCylinderShape_Create(half_height, top_radius, bottom_radius, convex_radius)
        })
    }

    pub fn convex_hull(points: &[Vec3], max_convex_radius: f32) -> Result<Self> {
        crate::ensure_init();
        let count = u32::try_from(points.len()).map_err(|_| Error::new("too many points"))?;
        // Vec3 is layout-compatible with JPC_Vec3.
        Self::from_raw_or_error(unsafe {
            sys::JPC_ConvexHullShape_Create(points.as_ptr().cast(), count, max_convex_radius)
        })
    }

    /// A triangle mesh. Meshes can only be used for static or kinematic bodies.
    pub fn mesh(vertices: &[Vec3], triangles: &[[u32; 3]]) -> Result<Self> {
        crate::ensure_init();
        let num_vertices = u32::try_from(vertices.len()).map_err(|_| Error::new("too many vertices"))?;
        let num_triangles = u32::try_from(triangles.len()).map_err(|_| Error::new("too many triangles"))?;
        // [u32; 3] has the same layout as three consecutive u32 indices.
        Self::from_raw_or_error(unsafe {
            sys::JPC_MeshShape_Create(vertices.as_ptr().cast(), num_vertices, triangles.as_ptr().cast(), num_triangles)
        })
    }

    /// A triangle mesh where `material_indices[i]` picks the material of triangle `i` from `materials`.
    pub fn mesh_with_materials(vertices: &[Vec3], triangles: &[[u32; 3]], material_indices: &[u32], materials: &[PhysicsMaterial]) -> Result<Self> {
        crate::ensure_init();
        if material_indices.len() != triangles.len() {
            return Err(Error::new("need one material index per triangle"));
        }
        let num_vertices = u32::try_from(vertices.len()).map_err(|_| Error::new("too many vertices"))?;
        let num_triangles = u32::try_from(triangles.len()).map_err(|_| Error::new("too many triangles"))?;
        let num_materials = u32::try_from(materials.len()).map_err(|_| Error::new("too many materials"))?;
        let raw_materials: Vec<*const sys::JPC_PhysicsMaterial> = materials.iter().map(|m| m.as_raw()).collect();
        Self::from_raw_or_error(unsafe {
            sys::JPC_MeshShape_CreateWithMaterials(
                vertices.as_ptr().cast(),
                num_vertices,
                triangles.as_ptr().cast(),
                num_triangles,
                material_indices.as_ptr(),
                raw_materials.as_ptr(),
                num_materials,
            )
        })
    }

    /// A height field of `sample_count * sample_count` samples, stored row by row.
    pub fn height_field(samples: &[f32], offset: Vec3, scale: Vec3, sample_count: u32) -> Result<Self> {
        crate::ensure_init();
        if samples.len() as u64 != sample_count as u64 * sample_count as u64 {
            return Err(Error::new("height field needs sample_count * sample_count samples"));
        }
        Self::from_raw_or_error(unsafe {
            sys::JPC_HeightFieldShape_Create(samples.as_ptr(), offset.into(), scale.into(), sample_count)
        })
    }

    /// A height field where `material_indices[x + y * (sample_count - 1)]` picks the material of each cell.
    pub fn height_field_with_materials(
        samples: &[f32],
        offset: Vec3,
        scale: Vec3,
        sample_count: u32,
        material_indices: &[u8],
        materials: &[PhysicsMaterial],
    ) -> Result<Self> {
        crate::ensure_init();
        if samples.len() as u64 != sample_count as u64 * sample_count as u64 {
            return Err(Error::new("height field needs sample_count * sample_count samples"));
        }
        let cells = (sample_count.saturating_sub(1) as u64).pow(2);
        if material_indices.len() as u64 != cells {
            return Err(Error::new("height field needs (sample_count - 1)^2 material indices"));
        }
        let num_materials = u32::try_from(materials.len()).map_err(|_| Error::new("too many materials"))?;
        let raw_materials: Vec<*const sys::JPC_PhysicsMaterial> = materials.iter().map(|m| m.as_raw()).collect();
        Self::from_raw_or_error(unsafe {
            sys::JPC_HeightFieldShape_CreateWithMaterials(
                samples.as_ptr(),
                offset.into(),
                scale.into(),
                sample_count,
                material_indices.as_ptr(),
                raw_materials.as_ptr(),
                num_materials,
            )
        })
    }

    /// A plane through `normal * -distance`, limited to a square of `half_extent` for the broad phase.
    pub fn plane(normal: Vec3, distance: f32, half_extent: f32) -> Result<Self> {
        crate::ensure_init();
        Self::from_raw_or_error(unsafe { sys::JPC_PlaneShape_Create(normal.into(), distance, half_extent) })
    }

    /// A shape that collides with nothing.
    pub fn empty(center_of_mass: Vec3) -> Result<Self> {
        crate::ensure_init();
        Self::from_raw_or_error(unsafe { sys::JPC_EmptyShape_Create(center_of_mass.into()) })
    }

    pub fn static_compound(sub_shapes: &[CompoundSubShape]) -> Result<Self> {
        crate::ensure_init();
        let subs: Vec<sys::JPC_CompoundSubShape> = sub_shapes
            .iter()
            .map(|s| sys::JPC_CompoundSubShape {
                position: s.position.into(),
                rotation: s.rotation.into(),
                shape: s.shape.as_raw(),
                user_data: s.user_data,
            })
            .collect();
        let count = u32::try_from(subs.len()).map_err(|_| Error::new("too many sub shapes"))?;
        Self::from_raw_or_error(unsafe { sys::JPC_StaticCompoundShape_Create(subs.as_ptr(), count) })
    }

    pub fn rotated_translated(position: Vec3, rotation: Quat, inner: &Shape) -> Result<Self> {
        Self::from_raw_or_error(unsafe {
            sys::JPC_RotatedTranslatedShape_Create(position.into(), rotation.into(), inner.as_raw())
        })
    }

    pub fn scaled(inner: &Shape, scale: Vec3) -> Result<Self> {
        Self::from_raw_or_error(unsafe { sys::JPC_ScaledShape_Create(inner.as_raw(), scale.into()) })
    }

    pub fn offset_center_of_mass(offset: Vec3, inner: &Shape) -> Result<Self> {
        Self::from_raw_or_error(unsafe { sys::JPC_OffsetCenterOfMassShape_Create(offset.into(), inner.as_raw()) })
    }

    pub fn shape_type(&self) -> ShapeType {
        match unsafe { sys::JPC_Shape_GetType(self.as_raw()) } {
            sys::JPC_SHAPE_TYPE_CONVEX => ShapeType::Convex,
            sys::JPC_SHAPE_TYPE_COMPOUND => ShapeType::Compound,
            sys::JPC_SHAPE_TYPE_DECORATED => ShapeType::Decorated,
            sys::JPC_SHAPE_TYPE_MESH => ShapeType::Mesh,
            sys::JPC_SHAPE_TYPE_HEIGHT_FIELD => ShapeType::HeightField,
            sys::JPC_SHAPE_TYPE_SOFT_BODY => ShapeType::SoftBody,
            other => ShapeType::User(other as u8),
        }
    }

    pub fn sub_type(&self) -> ShapeSubType {
        match unsafe { sys::JPC_Shape_GetSubType(self.as_raw()) } {
            sys::JPC_SHAPE_SUB_TYPE_SPHERE => ShapeSubType::Sphere,
            sys::JPC_SHAPE_SUB_TYPE_BOX => ShapeSubType::Box,
            sys::JPC_SHAPE_SUB_TYPE_TRIANGLE => ShapeSubType::Triangle,
            sys::JPC_SHAPE_SUB_TYPE_CAPSULE => ShapeSubType::Capsule,
            sys::JPC_SHAPE_SUB_TYPE_TAPERED_CAPSULE => ShapeSubType::TaperedCapsule,
            sys::JPC_SHAPE_SUB_TYPE_CYLINDER => ShapeSubType::Cylinder,
            sys::JPC_SHAPE_SUB_TYPE_CONVEX_HULL => ShapeSubType::ConvexHull,
            sys::JPC_SHAPE_SUB_TYPE_STATIC_COMPOUND => ShapeSubType::StaticCompound,
            sys::JPC_SHAPE_SUB_TYPE_MUTABLE_COMPOUND => ShapeSubType::MutableCompound,
            sys::JPC_SHAPE_SUB_TYPE_ROTATED_TRANSLATED => ShapeSubType::RotatedTranslated,
            sys::JPC_SHAPE_SUB_TYPE_SCALED => ShapeSubType::Scaled,
            sys::JPC_SHAPE_SUB_TYPE_OFFSET_CENTER_OF_MASS => ShapeSubType::OffsetCenterOfMass,
            sys::JPC_SHAPE_SUB_TYPE_MESH => ShapeSubType::Mesh,
            sys::JPC_SHAPE_SUB_TYPE_HEIGHT_FIELD => ShapeSubType::HeightField,
            sys::JPC_SHAPE_SUB_TYPE_SOFT_BODY => ShapeSubType::SoftBody,
            sys::JPC_SHAPE_SUB_TYPE_PLANE => ShapeSubType::Plane,
            sys::JPC_SHAPE_SUB_TYPE_TAPERED_CYLINDER => ShapeSubType::TaperedCylinder,
            sys::JPC_SHAPE_SUB_TYPE_EMPTY => ShapeSubType::Empty,
            other => ShapeSubType::User(other as u8),
        }
    }

    pub fn local_bounds(&self) -> AABox {
        unsafe { sys::JPC_Shape_GetLocalBounds(self.as_raw()) }.into()
    }

    pub fn center_of_mass(&self) -> Vec3 {
        unsafe { sys::JPC_Shape_GetCenterOfMass(self.as_raw()) }.into()
    }

    pub fn inner_radius(&self) -> f32 {
        unsafe { sys::JPC_Shape_GetInnerRadius(self.as_raw()) }
    }

    pub fn volume(&self) -> f32 {
        unsafe { sys::JPC_Shape_GetVolume(self.as_raw()) }
    }

    pub fn mass_properties(&self) -> MassProperties {
        unsafe { sys::JPC_Shape_GetMassProperties(self.as_raw()) }.into()
    }

    pub fn user_data(&self) -> u64 {
        unsafe { sys::JPC_Shape_GetUserData(self.as_raw()) }
    }

    /// Density of a convex shape in kg/m^3, or `None` for other shapes.
    pub fn density(&self) -> Option<f32> {
        (self.shape_type() == ShapeType::Convex).then(|| unsafe { sys::JPC_ConvexShape_GetDensity(self.as_raw()) })
    }

    fn is_unique(&self) -> bool {
        unsafe { sys::JPC_Shape_GetRefCount(self.as_raw()) == 1 }
    }

    fn ensure_unique(&self) -> Result<()> {
        if self.is_unique() { Ok(()) } else { Err(Error::new("shape is shared; modify it before cloning it or using it in a body")) }
    }

    /// Sets the density (kg/m^3) used to compute the mass of bodies created from this shape afterwards.
    /// Fails if the shape is not convex or is shared with another handle or body.
    pub fn set_density(&mut self, density: f32) -> Result<()> {
        self.ensure_unique()?;
        if self.shape_type() != ShapeType::Convex {
            return Err(Error::new("density can only be set on convex shapes"));
        }
        unsafe { sys::JPC_ConvexShape_SetDensity(self.raw.as_ptr(), density) };
        Ok(())
    }

    /// Builder form of [`Shape::set_density`].
    pub fn with_density(mut self, density: f32) -> Result<Self> {
        self.set_density(density)?;
        Ok(self)
    }

    /// Sets the material of a convex shape (`None` for the default). Fails if the shape is not convex or is shared.
    pub fn set_material(&mut self, material: Option<&PhysicsMaterial>) -> Result<()> {
        self.ensure_unique()?;
        if self.shape_type() != ShapeType::Convex {
            return Err(Error::new("only convex shapes have a single material; use mesh_with_materials for meshes"));
        }
        unsafe { sys::JPC_ConvexShape_SetMaterial(self.raw.as_ptr(), material.map_or(std::ptr::null(), |m| m.as_raw())) };
        Ok(())
    }

    /// Builder form of [`Shape::set_material`].
    pub fn with_material(mut self, material: &PhysicsMaterial) -> Result<Self> {
        self.set_material(Some(material))?;
        Ok(self)
    }

    /// Material of a sub shape, `None` for the default material. Hits and contacts already report their material,
    /// so this is only needed when you stored a sub shape id yourself.
    ///
    /// # Safety
    /// `sub_shape_id` must have been produced by a query or contact against this exact shape; Jolt does not
    /// validate it and reads out of bounds otherwise.
    pub unsafe fn material_unchecked(&self, sub_shape_id: SubShapeId) -> Option<PhysicsMaterial> {
        PhysicsMaterial::from_borrowed(unsafe { sys::JPC_Shape_GetMaterialUnchecked(self.as_raw(), sub_shape_id.0) })
    }

    /// Fails if the shape is shared with another handle or body.
    pub fn set_user_data(&mut self, user_data: u64) -> Result<()> {
        self.ensure_unique()?;
        unsafe { sys::JPC_Shape_SetUserData(self.raw.as_ptr(), user_data) };
        Ok(())
    }
}

impl Clone for Shape {
    fn clone(&self) -> Self {
        unsafe { sys::JPC_Shape_AddRef(self.as_raw()) };
        Self { raw: self.raw }
    }
}

impl Drop for Shape {
    fn drop(&mut self) {
        unsafe { sys::JPC_Shape_Release(self.as_raw()) };
    }
}

impl fmt::Debug for Shape {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Shape").field("sub_type", &self.sub_type()).field("ptr", &self.raw).finish()
    }
}
