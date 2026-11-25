use std::ffi::c_void;
use std::fmt;
use std::marker::PhantomData;
use std::ptr::NonNull;

use jolt_sys as sys;

use crate::{
    AABox, CollideShapeSettings, Error, Mat44, PhysicsMaterial, Quat, RayCastHit, RayCastSettings, Result, ShapeCastSettings, ShapeHit, SubShapeId, Vec3,
};

/// Default convex radius Jolt uses to round off the corners of boxes and cylinders.
pub const DEFAULT_CONVEX_RADIUS: f32 = sys::JPC_DEFAULT_CONVEX_RADIUS as f32;

/// A reference counted, immutable collision shape. Cloning is cheap and shares the shape.
pub struct Shape {
    raw: NonNull<sys::JPC_Shape>,
}

// SAFETY: Jolt shapes are immutable after creation (the only mutators here require unique ownership, or edit a
// body's private copy under the body's write lock) and their reference count is atomic.
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

/// A child of a [`Shape::static_compound`] or [`Shape::mutable_compound`].
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

    /// A single triangle; ray casts hit its front (counter-clockwise) side and `convex_radius` rounds it off. It has
    /// no volume or mass, so use it for static or kinematic bodies.
    pub fn triangle(v1: Vec3, v2: Vec3, v3: Vec3, convex_radius: f32) -> Result<Self> {
        crate::ensure_init();
        Self::from_raw_or_error(unsafe { sys::JPC_TriangleShape_Create(v1.into(), v2.into(), v3.into(), convex_radius) })
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
        let subs = sys_sub_shapes(sub_shapes);
        let count = u32::try_from(subs.len()).map_err(|_| Error::new("too many sub shapes"))?;
        Self::from_raw_or_error(unsafe { sys::JPC_StaticCompoundShape_Create(subs.as_ptr(), count) })
    }

    /// A compound whose children can be added, removed and moved later: with [`edit_mutable_compound`](Self::edit_mutable_compound)
    /// while this is the only handle, or with [`BodyInterface::modify_mutable_compound`](crate::BodyInterface::modify_mutable_compound)
    /// once a body uses it. Static compounds are faster to query; use this one for things that change, like a ship
    /// being built or a destructible object.
    pub fn mutable_compound(sub_shapes: &[CompoundSubShape]) -> Result<Self> {
        crate::ensure_init();
        let subs = sys_sub_shapes(sub_shapes);
        let count = u32::try_from(subs.len()).map_err(|_| Error::new("too many sub shapes"))?;
        Self::from_raw_or_error(unsafe { sys::JPC_MutableCompoundShape_Create(subs.as_ptr(), count) })
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

    /// Number of children of a static or mutable compound, 0 for any other shape.
    pub fn sub_shape_count(&self) -> usize {
        unsafe { sys::JPC_CompoundShape_GetNumSubShapes(self.as_raw()) as usize }
    }

    /// A child of a static or mutable compound, placed as it was when added or last moved.
    pub fn sub_shape(&self, index: usize) -> Option<CompoundSubShape> {
        sub_shape(self.as_raw(), index)
    }

    /// All children of a static or mutable compound; empty for any other shape.
    pub fn sub_shapes(&self) -> Vec<CompoundSubShape> {
        (0..self.sub_shape_count()).filter_map(|i| self.sub_shape(i)).collect()
    }

    /// Splits a sub shape id from a hit on this compound into the index of the child that was hit and the id
    /// within that child. `None` if this is not a compound or the id names no child.
    pub fn compound_child(&self, sub_shape_id: SubShapeId) -> Option<(usize, SubShapeId)> {
        let (mut index, mut remainder) = (0u32, 0u32);
        unsafe { sys::JPC_CompoundShape_GetSubShapeIndexFromID(self.as_raw(), sub_shape_id.0, &mut index, &mut remainder) }
            .then_some((index as usize, SubShapeId(remainder)))
    }

    /// Edits a [`mutable_compound`](Self::mutable_compound) in place. Fails if the shape is another kind or is
    /// shared with another handle or body; to edit the shape of a body use
    /// [`BodyInterface::modify_mutable_compound`](crate::BodyInterface::modify_mutable_compound).
    pub fn edit_mutable_compound(&mut self) -> Result<MutableCompound<'_>> {
        self.ensure_unique()?;
        if self.sub_type() != ShapeSubType::MutableCompound {
            return Err(Error::new("shape is not a mutable compound"));
        }
        Ok(MutableCompound { raw: self.raw, _shape: PhantomData })
    }

    /// Saves the shape and all its children in Jolt's binary format, e.g. to bake meshes or compounds at build
    /// time. Children used more than once are stored once. Materials are saved by user data.
    pub fn to_bytes(&self) -> Vec<u8> {
        unsafe extern "C" fn append(user_data: *mut c_void, data: *const c_void, size: usize) {
            // SAFETY: user_data is the Vec below and data points at size readable bytes.
            unsafe { (*user_data.cast::<Vec<u8>>()).extend_from_slice(std::slice::from_raw_parts(data.cast::<u8>(), size)) };
        }
        let mut bytes = Vec::new();
        unsafe { sys::JPC_Shape_SaveBinary(self.as_raw(), Some(append), (&mut bytes as *mut Vec<u8>).cast()) };
        bytes
    }

    /// Loads a shape saved with [`to_bytes`](Self::to_bytes). Materials come back as new [`PhysicsMaterial`]s with
    /// the saved user data (one per distinct material). Data from another Jolt version, unknown shape types and
    /// truncated data are rejected.
    ///
    /// # Safety
    /// Jolt trusts the contents beyond that: the bytes must come from `to_bytes` of this library, with the same
    /// Jolt version, on a platform with the same endianness. Corrupted or untrusted data is undefined behaviour.
    pub unsafe fn from_bytes(bytes: &[u8]) -> Result<Self> {
        crate::ensure_init();
        Self::from_raw_or_error(unsafe { sys::JPC_Shape_RestoreBinary(bytes.as_ptr().cast(), bytes.len()) })
    }
}

/// A triangle from [`Shape::triangles`].
#[derive(Clone, Debug, PartialEq)]
pub struct ShapeTriangle {
    pub vertices: [Vec3; 3],
    /// `None` for the default material.
    pub material: Option<PhysicsMaterial>,
}

/// Where a shape is for [`Shape::triangles`] and the queries against a single shape.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ShapePlacement {
    /// Position of the shape's center of mass.
    pub position: Vec3,
    pub rotation: Quat,
    pub scale: Vec3,
}

impl ShapePlacement {
    pub fn new(position: Vec3, rotation: Quat) -> Self {
        Self { position, rotation, scale: Vec3::splat(1.0) }
    }
}

impl Default for ShapePlacement {
    fn default() -> Self {
        Self::new(Vec3::ZERO, Quat::IDENTITY)
    }
}

unsafe extern "C" fn collect_hit(user_data: *mut c_void, hit: *const sys::JPC_ShapeHit) -> bool {
    // SAFETY: user_data is the Vec of the calling function; the hit is valid during the call.
    unsafe { (*user_data.cast::<Vec<ShapeHit>>()).push(ShapeHit::from(&*hit)) };
    true
}

impl Shape {
    /// Triangulates the shape (convex shapes approximately) as placed, optionally only the part inside `bounds`: for
    /// rendering, navigation meshes and the like.
    pub fn triangles(&self, placement: &ShapePlacement, bounds: Option<AABox>) -> Vec<ShapeTriangle> {
        unsafe extern "C" fn collect(
            user_data: *mut c_void,
            vertices: *const sys::JPC_Vec3,
            materials: *const *const sys::JPC_PhysicsMaterial,
            count: u32,
        ) -> bool {
            // SAFETY: user_data is the Vec below; vertices holds 3 * count and materials count entries.
            let out = unsafe { &mut *user_data.cast::<Vec<ShapeTriangle>>() };
            let (vertices, materials) =
                unsafe { (std::slice::from_raw_parts(vertices, 3 * count as usize), std::slice::from_raw_parts(materials, count as usize)) };
            for (v, &m) in vertices.as_chunks::<3>().0.iter().zip(materials) {
                out.push(ShapeTriangle { vertices: [v[0].into(), v[1].into(), v[2].into()], material: PhysicsMaterial::from_borrowed(m) });
            }
            true
        }
        let mut out: Vec<ShapeTriangle> = Vec::new();
        let raw_bounds = bounds.map(sys::JPC_AABox::from);
        unsafe {
            sys::JPC_Shape_GetTriangles(
                self.as_raw(),
                placement.position.into(),
                placement.rotation.into(),
                placement.scale.into(),
                raw_bounds.as_ref().map_or(std::ptr::null(), |b| b as *const _),
                Some(collect),
                (&mut out as *mut Vec<ShapeTriangle>).cast(),
            )
        };
        out
    }

    /// Casts a ray (`direction` includes its length) against this shape alone, without a physics system. The hit's
    /// body is [`BodyId::INVALID`](crate::BodyId::INVALID).
    pub fn cast_ray(&self, placement: &ShapePlacement, origin: Vec3, direction: Vec3, settings: &RayCastSettings) -> Option<RayCastHit> {
        let raw_settings = settings.to_sys();
        let mut hit = sys::JPC_RayCastHit::default();
        let found = unsafe {
            sys::JPC_Shape_CastRay(
                self.as_raw(),
                placement.position.into(),
                placement.rotation.into(),
                placement.scale.into(),
                origin.into(),
                direction.into(),
                &raw_settings,
                &mut hit,
            )
        };
        if !found {
            return None;
        }
        let mut result = RayCastHit::from(&hit);
        // The C side handed out its own reference for the material; from() took another.
        if !hit.material.is_null() {
            unsafe { sys::JPC_PhysicsMaterial_Release(hit.material) };
        }
        result.body_id = crate::BodyId::INVALID;
        Some(result)
    }

    /// Whether `point` is inside this shape as placed.
    pub fn contains_point(&self, placement: &ShapePlacement, point: Vec3) -> bool {
        unsafe { sys::JPC_Shape_ContainsPoint(self.as_raw(), placement.position.into(), placement.rotation.into(), placement.scale.into(), point.into()) }
    }

    /// Where `query` (scaled, with its center of mass at `query_transform`) overlaps this shape as placed. Shape 1 of
    /// the hits is `query`.
    pub fn collide_shape(
        &self,
        placement: &ShapePlacement,
        query: &Shape,
        query_scale: Vec3,
        query_transform: &Mat44,
        settings: &CollideShapeSettings,
    ) -> Vec<ShapeHit> {
        let transform: sys::JPC_Mat44 = (*query_transform).into();
        let raw_settings = settings.to_sys();
        let mut hits: Vec<ShapeHit> = Vec::new();
        unsafe {
            sys::JPC_Shape_CollideShape(
                self.as_raw(),
                placement.position.into(),
                placement.rotation.into(),
                placement.scale.into(),
                query.as_raw(),
                query_scale.into(),
                &transform,
                &raw_settings,
                Some(collect_hit),
                (&mut hits as *mut Vec<ShapeHit>).cast(),
            )
        };
        hits
    }

    /// Sweeps `query` from `query_transform` along `direction` against this shape as placed.
    #[allow(clippy::too_many_arguments)]
    pub fn cast_shape(
        &self,
        placement: &ShapePlacement,
        query: &Shape,
        query_scale: Vec3,
        query_transform: &Mat44,
        direction: Vec3,
        settings: &ShapeCastSettings,
    ) -> Vec<ShapeHit> {
        let transform: sys::JPC_Mat44 = (*query_transform).into();
        let raw_settings = settings.to_sys();
        let mut hits: Vec<ShapeHit> = Vec::new();
        unsafe {
            sys::JPC_Shape_CastShape(
                self.as_raw(),
                placement.position.into(),
                placement.rotation.into(),
                placement.scale.into(),
                query.as_raw(),
                query_scale.into(),
                &transform,
                direction.into(),
                &raw_settings,
                Some(collect_hit),
                (&mut hits as *mut Vec<ShapeHit>).cast(),
            )
        };
        hits
    }
}

fn sys_sub_shapes(sub_shapes: &[CompoundSubShape]) -> Vec<sys::JPC_CompoundSubShape> {
    sub_shapes
        .iter()
        .map(|s| sys::JPC_CompoundSubShape {
            position: s.position.into(),
            rotation: s.rotation.into(),
            shape: s.shape.as_raw(),
            user_data: s.user_data,
        })
        .collect()
}

fn sub_shape(raw: *const sys::JPC_Shape, index: usize) -> Option<CompoundSubShape> {
    let index = u32::try_from(index).ok()?;
    let mut out = sys::JPC_CompoundSubShape::default();
    if !unsafe { sys::JPC_CompoundShape_GetSubShape(raw, index, &mut out) } {
        return None;
    }
    // SAFETY: GetSubShape hands out a new reference to the child.
    let shape = unsafe { Shape::from_raw(NonNull::new(out.shape.cast_mut())?) };
    Some(CompoundSubShape { position: out.position.into(), rotation: out.rotation.into(), shape, user_data: out.user_data })
}

/// Edits the children of a mutable compound, from [`Shape::edit_mutable_compound`] or
/// [`BodyInterface::modify_mutable_compound`](crate::BodyInterface::modify_mutable_compound).
///
/// Child positions are where each child's origin sits in the compound. Editing changes what sub shape ids from
/// earlier hits refer to.
pub struct MutableCompound<'a> {
    raw: NonNull<sys::JPC_Shape>,
    _shape: PhantomData<&'a mut Shape>,
}

impl MutableCompound<'_> {
    /// # Safety
    /// `raw` must be a mutable compound that nothing else uses for `'a`.
    pub(crate) unsafe fn from_raw<'a>(raw: NonNull<sys::JPC_Shape>) -> MutableCompound<'a> {
        MutableCompound { raw, _shape: PhantomData }
    }

    fn ptr(&self) -> *mut sys::JPC_Shape {
        self.raw.as_ptr()
    }

    fn check(ok: bool) -> Result<()> {
        if ok { Ok(()) } else { Err(Error::last()) }
    }

    pub fn len(&self) -> usize {
        unsafe { sys::JPC_CompoundShape_GetNumSubShapes(self.ptr()) as usize }
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn sub_shape(&self, index: usize) -> Option<CompoundSubShape> {
        sub_shape(self.ptr(), index)
    }

    /// Appends a child and returns its index.
    pub fn push(&mut self, sub_shape: &CompoundSubShape) -> Result<usize> {
        self.add(u32::MAX, sub_shape)
    }

    /// Inserts a child before `index` (`index == len()` appends), shifting the later children up.
    pub fn insert(&mut self, index: usize, sub_shape: &CompoundSubShape) -> Result<usize> {
        if index > self.len() {
            return Err(Error::new("sub shape index out of range"));
        }
        self.add(index as u32, sub_shape)
    }

    fn add(&mut self, index: u32, s: &CompoundSubShape) -> Result<usize> {
        let added = unsafe {
            sys::JPC_MutableCompoundShape_AddShape(self.ptr(), s.position.into(), s.rotation.into(), s.shape.as_raw(), s.user_data, index)
        };
        if added == u32::MAX { Err(Error::last()) } else { Ok(added as usize) }
    }

    /// Removes a child, shifting the later children down.
    pub fn remove(&mut self, index: usize) -> Result<()> {
        let index = u32::try_from(index).map_err(|_| Error::new("sub shape index out of range"))?;
        Self::check(unsafe { sys::JPC_MutableCompoundShape_RemoveShape(self.ptr(), index) })
    }

    /// Moves a child.
    pub fn set_transform(&mut self, index: usize, position: Vec3, rotation: Quat) -> Result<()> {
        let index = u32::try_from(index).map_err(|_| Error::new("sub shape index out of range"))?;
        Self::check(unsafe {
            sys::JPC_MutableCompoundShape_ModifyShape(self.ptr(), index, position.into(), rotation.into(), std::ptr::null())
        })
    }

    /// Moves a child and swaps its shape (keeping its user data).
    pub fn replace(&mut self, index: usize, position: Vec3, rotation: Quat, shape: &Shape) -> Result<()> {
        let index = u32::try_from(index).map_err(|_| Error::new("sub shape index out of range"))?;
        Self::check(unsafe { sys::JPC_MutableCompoundShape_ModifyShape(self.ptr(), index, position.into(), rotation.into(), shape.as_raw()) })
    }

    /// Moves the children from `start` on to `transforms`, faster than one at a time.
    pub fn set_transforms(&mut self, start: usize, transforms: &[(Vec3, Quat)]) -> Result<()> {
        let start = u32::try_from(start).map_err(|_| Error::new("sub shape index out of range"))?;
        let count = u32::try_from(transforms.len()).map_err(|_| Error::new("sub shape index out of range"))?;
        let positions: Vec<sys::JPC_Vec3> = transforms.iter().map(|t| t.0.into()).collect();
        let rotations: Vec<sys::JPC_Quat> = transforms.iter().map(|t| t.1.into()).collect();
        Self::check(unsafe {
            sys::JPC_MutableCompoundShape_ModifyShapes(self.ptr(), start, count, positions.as_ptr(), rotations.as_ptr(), std::ptr::null())
        })
    }

    /// Moves the children from `start` on and swaps their shapes (keeping their user data), faster than one at a time.
    pub fn replace_many(&mut self, start: usize, children: &[(Vec3, Quat, &Shape)]) -> Result<()> {
        let start = u32::try_from(start).map_err(|_| Error::new("sub shape index out of range"))?;
        let count = u32::try_from(children.len()).map_err(|_| Error::new("sub shape index out of range"))?;
        let positions: Vec<sys::JPC_Vec3> = children.iter().map(|c| c.0.into()).collect();
        let rotations: Vec<sys::JPC_Quat> = children.iter().map(|c| c.1.into()).collect();
        let shapes: Vec<*const sys::JPC_Shape> = children.iter().map(|c| c.2.as_raw()).collect();
        Self::check(unsafe {
            sys::JPC_MutableCompoundShape_ModifyShapes(self.ptr(), start, count, positions.as_ptr(), rotations.as_ptr(), shapes.as_ptr())
        })
    }

    /// Moves the compound's center of mass to that of its children. Call it after changes that shift the mass, so
    /// the body rotates around the right point; child positions don't change.
    pub fn adjust_center_of_mass(&mut self) {
        unsafe { sys::JPC_MutableCompoundShape_AdjustCenterOfMass(self.ptr()) };
    }

    pub fn center_of_mass(&self) -> Vec3 {
        unsafe { sys::JPC_Shape_GetCenterOfMass(self.ptr()) }.into()
    }

    pub fn local_bounds(&self) -> AABox {
        unsafe { sys::JPC_Shape_GetLocalBounds(self.ptr()) }.into()
    }
}

impl fmt::Debug for MutableCompound<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("MutableCompound").field("len", &self.len()).finish()
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
