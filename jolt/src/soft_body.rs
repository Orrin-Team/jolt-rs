//! Soft bodies: cloth, ropes, balloons and jelly, simulated as particles (vertices) held together by constraints.
//!
//! ```
//! # use jolt::*;
//! # fn demo(system: &PhysicsSystem, layer: ObjectLayer) -> Result<()> {
//! // A 10 x 10 cloth hanging from its two top corners.
//! let n = 10u32;
//! let mut desc = SoftBodyDesc::default();
//! for y in 0..n {
//!     for x in 0..n {
//!         let pinned = y == n - 1 && (x == 0 || x == n - 1);
//!         let position = Vec3::new(x as f32 * 0.1, y as f32 * 0.1, 0.0);
//!         desc.vertices.push(if pinned { SoftBodyVertex::pinned(position) } else { SoftBodyVertex::new(position) });
//!     }
//! }
//! for y in 0..n - 1 {
//!     for x in 0..n - 1 {
//!         let i = x + y * n;
//!         desc.faces.push(SoftBodyFace::new([i, i + 1, i + n]));
//!         desc.faces.push(SoftBodyFace::new([i + 1, i + n + 1, i + n]));
//!     }
//! }
//! desc.vertex_attributes.push(VertexAttributes::default()); // edge, shear and bend constraints from the faces
//! let cloth = SoftBodySharedSettings::new(&desc)?;
//!
//! let settings = SoftBodyCreationSettings::new(cloth, Vec3::new(0.0, 5.0, 0.0), Quat::IDENTITY, layer);
//! let body = system.body_interface().create_and_add_soft_body(&settings, Activation::Activate)?;
//! let vertices = system.body_interface().soft_body_vertices(body, true).unwrap();
//! # Ok(()) }
//! ```

use std::fmt;
use std::ptr::NonNull;

use jolt_sys as sys;

use crate::{
    Activation, BodyId, BodyInterface, CollisionGroup, Error, Mat44, ObjectLayer, PhysicsMaterial, Quat, RMat44, RVec3, Result, TempAllocator, Vec3,
};

/// A particle as it is created.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoftBodyVertex {
    pub position: Vec3,
    pub velocity: Vec3,
    /// 0 pins the vertex: it only moves when you move it.
    pub inv_mass: f32,
}

impl SoftBodyVertex {
    /// A vertex with a mass of 1 kg.
    pub fn new(position: Vec3) -> Self {
        Self { position, velocity: Vec3::ZERO, inv_mass: 1.0 }
    }

    pub fn pinned(position: Vec3) -> Self {
        Self { position, velocity: Vec3::ZERO, inv_mass: 0.0 }
    }
}

/// A triangle of the surface; it is what queries hit and what touches other bodies.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct SoftBodyFace {
    pub vertices: [u32; 3],
    /// Index into [`SoftBodyDesc::materials`] (0 when there are none).
    pub material_index: u32,
}

impl SoftBodyFace {
    pub fn new(vertices: [u32; 3]) -> Self {
        Self { vertices, material_index: 0 }
    }
}

/// Keeps two vertices at their rest distance. Compliance is the inverse of stiffness: 0 is rigid.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoftBodyEdge {
    pub vertices: [u32; 2],
    pub compliance: f32,
}

/// Keeps the angle between the triangles `(v0, v1, v2)` and `(v1, v0, v3)`, which share the edge `v0-v1`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoftBodyDihedralBend {
    pub vertices: [u32; 4],
    pub compliance: f32,
}

/// Keeps the volume of a tetrahedron.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoftBodyVolume {
    pub vertices: [u32; 4],
    pub compliance: f32,
}

/// Long range attachment: keeps a dynamic vertex (`vertices[1]`) within its rest distance, times the multiplier, of a
/// pinned one (`vertices[0]`). Stops cloth from stretching under its own weight.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoftBodyLra {
    pub vertices: [u32; 2],
    pub max_distance_multiplier: f32,
}

/// Takes a vertex in its bind pose ([`SoftBodyVertex::position`]) into the space of a joint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoftBodyInvBind {
    pub joint_index: u32,
    pub inv_bind: Mat44,
}

/// One joint influence on a skinned vertex.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SkinWeight {
    /// Index into [`SoftBodyDesc::inv_binds`].
    pub inv_bind_index: u32,
    pub weight: f32,
}

/// Skins a vertex to up to 4 joints and limits how far the simulated vertex may move from its skinned position.
#[derive(Clone, Debug, PartialEq)]
pub struct SoftBodySkinned {
    pub vertex: u32,
    /// At most 4, normalized to add up to 1.
    pub weights: Vec<SkinWeight>,
    /// 0 hard skins the vertex; `f32::MAX` leaves it free.
    pub max_distance: f32,
    /// Keeps the vertex from going further than this behind the skinned surface; off when >= `max_distance`.
    pub back_stop_distance: f32,
    pub back_stop_radius: f32,
}

impl SoftBodySkinned {
    /// A vertex that follows the skin exactly.
    pub fn hard(vertex: u32, weights: Vec<SkinWeight>) -> Self {
        Self { vertex, weights, max_distance: 0.0, back_stop_distance: f32::MAX, back_stop_radius: 40.0 }
    }

    /// A vertex that may move up to `max_distance` from its skinned position.
    pub fn within(vertex: u32, weights: Vec<SkinWeight>, max_distance: f32) -> Self {
        Self { max_distance, ..Self::hard(vertex, weights) }
    }
}

/// A rigid segment between two vertices that also has an orientation, for hair, ropes and plants. Every rod should be
/// in at least one [`SoftBodyRodBendTwist`], or it spins freely around its axis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoftBodyRod {
    pub vertices: [u32; 2],
    pub compliance: f32,
}

/// Limits bending and twisting between two rods (indices into [`SoftBodyDesc::rods`]).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoftBodyRodBendTwist {
    pub rods: [u32; 2],
    pub compliance: f32,
}

/// Orientation of a rod of a live soft body, in world space.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoftBodyRodState {
    pub rotation: Quat,
    pub angular_velocity: Vec3,
}

/// Which bend constraints to generate from the faces.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum BendType {
    None,
    /// A distance constraint across each pair of triangles; cheap.
    #[default]
    Distance,
    /// Keeps the angle between triangles; more accurate, also for shapes that are not flat to begin with.
    Dihedral,
}

/// Which long range attachments to generate towards the nearest pinned vertex.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum LraType {
    #[default]
    None,
    EuclideanDistance,
    /// Distance along the edges; better for cloth that drapes over something.
    GeodesicDistance,
}

/// Per vertex settings for the constraints generated from the faces.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VertexAttributes {
    pub compliance: f32,
    pub shear_compliance: f32,
    /// `f32::MAX` for no bend constraints.
    pub bend_compliance: f32,
    pub lra_type: LraType,
    pub lra_max_distance_multiplier: f32,
}

impl Default for VertexAttributes {
    fn default() -> Self {
        let mut d = sys::JPC_SoftBodyVertexAttributes::default();
        unsafe { sys::JPC_SoftBodyVertexAttributes_SetDefault(&mut d) };
        Self {
            compliance: d.compliance,
            shear_compliance: d.shear_compliance,
            bend_compliance: d.bend_compliance,
            lra_type: lra_from_sys(d.lra_type),
            lra_max_distance_multiplier: d.lra_max_distance_multiplier,
        }
    }
}

fn lra_from_sys(raw: sys::JPC_SoftBodyLRAType) -> LraType {
    match raw {
        sys::JPC_SOFT_BODY_LRA_TYPE_EUCLIDEAN_DISTANCE => LraType::EuclideanDistance,
        sys::JPC_SOFT_BODY_LRA_TYPE_GEODESIC_DISTANCE => LraType::GeodesicDistance,
        _ => LraType::None,
    }
}

/// Everything a [`SoftBodySharedSettings`] is made from.
#[derive(Clone, Debug, PartialEq)]
pub struct SoftBodyDesc {
    pub vertices: Vec<SoftBodyVertex>,
    pub faces: Vec<SoftBodyFace>,
    /// Empty means every face uses the default material.
    pub materials: Vec<PhysicsMaterial>,
    /// When not empty, edge, shear, bend and long range constraints are generated from the faces. One per vertex;
    /// when there are fewer, the last one is repeated.
    pub vertex_attributes: Vec<VertexAttributes>,
    pub bend_type: BendType,
    /// Radians; how far from a square two triangles may be to get a shear edge.
    pub angle_tolerance: f32,
    /// Extra constraints on top of the generated ones. Their rest lengths, angles and volumes come from the vertex
    /// positions.
    pub edges: Vec<SoftBodyEdge>,
    pub dihedral_bends: Vec<SoftBodyDihedralBend>,
    pub volumes: Vec<SoftBodyVolume>,
    pub lras: Vec<SoftBodyLra>,
    /// Skinning, e.g. clothing on an animated character; drive it with [`BodyInterface::skin_soft_body`].
    pub inv_binds: Vec<SoftBodyInvBind>,
    pub skinned: Vec<SoftBodySkinned>,
    pub rods: Vec<SoftBodyRod>,
    pub rod_bend_twists: Vec<SoftBodyRodBendTwist>,
}

impl Default for SoftBodyDesc {
    fn default() -> Self {
        let mut d = sys::JPC_SoftBodySharedSettingsDesc::default();
        unsafe { sys::JPC_SoftBodySharedSettingsDesc_SetDefault(&mut d) };
        Self {
            vertices: Vec::new(),
            faces: Vec::new(),
            materials: Vec::new(),
            vertex_attributes: Vec::new(),
            bend_type: BendType::Distance,
            angle_tolerance: d.angle_tolerance,
            edges: Vec::new(),
            dihedral_bends: Vec::new(),
            volumes: Vec::new(),
            lras: Vec::new(),
            inv_binds: Vec::new(),
            skinned: Vec::new(),
            rods: Vec::new(),
            rod_bend_twists: Vec::new(),
        }
    }
}

/// How many of each element a [`SoftBodySharedSettings`] has, generated constraints included.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SoftBodyCounts {
    pub vertices: usize,
    pub faces: usize,
    pub edges: usize,
    pub dihedral_bends: usize,
    pub volumes: usize,
    pub lras: usize,
    pub skinned: usize,
    pub rods: usize,
    pub rod_bend_twists: usize,
    /// Joint matrices [`BodyInterface::skin_soft_body`] needs (highest joint index + 1).
    pub skin_joints: usize,
}

/// The particles and constraints of a soft body. Immutable and reference counted: one settings object can create
/// many bodies, and cloning is cheap.
pub struct SoftBodySharedSettings {
    raw: NonNull<sys::JPC_SoftBodySharedSettings>,
}

// SAFETY: immutable after creation, with an atomic reference count.
unsafe impl Send for SoftBodySharedSettings {}
unsafe impl Sync for SoftBodySharedSettings {}

fn len(len: usize, what: &str) -> Result<u32> {
    u32::try_from(len).map_err(|_| Error::new(format!("too many {what}")))
}

impl SoftBodySharedSettings {
    /// Fails for out of range or repeated vertex indices, degenerate faces, bad material indices, constraints
    /// between vertices at the same position, or negative masses or compliances.
    pub fn new(desc: &SoftBodyDesc) -> Result<Self> {
        crate::ensure_init();
        let vertices: Vec<sys::JPC_SoftBodyVertex> = desc
            .vertices
            .iter()
            .map(|v| sys::JPC_SoftBodyVertex { position: v.position.into(), velocity: v.velocity.into(), inv_mass: v.inv_mass })
            .collect();
        let faces: Vec<sys::JPC_SoftBodyFace> =
            desc.faces.iter().map(|f| sys::JPC_SoftBodyFace { vertices: f.vertices, material_index: f.material_index }).collect();
        let materials: Vec<*const sys::JPC_PhysicsMaterial> = desc.materials.iter().map(|m| m.as_raw()).collect();
        let attributes: Vec<sys::JPC_SoftBodyVertexAttributes> = desc
            .vertex_attributes
            .iter()
            .map(|a| sys::JPC_SoftBodyVertexAttributes {
                compliance: a.compliance,
                shear_compliance: a.shear_compliance,
                bend_compliance: a.bend_compliance,
                lra_type: match a.lra_type {
                    LraType::None => sys::JPC_SOFT_BODY_LRA_TYPE_NONE,
                    LraType::EuclideanDistance => sys::JPC_SOFT_BODY_LRA_TYPE_EUCLIDEAN_DISTANCE,
                    LraType::GeodesicDistance => sys::JPC_SOFT_BODY_LRA_TYPE_GEODESIC_DISTANCE,
                },
                lra_max_distance_multiplier: a.lra_max_distance_multiplier,
            })
            .collect();
        let edges: Vec<sys::JPC_SoftBodyEdge> =
            desc.edges.iter().map(|e| sys::JPC_SoftBodyEdge { vertices: e.vertices, compliance: e.compliance }).collect();
        let bends: Vec<sys::JPC_SoftBodyDihedralBend> =
            desc.dihedral_bends.iter().map(|b| sys::JPC_SoftBodyDihedralBend { vertices: b.vertices, compliance: b.compliance }).collect();
        let volumes: Vec<sys::JPC_SoftBodyVolume> =
            desc.volumes.iter().map(|v| sys::JPC_SoftBodyVolume { vertices: v.vertices, compliance: v.compliance }).collect();
        let inv_binds: Vec<sys::JPC_SoftBodyInvBind> =
            desc.inv_binds.iter().map(|b| sys::JPC_SoftBodyInvBind { joint_index: b.joint_index, inv_bind: b.inv_bind.into() }).collect();
        let skinned: Vec<sys::JPC_SoftBodySkinned> = desc
            .skinned
            .iter()
            .map(|k| {
                if k.weights.len() > 4 {
                    return Err(Error::new("a skinned vertex has at most 4 weights"));
                }
                let mut weights = [sys::JPC_SoftBodySkinWeight::default(); 4];
                for (out, w) in weights.iter_mut().zip(&k.weights) {
                    *out = sys::JPC_SoftBodySkinWeight { inv_bind_index: w.inv_bind_index, weight: w.weight };
                }
                Ok(sys::JPC_SoftBodySkinned {
                    vertex: k.vertex,
                    weights,
                    max_distance: k.max_distance,
                    back_stop_distance: k.back_stop_distance,
                    back_stop_radius: k.back_stop_radius,
                })
            })
            .collect::<Result<_>>()?;
        let rods: Vec<sys::JPC_SoftBodyRod> =
            desc.rods.iter().map(|r| sys::JPC_SoftBodyRod { vertices: r.vertices, compliance: r.compliance }).collect();
        let bend_twists: Vec<sys::JPC_SoftBodyRodBendTwist> =
            desc.rod_bend_twists.iter().map(|b| sys::JPC_SoftBodyRodBendTwist { rods: b.rods, compliance: b.compliance }).collect();
        let lras: Vec<sys::JPC_SoftBodyLRA> = desc
            .lras
            .iter()
            .map(|l| sys::JPC_SoftBodyLRA { vertices: l.vertices, max_distance_multiplier: l.max_distance_multiplier })
            .collect();
        let raw = sys::JPC_SoftBodySharedSettingsDesc {
            vertices: vertices.as_ptr(),
            num_vertices: len(vertices.len(), "vertices")?,
            faces: faces.as_ptr(),
            num_faces: len(faces.len(), "faces")?,
            materials: materials.as_ptr(),
            num_materials: len(materials.len(), "materials")?,
            vertex_attributes: attributes.as_ptr(),
            num_vertex_attributes: len(attributes.len(), "vertex attributes")?,
            bend_type: match desc.bend_type {
                BendType::None => sys::JPC_SOFT_BODY_BEND_TYPE_NONE,
                BendType::Distance => sys::JPC_SOFT_BODY_BEND_TYPE_DISTANCE,
                BendType::Dihedral => sys::JPC_SOFT_BODY_BEND_TYPE_DIHEDRAL,
            },
            angle_tolerance: desc.angle_tolerance,
            edges: edges.as_ptr(),
            num_edges: len(edges.len(), "edges")?,
            dihedral_bends: bends.as_ptr(),
            num_dihedral_bends: len(bends.len(), "bend constraints")?,
            volumes: volumes.as_ptr(),
            num_volumes: len(volumes.len(), "volume constraints")?,
            lras: lras.as_ptr(),
            num_lras: len(lras.len(), "LRA constraints")?,
            inv_binds: inv_binds.as_ptr(),
            num_inv_binds: len(inv_binds.len(), "inverse bind matrices")?,
            skinned: skinned.as_ptr(),
            num_skinned: len(skinned.len(), "skinned vertices")?,
            rods: rods.as_ptr(),
            num_rods: len(rods.len(), "rods")?,
            rod_bend_twists: bend_twists.as_ptr(),
            num_rod_bend_twists: len(bend_twists.len(), "rod bend twist constraints")?,
        };
        Self::wrap(unsafe { sys::JPC_SoftBodySharedSettings_Create(&raw) })
    }

    /// A cube of `grid_size`³ vertices (2 to 64 per side) `spacing` apart, with edge and volume constraints.
    pub fn cube(grid_size: u32, spacing: f32) -> Result<Self> {
        crate::ensure_init();
        Self::wrap(unsafe { sys::JPC_SoftBodySharedSettings_CreateCube(grid_size, spacing) })
    }

    fn wrap(raw: *mut sys::JPC_SoftBodySharedSettings) -> Result<Self> {
        NonNull::new(raw).map(|raw| Self { raw }).ok_or_else(Error::last)
    }

    fn as_raw(&self) -> *const sys::JPC_SoftBodySharedSettings {
        self.raw.as_ptr()
    }

    pub fn counts(&self) -> SoftBodyCounts {
        let mut c = sys::JPC_SoftBodySharedSettingsCounts::default();
        unsafe { sys::JPC_SoftBodySharedSettings_GetCounts(self.as_raw(), &mut c) };
        SoftBodyCounts {
            vertices: c.vertices as usize,
            faces: c.faces as usize,
            edges: c.edges as usize,
            dihedral_bends: c.dihedral_bends as usize,
            volumes: c.volumes as usize,
            lras: c.lras as usize,
            skinned: c.skinned as usize,
            rods: c.rods as usize,
            rod_bend_twists: c.rod_bend_twists as usize,
            skin_joints: c.skin_joints as usize,
        }
    }

    /// The rods in simulation order, which differs from the order in [`SoftBodyDesc::rods`] (and a rod's two
    /// vertices may be swapped). [`BodyInterface::soft_body_rods`] reports rods in this order.
    pub fn rods(&self) -> Vec<SoftBodyRod> {
        let mut out = vec![sys::JPC_SoftBodyRod::default(); self.counts().rods];
        let n = unsafe { sys::JPC_SoftBodySharedSettings_GetRods(self.as_raw(), out.as_mut_ptr(), out.len() as u32) };
        out.truncate(n as usize);
        out.into_iter().map(|r| SoftBodyRod { vertices: r.vertices, compliance: r.compliance }).collect()
    }

    /// The faces, e.g. to render the body with [`BodyInterface::soft_body_vertices`].
    pub fn faces(&self) -> Vec<SoftBodyFace> {
        let mut out = vec![sys::JPC_SoftBodyFace::default(); self.counts().faces];
        let n = unsafe { sys::JPC_SoftBodySharedSettings_GetFaces(self.as_raw(), out.as_mut_ptr(), out.len() as u32) };
        out.truncate(n as usize);
        out.into_iter().map(|f| SoftBodyFace { vertices: f.vertices, material_index: f.material_index }).collect()
    }
}

impl Clone for SoftBodySharedSettings {
    fn clone(&self) -> Self {
        unsafe { sys::JPC_SoftBodySharedSettings_AddRef(self.as_raw()) };
        Self { raw: self.raw }
    }
}

impl Drop for SoftBodySharedSettings {
    fn drop(&mut self) {
        unsafe { sys::JPC_SoftBodySharedSettings_Release(self.as_raw()) }
    }
}

impl PartialEq for SoftBodySharedSettings {
    fn eq(&self, other: &Self) -> bool {
        self.raw == other.raw
    }
}

impl fmt::Debug for SoftBodySharedSettings {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SoftBodySharedSettings").field("counts", &self.counts()).finish()
    }
}

/// Everything needed to create a soft body.
#[derive(Clone, Debug, PartialEq)]
pub struct SoftBodyCreationSettings {
    pub settings: SoftBodySharedSettings,
    pub position: RVec3,
    pub rotation: Quat,
    pub user_data: u64,
    pub object_layer: ObjectLayer,
    pub collision_group: CollisionGroup,
    /// Solver iterations per step; more is stiffer and slower.
    pub num_iterations: u32,
    pub linear_damping: f32,
    pub max_linear_velocity: f32,
    pub restitution: f32,
    pub friction: f32,
    /// Inflates closed bodies (n·R·T of the ideal gas law); 0 for none.
    pub pressure: f32,
    pub gravity_factor: f32,
    /// Keeps vertices this far from other bodies.
    pub vertex_radius: f32,
    /// Keep the body's position at its vertices (turn off for things attached to the static world).
    pub update_position: bool,
    /// Bake the rotation into the vertices and keep the body's rotation identity (more accurate).
    pub make_rotation_identity: bool,
    pub allow_sleeping: bool,
    /// Queries hit faces from both sides.
    pub faces_double_sided: bool,
}

impl SoftBodyCreationSettings {
    pub fn new(settings: SoftBodySharedSettings, position: impl Into<RVec3>, rotation: Quat, object_layer: ObjectLayer) -> Self {
        let mut d = sys::JPC_SoftBodyCreationSettings::default();
        unsafe { sys::JPC_SoftBodyCreationSettings_SetDefault(&mut d) };
        Self {
            settings,
            position: position.into(),
            rotation,
            user_data: d.user_data,
            object_layer,
            collision_group: CollisionGroup::default(),
            num_iterations: d.num_iterations,
            linear_damping: d.linear_damping,
            max_linear_velocity: d.max_linear_velocity,
            restitution: d.restitution,
            friction: d.friction,
            pressure: d.pressure,
            gravity_factor: d.gravity_factor,
            vertex_radius: d.vertex_radius,
            update_position: d.update_position,
            make_rotation_identity: d.make_rotation_identity,
            allow_sleeping: d.allow_sleeping,
            faces_double_sided: d.faces_double_sided,
        }
    }

    fn to_sys(&self) -> sys::JPC_SoftBodyCreationSettings {
        sys::JPC_SoftBodyCreationSettings {
            settings: self.settings.as_raw(),
            position: self.position.into(),
            rotation: self.rotation.into(),
            user_data: self.user_data,
            object_layer: self.object_layer.0,
            collision_group: self.collision_group.to_sys(),
            num_iterations: self.num_iterations,
            linear_damping: self.linear_damping,
            max_linear_velocity: self.max_linear_velocity,
            restitution: self.restitution,
            friction: self.friction,
            pressure: self.pressure,
            gravity_factor: self.gravity_factor,
            vertex_radius: self.vertex_radius,
            update_position: self.update_position,
            make_rotation_identity: self.make_rotation_identity,
            allow_sleeping: self.allow_sleeping,
            faces_double_sided: self.faces_double_sided,
        }
    }
}

/// A particle of a live soft body.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SoftBodyVertexState {
    /// World space or relative to the body's center of mass transform, depending on how it was read or is written.
    pub position: RVec3,
    pub velocity: Vec3,
    /// 0 for pinned vertices.
    pub inv_mass: f32,
}

/// Settings of a live soft body that can be changed.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoftBodyProperties {
    pub num_iterations: u32,
    pub pressure: f32,
    pub vertex_radius: f32,
    pub update_position: bool,
    pub faces_double_sided: bool,
}

impl BodyInterface<'_> {
    /// Creates a soft body without adding it to the world.
    pub fn create_soft_body(&self, settings: &SoftBodyCreationSettings) -> Result<BodyId> {
        let raw = settings.to_sys();
        BodyId::from_sys(unsafe { sys::JPC_BodyInterface_CreateSoftBody(self.as_raw(), &raw) }).ok_or_else(Error::last)
    }

    pub fn create_and_add_soft_body(&self, settings: &SoftBodyCreationSettings, activation: Activation) -> Result<BodyId> {
        let raw = settings.to_sys();
        BodyId::from_sys(unsafe { sys::JPC_BodyInterface_CreateAndAddSoftBody(self.as_raw(), &raw, activation.to_sys()) })
            .ok_or_else(Error::last)
    }

    pub fn is_soft_body(&self, id: BodyId) -> bool {
        unsafe { sys::JPC_BodyInterface_IsSoftBody(self.as_raw(), id.0) }
    }

    /// The settings a soft body was created from; `None` for anything else.
    pub fn soft_body_shared_settings(&self, id: BodyId) -> Option<SoftBodySharedSettings> {
        NonNull::new(unsafe { sys::JPC_BodyInterface_GetSoftBodySharedSettings(self.as_raw(), id.0) }).map(|raw| SoftBodySharedSettings { raw })
    }

    /// The vertices of a soft body, in world space or relative to its center of mass transform; `None` for
    /// anything that is not a soft body.
    pub fn soft_body_vertices(&self, id: BodyId, world_space: bool) -> Option<Vec<SoftBodyVertexState>> {
        let count = unsafe { sys::JPC_BodyInterface_GetSoftBodyVertices(self.as_raw(), id.0, 0, std::ptr::null_mut(), 0, world_space) };
        if count == 0 {
            return None;
        }
        let mut out = vec![sys::JPC_SoftBodyVertexState::default(); count as usize];
        let count = unsafe { sys::JPC_BodyInterface_GetSoftBodyVertices(self.as_raw(), id.0, 0, out.as_mut_ptr(), count, world_space) };
        out.truncate(count as usize);
        Some(
            out.into_iter()
                .map(|v| SoftBodyVertexState { position: v.position.into(), velocity: v.velocity.into(), inv_mass: v.inv_mass })
                .collect(),
        )
    }

    /// Overwrites vertices from `first` on, e.g. to drag pinned vertices around or reset a body. Changing inverse
    /// masses updates the body's mass. The body's bounds catch up at the next update.
    pub fn set_soft_body_vertices(&self, id: BodyId, first: usize, vertices: &[SoftBodyVertexState], world_space: bool, activation: Activation) -> Result<()> {
        let first = u32::try_from(first).map_err(|_| Error::new("vertex range out of bounds"))?;
        let count = u32::try_from(vertices.len()).map_err(|_| Error::new("vertex range out of bounds"))?;
        let raw: Vec<sys::JPC_SoftBodyVertexState> = vertices
            .iter()
            .map(|v| sys::JPC_SoftBodyVertexState { position: v.position.into(), velocity: v.velocity.into(), inv_mass: v.inv_mass })
            .collect();
        let ok = unsafe { sys::JPC_BodyInterface_SetSoftBodyVertices(self.as_raw(), id.0, first, raw.as_ptr(), count, world_space, activation.to_sys()) };
        if ok { Ok(()) } else { Err(Error::last()) }
    }

    pub fn soft_body_properties(&self, id: BodyId) -> Option<SoftBodyProperties> {
        let mut p = sys::JPC_SoftBodyProperties::default();
        unsafe { sys::JPC_BodyInterface_GetSoftBodyProperties(self.as_raw(), id.0, &mut p) }.then_some(SoftBodyProperties {
            num_iterations: p.num_iterations,
            pressure: p.pressure,
            vertex_radius: p.vertex_radius,
            update_position: p.update_position,
            faces_double_sided: p.faces_double_sided,
        })
    }

    /// Fails for anything that is not a soft body or a negative vertex radius.
    pub fn set_soft_body_properties(&self, id: BodyId, properties: &SoftBodyProperties) -> Result<()> {
        let p = sys::JPC_SoftBodyProperties {
            num_iterations: properties.num_iterations,
            pressure: properties.pressure,
            vertex_radius: properties.vertex_radius,
            update_position: properties.update_position,
            faces_double_sided: properties.faces_double_sided,
        };
        if unsafe { sys::JPC_BodyInterface_SetSoftBodyProperties(self.as_raw(), id.0, &p) } { Ok(()) } else { Err(Error::last()) }
    }

    /// Rod orientations in world space, in the order of [`SoftBodySharedSettings::rods`]; `None` for anything that is
    /// not a soft body.
    pub fn soft_body_rods(&self, id: BodyId) -> Option<Vec<SoftBodyRodState>> {
        if !self.is_soft_body(id) {
            return None;
        }
        let count = unsafe { sys::JPC_BodyInterface_GetSoftBodyRods(self.as_raw(), id.0, 0, std::ptr::null_mut(), 0) };
        let mut out = vec![sys::JPC_SoftBodyRodState::default(); count as usize];
        let count = unsafe { sys::JPC_BodyInterface_GetSoftBodyRods(self.as_raw(), id.0, 0, out.as_mut_ptr(), count) };
        out.truncate(count as usize);
        Some(out.into_iter().map(|r| SoftBodyRodState { rotation: r.rotation.into(), angular_velocity: r.angular_velocity.into() }).collect())
    }

    /// Moves the skinned vertices along with world space joint matrices, e.g. from an animated skeleton. Call it
    /// before every update that should follow the animation, and before the first update. `hard_skin_all` teleports
    /// every skinned vertex to the skin. Fails for anything that is not a soft body with skinned vertices, or for
    /// fewer joints than [`SoftBodyCounts::skin_joints`].
    pub fn skin_soft_body(&self, id: BodyId, joint_matrices: &[RMat44], hard_skin_all: bool, temp: &mut TempAllocator) -> Result<()> {
        let joints: Vec<sys::JPC_RMat44> = joint_matrices.iter().map(|&m| m.into()).collect();
        let count = u32::try_from(joints.len()).map_err(|_| Error::new("too many joints"))?;
        let ok = unsafe { sys::JPC_BodyInterface_SkinSoftBody(self.as_raw(), id.0, joints.as_ptr(), count, hard_skin_all, temp.raw()) };
        if ok { Ok(()) } else { Err(Error::last()) }
    }

    /// Turns the distance and back stop limits of skinned vertices on or off (on by default), and scales their max
    /// distances. With them off only hard skinned vertices follow the skin.
    pub fn set_soft_body_skin_constraints(&self, id: BodyId, enabled: bool, max_distance_multiplier: f32) -> Result<()> {
        let ok = unsafe { sys::JPC_BodyInterface_SetSoftBodySkinConstraintsEnabled(self.as_raw(), id.0, enabled, max_distance_multiplier) };
        if ok { Ok(()) } else { Err(Error::last()) }
    }

    /// Volume enclosed by the faces (meaningful for closed bodies); 0 for anything that is not a soft body.
    pub fn soft_body_volume(&self, id: BodyId) -> f32 {
        unsafe { sys::JPC_BodyInterface_GetSoftBodyVolume(self.as_raw(), id.0) }
    }
}
