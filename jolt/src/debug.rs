//! Debug drawing of bodies, constraints and skeletons through your own line and triangle renderer. Needs the
//! `debug-renderer` feature; without it [`DebugRenderer::new`] fails.
//!
//! ```no_run
//! # use jolt::*;
//! struct Lines(Vec<(Vec3, Vec3, Color)>);
//!
//! impl DebugDraw for Lines {
//!     fn draw_line(&mut self, from: Vec3, to: Vec3, color: Color) {
//!         self.0.push((from, to, color));
//!     }
//! }
//!
//! # fn demo(system: &mut PhysicsSystem) -> Result<()> {
//! let mut renderer = DebugRenderer::new()?;
//! let mut lines = Lines(Vec::new());
//! system.draw_bodies(&mut renderer, &BodyDrawSettings::default(), &mut lines, None);
//! system.draw_constraints(&mut renderer, &mut lines);
//! # Ok(()) }
//! ```

use std::ffi::c_void;
use std::fmt;
use std::marker::PhantomData;
use std::ptr::NonNull;

use jolt_sys as sys;

use crate::listener::PanicSlot;
use crate::{Body, BodyId, ContactManifold, ContactSettings, Error, Mat44, PhysicsSystem, Result, Skeleton, Vec3};

/// An 8 bit per channel color.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct Color {
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub a: u8,
}

impl Color {
    pub const fn new(r: u8, g: u8, b: u8, a: u8) -> Self {
        Self { r, g, b, a }
    }
}

impl From<sys::JPC_Color> for Color {
    fn from(c: sys::JPC_Color) -> Self {
        Self { r: c.r, g: c.g, b: c.b, a: c.a }
    }
}

/// Where debug drawing ends up: implement it on top of your renderer.
pub trait DebugDraw {
    fn draw_line(&mut self, from: Vec3, to: Vec3, color: Color);

    /// Defaults to drawing the triangle's edges.
    fn draw_triangle(&mut self, v1: Vec3, v2: Vec3, v3: Vec3, color: Color, cast_shadow: bool) {
        let _ = cast_shadow;
        self.draw_line(v1, v2, color);
        self.draw_line(v2, v3, color);
        self.draw_line(v3, v1, color);
    }

    /// Defaults to drawing nothing.
    fn draw_text(&mut self, position: Vec3, text: &str, color: Color, height: f32) {
        let _ = (position, text, color, height);
    }
}

/// How [`PhysicsSystem::draw_bodies`] colors shapes.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ShapeColor {
    /// A random color per body.
    Instance,
    /// Convex green, scaled yellow, compound orange, mesh red.
    ShapeType,
    /// Static grey, kinematic green, dynamic a random color per body.
    #[default]
    MotionType,
    /// Static grey, kinematic green, dynamic yellow, sleeping red.
    Sleep,
    /// A color per simulation island.
    Island,
    Material,
}

/// How soft body constraints are colored.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SoftBodyConstraintColor {
    #[default]
    Type,
    /// Constraints solved in parallel share a color.
    Group,
    /// Like `Group`, with a gradient for the order within a group.
    Order,
}

/// What [`PhysicsSystem::draw_bodies`] draws.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BodyDrawSettings {
    pub draw_shape: bool,
    pub draw_shape_wireframe: bool,
    pub shape_color: ShapeColor,
    pub draw_bounding_box: bool,
    pub draw_center_of_mass_transform: bool,
    pub draw_world_transform: bool,
    pub draw_velocity: bool,
    pub draw_mass_and_inertia: bool,
    pub draw_sleep_stats: bool,
    /// Collision detection internals: support functions and supporting faces of convex shapes.
    pub draw_get_support_function: bool,
    pub draw_support_direction: bool,
    pub draw_get_supporting_face: bool,
    pub draw_soft_body_vertices: bool,
    pub draw_soft_body_vertex_velocities: bool,
    pub draw_soft_body_edge_constraints: bool,
    pub draw_soft_body_bend_constraints: bool,
    pub draw_soft_body_volume_constraints: bool,
    pub draw_soft_body_skin_constraints: bool,
    pub draw_soft_body_lra_constraints: bool,
    pub draw_soft_body_rods: bool,
    pub draw_soft_body_rod_states: bool,
    pub draw_soft_body_rod_bend_twist_constraints: bool,
    pub draw_soft_body_predicted_bounds: bool,
    pub soft_body_constraint_color: SoftBodyConstraintColor,
}

impl Default for BodyDrawSettings {
    fn default() -> Self {
        let mut d = sys::JPC_BodyDrawSettings::default();
        unsafe { sys::JPC_BodyDrawSettings_SetDefault(&mut d) };
        Self {
            draw_shape: d.draw_shape,
            draw_shape_wireframe: d.draw_shape_wireframe,
            shape_color: ShapeColor::MotionType,
            draw_bounding_box: d.draw_bounding_box,
            draw_center_of_mass_transform: d.draw_center_of_mass_transform,
            draw_world_transform: d.draw_world_transform,
            draw_velocity: d.draw_velocity,
            draw_mass_and_inertia: d.draw_mass_and_inertia,
            draw_sleep_stats: d.draw_sleep_stats,
            draw_get_support_function: d.draw_get_support_function,
            draw_support_direction: d.draw_support_direction,
            draw_get_supporting_face: d.draw_get_supporting_face,
            draw_soft_body_vertices: d.draw_soft_body_vertices,
            draw_soft_body_vertex_velocities: d.draw_soft_body_vertex_velocities,
            draw_soft_body_edge_constraints: d.draw_soft_body_edge_constraints,
            draw_soft_body_bend_constraints: d.draw_soft_body_bend_constraints,
            draw_soft_body_volume_constraints: d.draw_soft_body_volume_constraints,
            draw_soft_body_skin_constraints: d.draw_soft_body_skin_constraints,
            draw_soft_body_lra_constraints: d.draw_soft_body_lra_constraints,
            draw_soft_body_rods: d.draw_soft_body_rods,
            draw_soft_body_rod_states: d.draw_soft_body_rod_states,
            draw_soft_body_rod_bend_twist_constraints: d.draw_soft_body_rod_bend_twist_constraints,
            draw_soft_body_predicted_bounds: d.draw_soft_body_predicted_bounds,
            soft_body_constraint_color: SoftBodyConstraintColor::Type,
        }
    }
}

impl BodyDrawSettings {
    fn to_sys(self) -> sys::JPC_BodyDrawSettings {
        sys::JPC_BodyDrawSettings {
            draw_get_support_function: self.draw_get_support_function,
            draw_support_direction: self.draw_support_direction,
            draw_get_supporting_face: self.draw_get_supporting_face,
            draw_shape: self.draw_shape,
            draw_shape_wireframe: self.draw_shape_wireframe,
            shape_color: match self.shape_color {
                ShapeColor::Instance => sys::JPC_SHAPE_COLOR_INSTANCE,
                ShapeColor::ShapeType => sys::JPC_SHAPE_COLOR_SHAPE_TYPE,
                ShapeColor::MotionType => sys::JPC_SHAPE_COLOR_MOTION_TYPE,
                ShapeColor::Sleep => sys::JPC_SHAPE_COLOR_SLEEP,
                ShapeColor::Island => sys::JPC_SHAPE_COLOR_ISLAND,
                ShapeColor::Material => sys::JPC_SHAPE_COLOR_MATERIAL,
            },
            draw_bounding_box: self.draw_bounding_box,
            draw_center_of_mass_transform: self.draw_center_of_mass_transform,
            draw_world_transform: self.draw_world_transform,
            draw_velocity: self.draw_velocity,
            draw_mass_and_inertia: self.draw_mass_and_inertia,
            draw_sleep_stats: self.draw_sleep_stats,
            draw_soft_body_vertices: self.draw_soft_body_vertices,
            draw_soft_body_vertex_velocities: self.draw_soft_body_vertex_velocities,
            draw_soft_body_edge_constraints: self.draw_soft_body_edge_constraints,
            draw_soft_body_bend_constraints: self.draw_soft_body_bend_constraints,
            draw_soft_body_volume_constraints: self.draw_soft_body_volume_constraints,
            draw_soft_body_skin_constraints: self.draw_soft_body_skin_constraints,
            draw_soft_body_lra_constraints: self.draw_soft_body_lra_constraints,
            draw_soft_body_rods: self.draw_soft_body_rods,
            draw_soft_body_rod_states: self.draw_soft_body_rod_states,
            draw_soft_body_rod_bend_twist_constraints: self.draw_soft_body_rod_bend_twist_constraints,
            draw_soft_body_predicted_bounds: self.draw_soft_body_predicted_bounds,
            soft_body_constraint_color: match self.soft_body_constraint_color {
                SoftBodyConstraintColor::Type => sys::JPC_SOFT_BODY_CONSTRAINT_COLOR_TYPE,
                SoftBodyConstraintColor::Group => sys::JPC_SOFT_BODY_CONSTRAINT_COLOR_GROUP,
                SoftBodyConstraintColor::Order => sys::JPC_SOFT_BODY_CONSTRAINT_COLOR_ORDER,
            },
        }
    }
}

/// Jolt's debug renderer: turns bodies, constraints and skeletons into lines, triangles and text for a
/// [`DebugDraw`]. It keeps simplified geometry of the shapes it has drawn, so keep one around rather than creating one
/// per frame. Jolt allows only one at a time.
pub struct DebugRenderer {
    raw: NonNull<sys::JPC_DebugRenderer>,
    // Jolt's renderer is not thread-safe
    _not_sync: PhantomData<*mut ()>,
}

// SAFETY: the renderer is only used through `&mut self`, from whichever single thread owns it.
unsafe impl Send for DebugRenderer {}

impl DebugRenderer {
    /// Whether Jolt was built with its debug renderer (the `debug-renderer` feature).
    pub fn is_available() -> bool {
        crate::ensure_init();
        unsafe { sys::JPC_DebugRenderer_IsAvailable() }
    }

    /// Fails without the `debug-renderer` feature or while another `DebugRenderer` exists.
    pub fn new() -> Result<Self> {
        crate::ensure_init();
        let raw = unsafe { sys::JPC_DebugRenderer_Create(std::ptr::null(), std::ptr::null_mut()) };
        NonNull::new(raw).map(|raw| Self { raw, _not_sync: PhantomData }).ok_or_else(Error::last)
    }

    /// Detailed shapes are simplified with distance from the camera.
    pub fn set_camera_position(&mut self, position: Vec3) {
        unsafe { sys::JPC_DebugRenderer_SetCameraPosition(self.raw.as_ptr(), position.into()) }
    }

    /// Runs `f` with the renderer pointed at `draw`, then resumes any panic from `draw`.
    fn with<R>(&mut self, draw: &mut dyn DebugDraw, f: impl FnOnce(*mut sys::JPC_DebugRenderer) -> R) -> R {
        let mut sink = Sink { draw, panic: PanicSlot::default() };
        let fns = sys::JPC_DebugRendererFns { draw_line: Some(draw_line), draw_triangle: Some(draw_triangle), draw_text: Some(draw_text) };
        unsafe { sys::JPC_DebugRenderer_SetFns(self.raw.as_ptr(), &fns, (&mut sink as *mut Sink<'_>).cast()) };
        let result = f(self.raw.as_ptr());
        unsafe { sys::JPC_DebugRenderer_SetFns(self.raw.as_ptr(), std::ptr::null(), std::ptr::null_mut()) };
        sink.panic.resume();
        result
    }
}

impl Drop for DebugRenderer {
    fn drop(&mut self) {
        unsafe { sys::JPC_DebugRenderer_Destroy(self.raw.as_ptr()) }
    }
}

impl fmt::Debug for DebugRenderer {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("DebugRenderer").finish_non_exhaustive()
    }
}

struct Sink<'a> {
    draw: &'a mut dyn DebugDraw,
    panic: PanicSlot,
}

unsafe fn sink<'a>(user_data: *mut c_void) -> &'a mut Sink<'a> {
    unsafe { &mut *user_data.cast::<Sink<'a>>() }
}

unsafe extern "C" fn draw_line(user_data: *mut c_void, from: sys::JPC_Vec3, to: sys::JPC_Vec3, color: sys::JPC_Color) {
    let s = unsafe { sink(user_data) };
    let draw = &mut *s.draw;
    s.panic.guard((), || draw.draw_line(from.into(), to.into(), color.into()));
}

unsafe extern "C" fn draw_triangle(
    user_data: *mut c_void,
    v1: sys::JPC_Vec3,
    v2: sys::JPC_Vec3,
    v3: sys::JPC_Vec3,
    color: sys::JPC_Color,
    cast_shadow: bool,
) {
    let s = unsafe { sink(user_data) };
    let draw = &mut *s.draw;
    s.panic.guard((), || draw.draw_triangle(v1.into(), v2.into(), v3.into(), color.into(), cast_shadow));
}

unsafe extern "C" fn draw_text(user_data: *mut c_void, position: sys::JPC_Vec3, text: *const std::ffi::c_char, length: usize, color: sys::JPC_Color, height: f32) {
    let s = unsafe { sink(user_data) };
    let bytes = if length == 0 { &[][..] } else { unsafe { std::slice::from_raw_parts(text.cast::<u8>(), length) } };
    let text = String::from_utf8_lossy(bytes);
    let draw = &mut *s.draw;
    s.panic.guard((), || draw.draw_text(position.into(), &text, color.into(), height));
}

struct FilterContext<'a> {
    filter: &'a dyn Fn(BodyId) -> bool,
    panic: PanicSlot,
}

unsafe extern "C" fn body_filter(user_data: *mut c_void, body: sys::JPC_BodyID) -> bool {
    let ctx = unsafe { &*user_data.cast::<FilterContext<'_>>() };
    ctx.panic.guard(true, || (ctx.filter)(BodyId(body)))
}

impl PhysicsSystem {
    /// Draws the bodies in the world, optionally only those `filter` accepts. Every body is locked meanwhile, so
    /// neither `filter` nor `draw` may use the body interface (that deadlocks).
    pub fn draw_bodies(&self, renderer: &mut DebugRenderer, settings: &BodyDrawSettings, draw: &mut dyn DebugDraw, filter: Option<&dyn Fn(BodyId) -> bool>) {
        let raw_settings = settings.to_sys();
        let system = self.raw();
        let ctx = filter.map(|filter| FilterContext { filter, panic: PanicSlot::default() });
        renderer.with(draw, |r| unsafe {
            match &ctx {
                Some(ctx) => sys::JPC_PhysicsSystem_DrawBodies(system, &raw_settings, r, Some(body_filter), (ctx as *const FilterContext<'_>).cast_mut().cast()),
                None => sys::JPC_PhysicsSystem_DrawBodies(system, &raw_settings, r, None, std::ptr::null_mut()),
            }
        });
        if let Some(ctx) = ctx {
            ctx.panic.resume();
        }
    }

    /// Draws every constraint (vehicles included). Takes `&mut self` because Jolt reads the bodies without locking them.
    pub fn draw_constraints(&mut self, renderer: &mut DebugRenderer, draw: &mut dyn DebugDraw) {
        let system = self.raw();
        renderer.with(draw, |r| unsafe { sys::JPC_PhysicsSystem_DrawConstraints(system, r) });
    }

    /// Draws the limits of every constraint.
    pub fn draw_constraint_limits(&mut self, renderer: &mut DebugRenderer, draw: &mut dyn DebugDraw) {
        let system = self.raw();
        renderer.with(draw, |r| unsafe { sys::JPC_PhysicsSystem_DrawConstraintLimits(system, r) });
    }

    /// Draws the reference frames of every constraint.
    pub fn draw_constraint_reference_frames(&mut self, renderer: &mut DebugRenderer, draw: &mut dyn DebugDraw) {
        let system = self.raw();
        renderer.with(draw, |r| unsafe { sys::JPC_PhysicsSystem_DrawConstraintReferenceFrames(system, r) });
    }
}

impl Skeleton {
    /// Draws the skeleton posed with one model space matrix per joint, moved by `root_offset`. Panics unless there is
    /// one matrix per joint.
    pub fn draw_pose(&self, renderer: &mut DebugRenderer, joint_matrices: &[Mat44], root_offset: Vec3, draw_joint_names: bool, draw: &mut dyn DebugDraw) {
        assert_eq!(joint_matrices.len(), self.joint_count(), "one matrix per joint");
        let matrices: Vec<sys::JPC_Mat44> = joint_matrices.iter().map(|&m| m.into()).collect();
        let skeleton = self.as_raw();
        renderer.with(draw, |r| unsafe { sys::JPC_Skeleton_DrawPose(skeleton, matrices.as_ptr(), root_offset.into(), draw_joint_names, r) });
    }
}

/// Estimated outcome of a new contact, from [`estimate_collision_response`].
#[derive(Clone, Debug, PartialEq)]
pub struct CollisionEstimation {
    pub linear_velocity1: Vec3,
    pub angular_velocity1: Vec3,
    pub linear_velocity2: Vec3,
    pub angular_velocity2: Vec3,
    /// Relative to the manifold's base offset.
    pub friction_point: Vec3,
    pub friction_impulse: Vec3,
    pub angular_friction_impulse: f32,
    /// One per contact point (kg m/s).
    pub contact_impulses: Vec<f32>,
}

/// Estimates the impulses and velocities a new contact will cause, assuming no other bodies are involved: e.g. to play
/// an impact sound of the right strength from [`ContactListener::on_contact_added`](crate::ContactListener::on_contact_added).
/// Pass that callback's bodies, manifold and settings. `num_iterations` of 1 skips friction.
pub fn estimate_collision_response(
    body1: &Body,
    body2: &Body,
    manifold: &ContactManifold<'_>,
    settings: &ContactSettings,
    min_velocity_for_restitution: f32,
    num_iterations: u32,
) -> CollisionEstimation {
    let n = manifold.relative_points_on1.len().min(manifold.relative_points_on2.len());
    let raw_manifold = sys::JPC_ContactManifold {
        base_offset: manifold.base_offset.into(),
        world_space_normal: manifold.world_space_normal.into(),
        penetration_depth: manifold.penetration_depth,
        sub_shape_id1: manifold.sub_shape_id1.0,
        sub_shape_id2: manifold.sub_shape_id2.0,
        // Manifolds come from Jolt, which has at most 64 points.
        num_points: n.min(64) as u32,
        relative_points_on1: manifold.relative_points_on1.as_ptr().cast(),
        relative_points_on2: manifold.relative_points_on2.as_ptr().cast(),
        material1: std::ptr::null(),
        material2: std::ptr::null(),
    };
    let mut out = sys::JPC_CollisionEstimation::default();
    // ContactSettings has the C layout (checked in listener.rs).
    unsafe {
        sys::JPC_EstimateCollisionResponse(
            body1.raw(),
            body2.raw(),
            &raw_manifold,
            (settings as *const ContactSettings).cast(),
            min_velocity_for_restitution,
            num_iterations,
            &mut out,
        )
    };
    CollisionEstimation {
        linear_velocity1: out.linear_velocity1.into(),
        angular_velocity1: out.angular_velocity1.into(),
        linear_velocity2: out.linear_velocity2.into(),
        angular_velocity2: out.angular_velocity2.into(),
        friction_point: out.friction_point.into(),
        friction_impulse: out.friction_impulse.into(),
        angular_friction_impulse: out.angular_friction_impulse,
        contact_impulses: out.contact_impulses[..(out.num_contact_impulses as usize).min(64)].to_vec(),
    }
}
