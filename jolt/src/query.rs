use std::ffi::c_void;
use std::ops::ControlFlow;

use jolt_sys as sys;

use crate::listener::PanicSlot;
use crate::{AABox, BodyId, Mat44, ObjectLayer, PhysicsMaterial, PhysicsSystem, Shape, SubShapeId, Vec3};

/// Restricts which bodies a query can hit. The default hits everything.
#[derive(Clone, Copy, Default)]
pub struct QueryFilter<'a> {
    /// Only hit bodies whose layer collides with this layer.
    pub object_layer: Option<ObjectLayer>,
    pub ignore_body: Option<BodyId>,
    /// Return `false` to skip a body. Runs on the calling thread while Jolt holds the broad phase query lock,
    /// so it must not touch the physics system.
    pub body_filter: Option<&'a dyn Fn(BodyId) -> bool>,
}

impl<'a> QueryFilter<'a> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_object_layer(mut self, layer: ObjectLayer) -> Self {
        self.object_layer = Some(layer);
        self
    }

    pub fn ignoring(mut self, body: BodyId) -> Self {
        self.ignore_body = Some(body);
        self
    }

    pub fn with_body_filter(mut self, filter: &'a dyn Fn(BodyId) -> bool) -> Self {
        self.body_filter = Some(filter);
        self
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum BackFaceMode {
    #[default]
    Ignore,
    Collide,
}

impl BackFaceMode {
    fn to_sys(self) -> sys::JPC_BackFaceMode {
        match self {
            Self::Ignore => sys::JPC_BACK_FACE_MODE_IGNORE,
            Self::Collide => sys::JPC_BACK_FACE_MODE_COLLIDE,
        }
    }
}

/// Whether shape queries against meshes report hits on edges shared by coplanar-ish triangles.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ActiveEdgeMode {
    /// Skip inactive edges; fewer ghost collisions for simulation.
    #[default]
    CollideOnlyWithActive,
    /// Report every edge.
    CollideWithAll,
}

impl ActiveEdgeMode {
    fn to_sys(self) -> sys::JPC_ActiveEdgeMode {
        match self {
            Self::CollideOnlyWithActive => sys::JPC_ACTIVE_EDGE_MODE_COLLIDE_ONLY_WITH_ACTIVE,
            Self::CollideWithAll => sys::JPC_ACTIVE_EDGE_MODE_COLLIDE_WITH_ALL,
        }
    }
}

fn back_face_from_sys(raw: sys::JPC_BackFaceMode) -> BackFaceMode {
    if raw == sys::JPC_BACK_FACE_MODE_COLLIDE { BackFaceMode::Collide } else { BackFaceMode::Ignore }
}

fn active_edge_from_sys(raw: sys::JPC_ActiveEdgeMode) -> ActiveEdgeMode {
    if raw == sys::JPC_ACTIVE_EDGE_MODE_COLLIDE_WITH_ALL { ActiveEdgeMode::CollideWithAll } else { ActiveEdgeMode::CollideOnlyWithActive }
}

/// Settings for [`PhysicsSystem::collide_shape_with_settings`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CollideShapeSettings {
    /// Also report bodies closer than this (with a negative penetration depth).
    pub max_separation_distance: f32,
    pub back_face_mode: BackFaceMode,
    pub active_edge_mode: ActiveEdgeMode,
    /// Movement direction used to decide which edges are active.
    pub active_edge_movement_direction: Vec3,
    pub collision_tolerance: f32,
    pub penetration_tolerance: f32,
}

impl Default for CollideShapeSettings {
    fn default() -> Self {
        let mut d = sys::JPC_CollideShapeSettings::default();
        unsafe { sys::JPC_CollideShapeSettings_SetDefault(&mut d) };
        Self {
            max_separation_distance: d.max_separation_distance,
            back_face_mode: back_face_from_sys(d.back_face_mode),
            active_edge_mode: active_edge_from_sys(d.active_edge_mode),
            active_edge_movement_direction: d.active_edge_movement_direction.into(),
            collision_tolerance: d.collision_tolerance,
            penetration_tolerance: d.penetration_tolerance,
        }
    }
}

impl CollideShapeSettings {
    fn to_sys(self) -> sys::JPC_CollideShapeSettings {
        sys::JPC_CollideShapeSettings {
            max_separation_distance: self.max_separation_distance,
            back_face_mode: self.back_face_mode.to_sys(),
            active_edge_mode: self.active_edge_mode.to_sys(),
            active_edge_movement_direction: self.active_edge_movement_direction.into(),
            collision_tolerance: self.collision_tolerance,
            penetration_tolerance: self.penetration_tolerance,
        }
    }
}

/// Settings for [`PhysicsSystem::cast_shape_with_settings`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ShapeCastSettings {
    pub back_face_mode_triangles: BackFaceMode,
    pub back_face_mode_convex: BackFaceMode,
    /// Cast the shrunken shape plus convex radius: faster, slightly rounded corners.
    pub use_shrunken_shape_and_convex_radius: bool,
    /// For hits at fraction 0, report the deepest point instead of any point.
    pub return_deepest_point: bool,
    pub active_edge_mode: ActiveEdgeMode,
    pub active_edge_movement_direction: Vec3,
    pub collision_tolerance: f32,
    pub penetration_tolerance: f32,
}

impl Default for ShapeCastSettings {
    fn default() -> Self {
        let mut d = sys::JPC_ShapeCastSettings::default();
        unsafe { sys::JPC_ShapeCastSettings_SetDefault(&mut d) };
        Self {
            back_face_mode_triangles: back_face_from_sys(d.back_face_mode_triangles),
            back_face_mode_convex: back_face_from_sys(d.back_face_mode_convex),
            use_shrunken_shape_and_convex_radius: d.use_shrunken_shape_and_convex_radius,
            return_deepest_point: d.return_deepest_point,
            active_edge_mode: active_edge_from_sys(d.active_edge_mode),
            active_edge_movement_direction: d.active_edge_movement_direction.into(),
            collision_tolerance: d.collision_tolerance,
            penetration_tolerance: d.penetration_tolerance,
        }
    }
}

impl ShapeCastSettings {
    fn to_sys(self) -> sys::JPC_ShapeCastSettings {
        sys::JPC_ShapeCastSettings {
            back_face_mode_triangles: self.back_face_mode_triangles.to_sys(),
            back_face_mode_convex: self.back_face_mode_convex.to_sys(),
            use_shrunken_shape_and_convex_radius: self.use_shrunken_shape_and_convex_radius,
            return_deepest_point: self.return_deepest_point,
            active_edge_mode: self.active_edge_mode.to_sys(),
            active_edge_movement_direction: self.active_edge_movement_direction.into(),
            collision_tolerance: self.collision_tolerance,
            penetration_tolerance: self.penetration_tolerance,
        }
    }
}

/// A body whose bounding box a broad phase cast hit.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BroadPhaseCastHit {
    pub body_id: BodyId,
    pub fraction: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RayCastSettings {
    pub back_face_mode_triangles: BackFaceMode,
    pub back_face_mode_convex: BackFaceMode,
    /// When true a ray starting inside a convex shape hits at fraction 0; otherwise it hits the far side.
    pub treat_convex_as_solid: bool,
}

impl RayCastSettings {
    fn to_sys(self) -> sys::JPC_RayCastSettings {
        sys::JPC_RayCastSettings {
            back_face_mode_triangles: self.back_face_mode_triangles.to_sys(),
            back_face_mode_convex: self.back_face_mode_convex.to_sys(),
            treat_convex_as_solid: self.treat_convex_as_solid,
        }
    }
}

impl Default for RayCastSettings {
    fn default() -> Self {
        Self { back_face_mode_triangles: BackFaceMode::Ignore, back_face_mode_convex: BackFaceMode::Ignore, treat_convex_as_solid: true }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct RayCastHit {
    pub body_id: BodyId,
    pub sub_shape_id: SubShapeId,
    /// Hit point = origin + fraction * direction.
    pub fraction: f32,
    pub point: Vec3,
    /// Surface normal at the hit point.
    pub normal: Vec3,
    /// `None` for the default material.
    pub material: Option<PhysicsMaterial>,
}

impl From<&sys::JPC_RayCastHit> for RayCastHit {
    fn from(h: &sys::JPC_RayCastHit) -> Self {
        Self {
            body_id: BodyId(h.body_id),
            sub_shape_id: SubShapeId(h.sub_shape_id),
            fraction: h.fraction,
            point: h.point.into(),
            normal: h.normal.into(),
            material: PhysicsMaterial::from_borrowed(h.material),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PointHit {
    pub body_id: BodyId,
    pub sub_shape_id: SubShapeId,
}

/// A hit from [`PhysicsSystem::collide_shape`] or [`PhysicsSystem::cast_shape`]. Shape 1 is the query shape.
#[derive(Clone, Debug, PartialEq)]
pub struct ShapeHit {
    pub body_id: BodyId,
    pub sub_shape_id1: SubShapeId,
    pub sub_shape_id2: SubShapeId,
    pub contact_point_on1: Vec3,
    pub contact_point_on2: Vec3,
    /// Direction to move the hit body out of collision. Its length is meaningless.
    pub penetration_axis: Vec3,
    pub penetration_depth: f32,
    /// How far along the cast the hit happened (0 for `collide_shape`).
    pub fraction: f32,
    pub is_back_face_hit: bool,
    /// Material of the hit body at `sub_shape_id2`, `None` for the default material.
    pub material: Option<PhysicsMaterial>,
}

impl ShapeHit {
    /// Contact normal pointing from the hit body towards the query shape.
    pub fn normal(&self) -> Vec3 {
        -self.penetration_axis.normalize_or_zero()
    }
}

impl From<&sys::JPC_ShapeHit> for ShapeHit {
    fn from(h: &sys::JPC_ShapeHit) -> Self {
        Self {
            body_id: BodyId(h.body_id),
            sub_shape_id1: SubShapeId(h.sub_shape_id1),
            sub_shape_id2: SubShapeId(h.sub_shape_id2),
            contact_point_on1: h.contact_point_on1.into(),
            contact_point_on2: h.contact_point_on2.into(),
            penetration_axis: h.penetration_axis.into(),
            penetration_depth: h.penetration_depth,
            fraction: h.fraction,
            is_back_face_hit: h.is_back_face_hit,
            material: PhysicsMaterial::from_borrowed(h.material),
        }
    }
}

/// Everything a query's callbacks need, kept alive on the stack for the duration of the call.
struct QueryContext<'a, F> {
    filter: QueryFilter<'a>,
    collector: F,
    panic: PanicSlot,
}

impl<'a, F> QueryContext<'a, F> {
    fn new(filter: &QueryFilter<'a>, collector: F) -> Self {
        Self { filter: *filter, collector, panic: PanicSlot::default() }
    }

    /// The returned filter points at `this`, which must stay put until the query finishes. The same
    /// pointer is handed to the collector so the C side never sees two competing borrows.
    fn sys_filter(this: *mut Self) -> sys::JPC_QueryFilter {
        let filter = unsafe { &(*this).filter };
        sys::JPC_QueryFilter {
            use_object_layer: filter.object_layer.is_some(),
            object_layer: filter.object_layer.map_or(0, |l| l.0),
            ignore_body: filter.ignore_body.map_or(sys::JPC_BODY_ID_INVALID, |b| b.0),
            body_filter: filter.body_filter.is_some().then_some(body_filter_trampoline::<F> as _),
            body_filter_user_data: this.cast(),
        }
    }

    fn finish(self) {
        self.panic.resume();
    }
}

/// Hits are copied out while Jolt holds the hit body's lock and handed to the user's closure only after the native
/// query returned, so the closure is free to use the body interface. Breaking only stops delivering hits.
fn deliver<T>(hits: Vec<T>, mut collector: impl FnMut(&T) -> ControlFlow<()>) {
    for hit in &hits {
        if collector(hit).is_break() {
            break;
        }
    }
}

unsafe extern "C" fn body_filter_trampoline<F>(user_data: *mut c_void, body: sys::JPC_BodyID) -> bool {
    let ctx = unsafe { &*user_data.cast::<QueryContext<'_, F>>() };
    let filter = ctx.filter.body_filter.expect("body filter set");
    ctx.panic.guard(false, || filter(BodyId(body)))
}

// The collector of a query context is the hit buffer. Converting a hit takes a reference on its material while
// the body is still locked, so the material outlives the query.
unsafe extern "C" fn ray_trampoline(user_data: *mut c_void, hit: *const sys::JPC_RayCastHit) -> bool {
    let ctx = unsafe { &mut *user_data.cast::<QueryContext<'_, Vec<RayCastHit>>>() };
    ctx.collector.push(RayCastHit::from(unsafe { &*hit }));
    true
}

unsafe extern "C" fn point_trampoline(user_data: *mut c_void, hit: *const sys::JPC_CollidePointHit) -> bool {
    let ctx = unsafe { &mut *user_data.cast::<QueryContext<'_, Vec<PointHit>>>() };
    let raw = unsafe { &*hit };
    ctx.collector.push(PointHit { body_id: BodyId(raw.body_id), sub_shape_id: SubShapeId(raw.sub_shape_id) });
    true
}

unsafe extern "C" fn body_trampoline(user_data: *mut c_void, body: sys::JPC_BodyID) -> bool {
    let ctx = unsafe { &mut *user_data.cast::<QueryContext<'_, Vec<BodyId>>>() };
    ctx.collector.push(BodyId(body));
    true
}

unsafe extern "C" fn cast_trampoline(user_data: *mut c_void, hit: *const sys::JPC_BroadPhaseCastHit) -> bool {
    let ctx = unsafe { &mut *user_data.cast::<QueryContext<'_, Vec<BroadPhaseCastHit>>>() };
    let hit = unsafe { &*hit };
    ctx.collector.push(BroadPhaseCastHit { body_id: BodyId(hit.body_id), fraction: hit.fraction });
    true
}

unsafe extern "C" fn shape_trampoline(user_data: *mut c_void, hit: *const sys::JPC_ShapeHit) -> bool {
    let ctx = unsafe { &mut *user_data.cast::<QueryContext<'_, Vec<ShapeHit>>>() };
    ctx.collector.push(ShapeHit::from(unsafe { &*hit }));
    true
}

impl PhysicsSystem {
    /// Closest hit along `direction` (whose length is the ray length).
    pub fn cast_ray(&self, origin: Vec3, direction: Vec3, filter: &QueryFilter<'_>) -> Option<RayCastHit> {
        let mut ctx = QueryContext::new(filter, ());
        let sys_filter = QueryContext::sys_filter(&mut ctx);
        let mut hit = sys::JPC_RayCastHit::default();
        let found = unsafe { sys::JPC_PhysicsSystem_CastRay(self.raw(), origin.into(), direction.into(), &sys_filter, &mut hit) };
        ctx.finish();
        // The material of the closest hit comes back as an owned reference.
        found.then(|| {
            let material = PhysicsMaterial::from_owned(hit.material);
            hit.material = std::ptr::null();
            RayCastHit { material, ..RayCastHit::from(&hit) }
        })
    }

    /// Calls `collector` for every hit, in no particular order, until it returns `ControlFlow::Break`. Hits are
    /// delivered after the query finished, so the collector may use the body interface.
    pub fn cast_ray_all<F>(&self, origin: Vec3, direction: Vec3, settings: &RayCastSettings, filter: &QueryFilter<'_>, collector: F)
    where
        F: FnMut(&RayCastHit) -> ControlFlow<()>,
    {
        let raw_settings = settings.to_sys();
        let mut ctx = QueryContext::new(filter, Vec::new());
        let ctx_ptr: *mut QueryContext<'_, _> = &mut ctx;
        let sys_filter = QueryContext::sys_filter(ctx_ptr);
        unsafe {
            sys::JPC_PhysicsSystem_CastRayAll(
                self.raw(),
                origin.into(),
                direction.into(),
                &raw_settings,
                &sys_filter,
                Some(ray_trampoline),
                ctx_ptr.cast(),
            )
        };
        let hits = std::mem::take(&mut ctx.collector);
        ctx.finish();
        deliver(hits, collector);
    }

    /// Bodies containing `point`.
    pub fn collide_point<F>(&self, point: Vec3, filter: &QueryFilter<'_>, collector: F)
    where
        F: FnMut(&PointHit) -> ControlFlow<()>,
    {
        let mut ctx = QueryContext::new(filter, Vec::new());
        let ctx_ptr: *mut QueryContext<'_, _> = &mut ctx;
        let sys_filter = QueryContext::sys_filter(ctx_ptr);
        unsafe {
            sys::JPC_PhysicsSystem_CollidePoint(self.raw(), point.into(), &sys_filter, Some(point_trampoline), ctx_ptr.cast())
        };
        let hits = std::mem::take(&mut ctx.collector);
        ctx.finish();
        deliver(hits, collector);
    }

    /// Bodies overlapping `shape` placed at `center_of_mass_transform`. With `max_separation_distance > 0`
    /// bodies closer than that distance are reported too, with a negative penetration depth.
    pub fn collide_shape<F>(
        &self,
        shape: &Shape,
        scale: Vec3,
        center_of_mass_transform: &Mat44,
        max_separation_distance: f32,
        filter: &QueryFilter<'_>,
        collector: F,
    ) where
        F: FnMut(&ShapeHit) -> ControlFlow<()>,
    {
        let settings = CollideShapeSettings { max_separation_distance, ..Default::default() };
        self.collide_shape_with_settings(shape, scale, center_of_mass_transform, &settings, filter, collector);
    }

    /// [`collide_shape`](Self::collide_shape) with full control over back faces, active edges and tolerances.
    pub fn collide_shape_with_settings<F>(
        &self,
        shape: &Shape,
        scale: Vec3,
        center_of_mass_transform: &Mat44,
        settings: &CollideShapeSettings,
        filter: &QueryFilter<'_>,
        collector: F,
    ) where
        F: FnMut(&ShapeHit) -> ControlFlow<()>,
    {
        let transform: sys::JPC_Mat44 = (*center_of_mass_transform).into();
        let raw_settings = settings.to_sys();
        let mut ctx = QueryContext::new(filter, Vec::new());
        let ctx_ptr: *mut QueryContext<'_, _> = &mut ctx;
        let sys_filter = QueryContext::sys_filter(ctx_ptr);
        unsafe {
            sys::JPC_PhysicsSystem_CollideShapeWithSettings(
                self.raw(),
                shape.as_raw(),
                scale.into(),
                &transform,
                &raw_settings,
                &sys_filter,
                Some(shape_trampoline),
                ctx_ptr.cast(),
            )
        };
        let hits = std::mem::take(&mut ctx.collector);
        ctx.finish();
        deliver(hits, collector);
    }

    /// Sweeps `shape` from `center_of_mass_transform` along `direction` and reports every hit.
    pub fn cast_shape<F>(
        &self,
        shape: &Shape,
        scale: Vec3,
        center_of_mass_transform: &Mat44,
        direction: Vec3,
        filter: &QueryFilter<'_>,
        collector: F,
    ) where
        F: FnMut(&ShapeHit) -> ControlFlow<()>,
    {
        self.cast_shape_with_settings(shape, scale, center_of_mass_transform, direction, &ShapeCastSettings::default(), filter, collector);
    }

    /// [`cast_shape`](Self::cast_shape) with full control over back faces, active edges and tolerances.
    #[allow(clippy::too_many_arguments)]
    pub fn cast_shape_with_settings<F>(
        &self,
        shape: &Shape,
        scale: Vec3,
        center_of_mass_transform: &Mat44,
        direction: Vec3,
        settings: &ShapeCastSettings,
        filter: &QueryFilter<'_>,
        collector: F,
    ) where
        F: FnMut(&ShapeHit) -> ControlFlow<()>,
    {
        let transform: sys::JPC_Mat44 = (*center_of_mass_transform).into();
        let raw_settings = settings.to_sys();
        let mut ctx = QueryContext::new(filter, Vec::new());
        let ctx_ptr: *mut QueryContext<'_, _> = &mut ctx;
        let sys_filter = QueryContext::sys_filter(ctx_ptr);
        unsafe {
            sys::JPC_PhysicsSystem_CastShapeWithSettings(
                self.raw(),
                shape.as_raw(),
                scale.into(),
                &transform,
                direction.into(),
                &raw_settings,
                &sys_filter,
                Some(shape_trampoline),
                ctx_ptr.cast(),
            )
        };
        let hits = std::mem::take(&mut ctx.collector);
        ctx.finish();
        deliver(hits, collector);
    }

    /// Closest hit with explicit back face and solidity settings.
    pub fn cast_ray_with_settings(
        &self,
        origin: Vec3,
        direction: Vec3,
        settings: &RayCastSettings,
        filter: &QueryFilter<'_>,
    ) -> Option<RayCastHit> {
        let raw_settings = settings.to_sys();
        let mut ctx = QueryContext::new(filter, ());
        let sys_filter = QueryContext::sys_filter(&mut ctx);
        let mut hit = sys::JPC_RayCastHit::default();
        let found = unsafe {
            sys::JPC_PhysicsSystem_CastRayWithSettings(self.raw(), origin.into(), direction.into(), &raw_settings, &sys_filter, &mut hit)
        };
        ctx.finish();
        found.then(|| {
            let material = PhysicsMaterial::from_owned(hit.material);
            hit.material = std::ptr::null();
            RayCastHit { material, ..RayCastHit::from(&hit) }
        })
    }

    /// Bodies whose bounding boxes overlap `bounds`. Broad phase queries only test bounding boxes, so they are
    /// cheap but may report bodies whose shapes don't actually touch.
    pub fn broad_phase_collide_aabox(&self, bounds: &AABox, filter: &QueryFilter<'_>, collector: impl FnMut(BodyId) -> ControlFlow<()>) {
        let raw = sys::JPC_AABox { min: bounds.min.into(), max: bounds.max.into() };
        self.broad_phase_bodies(filter, collector, |sys_filter, ud| unsafe {
            sys::JPC_PhysicsSystem_BroadPhaseCollideAABox(self.raw(), &raw, sys_filter, Some(body_trampoline), ud)
        });
    }

    pub fn broad_phase_collide_sphere(&self, center: Vec3, radius: f32, filter: &QueryFilter<'_>, collector: impl FnMut(BodyId) -> ControlFlow<()>) {
        self.broad_phase_bodies(filter, collector, |sys_filter, ud| unsafe {
            sys::JPC_PhysicsSystem_BroadPhaseCollideSphere(self.raw(), center.into(), radius, sys_filter, Some(body_trampoline), ud)
        });
    }

    pub fn broad_phase_collide_point(&self, point: Vec3, filter: &QueryFilter<'_>, collector: impl FnMut(BodyId) -> ControlFlow<()>) {
        self.broad_phase_bodies(filter, collector, |sys_filter, ud| unsafe {
            sys::JPC_PhysicsSystem_BroadPhaseCollidePoint(self.raw(), point.into(), sys_filter, Some(body_trampoline), ud)
        });
    }

    /// A box of `half_extent` placed by `orientation` (rotation and translation).
    pub fn broad_phase_collide_oriented_box(
        &self,
        orientation: &Mat44,
        half_extent: Vec3,
        filter: &QueryFilter<'_>,
        collector: impl FnMut(BodyId) -> ControlFlow<()>,
    ) {
        let raw: sys::JPC_Mat44 = (*orientation).into();
        self.broad_phase_bodies(filter, collector, |sys_filter, ud| unsafe {
            sys::JPC_PhysicsSystem_BroadPhaseCollideOrientedBox(self.raw(), &raw, half_extent.into(), sys_filter, Some(body_trampoline), ud)
        });
    }

    pub fn broad_phase_cast_ray(
        &self,
        origin: Vec3,
        direction: Vec3,
        filter: &QueryFilter<'_>,
        collector: impl FnMut(&BroadPhaseCastHit) -> ControlFlow<()>,
    ) {
        self.broad_phase_casts(filter, collector, |sys_filter, ud| unsafe {
            sys::JPC_PhysicsSystem_BroadPhaseCastRay(self.raw(), origin.into(), direction.into(), sys_filter, Some(cast_trampoline), ud)
        });
    }

    pub fn broad_phase_cast_aabox(
        &self,
        bounds: &AABox,
        direction: Vec3,
        filter: &QueryFilter<'_>,
        collector: impl FnMut(&BroadPhaseCastHit) -> ControlFlow<()>,
    ) {
        let raw = sys::JPC_AABox { min: bounds.min.into(), max: bounds.max.into() };
        self.broad_phase_casts(filter, collector, |sys_filter, ud| unsafe {
            sys::JPC_PhysicsSystem_BroadPhaseCastAABox(self.raw(), &raw, direction.into(), sys_filter, Some(cast_trampoline), ud)
        });
    }

    fn broad_phase_bodies(
        &self,
        filter: &QueryFilter<'_>,
        collector: impl FnMut(BodyId) -> ControlFlow<()>,
        query: impl FnOnce(&sys::JPC_QueryFilter, *mut c_void),
    ) {
        let mut ctx = QueryContext::new(filter, Vec::<BodyId>::new());
        let ctx_ptr: *mut QueryContext<'_, _> = &mut ctx;
        let sys_filter = QueryContext::sys_filter(ctx_ptr);
        query(&sys_filter, ctx_ptr.cast());
        let hits = std::mem::take(&mut ctx.collector);
        ctx.finish();
        let mut collector = collector;
        deliver(hits, |id: &BodyId| collector(*id));
    }

    fn broad_phase_casts(
        &self,
        filter: &QueryFilter<'_>,
        collector: impl FnMut(&BroadPhaseCastHit) -> ControlFlow<()>,
        query: impl FnOnce(&sys::JPC_QueryFilter, *mut c_void),
    ) {
        let mut ctx = QueryContext::new(filter, Vec::<BroadPhaseCastHit>::new());
        let ctx_ptr: *mut QueryContext<'_, _> = &mut ctx;
        let sys_filter = QueryContext::sys_filter(ctx_ptr);
        query(&sys_filter, ctx_ptr.cast());
        let hits = std::mem::take(&mut ctx.collector);
        ctx.finish();
        deliver(hits, collector);
    }

    /// Closest hit of a shape sweep.
    pub fn cast_shape_closest(
        &self,
        shape: &Shape,
        scale: Vec3,
        center_of_mass_transform: &Mat44,
        direction: Vec3,
        filter: &QueryFilter<'_>,
    ) -> Option<ShapeHit> {
        let mut closest: Option<ShapeHit> = None;
        self.cast_shape(shape, scale, center_of_mass_transform, direction, filter, |hit| {
            if closest.as_ref().is_none_or(|c| hit.fraction < c.fraction) {
                closest = Some(hit.clone());
            }
            ControlFlow::Continue(())
        });
        closest
    }
}
