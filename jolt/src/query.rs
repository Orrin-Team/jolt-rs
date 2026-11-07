use std::ffi::c_void;
use std::ops::ControlFlow;

use jolt_sys as sys;

use crate::listener::PanicSlot;
use crate::{BodyId, Mat44, ObjectLayer, PhysicsMaterial, PhysicsSystem, Shape, SubShapeId, Vec3};

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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RayCastSettings {
    pub back_face_mode_triangles: BackFaceMode,
    pub back_face_mode_convex: BackFaceMode,
    /// When true a ray starting inside a convex shape hits at fraction 0; otherwise it hits the far side.
    pub treat_convex_as_solid: bool,
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
        let raw_settings = sys::JPC_RayCastSettings {
            back_face_mode_triangles: settings.back_face_mode_triangles.to_sys(),
            back_face_mode_convex: settings.back_face_mode_convex.to_sys(),
            treat_convex_as_solid: settings.treat_convex_as_solid,
        };
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
        let transform: sys::JPC_Mat44 = (*center_of_mass_transform).into();
        let mut ctx = QueryContext::new(filter, Vec::new());
        let ctx_ptr: *mut QueryContext<'_, _> = &mut ctx;
        let sys_filter = QueryContext::sys_filter(ctx_ptr);
        unsafe {
            sys::JPC_PhysicsSystem_CollideShape(
                self.raw(),
                shape.as_raw(),
                scale.into(),
                &transform,
                max_separation_distance,
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
        let transform: sys::JPC_Mat44 = (*center_of_mass_transform).into();
        let mut ctx = QueryContext::new(filter, Vec::new());
        let ctx_ptr: *mut QueryContext<'_, _> = &mut ctx;
        let sys_filter = QueryContext::sys_filter(ctx_ptr);
        unsafe {
            sys::JPC_PhysicsSystem_CastShape(
                self.raw(),
                shape.as_raw(),
                scale.into(),
                &transform,
                direction.into(),
                &sys_filter,
                Some(shape_trampoline),
                ctx_ptr.cast(),
            )
        };
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
