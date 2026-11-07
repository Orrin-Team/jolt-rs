use std::any::Any;
use std::ffi::c_void;
use std::mem::offset_of;
use std::panic::{AssertUnwindSafe, catch_unwind, resume_unwind};
use std::sync::Mutex;

use jolt_sys as sys;

use crate::{Activation, Body, BodyId, MotionType, PhysicsMaterial, Quat, SubShapeId, Vec3};

/// Holds the first panic raised inside a callback that C++ called, so it can be resumed on the Rust side.
#[derive(Default)]
pub(crate) struct PanicSlot(Mutex<Option<Box<dyn Any + Send>>>);

impl PanicSlot {
    pub(crate) fn guard<R>(&self, fallback: R, f: impl FnOnce() -> R) -> R {
        match catch_unwind(AssertUnwindSafe(f)) {
            Ok(value) => value,
            Err(payload) => {
                let mut slot = self.0.lock().unwrap_or_else(|e| e.into_inner());
                slot.get_or_insert(payload);
                fallback
            }
        }
    }

    pub(crate) fn resume(&self) {
        let payload = self.0.lock().unwrap_or_else(|e| e.into_inner()).take();
        if let Some(payload) = payload {
            resume_unwind(payload);
        }
    }
}

/// What to do with a contact that is about to be created.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ValidateResult {
    #[default]
    AcceptAllContactsForThisBodyPair,
    AcceptContact,
    RejectContact,
    RejectAllContactsForThisBodyPair,
}

impl ValidateResult {
    fn to_sys(self) -> sys::JPC_ValidateResult {
        match self {
            Self::AcceptAllContactsForThisBodyPair => sys::JPC_VALIDATE_RESULT_ACCEPT_ALL_CONTACTS,
            Self::AcceptContact => sys::JPC_VALIDATE_RESULT_ACCEPT_CONTACT,
            Self::RejectContact => sys::JPC_VALIDATE_RESULT_REJECT_CONTACT,
            Self::RejectAllContactsForThisBodyPair => sys::JPC_VALIDATE_RESULT_REJECT_ALL_CONTACTS,
        }
    }
}

/// A potential contact passed to [`ContactListener::on_contact_validate`]. Points are relative to the base offset.
#[derive(Clone, Copy, Debug)]
pub struct CollideShapeResult {
    pub contact_point_on1: Vec3,
    pub contact_point_on2: Vec3,
    pub penetration_axis: Vec3,
    pub penetration_depth: f32,
    pub sub_shape_id1: SubShapeId,
    pub sub_shape_id2: SubShapeId,
    pub body_id2: BodyId,
}

/// The contact points between two bodies.
#[derive(Clone, Debug)]
pub struct ContactManifold<'a> {
    pub base_offset: Vec3,
    /// Direction to move body 2 out of collision.
    pub world_space_normal: Vec3,
    pub penetration_depth: f32,
    pub sub_shape_id1: SubShapeId,
    pub sub_shape_id2: SubShapeId,
    /// Contact points on body 1, relative to `base_offset`.
    pub relative_points_on1: &'a [Vec3],
    /// Contact points on body 2, relative to `base_offset`.
    pub relative_points_on2: &'a [Vec3],
    /// Material of body 1 at `sub_shape_id1`, `None` for the default material.
    pub material1: Option<PhysicsMaterial>,
    pub material2: Option<PhysicsMaterial>,
}

impl ContactManifold<'_> {
    pub fn world_point_on1(&self, index: usize) -> Vec3 {
        self.base_offset + self.relative_points_on1[index]
    }

    pub fn world_point_on2(&self, index: usize) -> Vec3 {
        self.base_offset + self.relative_points_on2[index]
    }
}

/// Contact properties that [`ContactListener::on_contact_added`] and
/// [`ContactListener::on_contact_persisted`] may change.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct ContactSettings {
    pub combined_friction: f32,
    pub combined_restitution: f32,
    pub inv_mass_scale1: f32,
    pub inv_inertia_scale1: f32,
    pub inv_mass_scale2: f32,
    pub inv_inertia_scale2: f32,
    /// Report the contact but don't resolve it.
    pub is_sensor: bool,
    /// Surface velocity of body 2 relative to body 1, e.g. for conveyor belts.
    pub relative_linear_surface_velocity: Vec3,
    pub relative_angular_surface_velocity: Vec3,
}

const _: () = {
    type S = sys::JPC_ContactSettings;
    assert!(size_of::<ContactSettings>() == size_of::<S>());
    assert!(offset_of!(ContactSettings, combined_friction) == offset_of!(S, combined_friction));
    assert!(offset_of!(ContactSettings, inv_inertia_scale2) == offset_of!(S, inv_inertia_scale2));
    assert!(offset_of!(ContactSettings, is_sensor) == offset_of!(S, is_sensor));
    assert!(offset_of!(ContactSettings, relative_linear_surface_velocity) == offset_of!(S, relative_linear_surface_velocity));
    assert!(offset_of!(ContactSettings, relative_angular_surface_velocity) == offset_of!(S, relative_angular_surface_velocity));
};

/// Identifies a contact that went away. The bodies may already be gone, so only ids are given.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct SubShapeIdPair {
    pub body1: BodyId,
    pub sub_shape_id1: SubShapeId,
    pub body2: BodyId,
    pub sub_shape_id2: SubShapeId,
}

/// Receives contact events during [`PhysicsSystem::update`](crate::PhysicsSystem::update).
///
/// Callbacks run on Jolt's worker threads, possibly several at once, so implementations must be
/// thread-safe. A panic inside a callback is caught and resumed when `update` returns.
pub trait ContactListener: Send + Sync + 'static {
    /// Called before a contact is created; can reject it. Expensive work here slows down the simulation.
    fn on_contact_validate(&self, body1: &Body, body2: &Body, base_offset: Vec3, result: &CollideShapeResult) -> ValidateResult {
        let _ = (body1, body2, base_offset, result);
        ValidateResult::AcceptAllContactsForThisBodyPair
    }

    /// Two bodies started touching.
    fn on_contact_added(&self, body1: &Body, body2: &Body, manifold: &ContactManifold<'_>, settings: &mut ContactSettings) {
        let _ = (body1, body2, manifold, settings);
    }

    /// Two bodies that were touching last update are still touching.
    fn on_contact_persisted(&self, body1: &Body, body2: &Body, manifold: &ContactManifold<'_>, settings: &mut ContactSettings) {
        let _ = (body1, body2, manifold, settings);
    }

    /// Two bodies stopped touching.
    fn on_contact_removed(&self, pair: &SubShapeIdPair) {
        let _ = pair;
    }
}

/// Receives sleep and wake events. Called from worker threads during updates and from whichever thread
/// wakes a body through the body interface. A panic is caught and resumed by the next `update`.
pub trait BodyActivationListener: Send + Sync + 'static {
    fn on_body_activated(&self, body: BodyId, user_data: u64) {
        let _ = (body, user_data);
    }

    fn on_body_deactivated(&self, body: BodyId, user_data: u64) {
        let _ = (body, user_data);
    }
}

pub(crate) struct ContactListenerHolder {
    pub(crate) listener: Box<dyn ContactListener>,
    pub(crate) panic: PanicSlot,
}

pub(crate) struct ActivationListenerHolder {
    pub(crate) listener: Box<dyn BodyActivationListener>,
    pub(crate) panic: PanicSlot,
}

impl ContactListenerHolder {
    pub(crate) fn fns() -> sys::JPC_ContactListenerFns {
        sys::JPC_ContactListenerFns {
            on_contact_validate: Some(on_contact_validate),
            on_contact_added: Some(on_contact_added),
            on_contact_persisted: Some(on_contact_persisted),
            on_contact_removed: Some(on_contact_removed),
        }
    }
}

impl ActivationListenerHolder {
    pub(crate) fn fns() -> sys::JPC_BodyActivationListenerFns {
        sys::JPC_BodyActivationListenerFns {
            on_body_activated: Some(on_body_activated),
            on_body_deactivated: Some(on_body_deactivated),
        }
    }
}

/// # Safety
/// The pointers come straight from the C layer and are valid for the duration of the callback.
unsafe fn manifold<'a>(raw: &'a sys::JPC_ContactManifold) -> ContactManifold<'a> {
    let n = raw.num_points as usize;
    unsafe {
        ContactManifold {
            base_offset: raw.base_offset.into(),
            world_space_normal: raw.world_space_normal.into(),
            penetration_depth: raw.penetration_depth,
            sub_shape_id1: SubShapeId(raw.sub_shape_id1),
            sub_shape_id2: SubShapeId(raw.sub_shape_id2),
            relative_points_on1: std::slice::from_raw_parts(raw.relative_points_on1.cast(), n),
            relative_points_on2: std::slice::from_raw_parts(raw.relative_points_on2.cast(), n),
            material1: PhysicsMaterial::from_borrowed(raw.material1),
            material2: PhysicsMaterial::from_borrowed(raw.material2),
        }
    }
}

unsafe extern "C" fn on_contact_validate(
    user_data: *mut c_void,
    body1: *const sys::JPC_Body,
    body2: *const sys::JPC_Body,
    base_offset: *const sys::JPC_Vec3,
    result: *const sys::JPC_CollideShapeResult,
) -> sys::JPC_ValidateResult {
    let holder = unsafe { &*user_data.cast::<ContactListenerHolder>() };
    let accept = ValidateResult::AcceptAllContactsForThisBodyPair.to_sys();
    holder.panic.guard(accept, || unsafe {
        let r = &*result;
        let result = CollideShapeResult {
            contact_point_on1: r.contact_point_on1.into(),
            contact_point_on2: r.contact_point_on2.into(),
            penetration_axis: r.penetration_axis.into(),
            penetration_depth: r.penetration_depth,
            sub_shape_id1: SubShapeId(r.sub_shape_id1),
            sub_shape_id2: SubShapeId(r.sub_shape_id2),
            body_id2: BodyId(r.body_id2),
        };
        holder
            .listener
            .on_contact_validate(Body::from_raw(body1), Body::from_raw(body2), (*base_offset).into(), &result)
            .to_sys()
    })
}

unsafe extern "C" fn on_contact_added(
    user_data: *mut c_void,
    body1: *const sys::JPC_Body,
    body2: *const sys::JPC_Body,
    raw_manifold: *const sys::JPC_ContactManifold,
    settings: *mut sys::JPC_ContactSettings,
) {
    let holder = unsafe { &*user_data.cast::<ContactListenerHolder>() };
    holder.panic.guard((), || unsafe {
        let manifold = manifold(&*raw_manifold);
        let settings = &mut *settings.cast::<ContactSettings>();
        holder.listener.on_contact_added(Body::from_raw(body1), Body::from_raw(body2), &manifold, settings);
    })
}

unsafe extern "C" fn on_contact_persisted(
    user_data: *mut c_void,
    body1: *const sys::JPC_Body,
    body2: *const sys::JPC_Body,
    raw_manifold: *const sys::JPC_ContactManifold,
    settings: *mut sys::JPC_ContactSettings,
) {
    let holder = unsafe { &*user_data.cast::<ContactListenerHolder>() };
    holder.panic.guard((), || unsafe {
        let manifold = manifold(&*raw_manifold);
        let settings = &mut *settings.cast::<ContactSettings>();
        holder.listener.on_contact_persisted(Body::from_raw(body1), Body::from_raw(body2), &manifold, settings);
    })
}

unsafe extern "C" fn on_contact_removed(user_data: *mut c_void, pair: *const sys::JPC_SubShapeIDPair) {
    let holder = unsafe { &*user_data.cast::<ContactListenerHolder>() };
    holder.panic.guard((), || unsafe {
        let p = &*pair;
        holder.listener.on_contact_removed(&SubShapeIdPair {
            body1: BodyId(p.body1_id),
            sub_shape_id1: SubShapeId(p.sub_shape_id1),
            body2: BodyId(p.body2_id),
            sub_shape_id2: SubShapeId(p.sub_shape_id2),
        });
    })
}

unsafe extern "C" fn on_body_activated(user_data: *mut c_void, body: sys::JPC_BodyID, body_user_data: u64) {
    let holder = unsafe { &*user_data.cast::<ActivationListenerHolder>() };
    holder.panic.guard((), || holder.listener.on_body_activated(BodyId(body), body_user_data))
}

unsafe extern "C" fn on_body_deactivated(user_data: *mut c_void, body: sys::JPC_BodyID, body_user_data: u64) {
    let holder = unsafe { &*user_data.cast::<ActivationListenerHolder>() };
    holder.panic.guard((), || holder.listener.on_body_deactivated(BodyId(body), body_user_data))
}

/// Runs before every collision step of [`PhysicsSystem::update`](crate::PhysicsSystem::update), on a worker
/// thread, while the simulation is paused. Use it for per-step velocity changes and forces that depend on the
/// current state. Note that forces added with [`StepBodies::add_force`] accumulate until the end of the whole
/// update, not just the current step. A panic is caught and resumed when `update` returns.
pub trait StepListener: Send + Sync + 'static {
    fn on_step(&self, context: &StepContext<'_>);
}

/// Passed to [`StepListener::on_step`].
pub struct StepContext<'a> {
    pub delta_time: f32,
    pub is_first_step: bool,
    pub is_last_step: bool,
    /// Access to the bodies for this step.
    pub bodies: StepBodies<'a>,
}

/// Body access from inside a step listener. Bodies can be read and pushed around but not added or removed.
pub struct StepBodies<'a> {
    raw: *mut sys::JPC_BodyInterface,
    _step: std::marker::PhantomData<&'a ()>,
}

impl StepBodies<'_> {
    pub fn position(&self, id: BodyId) -> Vec3 {
        unsafe { sys::JPC_BodyInterface_GetPosition(self.raw, id.0) }.into()
    }

    pub fn rotation(&self, id: BodyId) -> Quat {
        unsafe { sys::JPC_BodyInterface_GetRotation(self.raw, id.0) }.into()
    }

    pub fn center_of_mass_position(&self, id: BodyId) -> Vec3 {
        unsafe { sys::JPC_BodyInterface_GetCenterOfMassPosition(self.raw, id.0) }.into()
    }

    pub fn linear_velocity(&self, id: BodyId) -> Vec3 {
        unsafe { sys::JPC_BodyInterface_GetLinearVelocity(self.raw, id.0) }.into()
    }

    pub fn set_linear_velocity(&self, id: BodyId, velocity: Vec3) {
        unsafe { sys::JPC_BodyInterface_SetLinearVelocity(self.raw, id.0, velocity.into()) }
    }

    pub fn angular_velocity(&self, id: BodyId) -> Vec3 {
        unsafe { sys::JPC_BodyInterface_GetAngularVelocity(self.raw, id.0) }.into()
    }

    pub fn set_angular_velocity(&self, id: BodyId, velocity: Vec3) {
        unsafe { sys::JPC_BodyInterface_SetAngularVelocity(self.raw, id.0, velocity.into()) }
    }

    pub fn add_force(&self, id: BodyId, force: Vec3) {
        unsafe { sys::JPC_BodyInterface_AddForce(self.raw, id.0, force.into(), Activation::Activate.to_sys()) }
    }

    pub fn add_force_at_point(&self, id: BodyId, force: Vec3, point: Vec3) {
        unsafe { sys::JPC_BodyInterface_AddForceAtPoint(self.raw, id.0, force.into(), point.into(), Activation::Activate.to_sys()) }
    }

    pub fn add_torque(&self, id: BodyId, torque: Vec3) {
        unsafe { sys::JPC_BodyInterface_AddTorque(self.raw, id.0, torque.into(), Activation::Activate.to_sys()) }
    }

    pub fn add_impulse(&self, id: BodyId, impulse: Vec3) {
        unsafe { sys::JPC_BodyInterface_AddImpulse(self.raw, id.0, impulse.into()) }
    }

    pub fn is_active(&self, id: BodyId) -> bool {
        unsafe { sys::JPC_BodyInterface_IsActive(self.raw, id.0) }
    }

    pub fn motion_type(&self, id: BodyId) -> MotionType {
        MotionType::from_sys(unsafe { sys::JPC_BodyInterface_GetMotionType(self.raw, id.0) })
    }

    pub fn user_data(&self, id: BodyId) -> u64 {
        unsafe { sys::JPC_BodyInterface_GetUserData(self.raw, id.0) }
    }
}

pub(crate) struct StepListenerHolder {
    pub(crate) listener: Box<dyn StepListener>,
    pub(crate) panic: PanicSlot,
}

pub(crate) unsafe extern "C" fn on_step(user_data: *mut c_void, system: *mut sys::JPC_PhysicsSystem, context: *const sys::JPC_PhysicsStepContext) {
    let holder = unsafe { &*user_data.cast::<StepListenerHolder>() };
    let ctx = unsafe { &*context };
    // Every body is locked during the step, so only the no-lock interface may be used. A system only ever
    // has one Rust step listener, so no other listener touches the bodies concurrently.
    let bodies = StepBodies { raw: unsafe { sys::JPC_PhysicsSystem_GetBodyInterfaceNoLock(system) }, _step: std::marker::PhantomData };
    let context = StepContext { delta_time: ctx.delta_time, is_first_step: ctx.is_first_step, is_last_step: ctx.is_last_step, bodies };
    holder.panic.guard((), || holder.listener.on_step(&context));
}
