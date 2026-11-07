use std::ffi::c_void;
use std::fmt;
use std::ptr::NonNull;
use std::sync::{Arc, Mutex};

use jolt_sys as sys;

use crate::constraint::{ConstraintEntry, ConstraintRegistry};
use crate::character::Deferred;
use crate::listener::{ActivationListenerHolder, ContactListenerHolder, PanicSlot, StepListenerHolder};
use crate::{
    BodyActivationListener, BodyId, BodyInterface, ConstraintId, ConstraintMut, ConstraintSettings, ContactListener, Error,
    LayerConfig, Result, StepListener, Vec3,
};

/// Scratch memory for [`PhysicsSystem::update`]. 10 MB is plenty for most scenes.
pub struct TempAllocator {
    raw: NonNull<sys::JPC_TempAllocator>,
}

// SAFETY: the allocator is only used by the thread that owns `&mut` access.
unsafe impl Send for TempAllocator {}

impl TempAllocator {
    pub(crate) fn raw(&mut self) -> *mut sys::JPC_TempAllocator {
        self.raw.as_ptr()
    }

    pub fn new(size_in_bytes: u32) -> Self {
        crate::ensure_init();
        let raw = unsafe { sys::JPC_TempAllocator_Create(size_in_bytes) };
        Self { raw: NonNull::new(raw).expect("JPC_TempAllocator_Create returned null") }
    }
}

impl Default for TempAllocator {
    fn default() -> Self {
        Self::new(10 * 1024 * 1024)
    }
}

impl Drop for TempAllocator {
    fn drop(&mut self) {
        unsafe { sys::JPC_TempAllocator_Destroy(self.raw.as_ptr()) }
    }
}

/// Runs the simulation's jobs. One job system can be shared by several physics systems; at most
/// `max_barriers` of them update at the same time and the rest wait for a free slot.
pub struct JobSystem {
    raw: NonNull<sys::JPC_JobSystem>,
}

// SAFETY: Jolt's job systems are thread-safe.
unsafe impl Send for JobSystem {}
unsafe impl Sync for JobSystem {}

impl JobSystem {
    pub const MAX_PHYSICS_JOBS: u32 = sys::JPC_MAX_PHYSICS_JOBS;
    pub const MAX_PHYSICS_BARRIERS: u32 = sys::JPC_MAX_PHYSICS_BARRIERS;

    /// A pool of worker threads. `None` uses one thread less than the number of CPUs. Allows 8 concurrent updates.
    pub fn thread_pool(num_threads: Option<u32>) -> Self {
        Self::thread_pool_with_limits(Self::MAX_PHYSICS_JOBS, Self::MAX_PHYSICS_BARRIERS, num_threads)
    }

    /// `max_jobs` is raised to at least [`MAX_PHYSICS_JOBS`](Self::MAX_PHYSICS_JOBS) and rounded up to a power of two;
    /// `max_barriers` (concurrent updates) is raised to at least 1.
    pub fn thread_pool_with_limits(max_jobs: u32, max_barriers: u32, num_threads: Option<u32>) -> Self {
        crate::ensure_init();
        let threads = num_threads.map_or(-1, |n| n.min(i32::MAX as u32) as i32);
        let raw = unsafe { sys::JPC_JobSystemThreadPool_Create(max_jobs, max_barriers, threads) };
        Self { raw: NonNull::new(raw).expect("JPC_JobSystemThreadPool_Create returned null") }
    }

    /// Runs every job on the thread that calls `update`. Handy for deterministic debugging.
    pub fn single_threaded() -> Self {
        crate::ensure_init();
        let raw = unsafe { sys::JPC_JobSystemSingleThreaded_Create(Self::MAX_PHYSICS_JOBS) };
        Self { raw: NonNull::new(raw).expect("JPC_JobSystemSingleThreaded_Create returned null") }
    }
}

impl Drop for JobSystem {
    fn drop(&mut self) {
        unsafe { sys::JPC_JobSystem_Destroy(self.raw.as_ptr()) }
    }
}

/// Capacity limits for a [`PhysicsSystem`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PhysicsSystemSettings {
    /// At most 8388608.
    pub max_bodies: u32,
    /// 0 picks a sensible default.
    pub num_body_mutexes: u32,
    /// Maximum pairs of bodies whose bounding boxes overlap in one step.
    pub max_body_pairs: u32,
    /// Maximum contacts between bodies in one step.
    pub max_contact_constraints: u32,
}

impl Default for PhysicsSystemSettings {
    fn default() -> Self {
        Self { max_bodies: 65536, num_body_mutexes: 0, max_body_pairs: 65536, max_contact_constraints: 10240 }
    }
}

/// Global solver settings; see Jolt's `PhysicsSettings` for what each field does.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PhysicsSettings {
    /// Pairs of bodies processed at once; at least 256.
    pub max_in_flight_body_pairs: i32,
    /// Step listeners per batch; at least 1.
    pub step_listeners_batch_size: i32,
    /// At least 1.
    pub step_listener_batches_per_job: i32,
    /// Fraction of position error corrected per step.
    pub baumgarte: f32,
    /// Contacts are created this far before bodies touch (m).
    pub speculative_contact_distance: f32,
    /// Allowed penetration (m).
    pub penetration_slop: f32,
    pub linear_cast_threshold: f32,
    pub linear_cast_max_penetration: f32,
    pub manifold_tolerance: f32,
    pub max_penetration_distance: f32,
    pub body_pair_cache_max_delta_position_sq: f32,
    pub body_pair_cache_cos_max_delta_rotation_div2: f32,
    pub contact_normal_cos_max_delta_rotation: f32,
    pub contact_point_preserve_lambda_max_dist_sq: f32,
    /// Solver velocity iterations; at least 1.
    pub num_velocity_steps: u32,
    /// Solver position iterations.
    pub num_position_steps: u32,
    pub min_velocity_for_restitution: f32,
    /// Seconds a body must be still before it sleeps.
    pub time_before_sleep: f32,
    pub point_velocity_sleep_threshold: f32,
    pub deterministic_simulation: bool,
    pub constraint_warm_start: bool,
    pub use_body_pair_contact_cache: bool,
    pub use_manifold_reduction: bool,
    pub use_large_island_splitter: bool,
    pub allow_sleeping: bool,
    pub check_active_edges: bool,
}

impl PhysicsSettings {
    fn from_sys(r: &sys::JPC_PhysicsSettings) -> Self {
        Self {
            max_in_flight_body_pairs: r.max_in_flight_body_pairs,
            step_listeners_batch_size: r.step_listeners_batch_size,
            step_listener_batches_per_job: r.step_listener_batches_per_job,
            baumgarte: r.baumgarte,
            speculative_contact_distance: r.speculative_contact_distance,
            penetration_slop: r.penetration_slop,
            linear_cast_threshold: r.linear_cast_threshold,
            linear_cast_max_penetration: r.linear_cast_max_penetration,
            manifold_tolerance: r.manifold_tolerance,
            max_penetration_distance: r.max_penetration_distance,
            body_pair_cache_max_delta_position_sq: r.body_pair_cache_max_delta_position_sq,
            body_pair_cache_cos_max_delta_rotation_div2: r.body_pair_cache_cos_max_delta_rotation_div2,
            contact_normal_cos_max_delta_rotation: r.contact_normal_cos_max_delta_rotation,
            contact_point_preserve_lambda_max_dist_sq: r.contact_point_preserve_lambda_max_dist_sq,
            num_velocity_steps: r.num_velocity_steps,
            num_position_steps: r.num_position_steps,
            min_velocity_for_restitution: r.min_velocity_for_restitution,
            time_before_sleep: r.time_before_sleep,
            point_velocity_sleep_threshold: r.point_velocity_sleep_threshold,
            deterministic_simulation: r.deterministic_simulation,
            constraint_warm_start: r.constraint_warm_start,
            use_body_pair_contact_cache: r.use_body_pair_contact_cache,
            use_manifold_reduction: r.use_manifold_reduction,
            use_large_island_splitter: r.use_large_island_splitter,
            allow_sleeping: r.allow_sleeping,
            check_active_edges: r.check_active_edges,
        }
    }

    fn to_sys(self) -> sys::JPC_PhysicsSettings {
        sys::JPC_PhysicsSettings {
            max_in_flight_body_pairs: self.max_in_flight_body_pairs,
            step_listeners_batch_size: self.step_listeners_batch_size,
            step_listener_batches_per_job: self.step_listener_batches_per_job,
            baumgarte: self.baumgarte,
            speculative_contact_distance: self.speculative_contact_distance,
            penetration_slop: self.penetration_slop,
            linear_cast_threshold: self.linear_cast_threshold,
            linear_cast_max_penetration: self.linear_cast_max_penetration,
            manifold_tolerance: self.manifold_tolerance,
            max_penetration_distance: self.max_penetration_distance,
            body_pair_cache_max_delta_position_sq: self.body_pair_cache_max_delta_position_sq,
            body_pair_cache_cos_max_delta_rotation_div2: self.body_pair_cache_cos_max_delta_rotation_div2,
            contact_normal_cos_max_delta_rotation: self.contact_normal_cos_max_delta_rotation,
            contact_point_preserve_lambda_max_dist_sq: self.contact_point_preserve_lambda_max_dist_sq,
            num_velocity_steps: self.num_velocity_steps,
            num_position_steps: self.num_position_steps,
            min_velocity_for_restitution: self.min_velocity_for_restitution,
            time_before_sleep: self.time_before_sleep,
            point_velocity_sleep_threshold: self.point_velocity_sleep_threshold,
            deterministic_simulation: self.deterministic_simulation,
            constraint_warm_start: self.constraint_warm_start,
            use_body_pair_contact_cache: self.use_body_pair_contact_cache,
            use_manifold_reduction: self.use_manifold_reduction,
            use_large_island_splitter: self.use_large_island_splitter,
            allow_sleeping: self.allow_sleeping,
            check_active_edges: self.check_active_edges,
        }
    }
}

/// The step completed, but some contacts were dropped because a buffer was full.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct UpdateError(u32);

impl UpdateError {
    /// Raise `max_contact_constraints`.
    pub fn manifold_cache_full(self) -> bool {
        self.0 & sys::JPC_PHYSICS_UPDATE_ERROR_MANIFOLD_CACHE_FULL != 0
    }

    /// Raise `max_body_pairs`.
    pub fn body_pair_cache_full(self) -> bool {
        self.0 & sys::JPC_PHYSICS_UPDATE_ERROR_BODY_PAIR_CACHE_FULL != 0
    }

    /// Raise `max_contact_constraints`.
    pub fn contact_constraints_full(self) -> bool {
        self.0 & sys::JPC_PHYSICS_UPDATE_ERROR_CONTACT_CONSTRAINTS_FULL != 0
    }

    pub fn bits(self) -> u32 {
        self.0
    }
}

impl fmt::Debug for UpdateError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("UpdateError")
            .field("manifold_cache_full", &self.manifold_cache_full())
            .field("body_pair_cache_full", &self.body_pair_cache_full())
            .field("contact_constraints_full", &self.contact_constraints_full())
            .finish()
    }
}

impl fmt::Display for UpdateError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut parts = Vec::new();
        if self.manifold_cache_full() {
            parts.push("manifold cache full");
        }
        if self.body_pair_cache_full() {
            parts.push("body pair cache full");
        }
        if self.contact_constraints_full() {
            parts.push("contact constraints full");
        }
        write!(f, "physics update dropped contacts: {}", parts.join(", "))
    }
}

impl std::error::Error for UpdateError {}

/// The native system, shared with characters so it outlives them.
pub(crate) struct SystemInner {
    raw: NonNull<sys::JPC_PhysicsSystem>,
    /// Characters dropped while an update could have been running; destroyed at the next safe point.
    deferred: Mutex<Vec<Deferred>>,
}

// SAFETY: see PhysicsSystem.
unsafe impl Send for SystemInner {}
unsafe impl Sync for SystemInner {}

impl SystemInner {
    pub(crate) fn raw(&self) -> *mut sys::JPC_PhysicsSystem {
        self.raw.as_ptr()
    }

    pub(crate) fn defer(&self, item: Deferred) {
        self.deferred.lock().unwrap_or_else(|e| e.into_inner()).push(item);
    }

    /// Must only be called when no update can be running.
    pub(crate) fn flush_deferred(&self) {
        let items = std::mem::take(&mut *self.deferred.lock().unwrap_or_else(|e| e.into_inner()));
        for item in items {
            unsafe { item.destroy() };
        }
    }
}

impl Drop for SystemInner {
    fn drop(&mut self) {
        self.flush_deferred();
        unsafe { sys::JPC_PhysicsSystem_Destroy(self.raw.as_ptr()) };
    }
}

/// A physics world.
///
/// Bodies are created and manipulated through [`body_interface`](Self::body_interface), which can be used
/// from several threads at once. Stepping the simulation needs `&mut self`, so no other access can
/// overlap with an update.
pub struct PhysicsSystem {
    pub(crate) inner: Arc<SystemInner>,
    contact_listener: Option<Box<ContactListenerHolder>>,
    activation_listener: Option<Box<ActivationListenerHolder>>,
    step_listener: Option<(Box<StepListenerHolder>, NonNull<sys::JPC_StepListener>)>,
    constraints: Mutex<ConstraintRegistry>,
}

// SAFETY: shared access only goes through Jolt's locking interfaces and internally synchronised queries;
// everything else needs `&mut self`.
unsafe impl Send for PhysicsSystem {}
unsafe impl Sync for PhysicsSystem {}

impl PhysicsSystem {
    pub fn new(settings: &PhysicsSystemSettings, layers: &LayerConfig) -> Result<Self> {
        crate::ensure_init();
        let raw_settings = sys::JPC_PhysicsSystemSettings {
            max_bodies: settings.max_bodies,
            num_body_mutexes: settings.num_body_mutexes,
            max_body_pairs: settings.max_body_pairs,
            max_contact_constraints: settings.max_contact_constraints,
            num_object_layers: layers.num_object_layers(),
            num_broad_phase_layers: layers.num_broad_phase_layers(),
            object_to_broad_phase_layer: layers.object_to_broad_phase.as_ptr(),
            colliding_pairs: layers.colliding_pairs.as_ptr(),
            num_colliding_pairs: layers.colliding_pairs.len() as u32,
        };
        let raw = NonNull::new(unsafe { sys::JPC_PhysicsSystem_Create(&raw_settings) }).ok_or_else(Error::last)?;
        let inner = Arc::new(SystemInner { raw, deferred: Mutex::default() });
        Ok(Self { inner, contact_listener: None, activation_listener: None, step_listener: None, constraints: Mutex::default() })
    }

    pub(crate) fn raw(&self) -> *mut sys::JPC_PhysicsSystem {
        self.inner.raw()
    }

    pub fn as_raw(&self) -> *mut sys::JPC_PhysicsSystem {
        self.raw()
    }

    /// Advances the simulation by `delta_time` seconds, split into `collision_steps` sub steps
    /// (use 1 at 60 Hz, more when stepping less often).
    ///
    /// Returns an error if contacts were dropped; the step itself still happened. Panics raised by
    /// listeners during the step are resumed here.
    pub fn update(&mut self, delta_time: f32, collision_steps: u32, temp_allocator: &mut TempAllocator, job_system: &JobSystem) -> Result<(), UpdateError> {
        self.inner.flush_deferred();
        let steps = collision_steps.clamp(1, i32::MAX as u32) as i32;
        let error = unsafe { sys::JPC_PhysicsSystem_Update(self.raw(), delta_time, steps, temp_allocator.raw.as_ptr(), job_system.raw.as_ptr()) };
        self.resume_listener_panics();
        if error == sys::JPC_PHYSICS_UPDATE_ERROR_NONE { Ok(()) } else { Err(UpdateError(error)) }
    }

    fn resume_listener_panics(&self) {
        if let Some(holder) = &self.contact_listener {
            holder.panic.resume();
        }
        if let Some(holder) = &self.activation_listener {
            holder.panic.resume();
        }
        if let Some((holder, _)) = &self.step_listener {
            holder.panic.resume();
        }
    }

    /// Destroys characters that were dropped since the last update. Updates do this automatically.
    pub fn flush_dropped_characters(&mut self) {
        self.inner.flush_deferred();
    }

    /// Rebuilds the broad phase trees. Call after adding many bodies at once, before the first update.
    pub fn optimize_broad_phase(&mut self) {
        unsafe { sys::JPC_PhysicsSystem_OptimizeBroadPhase(self.raw()) }
    }

    pub fn gravity(&self) -> Vec3 {
        unsafe { sys::JPC_PhysicsSystem_GetGravity(self.raw()) }.into()
    }

    pub fn set_gravity(&mut self, gravity: Vec3) {
        unsafe { sys::JPC_PhysicsSystem_SetGravity(self.raw(), gravity.into()) }
    }

    pub fn physics_settings(&self) -> PhysicsSettings {
        let mut settings = sys::JPC_PhysicsSettings::default();
        unsafe { sys::JPC_PhysicsSystem_GetPhysicsSettings(self.raw(), &mut settings) };
        PhysicsSettings::from_sys(&settings)
    }

    /// Fails, changing nothing, for values Jolt cannot run with (see the field docs).
    pub fn set_physics_settings(&mut self, settings: &PhysicsSettings) -> Result<()> {
        let raw = settings.to_sys();
        if unsafe { sys::JPC_PhysicsSystem_SetPhysicsSettings(self.raw(), &raw) } { Ok(()) } else { Err(Error::last()) }
    }

    /// Removes the body from the world (if needed) and frees it, along with every constraint that uses it.
    /// Needs exclusive access because a body freed while another thread reads it would be a use after free.
    /// Bodies owned by a character are left alone; destroy the character instead.
    pub fn destroy_body(&mut self, id: BodyId) {
        self.destroy_constraints_of(id);
        unsafe { sys::JPC_BodyInterface_DestroyBody(self.body_interface().as_raw(), id.0) }
    }

    pub fn destroy_bodies(&mut self, ids: &[BodyId]) {
        for &id in ids {
            self.destroy_body(id);
        }
    }

    pub fn num_bodies(&self) -> u32 {
        unsafe { sys::JPC_PhysicsSystem_GetNumBodies(self.raw()) }
    }

    pub fn num_active_bodies(&self) -> u32 {
        unsafe { sys::JPC_PhysicsSystem_GetNumActiveBodies(self.raw()) }
    }

    pub fn max_bodies(&self) -> u32 {
        unsafe { sys::JPC_PhysicsSystem_GetMaxBodies(self.raw()) }
    }

    /// Every body that exists, added to the world or not.
    pub fn bodies(&self) -> Vec<BodyId> {
        collect_ids(self.num_bodies(), |ptr, cap| unsafe { sys::JPC_PhysicsSystem_GetBodies(self.raw(), ptr, cap) })
    }

    pub fn active_bodies(&self) -> Vec<BodyId> {
        collect_ids(self.num_active_bodies(), |ptr, cap| unsafe { sys::JPC_PhysicsSystem_GetActiveBodies(self.raw(), ptr, cap) })
    }

    pub fn body_interface(&self) -> BodyInterface<'_> {
        BodyInterface::new(unsafe { sys::JPC_PhysicsSystem_GetBodyInterface(self.raw()) }, self)
    }

    /// The body interface that skips body locks.
    ///
    /// # Safety
    /// No other thread may access the same bodies while it is used, including through queries.
    pub unsafe fn body_interface_no_lock(&self) -> BodyInterface<'_> {
        BodyInterface::new(unsafe { sys::JPC_PhysicsSystem_GetBodyInterfaceNoLock(self.raw()) }, self)
    }

    /// Replaces the contact listener.
    pub fn set_contact_listener(&mut self, listener: impl ContactListener) {
        let holder = Box::new(ContactListenerHolder { listener: Box::new(listener), panic: PanicSlot::default() });
        let fns = ContactListenerHolder::fns();
        let user_data = (&*holder as *const ContactListenerHolder).cast_mut().cast();
        unsafe { sys::JPC_PhysicsSystem_SetContactListener(self.raw(), &fns, user_data) };
        // The old holder is dropped only after the system stopped pointing at it.
        self.contact_listener = Some(holder);
    }

    pub fn clear_contact_listener(&mut self) {
        unsafe { sys::JPC_PhysicsSystem_SetContactListener(self.raw(), std::ptr::null(), std::ptr::null_mut()) };
        self.contact_listener = None;
    }

    pub fn set_body_activation_listener(&mut self, listener: impl BodyActivationListener) {
        let holder = Box::new(ActivationListenerHolder { listener: Box::new(listener), panic: PanicSlot::default() });
        let fns = ActivationListenerHolder::fns();
        let user_data = (&*holder as *const ActivationListenerHolder).cast_mut().cast();
        unsafe { sys::JPC_PhysicsSystem_SetBodyActivationListener(self.raw(), &fns, user_data) };
        self.activation_listener = Some(holder);
    }

    pub fn clear_body_activation_listener(&mut self) {
        unsafe { sys::JPC_PhysicsSystem_SetBodyActivationListener(self.raw(), std::ptr::null(), std::ptr::null_mut()) };
        self.activation_listener = None;
    }

    /// Replaces the step listener, which runs before every collision step of an update.
    pub fn set_step_listener(&mut self, listener: impl StepListener) {
        self.clear_step_listener();
        let holder = Box::new(StepListenerHolder { listener: Box::new(listener), panic: PanicSlot::default() });
        let user_data = (&*holder as *const StepListenerHolder).cast_mut().cast::<c_void>();
        let raw = unsafe { sys::JPC_PhysicsSystem_AddStepListener(self.raw(), Some(crate::listener::on_step), user_data) };
        self.step_listener = Some((holder, NonNull::new(raw).expect("JPC_PhysicsSystem_AddStepListener returned null")));
    }

    pub fn clear_step_listener(&mut self) {
        if let Some((_, raw)) = self.step_listener.take() {
            unsafe { sys::JPC_PhysicsSystem_RemoveStepListener(self.raw(), raw.as_ptr()) };
        }
    }

    /// Creates a constraint between two bodies and adds it to the simulation.
    pub fn add_constraint(&mut self, settings: &impl ConstraintSettings, body1: BodyId, body2: BodyId) -> Result<ConstraintId> {
        if !body1.is_valid() || !body2.is_valid() {
            return Err(Error::new("use add_constraint_to_world to attach a body to the world"));
        }
        self.create_constraint(settings, body1, body2)
    }

    /// Creates a constraint between a body and the static world and adds it to the simulation.
    pub fn add_constraint_to_world(&mut self, settings: &impl ConstraintSettings, body: BodyId) -> Result<ConstraintId> {
        if !body.is_valid() {
            return Err(Error::new("invalid body id"));
        }
        self.create_constraint(settings, BodyId::INVALID, body)
    }

    fn create_constraint(&mut self, settings: &impl ConstraintSettings, body1: BodyId, body2: BodyId) -> Result<ConstraintId> {
        let raw = NonNull::new(unsafe { settings.create(self.raw(), body1.0, body2.0) }).ok_or_else(Error::last)?;
        unsafe {
            sys::JPC_PhysicsSystem_AddConstraint(self.raw(), raw.as_ptr());
            sys::JPC_BodyInterface_ActivateConstraint(self.body_interface().as_raw(), raw.as_ptr());
        }
        let entry = ConstraintEntry { raw, kind: settings.kind(), body1, body2 };
        Ok(self.constraints.get_mut().unwrap_or_else(|e| e.into_inner()).insert(entry))
    }

    /// Removes and destroys a constraint. Returns false if it no longer exists.
    pub fn remove_constraint(&mut self, id: ConstraintId) -> bool {
        let raw = self.raw();
        match self.constraints.get_mut().unwrap_or_else(|e| e.into_inner()).entries.remove(&id) {
            Some(entry) => {
                unsafe { ConstraintRegistry::release(raw, entry) };
                true
            }
            None => false,
        }
    }

    /// `None` once the constraint was removed or one of its bodies was destroyed.
    pub fn constraint(&mut self, id: ConstraintId) -> Option<ConstraintMut<'_>> {
        let body_interface = self.body_interface().as_raw();
        let registry = self.constraints.get_mut().unwrap_or_else(|e| e.into_inner());
        registry.entries.get(&id).map(|entry| ConstraintMut::new(entry, body_interface))
    }

    pub fn constraint_ids(&self) -> Vec<ConstraintId> {
        self.constraints.lock().unwrap_or_else(|e| e.into_inner()).entries.keys().copied().collect()
    }

    fn destroy_constraints_of(&mut self, body: BodyId) {
        let raw = self.raw();
        let registry = self.constraints.get_mut().unwrap_or_else(|e| e.into_inner());
        let doomed: Vec<ConstraintId> =
            registry.entries.iter().filter(|(_, e)| e.body1 == body || e.body2 == body).map(|(&id, _)| id).collect();
        for id in doomed {
            if let Some(entry) = registry.entries.remove(&id) {
                unsafe { ConstraintRegistry::release(raw, entry) };
            }
        }
    }
}

impl Drop for PhysicsSystem {
    fn drop(&mut self) {
        let raw = self.raw();
        let registry = self.constraints.get_mut().unwrap_or_else(|e| e.into_inner());
        for (_, entry) in registry.entries.drain() {
            unsafe { ConstraintRegistry::release(raw, entry) };
        }
        self.clear_step_listener();
        unsafe {
            sys::JPC_PhysicsSystem_SetContactListener(raw, std::ptr::null(), std::ptr::null_mut());
            sys::JPC_PhysicsSystem_SetBodyActivationListener(raw, std::ptr::null(), std::ptr::null_mut());
        }
        // Nothing can update the system anymore, so dropped characters can go now. The native system itself is
        // destroyed once the last character holding on to it is gone.
        self.inner.flush_deferred();
    }
}

impl fmt::Debug for PhysicsSystem {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PhysicsSystem")
            .field("num_bodies", &self.num_bodies())
            .field("num_active_bodies", &self.num_active_bodies())
            .finish()
    }
}

fn collect_ids(hint: u32, mut fill: impl FnMut(*mut sys::JPC_BodyID, u32) -> u32) -> Vec<BodyId> {
    let mut ids: Vec<u32> = vec![0; hint as usize];
    loop {
        let capacity = ids.len() as u32;
        let total = fill(ids.as_mut_ptr(), capacity);
        if total <= capacity {
            ids.truncate(total as usize);
            return ids.into_iter().map(BodyId).collect();
        }
        ids.resize(total as usize, 0);
    }
}
