//! Character controllers: [`CharacterVirtual`] (kinematic, swept through the world, the usual choice for players)
//! and [`Character`] (a dynamic rigid body kept upright).
//!
//! Both keep their physics system alive. Operations that touch the world take the [`PhysicsSystem`] as an argument,
//! which keeps them from overlapping with [`PhysicsSystem::update`]. Dropping a character defers its destruction to
//! the next update (or [`PhysicsSystem::flush_dropped_characters`]), since a drop could race other threads using the
//! system; call `destroy` to remove it immediately.
//!
//! Character bodies can't be destroyed through the system or used in constraints, since the character frees them
//! without knowing about either. Listener callbacks must not use the body interface: Jolt may be holding body locks.

use std::ffi::c_void;
use std::fmt;
use std::mem::ManuallyDrop;
use std::ops::Deref;
use std::ptr::NonNull;
use std::sync::{Arc, Mutex, MutexGuard};

use jolt_sys as sys;

use crate::listener::PanicSlot;
use crate::system::SystemInner;
use crate::{
    Activation, AllowedDofs, BackFaceMode, Body, BodyId, Error, Mat44, MotionType, ObjectLayer, PhysicsMaterial, PhysicsSystem,
    QueryFilter, Quat, Result, Shape, SubShapeId, TempAllocator, Vec3,
};

/// Identifies a [`CharacterVirtual`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct CharacterId(pub u32);

impl CharacterId {
    fn from_sys(raw: u32) -> Option<Self> {
        (raw != sys::JPC_CHARACTER_ID_INVALID).then_some(Self(raw))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GroundState {
    /// Standing on walkable ground.
    OnGround,
    /// On a slope too steep to climb; apply gravity to slide down.
    OnSteepGround,
    /// Touching something that does not support the character; it should fall.
    NotSupported,
    InAir,
}

impl GroundState {
    fn from_sys(raw: sys::JPC_GroundState) -> Self {
        match raw {
            sys::JPC_GROUND_STATE_ON_GROUND => Self::OnGround,
            sys::JPC_GROUND_STATE_ON_STEEP_GROUND => Self::OnSteepGround,
            sys::JPC_GROUND_STATE_NOT_SUPPORTED => Self::NotSupported,
            _ => Self::InAir,
        }
    }
}

/// What the character is standing on. The fields other than `state` describe the touched object, if any.
#[derive(Clone, Debug, PartialEq)]
pub struct CharacterGround {
    pub state: GroundState,
    pub position: Vec3,
    pub normal: Vec3,
    /// Velocity of the ground at the contact point, e.g. of a moving platform.
    pub velocity: Vec3,
    pub body_id: Option<BodyId>,
    pub sub_shape_id: SubShapeId,
    pub user_data: u64,
    pub material: Option<PhysicsMaterial>,
}

impl CharacterGround {
    fn from_sys(raw: &sys::JPC_CharacterGround) -> Self {
        Self {
            state: GroundState::from_sys(raw.state),
            position: raw.position.into(),
            normal: raw.normal.into(),
            velocity: raw.velocity.into(),
            body_id: BodyId::from_sys(raw.body_id),
            sub_shape_id: SubShapeId(raw.sub_shape_id),
            user_data: raw.user_data,
            material: PhysicsMaterial::from_borrowed(raw.material),
        }
    }

    pub fn is_supported(&self) -> bool {
        matches!(self.state, GroundState::OnGround | GroundState::OnSteepGround)
    }
}

/// A contact between a [`CharacterVirtual`] and a body or another character.
#[derive(Clone, Debug, PartialEq)]
pub struct CharacterContact {
    /// Set when the contact is with a body.
    pub body_id: Option<BodyId>,
    /// Set when the contact is with another character.
    pub character_id: Option<CharacterId>,
    pub sub_shape_id: SubShapeId,
    pub position: Vec3,
    /// Velocity of the contact point.
    pub linear_velocity: Vec3,
    /// Points towards the character.
    pub contact_normal: Vec3,
    pub surface_normal: Vec3,
    /// Negative or zero for real contacts, positive for predictive ones.
    pub distance: f32,
    pub fraction: f32,
    pub motion_type: MotionType,
    pub is_sensor: bool,
    pub user_data: u64,
    pub material: Option<PhysicsMaterial>,
    pub had_collision: bool,
    pub was_discarded: bool,
    pub can_push_character: bool,
    pub is_back_facing: bool,
}

impl CharacterContact {
    fn from_sys(raw: &sys::JPC_CharacterContact) -> Self {
        Self {
            body_id: BodyId::from_sys(raw.body_id),
            character_id: CharacterId::from_sys(raw.character_id),
            sub_shape_id: SubShapeId(raw.sub_shape_id),
            position: raw.position.into(),
            linear_velocity: raw.linear_velocity.into(),
            contact_normal: raw.contact_normal.into(),
            surface_normal: raw.surface_normal.into(),
            distance: raw.distance,
            fraction: raw.fraction,
            motion_type: MotionType::from_sys(raw.motion_type),
            is_sensor: raw.is_sensor,
            user_data: raw.user_data,
            material: PhysicsMaterial::from_borrowed(raw.material),
            had_collision: raw.had_collision,
            was_discarded: raw.was_discarded,
            can_push_character: raw.can_push_character,
            is_back_facing: raw.is_back_facing,
        }
    }
}

/// Settings that [`CharacterContactListener::on_contact_added`] and friends may change.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CharacterContactSettings {
    /// Whether the other object can push the character.
    pub can_push_character: bool,
    /// Whether the character pushes the other body.
    pub can_receive_impulses: bool,
}

/// Receives contact events of one [`CharacterVirtual`]. Called synchronously from the character's update
/// functions; panics are resumed when that function returns.
pub trait CharacterContactListener: Send + 'static {
    /// Adjust the velocity of a body the character is standing on or touching, e.g. to make a conveyor belt.
    fn on_adjust_body_velocity(&mut self, character: &CharacterView, body: &Body, linear_velocity: &mut Vec3, angular_velocity: &mut Vec3) {
        let _ = (character, body, linear_velocity, angular_velocity);
    }

    /// Return false to ignore the contact.
    fn on_contact_validate(&mut self, character: &CharacterView, contact: &CharacterContact) -> bool {
        let _ = (character, contact);
        true
    }

    fn on_contact_added(&mut self, character: &CharacterView, contact: &CharacterContact, settings: &mut CharacterContactSettings) {
        let _ = (character, contact, settings);
    }

    fn on_contact_persisted(&mut self, character: &CharacterView, contact: &CharacterContact, settings: &mut CharacterContactSettings) {
        let _ = (character, contact, settings);
    }

    fn on_contact_removed(&mut self, character: &CharacterView, body: BodyId, sub_shape_id: SubShapeId) {
        let _ = (character, body, sub_shape_id);
    }

    fn on_character_contact_validate(&mut self, character: &CharacterView, contact: &CharacterContact) -> bool {
        let _ = (character, contact);
        true
    }

    fn on_character_contact_added(&mut self, character: &CharacterView, contact: &CharacterContact, settings: &mut CharacterContactSettings) {
        let _ = (character, contact, settings);
    }

    fn on_character_contact_persisted(&mut self, character: &CharacterView, contact: &CharacterContact, settings: &mut CharacterContactSettings) {
        let _ = (character, contact, settings);
    }

    fn on_character_contact_removed(&mut self, character: &CharacterView, other: CharacterId, sub_shape_id: SubShapeId) {
        let _ = (character, other, sub_shape_id);
    }

    /// Called while solving the character's movement against a contact. `new_velocity` holds the velocity after
    /// sliding along the contact and may be changed (e.g. to stop sliding down slopes).
    fn on_contact_solve(&mut self, character: &CharacterView, contact: &CharacterContact, character_velocity: Vec3, new_velocity: &mut Vec3) {
        let _ = (character, contact, character_velocity, new_velocity);
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CharacterVirtualSettings {
    pub up: Vec3,
    /// Contacts below this plane (normal, constant) in character space support the character. The default
    /// accepts any contact; use e.g. `(Vec3::Y, -radius)` to only accept contacts on the bottom sphere of a capsule.
    pub supporting_volume: (Vec3, f32),
    /// Steepest walkable slope in radians.
    pub max_slope_angle: f32,
    pub enhanced_internal_edge_removal: bool,
    pub mass: f32,
    /// Maximum force the character can push other bodies with (N).
    pub max_strength: f32,
    pub shape_offset: Vec3,
    pub back_face_mode: BackFaceMode,
    pub predictive_contact_distance: f32,
    pub max_collision_iterations: u32,
    pub max_constraint_iterations: u32,
    pub min_time_remaining: f32,
    pub collision_tolerance: f32,
    pub character_padding: f32,
    pub max_num_hits: u32,
    pub hit_reduction_cos_max_angle: f32,
    pub penetration_recovery_speed: f32,
    /// Layer of the optional inner body (see [`CharacterVirtual::new`]).
    pub inner_body_layer: ObjectLayer,
}

impl Default for CharacterVirtualSettings {
    fn default() -> Self {
        let mut d = sys::JPC_CharacterVirtualSettings::default();
        unsafe { sys::JPC_CharacterVirtualSettings_SetDefault(&mut d) };
        Self {
            up: d.up.into(),
            supporting_volume: (d.supporting_volume_normal.into(), d.supporting_volume_constant),
            max_slope_angle: d.max_slope_angle,
            enhanced_internal_edge_removal: d.enhanced_internal_edge_removal,
            mass: d.mass,
            max_strength: d.max_strength,
            shape_offset: d.shape_offset.into(),
            back_face_mode: if d.back_face_mode == sys::JPC_BACK_FACE_MODE_COLLIDE { BackFaceMode::Collide } else { BackFaceMode::Ignore },
            predictive_contact_distance: d.predictive_contact_distance,
            max_collision_iterations: d.max_collision_iterations,
            max_constraint_iterations: d.max_constraint_iterations,
            min_time_remaining: d.min_time_remaining,
            collision_tolerance: d.collision_tolerance,
            character_padding: d.character_padding,
            max_num_hits: d.max_num_hits,
            hit_reduction_cos_max_angle: d.hit_reduction_cos_max_angle,
            penetration_recovery_speed: d.penetration_recovery_speed,
            inner_body_layer: ObjectLayer(d.inner_body_layer),
        }
    }
}

/// Settings for [`CharacterVirtual::extended_update`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ExtendedUpdateSettings {
    /// How far down to look for floor to stick to after moving. Zero disables sticking.
    pub stick_to_floor_step_down: Vec3,
    /// Highest step that can be walked up. Zero disables stair walking.
    pub walk_stairs_step_up: Vec3,
    pub walk_stairs_min_step_forward: f32,
    pub walk_stairs_step_forward_test: f32,
    pub walk_stairs_cos_angle_forward_contact: f32,
    pub walk_stairs_step_down_extra: Vec3,
}

impl Default for ExtendedUpdateSettings {
    fn default() -> Self {
        let mut d = sys::JPC_ExtendedUpdateSettings::default();
        unsafe { sys::JPC_ExtendedUpdateSettings_SetDefault(&mut d) };
        Self {
            stick_to_floor_step_down: d.stick_to_floor_step_down.into(),
            walk_stairs_step_up: d.walk_stairs_step_up.into(),
            walk_stairs_min_step_forward: d.walk_stairs_min_step_forward,
            walk_stairs_step_forward_test: d.walk_stairs_step_forward_test,
            walk_stairs_cos_angle_forward_contact: d.walk_stairs_cos_angle_forward_contact,
            walk_stairs_step_down_extra: d.walk_stairs_step_down_extra.into(),
        }
    }
}

impl ExtendedUpdateSettings {
    fn to_sys(self) -> sys::JPC_ExtendedUpdateSettings {
        sys::JPC_ExtendedUpdateSettings {
            stick_to_floor_step_down: self.stick_to_floor_step_down.into(),
            walk_stairs_step_up: self.walk_stairs_step_up.into(),
            walk_stairs_min_step_forward: self.walk_stairs_min_step_forward,
            walk_stairs_step_forward_test: self.walk_stairs_step_forward_test,
            walk_stairs_cos_angle_forward_contact: self.walk_stairs_cos_angle_forward_contact,
            walk_stairs_step_down_extra: self.walk_stairs_step_down_extra.into(),
        }
    }
}

/// Read-only view of a [`CharacterVirtual`]. Listener callbacks get one; a `CharacterVirtual` derefs to it.
#[repr(transparent)]
pub struct CharacterView(sys::JPC_CharacterVirtual);

impl CharacterView {
    unsafe fn from_raw<'a>(raw: *const sys::JPC_CharacterVirtual) -> &'a Self {
        unsafe { &*raw.cast::<Self>() }
    }

    fn raw(&self) -> *const sys::JPC_CharacterVirtual {
        (self as *const Self).cast()
    }

    pub fn id(&self) -> CharacterId {
        CharacterId(unsafe { sys::JPC_CharacterVirtual_GetID(self.raw()) })
    }

    pub fn position(&self) -> Vec3 {
        unsafe { sys::JPC_CharacterVirtual_GetPosition(self.raw()) }.into()
    }

    pub fn rotation(&self) -> Quat {
        unsafe { sys::JPC_CharacterVirtual_GetRotation(self.raw()) }.into()
    }

    pub fn linear_velocity(&self) -> Vec3 {
        unsafe { sys::JPC_CharacterVirtual_GetLinearVelocity(self.raw()) }.into()
    }

    pub fn center_of_mass_position(&self) -> Vec3 {
        unsafe { sys::JPC_CharacterVirtual_GetCenterOfMassPosition(self.raw()) }.into()
    }

    pub fn world_transform(&self) -> Mat44 {
        unsafe { sys::JPC_CharacterVirtual_GetWorldTransform(self.raw()) }.into()
    }

    pub fn up(&self) -> Vec3 {
        unsafe { sys::JPC_CharacterVirtual_GetUp(self.raw()) }.into()
    }

    pub fn cos_max_slope_angle(&self) -> f32 {
        unsafe { sys::JPC_CharacterVirtual_GetCosMaxSlopeAngle(self.raw()) }
    }

    pub fn is_slope_too_steep(&self, normal: Vec3) -> bool {
        unsafe { sys::JPC_CharacterVirtual_IsSlopeTooSteep(self.raw(), normal.into()) }
    }

    pub fn mass(&self) -> f32 {
        unsafe { sys::JPC_CharacterVirtual_GetMass(self.raw()) }
    }

    pub fn max_strength(&self) -> f32 {
        unsafe { sys::JPC_CharacterVirtual_GetMaxStrength(self.raw()) }
    }

    pub fn penetration_recovery_speed(&self) -> f32 {
        unsafe { sys::JPC_CharacterVirtual_GetPenetrationRecoverySpeed(self.raw()) }
    }

    pub fn shape_offset(&self) -> Vec3 {
        unsafe { sys::JPC_CharacterVirtual_GetShapeOffset(self.raw()) }.into()
    }

    pub fn user_data(&self) -> u64 {
        unsafe { sys::JPC_CharacterVirtual_GetUserData(self.raw()) }
    }

    pub fn enhanced_internal_edge_removal(&self) -> bool {
        unsafe { sys::JPC_CharacterVirtual_GetEnhancedInternalEdgeRemoval(self.raw()) }
    }

    pub fn shape(&self) -> Shape {
        let raw = unsafe { sys::JPC_CharacterVirtual_GetShape(self.raw()) };
        unsafe { sys::JPC_Shape_AddRef(raw) };
        // SAFETY: we just took the reference the handle owns.
        unsafe { Shape::from_raw(NonNull::new_unchecked(raw.cast_mut())) }
    }

    /// The rigid body that follows the character, if it was created with one.
    pub fn inner_body_id(&self) -> Option<BodyId> {
        BodyId::from_sys(unsafe { sys::JPC_CharacterVirtual_GetInnerBodyID(self.raw()) })
    }

    pub fn ground(&self) -> CharacterGround {
        let mut raw = sys::JPC_CharacterGround::default();
        unsafe { sys::JPC_CharacterVirtual_GetGround(self.raw(), &mut raw) };
        CharacterGround::from_sys(&raw)
    }

    pub fn ground_state(&self) -> GroundState {
        self.ground().state
    }

    pub fn is_supported(&self) -> bool {
        unsafe { sys::JPC_CharacterVirtual_IsSupported(self.raw()) }
    }

    pub fn has_collided_with(&self, body: BodyId) -> bool {
        unsafe { sys::JPC_CharacterVirtual_HasCollidedWith(self.raw(), body.0) }
    }

    /// True if the last update found more hits than `max_num_hits` and some were ignored.
    pub fn max_hits_exceeded(&self) -> bool {
        unsafe { sys::JPC_CharacterVirtual_GetMaxHitsExceeded(self.raw()) }
    }

    pub fn active_contacts(&self) -> Vec<CharacterContact> {
        let total = unsafe { sys::JPC_CharacterVirtual_GetActiveContacts(self.raw(), std::ptr::null_mut(), 0) };
        let mut raw = vec![sys::JPC_CharacterContact::default(); total as usize];
        let written = unsafe { sys::JPC_CharacterVirtual_GetActiveContacts(self.raw(), raw.as_mut_ptr(), total) }.min(total);
        raw[..written as usize].iter().map(CharacterContact::from_sys).collect()
    }

    pub fn can_walk_stairs(&self, linear_velocity: Vec3) -> bool {
        unsafe { sys::JPC_CharacterVirtual_CanWalkStairs(self.raw(), linear_velocity.into()) }
    }

    /// Removes the part of `desired_velocity` that would push the character up slopes that are too steep.
    pub fn cancel_velocity_towards_steep_slopes(&self, desired_velocity: Vec3) -> Vec3 {
        unsafe { sys::JPC_CharacterVirtual_CancelVelocityTowardsSteepSlopes(self.raw(), desired_velocity.into()) }.into()
    }
}

impl fmt::Debug for CharacterView {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CharacterVirtual").field("id", &self.id()).field("position", &self.position()).finish()
    }
}

struct CharacterListenerHolder {
    listener: Box<dyn CharacterContactListener>,
    panic: PanicSlot,
}

pub(crate) struct CharacterGroupInner {
    raw: NonNull<sys::JPC_CharacterVsCharacterCollision>,
    /// Characters in a group read each other during updates, so updates of members are serialised.
    lock: Mutex<()>,
}

// SAFETY: all access to the native object happens under `lock`.
unsafe impl Send for CharacterGroupInner {}
unsafe impl Sync for CharacterGroupInner {}

impl Drop for CharacterGroupInner {
    fn drop(&mut self) {
        // Every member holds an Arc, so the group is empty by now.
        unsafe { sys::JPC_CharacterVsCharacterCollision_Destroy(self.raw.as_ptr()) }
    }
}

/// A set of [`CharacterVirtual`]s that collide with each other. Updates of characters in the same group are
/// serialised internally.
#[derive(Clone)]
pub struct CharacterGroup {
    inner: Arc<CharacterGroupInner>,
}

impl CharacterGroup {
    pub fn new() -> Self {
        crate::ensure_init();
        let raw = unsafe { sys::JPC_CharacterVsCharacterCollision_Create() };
        Self { inner: Arc::new(CharacterGroupInner { raw: NonNull::new(raw).expect("allocation failed"), lock: Mutex::new(()) }) }
    }
}

impl Default for CharacterGroup {
    fn default() -> Self {
        Self::new()
    }
}

/// Native objects whose destruction waits until no update can be running.
pub(crate) enum Deferred {
    CharacterVirtual { raw: NonNull<sys::JPC_CharacterVirtual>, _group: Option<Arc<CharacterGroupInner>> },
    Character(NonNull<sys::JPC_Character>),
    Ragdoll(NonNull<sys::JPC_Ragdoll>),
}

// SAFETY: the pointers are owned by the entry and only used once, when it is destroyed.
unsafe impl Send for Deferred {}

impl Deferred {
    /// # Safety
    /// No update of the owning system may be running.
    pub(crate) unsafe fn destroy(self) {
        match self {
            Deferred::CharacterVirtual { raw, _group } => {
                let _lock = _group.as_ref().map(|g| g.lock.lock().unwrap_or_else(|e| e.into_inner()));
                unsafe { sys::JPC_CharacterVirtual_Destroy(raw.as_ptr()) }
            }
            Deferred::Character(raw) => unsafe { sys::JPC_Character_Destroy(raw.as_ptr()) },
            Deferred::Ragdoll(raw) => unsafe { sys::JPC_Ragdoll_Destroy(raw.as_ptr()) },
        }
    }
}

/// A kinematic character controller that moves by sweeping its shape through the world, sliding along walls,
/// walking up stairs and sticking to floors. Other bodies don't affect it unless you say so in a listener;
/// it pushes dynamic bodies with up to `max_strength`.
///
/// Typical frame: set the desired velocity, call [`extended_update`](Self::extended_update), then step the world.
pub struct CharacterVirtual {
    raw: NonNull<sys::JPC_CharacterVirtual>,
    system: Arc<SystemInner>,
    listener: Option<Box<CharacterListenerHolder>>,
    group: Option<Arc<CharacterGroupInner>>,
}

// SAFETY: mutation needs `&mut self`, shared access only reads, and group members are serialised by the group lock.
unsafe impl Send for CharacterVirtual {}
unsafe impl Sync for CharacterVirtual {}

impl Deref for CharacterVirtual {
    type Target = CharacterView;

    fn deref(&self) -> &CharacterView {
        unsafe { CharacterView::from_raw(self.raw.as_ptr()) }
    }
}

impl fmt::Debug for CharacterVirtual {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        (**self).fmt(f)
    }
}

impl CharacterVirtual {
    /// Creates a character. With `inner_body_shape` set, a kinematic body with that shape follows the character
    /// so that ray casts and sensors see it; it lives in `settings.inner_body_layer`.
    pub fn new(
        system: &PhysicsSystem,
        settings: &CharacterVirtualSettings,
        shape: &Shape,
        inner_body_shape: Option<&Shape>,
        position: Vec3,
        rotation: Quat,
        user_data: u64,
    ) -> Result<Self> {
        let s = settings;
        let raw_settings = sys::JPC_CharacterVirtualSettings {
            up: s.up.into(),
            supporting_volume_normal: s.supporting_volume.0.into(),
            supporting_volume_constant: s.supporting_volume.1,
            max_slope_angle: s.max_slope_angle,
            enhanced_internal_edge_removal: s.enhanced_internal_edge_removal,
            shape: shape.as_raw(),
            mass: s.mass,
            max_strength: s.max_strength,
            shape_offset: s.shape_offset.into(),
            back_face_mode: match s.back_face_mode {
                BackFaceMode::Ignore => sys::JPC_BACK_FACE_MODE_IGNORE,
                BackFaceMode::Collide => sys::JPC_BACK_FACE_MODE_COLLIDE,
            },
            predictive_contact_distance: s.predictive_contact_distance,
            max_collision_iterations: s.max_collision_iterations,
            max_constraint_iterations: s.max_constraint_iterations,
            min_time_remaining: s.min_time_remaining,
            collision_tolerance: s.collision_tolerance,
            character_padding: s.character_padding,
            max_num_hits: s.max_num_hits,
            hit_reduction_cos_max_angle: s.hit_reduction_cos_max_angle,
            penetration_recovery_speed: s.penetration_recovery_speed,
            inner_body_shape: inner_body_shape.map_or(std::ptr::null(), |s| s.as_raw()),
            inner_body_layer: s.inner_body_layer.0,
        };
        let raw = unsafe {
            sys::JPC_CharacterVirtual_Create(&raw_settings, position.into(), rotation.into(), user_data, system.inner.raw())
        };
        let raw = NonNull::new(raw).ok_or_else(Error::last)?;
        Ok(Self { raw, system: system.inner.clone(), listener: None, group: None })
    }

    fn check(&self, system: &PhysicsSystem) {
        assert!(Arc::ptr_eq(&self.system, &system.inner), "character used with a different physics system");
    }

    /// Other members of the group read this character's state during their updates.
    fn group_lock(&self) -> Option<MutexGuard<'_, ()>> {
        self.group.as_ref().map(|g| g.lock.lock().unwrap_or_else(|e| e.into_inner()))
    }

    fn ptr(&self) -> *mut sys::JPC_CharacterVirtual {
        self.raw.as_ptr()
    }

    fn resume_listener_panic(&self) {
        if let Some(holder) = &self.listener {
            holder.panic.resume();
        }
    }

    /// Runs `f` with everything a world-touching operation needs: the right system, the group lock, and
    /// listener panics resumed afterwards.
    fn world_op<R>(&mut self, system: &PhysicsSystem, f: impl FnOnce(*mut sys::JPC_CharacterVirtual) -> R) -> R {
        self.check(system);
        let result = {
            let _lock = self.group_lock();
            f(self.ptr())
        };
        self.resume_listener_panic();
        result
    }

    pub fn set_listener(&mut self, listener: impl CharacterContactListener) {
        let holder = Box::new(CharacterListenerHolder { listener: Box::new(listener), panic: PanicSlot::default() });
        let fns = listener_fns();
        let user_data = (&*holder as *const CharacterListenerHolder).cast_mut().cast::<c_void>();
        unsafe { sys::JPC_CharacterVirtual_SetListener(self.ptr(), &fns, user_data) };
        self.listener = Some(holder);
    }

    pub fn clear_listener(&mut self) {
        unsafe { sys::JPC_CharacterVirtual_SetListener(self.ptr(), std::ptr::null(), std::ptr::null_mut()) };
        self.listener = None;
    }

    /// Makes this character collide with the other members of `group`, leaving any previous group.
    pub fn join_group(&mut self, group: &CharacterGroup) {
        self.leave_group();
        let _lock = group.inner.lock.lock().unwrap_or_else(|e| e.into_inner());
        unsafe { sys::JPC_CharacterVsCharacterCollision_Add(group.inner.raw.as_ptr(), self.ptr()) };
        drop(_lock);
        self.group = Some(group.inner.clone());
    }

    pub fn leave_group(&mut self) {
        if let Some(group) = self.group.take() {
            let _lock = group.lock.lock().unwrap_or_else(|e| e.into_inner());
            unsafe { sys::JPC_CharacterVsCharacterCollision_Remove(group.raw.as_ptr(), self.ptr()) };
        }
    }

    pub fn set_linear_velocity(&mut self, velocity: Vec3) {
        let _lock = self.group_lock();
        unsafe { sys::JPC_CharacterVirtual_SetLinearVelocity(self.ptr(), velocity.into()) }
    }

    /// Moves the character by its velocity, sliding along whatever it hits.
    pub fn update(&mut self, system: &PhysicsSystem, delta_time: f32, gravity: Vec3, filter: &QueryFilter<'_>, temp: &mut TempAllocator) {
        let (f, _ctx) = sys_filter(filter);
        self.world_op(system, |c| unsafe { sys::JPC_CharacterVirtual_Update(c, delta_time, gravity.into(), &f, temp.raw()) });
    }

    /// [`update`](Self::update) plus stair walking and sticking to the floor.
    pub fn extended_update(
        &mut self,
        system: &PhysicsSystem,
        delta_time: f32,
        gravity: Vec3,
        settings: &ExtendedUpdateSettings,
        filter: &QueryFilter<'_>,
        temp: &mut TempAllocator,
    ) {
        let (f, _ctx) = sys_filter(filter);
        let s = settings.to_sys();
        self.world_op(system, |c| unsafe { sys::JPC_CharacterVirtual_ExtendedUpdate(c, delta_time, gravity.into(), &s, &f, temp.raw()) });
    }

    #[allow(clippy::too_many_arguments)]
    pub fn walk_stairs(
        &mut self,
        system: &PhysicsSystem,
        delta_time: f32,
        step_up: Vec3,
        step_forward: Vec3,
        step_forward_test: Vec3,
        step_down_extra: Vec3,
        filter: &QueryFilter<'_>,
        temp: &mut TempAllocator,
    ) -> bool {
        let (f, _ctx) = sys_filter(filter);
        self.world_op(system, |c| unsafe {
            sys::JPC_CharacterVirtual_WalkStairs(
                c,
                delta_time,
                step_up.into(),
                step_forward.into(),
                step_forward_test.into(),
                step_down_extra.into(),
                &f,
                temp.raw(),
            )
        })
    }

    pub fn stick_to_floor(&mut self, system: &PhysicsSystem, step_down: Vec3, filter: &QueryFilter<'_>, temp: &mut TempAllocator) -> bool {
        let (f, _ctx) = sys_filter(filter);
        self.world_op(system, |c| unsafe { sys::JPC_CharacterVirtual_StickToFloor(c, step_down.into(), &f, temp.raw()) })
    }

    /// Recomputes contacts at the current position, e.g. after teleporting.
    pub fn refresh_contacts(&mut self, system: &PhysicsSystem, filter: &QueryFilter<'_>, temp: &mut TempAllocator) {
        let (f, _ctx) = sys_filter(filter);
        self.world_op(system, |c| unsafe { sys::JPC_CharacterVirtual_RefreshContacts(c, &f, temp.raw()) });
    }

    /// Re-reads the velocity of the ground body. Call after stepping the world if you read the ground velocity.
    pub fn update_ground_velocity(&mut self, system: &PhysicsSystem) {
        self.world_op(system, |c| unsafe { sys::JPC_CharacterVirtual_UpdateGroundVelocity(c) });
    }

    /// Switches to `shape` if it fits where the character is (e.g. standing up from crouching). With
    /// `max_penetration_depth = f32::MAX` it always switches. Returns whether it switched.
    pub fn set_shape(
        &mut self,
        system: &PhysicsSystem,
        shape: &Shape,
        max_penetration_depth: f32,
        filter: &QueryFilter<'_>,
        temp: &mut TempAllocator,
    ) -> bool {
        let (f, _ctx) = sys_filter(filter);
        self.world_op(system, |c| unsafe { sys::JPC_CharacterVirtual_SetShape(c, shape.as_raw(), max_penetration_depth, &f, temp.raw()) })
    }

    pub fn set_inner_body_shape(&mut self, system: &PhysicsSystem, shape: &Shape) {
        self.world_op(system, |c| unsafe { sys::JPC_CharacterVirtual_SetInnerBodyShape(c, shape.as_raw()) });
    }

    /// Teleports the character (and its inner body).
    pub fn set_position(&mut self, system: &PhysicsSystem, position: Vec3) {
        self.world_op(system, |c| unsafe { sys::JPC_CharacterVirtual_SetPosition(c, position.into()) });
    }

    pub fn set_rotation(&mut self, system: &PhysicsSystem, rotation: Quat) {
        self.world_op(system, |c| unsafe { sys::JPC_CharacterVirtual_SetRotation(c, rotation.into()) });
    }

    pub fn set_shape_offset(&mut self, system: &PhysicsSystem, offset: Vec3) {
        self.world_op(system, |c| unsafe { sys::JPC_CharacterVirtual_SetShapeOffset(c, offset.into()) });
    }

    /// Also updates the user data of the inner body.
    pub fn set_user_data(&mut self, system: &PhysicsSystem, user_data: u64) {
        self.world_op(system, |c| unsafe { sys::JPC_CharacterVirtual_SetUserData(c, user_data) });
    }

    pub fn set_up(&mut self, up: Vec3) {
        let _lock = self.group_lock();
        unsafe { sys::JPC_CharacterVirtual_SetUp(self.ptr(), up.into()) }
    }

    pub fn set_max_slope_angle(&mut self, angle: f32) {
        let _lock = self.group_lock();
        unsafe { sys::JPC_CharacterVirtual_SetMaxSlopeAngle(self.ptr(), angle) }
    }

    pub fn set_mass(&mut self, mass: f32) {
        let _lock = self.group_lock();
        unsafe { sys::JPC_CharacterVirtual_SetMass(self.ptr(), mass) }
    }

    pub fn set_max_strength(&mut self, strength: f32) {
        let _lock = self.group_lock();
        unsafe { sys::JPC_CharacterVirtual_SetMaxStrength(self.ptr(), strength) }
    }

    pub fn set_penetration_recovery_speed(&mut self, speed: f32) {
        let _lock = self.group_lock();
        unsafe { sys::JPC_CharacterVirtual_SetPenetrationRecoverySpeed(self.ptr(), speed) }
    }

    pub fn set_enhanced_internal_edge_removal(&mut self, enabled: bool) {
        let _lock = self.group_lock();
        unsafe { sys::JPC_CharacterVirtual_SetEnhancedInternalEdgeRemoval(self.ptr(), enabled) }
    }

    /// Destroys the character and its inner body right away. Needs exclusive access to the system because the
    /// inner body is freed.
    pub fn destroy(self, system: &mut PhysicsSystem) {
        self.check(system);
        let mut this = ManuallyDrop::new(self);
        unsafe { sys::JPC_CharacterVirtual_SetListener(this.ptr(), std::ptr::null(), std::ptr::null_mut()) };
        let entry = Deferred::CharacterVirtual { raw: this.raw, _group: this.group.take() };
        // SAFETY: `this` is never used or dropped again, so moving the remaining fields out is fine.
        let (_listener, _system) = unsafe { (std::ptr::read(&this.listener), std::ptr::read(&this.system)) };
        // SAFETY: `&mut PhysicsSystem` proves nothing else is using the system.
        unsafe { entry.destroy() };
    }
}

impl Drop for CharacterVirtual {
    fn drop(&mut self) {
        // The listener holder is freed with self, so the native side must stop pointing at it first.
        unsafe { sys::JPC_CharacterVirtual_SetListener(self.ptr(), std::ptr::null(), std::ptr::null_mut()) };
        self.system.defer(Deferred::CharacterVirtual { raw: self.raw, _group: self.group.take() });
    }
}

/// Keeps a query filter's body filter closure reachable by the C callback for as long as the filter is in use.
struct FilterContext<'a> {
    filter: Option<&'a dyn Fn(BodyId) -> bool>,
    panic: PanicSlot,
}

impl Drop for FilterContext<'_> {
    fn drop(&mut self) {
        if !std::thread::panicking() {
            self.panic.resume();
        }
    }
}

unsafe extern "C" fn character_body_filter(user_data: *mut c_void, body: sys::JPC_BodyID) -> bool {
    let ctx = unsafe { &*user_data.cast::<FilterContext<'_>>() };
    let filter = ctx.filter.expect("body filter set");
    ctx.panic.guard(false, || filter(BodyId(body)))
}

/// The returned context resumes a panic raised by the body filter when it is dropped, so keep it alive until
/// the native call returns.
fn sys_filter<'a>(filter: &QueryFilter<'a>) -> (sys::JPC_QueryFilter, Box<FilterContext<'a>>) {
    let ctx = Box::new(FilterContext { filter: filter.body_filter, panic: PanicSlot::default() });
    let raw = sys::JPC_QueryFilter {
        use_object_layer: filter.object_layer.is_some(),
        object_layer: filter.object_layer.map_or(0, |l| l.0),
        ignore_body: filter.ignore_body.map_or(sys::JPC_BODY_ID_INVALID, |b| b.0),
        body_filter: filter.body_filter.is_some().then_some(character_body_filter as _),
        body_filter_user_data: (&*ctx as *const FilterContext<'_>).cast_mut().cast(),
    };
    (raw, ctx)
}

fn listener_fns() -> sys::JPC_CharacterContactListenerFns {
    sys::JPC_CharacterContactListenerFns {
        on_adjust_body_velocity: Some(on_adjust_body_velocity),
        on_contact_validate: Some(on_contact_validate),
        on_contact_added: Some(on_contact_added),
        on_contact_persisted: Some(on_contact_persisted),
        on_contact_removed: Some(on_contact_removed),
        on_character_contact_validate: Some(on_character_contact_validate),
        on_character_contact_added: Some(on_character_contact_added),
        on_character_contact_persisted: Some(on_character_contact_persisted),
        on_character_contact_removed: Some(on_character_contact_removed),
        on_contact_solve: Some(on_contact_solve),
    }
}

/// # Safety
/// `user_data` must be the holder registered with the character, which is only called back from the thread that
/// holds `&mut CharacterVirtual`.
unsafe fn holder<'a>(user_data: *mut c_void) -> &'a mut CharacterListenerHolder {
    unsafe { &mut *user_data.cast::<CharacterListenerHolder>() }
}

unsafe extern "C" fn on_adjust_body_velocity(
    user_data: *mut c_void,
    character: *const sys::JPC_CharacterVirtual,
    body: *const sys::JPC_Body,
    linear: *mut sys::JPC_Vec3,
    angular: *mut sys::JPC_Vec3,
) {
    let h = unsafe { holder(user_data) };
    let (listener, panic) = (&mut h.listener, &h.panic);
    panic.guard((), || unsafe {
        let mut l: Vec3 = (*linear).into();
        let mut a: Vec3 = (*angular).into();
        listener.on_adjust_body_velocity(CharacterView::from_raw(character), Body::from_raw(body), &mut l, &mut a);
        *linear = l.into();
        *angular = a.into();
    })
}

unsafe extern "C" fn on_contact_validate(user_data: *mut c_void, character: *const sys::JPC_CharacterVirtual, contact: *const sys::JPC_CharacterContact) -> bool {
    let h = unsafe { holder(user_data) };
    let (listener, panic) = (&mut h.listener, &h.panic);
    panic.guard(true, || unsafe { listener.on_contact_validate(CharacterView::from_raw(character), &CharacterContact::from_sys(&*contact)) })
}

unsafe extern "C" fn on_character_contact_validate(
    user_data: *mut c_void,
    character: *const sys::JPC_CharacterVirtual,
    contact: *const sys::JPC_CharacterContact,
) -> bool {
    let h = unsafe { holder(user_data) };
    let (listener, panic) = (&mut h.listener, &h.panic);
    panic.guard(true, || unsafe { listener.on_character_contact_validate(CharacterView::from_raw(character), &CharacterContact::from_sys(&*contact)) })
}

type ContactCallback = fn(&mut dyn CharacterContactListener, &CharacterView, &CharacterContact, &mut CharacterContactSettings);

unsafe fn forward_contact(
    user_data: *mut c_void,
    character: *const sys::JPC_CharacterVirtual,
    contact: *const sys::JPC_CharacterContact,
    settings: *mut sys::JPC_CharacterContactSettings,
    callback: ContactCallback,
) {
    let h = unsafe { holder(user_data) };
    let (listener, panic) = (&mut h.listener, &h.panic);
    panic.guard((), || unsafe {
        let raw = &mut *settings;
        let mut s = CharacterContactSettings { can_push_character: raw.can_push_character, can_receive_impulses: raw.can_receive_impulses };
        callback(listener.as_mut(), CharacterView::from_raw(character), &CharacterContact::from_sys(&*contact), &mut s);
        raw.can_push_character = s.can_push_character;
        raw.can_receive_impulses = s.can_receive_impulses;
    })
}

unsafe extern "C" fn on_contact_added(
    user_data: *mut c_void,
    character: *const sys::JPC_CharacterVirtual,
    contact: *const sys::JPC_CharacterContact,
    settings: *mut sys::JPC_CharacterContactSettings,
) {
    unsafe { forward_contact(user_data, character, contact, settings, |l, c, k, s| l.on_contact_added(c, k, s)) }
}

unsafe extern "C" fn on_contact_persisted(
    user_data: *mut c_void,
    character: *const sys::JPC_CharacterVirtual,
    contact: *const sys::JPC_CharacterContact,
    settings: *mut sys::JPC_CharacterContactSettings,
) {
    unsafe { forward_contact(user_data, character, contact, settings, |l, c, k, s| l.on_contact_persisted(c, k, s)) }
}

unsafe extern "C" fn on_character_contact_added(
    user_data: *mut c_void,
    character: *const sys::JPC_CharacterVirtual,
    contact: *const sys::JPC_CharacterContact,
    settings: *mut sys::JPC_CharacterContactSettings,
) {
    unsafe { forward_contact(user_data, character, contact, settings, |l, c, k, s| l.on_character_contact_added(c, k, s)) }
}

unsafe extern "C" fn on_character_contact_persisted(
    user_data: *mut c_void,
    character: *const sys::JPC_CharacterVirtual,
    contact: *const sys::JPC_CharacterContact,
    settings: *mut sys::JPC_CharacterContactSettings,
) {
    unsafe { forward_contact(user_data, character, contact, settings, |l, c, k, s| l.on_character_contact_persisted(c, k, s)) }
}

unsafe extern "C" fn on_contact_removed(user_data: *mut c_void, character: *const sys::JPC_CharacterVirtual, body: sys::JPC_BodyID, sub_shape: sys::JPC_SubShapeID) {
    let h = unsafe { holder(user_data) };
    let (listener, panic) = (&mut h.listener, &h.panic);
    panic.guard((), || unsafe { listener.on_contact_removed(CharacterView::from_raw(character), BodyId(body), SubShapeId(sub_shape)) })
}

unsafe extern "C" fn on_character_contact_removed(user_data: *mut c_void, character: *const sys::JPC_CharacterVirtual, other: u32, sub_shape: sys::JPC_SubShapeID) {
    let h = unsafe { holder(user_data) };
    let (listener, panic) = (&mut h.listener, &h.panic);
    panic.guard((), || unsafe {
        listener.on_character_contact_removed(CharacterView::from_raw(character), CharacterId(other), SubShapeId(sub_shape))
    })
}

unsafe extern "C" fn on_contact_solve(
    user_data: *mut c_void,
    character: *const sys::JPC_CharacterVirtual,
    contact: *const sys::JPC_CharacterContact,
    character_velocity: *const sys::JPC_Vec3,
    new_velocity: *mut sys::JPC_Vec3,
) {
    let h = unsafe { holder(user_data) };
    let (listener, panic) = (&mut h.listener, &h.panic);
    panic.guard((), || unsafe {
        let mut v: Vec3 = (*new_velocity).into();
        listener.on_contact_solve(CharacterView::from_raw(character), &CharacterContact::from_sys(&*contact), (*character_velocity).into(), &mut v);
        *new_velocity = v.into();
    })
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CharacterSettings {
    pub up: Vec3,
    /// See [`CharacterVirtualSettings::supporting_volume`].
    pub supporting_volume: (Vec3, f32),
    pub max_slope_angle: f32,
    pub enhanced_internal_edge_removal: bool,
    pub layer: ObjectLayer,
    pub mass: f32,
    pub friction: f32,
    pub gravity_factor: f32,
    /// Translation only by default, which keeps the character upright.
    pub allowed_dofs: AllowedDofs,
}

impl Default for CharacterSettings {
    fn default() -> Self {
        let mut d = sys::JPC_CharacterSettings::default();
        unsafe { sys::JPC_CharacterSettings_SetDefault(&mut d) };
        Self {
            up: d.up.into(),
            supporting_volume: (d.supporting_volume_normal.into(), d.supporting_volume_constant),
            max_slope_angle: d.max_slope_angle,
            enhanced_internal_edge_removal: d.enhanced_internal_edge_removal,
            layer: ObjectLayer(d.layer),
            mass: d.mass,
            friction: d.friction,
            gravity_factor: d.gravity_factor,
            allowed_dofs: AllowedDofs::TRANSLATION_X | AllowedDofs::TRANSLATION_Y | AllowedDofs::TRANSLATION_Z,
        }
    }
}

/// A character made of a dynamic rigid body. It interacts with the world like any body (it gets pushed, it
/// pushes back) and is simulated by [`PhysicsSystem::update`]; call [`post_simulation`](Self::post_simulation)
/// after every update to refresh its ground state.
pub struct Character {
    raw: NonNull<sys::JPC_Character>,
    system: Arc<SystemInner>,
}

// SAFETY: body access goes through Jolt's locking interface; character state is only mutated with `&mut self`.
unsafe impl Send for Character {}
unsafe impl Sync for Character {}

impl Character {
    /// Creates the character's body. It is not in the world until [`add_to_physics_system`](Self::add_to_physics_system).
    pub fn new(system: &PhysicsSystem, settings: &CharacterSettings, shape: &Shape, position: Vec3, rotation: Quat, user_data: u64) -> Result<Self> {
        let s = settings;
        let raw_settings = sys::JPC_CharacterSettings {
            up: s.up.into(),
            supporting_volume_normal: s.supporting_volume.0.into(),
            supporting_volume_constant: s.supporting_volume.1,
            max_slope_angle: s.max_slope_angle,
            enhanced_internal_edge_removal: s.enhanced_internal_edge_removal,
            shape: shape.as_raw(),
            layer: s.layer.0,
            mass: s.mass,
            friction: s.friction,
            gravity_factor: s.gravity_factor,
            allowed_dofs: s.allowed_dofs.bits() as sys::JPC_AllowedDOFs,
        };
        let raw = unsafe { sys::JPC_Character_Create(&raw_settings, position.into(), rotation.into(), user_data, system.inner.raw()) };
        Ok(Self { raw: NonNull::new(raw).ok_or_else(Error::last)?, system: system.inner.clone() })
    }

    fn check(&self, system: &PhysicsSystem) -> *mut sys::JPC_Character {
        assert!(Arc::ptr_eq(&self.system, &system.inner), "character used with a different physics system");
        self.raw.as_ptr()
    }

    /// The character's body. It cannot be destroyed through the body interface.
    pub fn body_id(&self) -> BodyId {
        BodyId(unsafe { sys::JPC_Character_GetBodyID(self.raw.as_ptr()) })
    }

    pub fn add_to_physics_system(&self, system: &PhysicsSystem, activation: Activation) {
        unsafe { sys::JPC_Character_AddToPhysicsSystem(self.check(system), activation.to_sys()) }
    }

    pub fn remove_from_physics_system(&self, system: &PhysicsSystem) {
        unsafe { sys::JPC_Character_RemoveFromPhysicsSystem(self.check(system)) }
    }

    pub fn activate(&self, system: &PhysicsSystem) {
        unsafe { sys::JPC_Character_Activate(self.check(system)) }
    }

    /// Updates the ground state from the body's contacts. `max_separation_distance` is how far below the
    /// character a floor still counts (0.05 is typical).
    pub fn post_simulation(&mut self, system: &PhysicsSystem, max_separation_distance: f32) {
        unsafe { sys::JPC_Character_PostSimulation(self.check(system), max_separation_distance) }
    }

    pub fn linear_velocity(&self, system: &PhysicsSystem) -> Vec3 {
        unsafe { sys::JPC_Character_GetLinearVelocity(self.check(system)) }.into()
    }

    pub fn set_linear_velocity(&self, system: &PhysicsSystem, velocity: Vec3) {
        unsafe { sys::JPC_Character_SetLinearVelocity(self.check(system), velocity.into()) }
    }

    pub fn add_linear_velocity(&self, system: &PhysicsSystem, velocity: Vec3) {
        unsafe { sys::JPC_Character_AddLinearVelocity(self.check(system), velocity.into()) }
    }

    pub fn set_linear_and_angular_velocity(&self, system: &PhysicsSystem, linear: Vec3, angular: Vec3) {
        unsafe { sys::JPC_Character_SetLinearAndAngularVelocity(self.check(system), linear.into(), angular.into()) }
    }

    pub fn add_impulse(&self, system: &PhysicsSystem, impulse: Vec3) {
        unsafe { sys::JPC_Character_AddImpulse(self.check(system), impulse.into()) }
    }

    pub fn position(&self, system: &PhysicsSystem) -> Vec3 {
        unsafe { sys::JPC_Character_GetPosition(self.check(system)) }.into()
    }

    pub fn set_position(&self, system: &PhysicsSystem, position: Vec3, activation: Activation) {
        unsafe { sys::JPC_Character_SetPosition(self.check(system), position.into(), activation.to_sys()) }
    }

    pub fn rotation(&self, system: &PhysicsSystem) -> Quat {
        unsafe { sys::JPC_Character_GetRotation(self.check(system)) }.into()
    }

    pub fn set_rotation(&self, system: &PhysicsSystem, rotation: Quat, activation: Activation) {
        unsafe { sys::JPC_Character_SetRotation(self.check(system), rotation.into(), activation.to_sys()) }
    }

    pub fn center_of_mass_position(&self, system: &PhysicsSystem) -> Vec3 {
        unsafe { sys::JPC_Character_GetCenterOfMassPosition(self.check(system)) }.into()
    }

    pub fn world_transform(&self, system: &PhysicsSystem) -> Mat44 {
        unsafe { sys::JPC_Character_GetWorldTransform(self.check(system)) }.into()
    }

    pub fn layer(&self) -> ObjectLayer {
        ObjectLayer(unsafe { sys::JPC_Character_GetLayer(self.raw.as_ptr()) })
    }

    /// Fails if the layer is not part of the system's [`LayerConfig`](crate::LayerConfig).
    pub fn set_layer(&mut self, system: &PhysicsSystem, layer: ObjectLayer) -> Result<()> {
        if unsafe { sys::JPC_Character_SetLayer(self.check(system), layer.0) } { Ok(()) } else { Err(Error::last()) }
    }

    /// Switches shape if the new one penetrates the world by less than `max_penetration_depth`.
    pub fn set_shape(&mut self, system: &PhysicsSystem, shape: &Shape, max_penetration_depth: f32) -> bool {
        unsafe { sys::JPC_Character_SetShape(self.check(system), shape.as_raw(), max_penetration_depth) }
    }

    pub fn shape(&self) -> Shape {
        let raw = unsafe { sys::JPC_Character_GetShape(self.raw.as_ptr()) };
        unsafe { sys::JPC_Shape_AddRef(raw) };
        // SAFETY: we just took the reference the handle owns.
        unsafe { Shape::from_raw(NonNull::new_unchecked(raw.cast_mut())) }
    }

    pub fn up(&self) -> Vec3 {
        unsafe { sys::JPC_Character_GetUp(self.raw.as_ptr()) }.into()
    }

    pub fn set_up(&mut self, up: Vec3) {
        unsafe { sys::JPC_Character_SetUp(self.raw.as_ptr(), up.into()) }
    }

    pub fn set_max_slope_angle(&mut self, angle: f32) {
        unsafe { sys::JPC_Character_SetMaxSlopeAngle(self.raw.as_ptr(), angle) }
    }

    /// Ground state as of the last [`post_simulation`](Self::post_simulation).
    pub fn ground(&self) -> CharacterGround {
        let mut raw = sys::JPC_CharacterGround::default();
        unsafe { sys::JPC_Character_GetGround(self.raw.as_ptr(), &mut raw) };
        CharacterGround::from_sys(&raw)
    }

    pub fn is_supported(&self) -> bool {
        unsafe { sys::JPC_Character_IsSupported(self.raw.as_ptr()) }
    }

    /// Removes and destroys the character and its body right away. Needs exclusive access to the system because
    /// the body is freed.
    pub fn destroy(self, system: &mut PhysicsSystem) {
        let raw = self.check(system);
        let this = ManuallyDrop::new(self);
        // SAFETY: `this` is never used or dropped again.
        let _system = unsafe { std::ptr::read(&this.system) };
        unsafe { sys::JPC_Character_Destroy(raw) }
    }
}

impl Drop for Character {
    fn drop(&mut self) {
        self.system.defer(Deferred::Character(self.raw));
    }
}

impl fmt::Debug for Character {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Character").field("body_id", &self.body_id()).finish()
    }
}
