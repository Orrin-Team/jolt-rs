//! Skeletons and ragdolls: a hierarchy of joints, and a set of bodies connected by constraints that follows it.
//!
//! ```
//! use jolt::*;
//!
//! let mut layers = LayerConfig::new();
//! let layer = layers.add_object_layer(BroadPhaseLayer(0));
//! layers.enable_collision(layer, layer);
//! let system = PhysicsSystem::new(&PhysicsSystemSettings::default(), &layers).unwrap();
//!
//! let mut skeleton = SkeletonBuilder::new();
//! let pelvis = skeleton.add_joint("pelvis", None).unwrap();
//! skeleton.add_joint("spine", Some(pelvis)).unwrap();
//! let skeleton = skeleton.build();
//!
//! let part = |y: f32| BodyCreationSettings::new(Shape::capsule(0.15, 0.1).unwrap(), Vec3::new(0.0, y, 0.0), Quat::IDENTITY, MotionType::Dynamic, layer);
//! let joint = SwingTwistConstraintSettings::at(Vec3::new(0.0, 1.25, 0.0), Vec3::Y, Vec3::X);
//! let parts = [RagdollPart::root(part(1.0)), RagdollPart::new(part(1.5), RagdollJoint::SwingTwist(joint))];
//! let settings = RagdollSettings::new(&skeleton, &parts, &RagdollOptions::default()).unwrap();
//!
//! let ragdoll = settings.create_ragdoll(&system, 1, 0).unwrap();
//! ragdoll.add_to_physics_system(&system, Activation::Activate);
//! ```

use std::ffi::{CStr, CString};
use std::fmt;
use std::mem::ManuallyDrop;
use std::ptr::NonNull;
use std::sync::Arc;

use jolt_sys as sys;

use crate::character::Deferred;
use crate::system::SystemInner;
use crate::{
    AABox, Activation, BodyCreationSettings, BodyId, ConeConstraintSettings, ConstraintMut, DistanceConstraintSettings, Error,
    FixedConstraintSettings, HingeConstraintSettings, Mat44, PhysicsSystem, PointConstraintSettings, Quat, Result, SixDofConstraintSettings,
    SliderConstraintSettings, SwingTwistConstraintSettings, Vec3,
};

/// An immutable joint hierarchy, built with [`SkeletonBuilder`]. Cheap to clone.
pub struct Skeleton {
    raw: NonNull<sys::JPC_Skeleton>,
}

// SAFETY: immutable once built, atomic reference count.
unsafe impl Send for Skeleton {}
unsafe impl Sync for Skeleton {}

/// Builds a [`Skeleton`]. Joints are added parent first.
#[derive(Clone, Debug, Default)]
pub struct SkeletonBuilder {
    joints: Vec<(String, Option<usize>)>,
}

impl SkeletonBuilder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds a joint and returns its index. `parent` must be a joint added earlier (`None` for a root).
    pub fn add_joint(&mut self, name: &str, parent: Option<usize>) -> Result<usize> {
        if parent.is_some_and(|p| p >= self.joints.len()) {
            return Err(Error::new("parent joint does not exist"));
        }
        if name.contains('\0') {
            return Err(Error::new("joint names can't contain NUL"));
        }
        self.joints.push((name.to_owned(), parent));
        Ok(self.joints.len() - 1)
    }

    pub fn build(&self) -> Skeleton {
        crate::ensure_init();
        let raw = unsafe { sys::JPC_Skeleton_Create() };
        for (name, parent) in &self.joints {
            let name = CString::new(name.as_str()).expect("checked in add_joint");
            let parent = parent.map_or(-1, |p| p as i32);
            let index = unsafe { sys::JPC_Skeleton_AddJoint(raw, name.as_ptr(), parent) };
            debug_assert!(index >= 0);
        }
        Skeleton { raw: NonNull::new(raw).expect("allocation failed") }
    }
}

/// A joint's transform relative to its parent joint (or the model, for roots).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct JointState {
    pub translation: Vec3,
    pub rotation: Quat,
}

impl Skeleton {
    fn as_raw(&self) -> *const sys::JPC_Skeleton {
        self.raw.as_ptr()
    }

    fn from_borrowed(raw: *const sys::JPC_Skeleton) -> Self {
        unsafe { sys::JPC_Skeleton_AddRef(raw) };
        Self { raw: NonNull::new(raw.cast_mut()).expect("skeleton is never null") }
    }

    pub fn joint_count(&self) -> usize {
        unsafe { sys::JPC_Skeleton_GetJointCount(self.as_raw()) as usize }
    }

    pub fn joint_index(&self, name: &str) -> Option<usize> {
        let name = CString::new(name).ok()?;
        usize::try_from(unsafe { sys::JPC_Skeleton_GetJointIndex(self.as_raw(), name.as_ptr()) }).ok()
    }

    pub fn joint_name(&self, joint: usize) -> Option<&str> {
        let raw = unsafe { sys::JPC_Skeleton_GetJointName(self.as_raw(), u32::try_from(joint).ok()?) };
        // SAFETY: the name lives as long as the (immutable) skeleton.
        (!raw.is_null()).then(|| unsafe { CStr::from_ptr(raw) }.to_str().unwrap_or(""))
    }

    pub fn parent(&self, joint: usize) -> Option<usize> {
        let joint = u32::try_from(joint).ok()?;
        usize::try_from(unsafe { sys::JPC_Skeleton_GetParentIndex(self.as_raw(), joint) }).ok()
    }

    /// Model space matrices from local joint states. Panics unless there is one state per joint.
    pub fn joint_matrices(&self, states: &[JointState]) -> Vec<Mat44> {
        assert_eq!(states.len(), self.joint_count(), "one state per joint");
        let raw: Vec<sys::JPC_JointState> =
            states.iter().map(|s| sys::JPC_JointState { translation: s.translation.into(), rotation: s.rotation.into() }).collect();
        let mut out = vec![sys::JPC_Mat44::default(); states.len()];
        unsafe { sys::JPC_Skeleton_CalculateJointMatrices(self.as_raw(), raw.as_ptr(), out.as_mut_ptr()) };
        out.into_iter().map(Into::into).collect()
    }

    /// Local joint states from model space matrices. Panics unless there is one matrix per joint.
    pub fn joint_states(&self, matrices: &[Mat44]) -> Vec<JointState> {
        assert_eq!(matrices.len(), self.joint_count(), "one matrix per joint");
        let raw: Vec<sys::JPC_Mat44> = matrices.iter().map(|&m| m.into()).collect();
        let mut out = vec![sys::JPC_JointState::default(); matrices.len()];
        unsafe { sys::JPC_Skeleton_CalculateJointStates(self.as_raw(), raw.as_ptr(), out.as_mut_ptr()) };
        out.into_iter().map(|s| JointState { translation: s.translation.into(), rotation: s.rotation.into() }).collect()
    }
}

impl Clone for Skeleton {
    fn clone(&self) -> Self {
        Self::from_borrowed(self.as_raw())
    }
}

impl Drop for Skeleton {
    fn drop(&mut self) {
        unsafe { sys::JPC_Skeleton_Release(self.as_raw()) }
    }
}

impl fmt::Debug for Skeleton {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let joints: Vec<_> = (0..self.joint_count()).map(|i| (self.joint_name(i).unwrap_or(""), self.parent(i))).collect();
        f.debug_struct("Skeleton").field("joints", &joints).finish()
    }
}

/// The constraint connecting a ragdoll part to another. Swing-twist is the usual choice for limbs, hinge for
/// knees and elbows; only those two are driven by [`Ragdoll::drive_to_pose_using_motors`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RagdollJoint {
    Fixed(FixedConstraintSettings),
    Point(PointConstraintSettings),
    Distance(DistanceConstraintSettings),
    Hinge(HingeConstraintSettings),
    Slider(SliderConstraintSettings),
    Cone(ConeConstraintSettings),
    SwingTwist(SwingTwistConstraintSettings),
    SixDof(SixDofConstraintSettings),
}

impl RagdollJoint {
    fn to_sys(self) -> sys::JPC_TwoBodyConstraintSettings {
        use sys::JPC_TwoBodyConstraintSettings__bindgen_ty_1 as U;
        let (type_, settings) = match self {
            Self::Fixed(s) => (sys::JPC_CONSTRAINT_TYPE_FIXED, U { fixed: s.to_sys() }),
            Self::Point(s) => (sys::JPC_CONSTRAINT_TYPE_POINT, U { point: s.to_sys() }),
            Self::Distance(s) => (sys::JPC_CONSTRAINT_TYPE_DISTANCE, U { distance: s.to_sys() }),
            Self::Hinge(s) => (sys::JPC_CONSTRAINT_TYPE_HINGE, U { hinge: s.to_sys() }),
            Self::Slider(s) => (sys::JPC_CONSTRAINT_TYPE_SLIDER, U { slider: s.to_sys() }),
            Self::Cone(s) => (sys::JPC_CONSTRAINT_TYPE_CONE, U { cone: s.to_sys() }),
            Self::SwingTwist(s) => (sys::JPC_CONSTRAINT_TYPE_SWING_TWIST, U { swing_twist: s.to_sys() }),
            Self::SixDof(s) => (sys::JPC_CONSTRAINT_TYPE_SIX_DOF, U { six_dof: s.to_sys() }),
        };
        sys::JPC_TwoBodyConstraintSettings { type_, settings }
    }
}

fn no_joint() -> sys::JPC_TwoBodyConstraintSettings {
    // SAFETY: the union only holds plain C structs; NONE means none of them is read.
    sys::JPC_TwoBodyConstraintSettings { type_: sys::JPC_CONSTRAINT_TYPE_NONE, settings: unsafe { std::mem::zeroed() } }
}

/// One body of a ragdoll, matching the skeleton joint with the same index.
#[derive(Clone, Debug)]
pub struct RagdollPart {
    pub body: BodyCreationSettings,
    /// Constraint to the parent joint's part; `None` for roots (or parts that should float free).
    pub to_parent: Option<RagdollJoint>,
}

impl RagdollPart {
    pub fn new(body: BodyCreationSettings, to_parent: RagdollJoint) -> Self {
        Self { body, to_parent: Some(to_parent) }
    }

    pub fn root(body: BodyCreationSettings) -> Self {
        Self { body, to_parent: None }
    }
}

/// An extra constraint between two parts, e.g. to close a loop.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RagdollConstraint {
    pub body1: usize,
    pub body2: usize,
    pub joint: RagdollJoint,
}

#[derive(Clone, Debug, PartialEq)]
pub struct RagdollOptions {
    pub additional_constraints: Vec<RagdollConstraint>,
    /// Limit mass ratios between parents and children so joints don't jitter. On by default.
    pub stabilize: bool,
    /// Stop parents and children colliding with each other. On by default.
    pub disable_parent_child_collisions: bool,
    /// Also stop parts closer than `min_separation_distance` in this pose (one model space matrix per joint).
    pub pose: Option<Vec<Mat44>>,
    pub min_separation_distance: f32,
    /// Give joints closer to the root a higher priority, starting at this value.
    pub constraint_priorities: Option<u32>,
}

impl Default for RagdollOptions {
    fn default() -> Self {
        Self {
            additional_constraints: Vec::new(),
            stabilize: true,
            disable_parent_child_collisions: true,
            pose: None,
            min_separation_distance: 0.0,
            constraint_priorities: None,
        }
    }
}

/// A ragdoll template. Immutable and cheap to clone; one settings object can spawn many ragdolls.
pub struct RagdollSettings {
    raw: NonNull<sys::JPC_RagdollSettings>,
}

// SAFETY: immutable once created, atomic reference count.
unsafe impl Send for RagdollSettings {}
unsafe impl Sync for RagdollSettings {}

impl RagdollSettings {
    /// `parts` holds one part per skeleton joint, in joint order.
    pub fn new(skeleton: &Skeleton, parts: &[RagdollPart], options: &RagdollOptions) -> Result<Self> {
        if let Some(pose) = &options.pose
            && pose.len() != parts.len()
        {
            return Err(Error::new("the pose needs one matrix per part"));
        }
        let raw_parts: Vec<sys::JPC_RagdollPart> = parts
            .iter()
            .map(|p| sys::JPC_RagdollPart { body: p.body.to_sys(), to_parent: p.to_parent.map_or_else(no_joint, RagdollJoint::to_sys) })
            .collect();
        let extra: Vec<sys::JPC_RagdollConstraint> = options
            .additional_constraints
            .iter()
            .map(|c| sys::JPC_RagdollConstraint {
                body1: u32::try_from(c.body1).unwrap_or(u32::MAX),
                body2: u32::try_from(c.body2).unwrap_or(u32::MAX),
                constraint: c.joint.to_sys(),
            })
            .collect();
        let pose: Option<Vec<sys::JPC_Mat44>> = options.pose.as_ref().map(|p| p.iter().map(|&m| m.into()).collect());
        let desc = sys::JPC_RagdollSettingsDesc {
            skeleton: skeleton.as_raw(),
            parts: raw_parts.as_ptr(),
            num_parts: raw_parts.len() as u32,
            additional_constraints: extra.as_ptr(),
            num_additional_constraints: extra.len() as u32,
            stabilize: options.stabilize,
            disable_parent_child_collisions: options.disable_parent_child_collisions,
            pose_matrices: pose.as_ref().map_or(std::ptr::null(), |p| p.as_ptr()),
            min_separation_distance: options.min_separation_distance,
            calculate_constraint_priorities: options.constraint_priorities.is_some(),
            base_priority: options.constraint_priorities.unwrap_or(0),
        };
        let raw = unsafe { sys::JPC_RagdollSettings_Create(&desc) };
        NonNull::new(raw).map(|raw| Self { raw }).ok_or_else(Error::last)
    }

    pub fn skeleton(&self) -> Skeleton {
        Skeleton::from_borrowed(unsafe { sys::JPC_RagdollSettings_GetSkeleton(self.raw.as_ptr()) })
    }

    /// Creates the ragdoll's bodies and constraints (not yet in the world). All bodies get collision group id
    /// `group_id`; ragdolls with different ids collide with each other.
    pub fn create_ragdoll(&self, system: &PhysicsSystem, group_id: u32, user_data: u64) -> Result<Ragdoll> {
        let raw = unsafe { sys::JPC_RagdollSettings_CreateRagdoll(self.raw.as_ptr(), system.inner.raw(), group_id, user_data) };
        Ok(Ragdoll { raw: NonNull::new(raw).ok_or_else(Error::last)?, system: system.inner.clone() })
    }
}

impl Clone for RagdollSettings {
    fn clone(&self) -> Self {
        unsafe { sys::JPC_RagdollSettings_AddRef(self.raw.as_ptr()) };
        Self { raw: self.raw }
    }
}

impl Drop for RagdollSettings {
    fn drop(&mut self) {
        unsafe { sys::JPC_RagdollSettings_Release(self.raw.as_ptr()) }
    }
}

impl fmt::Debug for RagdollSettings {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RagdollSettings").field("joints", &self.skeleton().joint_count()).finish()
    }
}

/// A live ragdoll. It owns its bodies and constraints: they can't be destroyed through the system or used in
/// other constraints. Like characters, it keeps the system alive, takes the system for anything that touches the
/// world, and defers its destruction to the next update when dropped (use [`destroy`](Self::destroy) to remove it
/// right away).
pub struct Ragdoll {
    raw: NonNull<sys::JPC_Ragdoll>,
    system: Arc<SystemInner>,
}

// SAFETY: world access goes through Jolt's locking interfaces; constraint changes need `&mut self`.
unsafe impl Send for Ragdoll {}
unsafe impl Sync for Ragdoll {}

impl Ragdoll {
    fn check(&self, system: &PhysicsSystem) -> *mut sys::JPC_Ragdoll {
        assert!(Arc::ptr_eq(&self.system, &system.inner), "ragdoll used with a different physics system");
        self.raw.as_ptr()
    }

    fn ptr(&self) -> *mut sys::JPC_Ragdoll {
        self.raw.as_ptr()
    }

    fn matrices(&self, matrices: &[Mat44]) -> Result<Vec<sys::JPC_Mat44>> {
        if matrices.len() != self.body_count() {
            return Err(Error::new("the pose needs one matrix per joint"));
        }
        Ok(matrices.iter().map(|&m| m.into()).collect())
    }

    pub fn settings(&self) -> RagdollSettings {
        let raw = unsafe { sys::JPC_Ragdoll_GetSettings(self.ptr()) };
        unsafe { sys::JPC_RagdollSettings_AddRef(raw) };
        RagdollSettings { raw: NonNull::new(raw.cast_mut()).expect("ragdolls always have settings") }
    }

    pub fn body_count(&self) -> usize {
        unsafe { sys::JPC_Ragdoll_GetBodyCount(self.ptr()) as usize }
    }

    /// The body of a joint.
    pub fn body_id(&self, joint: usize) -> Option<BodyId> {
        BodyId::from_sys(unsafe { sys::JPC_Ragdoll_GetBodyID(self.ptr(), u32::try_from(joint).ok()?) })
    }

    pub fn body_ids(&self) -> Vec<BodyId> {
        (0..self.body_count()).filter_map(|i| self.body_id(i)).collect()
    }

    pub fn constraint_count(&self) -> usize {
        unsafe { sys::JPC_Ragdoll_GetConstraintCount(self.ptr()) as usize }
    }

    /// One of the ragdoll's constraints (joints to parents first, then the additional ones).
    pub fn constraint<'a>(&'a mut self, system: &'a PhysicsSystem, index: usize) -> Option<ConstraintMut<'a>> {
        let raw = unsafe { sys::JPC_Ragdoll_GetConstraint(self.check(system), u32::try_from(index).ok()?) };
        ConstraintMut::from_raw(NonNull::new(raw)?, system.body_interface().as_raw())
    }

    pub fn add_to_physics_system(&self, system: &PhysicsSystem, activation: Activation) {
        unsafe { sys::JPC_Ragdoll_AddToPhysicsSystem(self.check(system), activation.to_sys()) }
    }

    pub fn remove_from_physics_system(&self, system: &PhysicsSystem) {
        unsafe { sys::JPC_Ragdoll_RemoveFromPhysicsSystem(self.check(system)) }
    }

    pub fn is_added(&self) -> bool {
        unsafe { sys::JPC_Ragdoll_IsAdded(self.ptr()) }
    }

    pub fn activate(&self, system: &PhysicsSystem) {
        unsafe { sys::JPC_Ragdoll_Activate(self.check(system)) }
    }

    pub fn is_active(&self, system: &PhysicsSystem) -> bool {
        unsafe { sys::JPC_Ragdoll_IsActive(self.check(system)) }
    }

    pub fn set_group_id(&self, system: &PhysicsSystem, group_id: u32) {
        unsafe { sys::JPC_Ragdoll_SetGroupID(self.check(system), group_id) }
    }

    /// Teleports every part to `matrices` (one model space matrix per joint) offset by `root_offset`.
    pub fn set_pose(&self, system: &PhysicsSystem, root_offset: Vec3, matrices: &[Mat44]) -> Result<()> {
        let m = self.matrices(matrices)?;
        unsafe { sys::JPC_Ragdoll_SetPose(self.check(system), root_offset.into(), m.as_ptr()) };
        Ok(())
    }

    /// The current pose as a root offset and one model space matrix per joint.
    pub fn pose(&self, system: &PhysicsSystem) -> (Vec3, Vec<Mat44>) {
        let mut root = sys::JPC_Vec3::default();
        let mut out = vec![sys::JPC_Mat44::default(); self.body_count()];
        unsafe { sys::JPC_Ragdoll_GetPose(self.check(system), &mut root, out.as_mut_ptr()) };
        (root.into(), out.into_iter().map(Into::into).collect())
    }

    /// Moves the parts towards a pose by setting their velocities, for ragdolls that follow an animation.
    pub fn drive_to_pose_using_kinematics(&self, system: &PhysicsSystem, root_offset: Vec3, matrices: &[Mat44], delta_time: f32) -> Result<()> {
        let m = self.matrices(matrices)?;
        unsafe { sys::JPC_Ragdoll_DriveToPoseUsingKinematics(self.check(system), root_offset.into(), m.as_ptr(), delta_time) };
        Ok(())
    }

    /// Sets the motors of swing-twist and hinge joints to reach a pose while staying physically simulated
    /// (active ragdolls). The joints' motor settings decide how strongly they pull.
    pub fn drive_to_pose_using_motors(&mut self, system: &PhysicsSystem, root_offset: Vec3, matrices: &[Mat44]) -> Result<()> {
        let m = self.matrices(matrices)?;
        unsafe { sys::JPC_Ragdoll_DriveToPoseUsingMotors(self.check(system), root_offset.into(), m.as_ptr()) };
        Ok(())
    }

    /// Forget accumulated constraint impulses, e.g. after teleporting.
    pub fn reset_warm_start(&mut self, system: &PhysicsSystem) {
        unsafe { sys::JPC_Ragdoll_ResetWarmStart(self.check(system)) }
    }

    pub fn set_linear_and_angular_velocity(&self, system: &PhysicsSystem, linear: Vec3, angular: Vec3) {
        unsafe { sys::JPC_Ragdoll_SetLinearAndAngularVelocity(self.check(system), linear.into(), angular.into()) }
    }

    pub fn add_linear_velocity(&self, system: &PhysicsSystem, velocity: Vec3) {
        unsafe { sys::JPC_Ragdoll_AddLinearVelocity(self.check(system), velocity.into()) }
    }

    /// The same impulse on every part. Does nothing while the ragdoll is not in the world.
    pub fn add_impulse(&self, system: &PhysicsSystem, impulse: Vec3) {
        unsafe { sys::JPC_Ragdoll_AddImpulse(self.check(system), impulse.into()) }
    }

    /// Position and rotation of the root part.
    pub fn root_transform(&self, system: &PhysicsSystem) -> (Vec3, Quat) {
        let mut position = sys::JPC_Vec3::default();
        let mut rotation = sys::JPC_Quat::default();
        unsafe { sys::JPC_Ragdoll_GetRootTransform(self.check(system), &mut position, &mut rotation) };
        (position.into(), rotation.into())
    }

    pub fn world_space_bounds(&self, system: &PhysicsSystem) -> AABox {
        unsafe { sys::JPC_Ragdoll_GetWorldSpaceBounds(self.check(system)) }.into()
    }

    /// Removes the ragdoll from the world and destroys its bodies and constraints right away.
    pub fn destroy(self, system: &mut PhysicsSystem) {
        let raw = self.check(system);
        let this = ManuallyDrop::new(self);
        // SAFETY: `this` is never used or dropped again.
        let _system = unsafe { std::ptr::read(&this.system) };
        unsafe { sys::JPC_Ragdoll_Destroy(raw) }
    }
}

impl Drop for Ragdoll {
    fn drop(&mut self) {
        self.system.defer(Deferred::Ragdoll(self.raw));
    }
}

impl fmt::Debug for Ragdoll {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Ragdoll").field("bodies", &self.body_ids()).finish()
    }
}
