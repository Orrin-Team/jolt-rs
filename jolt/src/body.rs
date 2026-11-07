use std::fmt;
use std::marker::PhantomData;

use jolt_sys as sys;

use crate::{AABox, CollisionGroup, Error, Mat44, MassProperties, ObjectLayer, PhysicsSystem, Quat, Result, Shape, Vec3};

/// Identifies a body. Ids of destroyed bodies are never reused for a different body
/// (they carry a sequence number), so a stale id simply stops matching anything.
#[repr(transparent)]
#[derive(Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct BodyId(pub(crate) u32);

impl BodyId {
    pub const INVALID: Self = Self(sys::JPC_BODY_ID_INVALID);

    pub const fn from_raw(raw: u32) -> Self {
        Self(raw)
    }

    pub const fn raw(self) -> u32 {
        self.0
    }

    pub const fn is_valid(self) -> bool {
        self.0 != sys::JPC_BODY_ID_INVALID
    }

    /// Index of the body in the system's body array.
    pub const fn index(self) -> u32 {
        self.0 & 0x007f_ffff
    }

    pub const fn sequence_number(self) -> u8 {
        (self.0 >> 23) as u8
    }

    pub(crate) fn from_sys(raw: u32) -> Option<Self> {
        let id = Self(raw);
        id.is_valid().then_some(id)
    }
}

impl fmt::Debug for BodyId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.is_valid() {
            write!(f, "BodyId({}:{})", self.index(), self.sequence_number())
        } else {
            f.write_str("BodyId(INVALID)")
        }
    }
}

/// Identifies a part of a compound shape, or a triangle of a mesh.
#[repr(transparent)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct SubShapeId(pub u32);

impl SubShapeId {
    pub const EMPTY: Self = Self(sys::JPC_SUB_SHAPE_ID_EMPTY);
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum MotionType {
    /// Never moves.
    Static,
    /// Moved by setting velocities; not affected by forces or collisions.
    Kinematic,
    /// Fully simulated.
    Dynamic,
}

impl MotionType {
    pub(crate) fn to_sys(self) -> sys::JPC_MotionType {
        match self {
            Self::Static => sys::JPC_MOTION_TYPE_STATIC,
            Self::Kinematic => sys::JPC_MOTION_TYPE_KINEMATIC,
            Self::Dynamic => sys::JPC_MOTION_TYPE_DYNAMIC,
        }
    }

    pub(crate) fn from_sys(raw: sys::JPC_MotionType) -> Self {
        match raw {
            sys::JPC_MOTION_TYPE_KINEMATIC => Self::Kinematic,
            sys::JPC_MOTION_TYPE_DYNAMIC => Self::Dynamic,
            _ => Self::Static,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum MotionQuality {
    /// Cheapest; fast bodies can tunnel through thin geometry.
    #[default]
    Discrete,
    /// Continuous collision detection along the linear motion.
    LinearCast,
}

impl MotionQuality {
    fn to_sys(self) -> sys::JPC_MotionQuality {
        match self {
            Self::Discrete => sys::JPC_MOTION_QUALITY_DISCRETE,
            Self::LinearCast => sys::JPC_MOTION_QUALITY_LINEAR_CAST,
        }
    }

    fn from_sys(raw: sys::JPC_MotionQuality) -> Self {
        if raw == sys::JPC_MOTION_QUALITY_LINEAR_CAST { Self::LinearCast } else { Self::Discrete }
    }
}

/// Whether an operation should wake the body up.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum Activation {
    #[default]
    Activate,
    DontActivate,
}

impl Activation {
    pub(crate) fn to_sys(self) -> sys::JPC_Activation {
        match self {
            Self::Activate => sys::JPC_ACTIVATION_ACTIVATE,
            Self::DontActivate => sys::JPC_ACTIVATION_DONT_ACTIVATE,
        }
    }
}

/// Degrees of freedom a body may move in. Combine with `|`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct AllowedDofs(u8);

impl AllowedDofs {
    pub const NONE: Self = Self(0);
    pub const TRANSLATION_X: Self = Self(sys::JPC_ALLOWED_DOFS_TRANSLATION_X as u8);
    pub const TRANSLATION_Y: Self = Self(sys::JPC_ALLOWED_DOFS_TRANSLATION_Y as u8);
    pub const TRANSLATION_Z: Self = Self(sys::JPC_ALLOWED_DOFS_TRANSLATION_Z as u8);
    pub const ROTATION_X: Self = Self(sys::JPC_ALLOWED_DOFS_ROTATION_X as u8);
    pub const ROTATION_Y: Self = Self(sys::JPC_ALLOWED_DOFS_ROTATION_Y as u8);
    pub const ROTATION_Z: Self = Self(sys::JPC_ALLOWED_DOFS_ROTATION_Z as u8);
    pub const PLANE_2D: Self = Self(sys::JPC_ALLOWED_DOFS_PLANE_2D as u8);
    pub const ALL: Self = Self(sys::JPC_ALLOWED_DOFS_ALL as u8);

    pub const fn bits(self) -> u8 {
        self.0
    }

    pub const fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }
}

impl Default for AllowedDofs {
    fn default() -> Self {
        Self::ALL
    }
}

impl std::ops::BitOr for AllowedDofs {
    type Output = Self;
    fn bitor(self, rhs: Self) -> Self {
        Self(self.0 | rhs.0)
    }
}

/// How the mass and inertia of a body are determined.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum OverrideMassProperties {
    /// Calculate both from the shape's density.
    #[default]
    CalculateMassAndInertia,
    /// Use the given mass and scale the calculated inertia to match.
    CalculateInertia { mass: f32 },
    /// Use exactly these values.
    MassAndInertiaProvided(MassProperties),
}

/// Everything needed to create a body.
#[derive(Clone, Debug)]
pub struct BodyCreationSettings {
    pub shape: Shape,
    pub position: Vec3,
    pub rotation: Quat,
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
    pub user_data: u64,
    pub object_layer: ObjectLayer,
    pub collision_group: CollisionGroup,
    pub motion_type: MotionType,
    pub allowed_dofs: AllowedDofs,
    /// Lets a body created as static later become kinematic or dynamic.
    pub allow_dynamic_or_kinematic: bool,
    pub is_sensor: bool,
    pub collide_kinematic_vs_non_dynamic: bool,
    pub use_manifold_reduction: bool,
    pub apply_gyroscopic_force: bool,
    pub motion_quality: MotionQuality,
    pub enhanced_internal_edge_removal: bool,
    pub allow_sleeping: bool,
    pub friction: f32,
    pub restitution: f32,
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub max_linear_velocity: f32,
    pub max_angular_velocity: f32,
    pub gravity_factor: f32,
    pub num_velocity_steps_override: u32,
    pub num_position_steps_override: u32,
    pub override_mass_properties: OverrideMassProperties,
    pub inertia_multiplier: f32,
}

impl BodyCreationSettings {
    /// Jolt's defaults for everything else.
    pub fn new(shape: Shape, position: Vec3, rotation: Quat, motion_type: MotionType, object_layer: ObjectLayer) -> Self {
        let mut d = sys::JPC_BodyCreationSettings::default();
        unsafe { sys::JPC_BodyCreationSettings_SetDefault(&mut d) };
        Self {
            shape,
            position,
            rotation,
            linear_velocity: d.linear_velocity.into(),
            angular_velocity: d.angular_velocity.into(),
            user_data: d.user_data,
            object_layer,
            collision_group: CollisionGroup::default(),
            motion_type,
            allowed_dofs: AllowedDofs(d.allowed_dofs as u8),
            allow_dynamic_or_kinematic: d.allow_dynamic_or_kinematic,
            is_sensor: d.is_sensor,
            collide_kinematic_vs_non_dynamic: d.collide_kinematic_vs_non_dynamic,
            use_manifold_reduction: d.use_manifold_reduction,
            apply_gyroscopic_force: d.apply_gyroscopic_force,
            motion_quality: MotionQuality::from_sys(d.motion_quality),
            enhanced_internal_edge_removal: d.enhanced_internal_edge_removal,
            allow_sleeping: d.allow_sleeping,
            friction: d.friction,
            restitution: d.restitution,
            linear_damping: d.linear_damping,
            angular_damping: d.angular_damping,
            max_linear_velocity: d.max_linear_velocity,
            max_angular_velocity: d.max_angular_velocity,
            gravity_factor: d.gravity_factor,
            num_velocity_steps_override: d.num_velocity_steps_override,
            num_position_steps_override: d.num_position_steps_override,
            override_mass_properties: OverrideMassProperties::CalculateMassAndInertia,
            inertia_multiplier: d.inertia_multiplier,
        }
    }

    pub(crate) fn to_sys(&self) -> sys::JPC_BodyCreationSettings {
        let (override_mode, mass_properties) = match self.override_mass_properties {
            OverrideMassProperties::CalculateMassAndInertia => {
                (sys::JPC_OVERRIDE_MASS_PROPERTIES_CALCULATE_MASS_AND_INERTIA, MassProperties::default())
            }
            OverrideMassProperties::CalculateInertia { mass } => (
                sys::JPC_OVERRIDE_MASS_PROPERTIES_CALCULATE_INERTIA,
                MassProperties { mass, ..Default::default() },
            ),
            OverrideMassProperties::MassAndInertiaProvided(m) => (sys::JPC_OVERRIDE_MASS_PROPERTIES_MASS_AND_INERTIA_PROVIDED, m),
        };
        sys::JPC_BodyCreationSettings {
            position: self.position.into(),
            rotation: self.rotation.into(),
            linear_velocity: self.linear_velocity.into(),
            angular_velocity: self.angular_velocity.into(),
            user_data: self.user_data,
            object_layer: self.object_layer.0,
            motion_type: self.motion_type.to_sys(),
            allowed_dofs: self.allowed_dofs.0 as sys::JPC_AllowedDOFs,
            allow_dynamic_or_kinematic: self.allow_dynamic_or_kinematic,
            is_sensor: self.is_sensor,
            collide_kinematic_vs_non_dynamic: self.collide_kinematic_vs_non_dynamic,
            use_manifold_reduction: self.use_manifold_reduction,
            apply_gyroscopic_force: self.apply_gyroscopic_force,
            motion_quality: self.motion_quality.to_sys(),
            enhanced_internal_edge_removal: self.enhanced_internal_edge_removal,
            allow_sleeping: self.allow_sleeping,
            friction: self.friction,
            restitution: self.restitution,
            linear_damping: self.linear_damping,
            angular_damping: self.angular_damping,
            max_linear_velocity: self.max_linear_velocity,
            max_angular_velocity: self.max_angular_velocity,
            gravity_factor: self.gravity_factor,
            num_velocity_steps_override: self.num_velocity_steps_override,
            num_position_steps_override: self.num_position_steps_override,
            override_mass_properties: override_mode,
            inertia_multiplier: self.inertia_multiplier,
            mass_properties_override: mass_properties.into(),
            collision_group: self.collision_group.to_sys(),
            shape: self.shape.as_raw(),
        }
    }
}

/// Thread-safe access to the bodies of a [`PhysicsSystem`]. Every call locks the body it touches,
/// so it can be used from several threads at once (but not while the system updates, which the
/// borrow checker already rules out). Destroying bodies needs exclusive access and lives on
/// [`PhysicsSystem::destroy_body`].
///
/// Calls with an id that does not refer to a live body do nothing and getters return zeroed values.
///
/// Don't use it from inside query body filters or activation listeners: Jolt holds locks there and the
/// call may deadlock.
#[derive(Clone, Copy)]
pub struct BodyInterface<'a> {
    raw: *mut sys::JPC_BodyInterface,
    _system: PhantomData<&'a PhysicsSystem>,
}

// SAFETY: the locking body interface is designed for concurrent use.
unsafe impl Send for BodyInterface<'_> {}
unsafe impl Sync for BodyInterface<'_> {}

impl<'a> BodyInterface<'a> {
    pub(crate) fn new(raw: *mut sys::JPC_BodyInterface, _system: &'a PhysicsSystem) -> Self {
        Self { raw, _system: PhantomData }
    }

    pub fn as_raw(&self) -> *mut sys::JPC_BodyInterface {
        self.raw
    }

    /// Creates a body without adding it to the world.
    pub fn create_body(&self, settings: &BodyCreationSettings) -> Result<BodyId> {
        let raw = settings.to_sys();
        BodyId::from_sys(unsafe { sys::JPC_BodyInterface_CreateBody(self.raw, &raw) }).ok_or_else(Error::last)
    }

    pub fn create_and_add_body(&self, settings: &BodyCreationSettings, activation: Activation) -> Result<BodyId> {
        let raw = settings.to_sys();
        BodyId::from_sys(unsafe { sys::JPC_BodyInterface_CreateAndAddBody(self.raw, &raw, activation.to_sys()) })
            .ok_or_else(Error::last)
    }

    /// Does nothing if the body is already in the world.
    pub fn add_body(&self, id: BodyId, activation: Activation) {
        unsafe { sys::JPC_BodyInterface_AddBody(self.raw, id.0, activation.to_sys()) }
    }

    pub fn remove_body(&self, id: BodyId) {
        unsafe { sys::JPC_BodyInterface_RemoveBody(self.raw, id.0) }
    }

    /// Adds many bodies at once, which is much faster than adding them one by one. Ids that are invalid,
    /// already added or repeated are skipped. Call [`PhysicsSystem::optimize_broad_phase`] after big batches.
    pub fn add_bodies(&self, ids: &[BodyId], activation: Activation) {
        let count = u32::try_from(ids.len()).expect("too many bodies");
        // BodyId is a transparent u32.
        unsafe { sys::JPC_BodyInterface_AddBodies(self.raw, ids.as_ptr().cast(), count, activation.to_sys()) }
    }

    pub fn remove_bodies(&self, ids: &[BodyId]) {
        let count = u32::try_from(ids.len()).expect("too many bodies");
        unsafe { sys::JPC_BodyInterface_RemoveBodies(self.raw, ids.as_ptr().cast(), count) }
    }

    /// Creates every body and then adds them in one batch. If a body cannot be created (usually because
    /// `max_bodies` was reached) nothing is added and the error lists the bodies that were created, so they can be
    /// destroyed with [`PhysicsSystem::destroy_bodies`].
    pub fn create_and_add_bodies(&self, settings: &[BodyCreationSettings], activation: Activation) -> Result<Vec<BodyId>, BatchError> {
        let mut ids = Vec::with_capacity(settings.len());
        for s in settings {
            match self.create_body(s) {
                Ok(id) => ids.push(id),
                Err(error) => return Err(BatchError { error, created: ids }),
            }
        }
        self.add_bodies(&ids, activation);
        Ok(ids)
    }

    pub fn is_added(&self, id: BodyId) -> bool {
        unsafe { sys::JPC_BodyInterface_IsAdded(self.raw, id.0) }
    }

    pub fn activate_body(&self, id: BodyId) {
        unsafe { sys::JPC_BodyInterface_ActivateBody(self.raw, id.0) }
    }

    pub fn deactivate_body(&self, id: BodyId) {
        unsafe { sys::JPC_BodyInterface_DeactivateBody(self.raw, id.0) }
    }

    pub fn is_active(&self, id: BodyId) -> bool {
        unsafe { sys::JPC_BodyInterface_IsActive(self.raw, id.0) }
    }

    pub fn reset_sleep_timer(&self, id: BodyId) {
        unsafe { sys::JPC_BodyInterface_ResetSleepTimer(self.raw, id.0) }
    }

    pub fn shape(&self, id: BodyId) -> Option<Shape> {
        let raw = unsafe { sys::JPC_BodyInterface_GetShape(self.raw, id.0) };
        // SAFETY: GetShape hands us a new reference.
        std::ptr::NonNull::new(raw).map(|raw| unsafe { Shape::from_raw(raw) })
    }

    pub fn set_shape(&self, id: BodyId, shape: &Shape, update_mass_properties: bool, activation: Activation) {
        unsafe { sys::JPC_BodyInterface_SetShape(self.raw, id.0, shape.as_raw(), update_mass_properties, activation.to_sys()) }
    }

    pub fn object_layer(&self, id: BodyId) -> ObjectLayer {
        ObjectLayer(unsafe { sys::JPC_BodyInterface_GetObjectLayer(self.raw, id.0) })
    }

    /// Fails if the layer is not part of the system's [`LayerConfig`](crate::LayerConfig).
    pub fn set_object_layer(&self, id: BodyId, layer: ObjectLayer) -> Result<()> {
        if unsafe { sys::JPC_BodyInterface_SetObjectLayer(self.raw, id.0, layer.0) } { Ok(()) } else { Err(Error::last()) }
    }

    pub fn position(&self, id: BodyId) -> Vec3 {
        unsafe { sys::JPC_BodyInterface_GetPosition(self.raw, id.0) }.into()
    }

    pub fn set_position(&self, id: BodyId, position: Vec3, activation: Activation) {
        unsafe { sys::JPC_BodyInterface_SetPosition(self.raw, id.0, position.into(), activation.to_sys()) }
    }

    pub fn center_of_mass_position(&self, id: BodyId) -> Vec3 {
        unsafe { sys::JPC_BodyInterface_GetCenterOfMassPosition(self.raw, id.0) }.into()
    }

    pub fn rotation(&self, id: BodyId) -> Quat {
        unsafe { sys::JPC_BodyInterface_GetRotation(self.raw, id.0) }.into()
    }

    pub fn set_rotation(&self, id: BodyId, rotation: Quat, activation: Activation) {
        unsafe { sys::JPC_BodyInterface_SetRotation(self.raw, id.0, rotation.into(), activation.to_sys()) }
    }

    pub fn position_and_rotation(&self, id: BodyId) -> (Vec3, Quat) {
        let mut position = sys::JPC_Vec3::default();
        let mut rotation = sys::JPC_Quat::default();
        unsafe { sys::JPC_BodyInterface_GetPositionAndRotation(self.raw, id.0, &mut position, &mut rotation) };
        (position.into(), rotation.into())
    }

    pub fn set_position_and_rotation(&self, id: BodyId, position: Vec3, rotation: Quat, activation: Activation) {
        unsafe {
            sys::JPC_BodyInterface_SetPositionAndRotation(self.raw, id.0, position.into(), rotation.into(), activation.to_sys())
        }
    }

    /// Like [`set_position_and_rotation`](Self::set_position_and_rotation) but skips bodies that did not move,
    /// avoiding needless wake-ups and broad phase updates.
    pub fn set_position_and_rotation_when_changed(&self, id: BodyId, position: Vec3, rotation: Quat, activation: Activation) {
        unsafe {
            sys::JPC_BodyInterface_SetPositionAndRotationWhenChanged(
                self.raw,
                id.0,
                position.into(),
                rotation.into(),
                activation.to_sys(),
            )
        }
    }

    pub fn world_transform(&self, id: BodyId) -> Mat44 {
        unsafe { sys::JPC_BodyInterface_GetWorldTransform(self.raw, id.0) }.into()
    }

    pub fn center_of_mass_transform(&self, id: BodyId) -> Mat44 {
        unsafe { sys::JPC_BodyInterface_GetCenterOfMassTransform(self.raw, id.0) }.into()
    }

    /// Sets the velocities of a kinematic body so it reaches the target in `delta_time` seconds.
    pub fn move_kinematic(&self, id: BodyId, target_position: Vec3, target_rotation: Quat, delta_time: f32) {
        unsafe {
            sys::JPC_BodyInterface_MoveKinematic(self.raw, id.0, target_position.into(), target_rotation.into(), delta_time)
        }
    }

    pub fn linear_velocity(&self, id: BodyId) -> Vec3 {
        unsafe { sys::JPC_BodyInterface_GetLinearVelocity(self.raw, id.0) }.into()
    }

    pub fn set_linear_velocity(&self, id: BodyId, velocity: Vec3) {
        unsafe { sys::JPC_BodyInterface_SetLinearVelocity(self.raw, id.0, velocity.into()) }
    }

    pub fn add_linear_velocity(&self, id: BodyId, velocity: Vec3) {
        unsafe { sys::JPC_BodyInterface_AddLinearVelocity(self.raw, id.0, velocity.into()) }
    }

    pub fn angular_velocity(&self, id: BodyId) -> Vec3 {
        unsafe { sys::JPC_BodyInterface_GetAngularVelocity(self.raw, id.0) }.into()
    }

    pub fn set_angular_velocity(&self, id: BodyId, velocity: Vec3) {
        unsafe { sys::JPC_BodyInterface_SetAngularVelocity(self.raw, id.0, velocity.into()) }
    }

    pub fn set_linear_and_angular_velocity(&self, id: BodyId, linear: Vec3, angular: Vec3) {
        unsafe { sys::JPC_BodyInterface_SetLinearAndAngularVelocity(self.raw, id.0, linear.into(), angular.into()) }
    }

    /// Velocity of a world space point attached to the body.
    pub fn point_velocity(&self, id: BodyId, point: Vec3) -> Vec3 {
        unsafe { sys::JPC_BodyInterface_GetPointVelocity(self.raw, id.0, point.into()) }.into()
    }

    /// Applied at the center of mass during the next update.
    pub fn add_force(&self, id: BodyId, force: Vec3, activation: Activation) {
        unsafe { sys::JPC_BodyInterface_AddForce(self.raw, id.0, force.into(), activation.to_sys()) }
    }

    pub fn add_force_at_point(&self, id: BodyId, force: Vec3, point: Vec3, activation: Activation) {
        unsafe { sys::JPC_BodyInterface_AddForceAtPoint(self.raw, id.0, force.into(), point.into(), activation.to_sys()) }
    }

    pub fn add_torque(&self, id: BodyId, torque: Vec3, activation: Activation) {
        unsafe { sys::JPC_BodyInterface_AddTorque(self.raw, id.0, torque.into(), activation.to_sys()) }
    }

    pub fn add_impulse(&self, id: BodyId, impulse: Vec3) {
        unsafe { sys::JPC_BodyInterface_AddImpulse(self.raw, id.0, impulse.into()) }
    }

    pub fn add_impulse_at_point(&self, id: BodyId, impulse: Vec3, point: Vec3) {
        unsafe { sys::JPC_BodyInterface_AddImpulseAtPoint(self.raw, id.0, impulse.into(), point.into()) }
    }

    pub fn add_angular_impulse(&self, id: BodyId, angular_impulse: Vec3) {
        unsafe { sys::JPC_BodyInterface_AddAngularImpulse(self.raw, id.0, angular_impulse.into()) }
    }

    pub fn motion_type(&self, id: BodyId) -> MotionType {
        MotionType::from_sys(unsafe { sys::JPC_BodyInterface_GetMotionType(self.raw, id.0) })
    }

    /// Fails when making a body movable that was created static without `allow_dynamic_or_kinematic`.
    pub fn set_motion_type(&self, id: BodyId, motion_type: MotionType, activation: Activation) -> Result<()> {
        if unsafe { sys::JPC_BodyInterface_SetMotionType(self.raw, id.0, motion_type.to_sys(), activation.to_sys()) } {
            Ok(())
        } else {
            Err(Error::last())
        }
    }

    pub fn motion_quality(&self, id: BodyId) -> MotionQuality {
        MotionQuality::from_sys(unsafe { sys::JPC_BodyInterface_GetMotionQuality(self.raw, id.0) })
    }

    pub fn set_motion_quality(&self, id: BodyId, quality: MotionQuality) {
        unsafe { sys::JPC_BodyInterface_SetMotionQuality(self.raw, id.0, quality.to_sys()) }
    }

    pub fn friction(&self, id: BodyId) -> f32 {
        unsafe { sys::JPC_BodyInterface_GetFriction(self.raw, id.0) }
    }

    pub fn set_friction(&self, id: BodyId, friction: f32) {
        unsafe { sys::JPC_BodyInterface_SetFriction(self.raw, id.0, friction) }
    }

    pub fn restitution(&self, id: BodyId) -> f32 {
        unsafe { sys::JPC_BodyInterface_GetRestitution(self.raw, id.0) }
    }

    pub fn set_restitution(&self, id: BodyId, restitution: f32) {
        unsafe { sys::JPC_BodyInterface_SetRestitution(self.raw, id.0, restitution) }
    }

    pub fn gravity_factor(&self, id: BodyId) -> f32 {
        unsafe { sys::JPC_BodyInterface_GetGravityFactor(self.raw, id.0) }
    }

    pub fn set_gravity_factor(&self, id: BodyId, factor: f32) {
        unsafe { sys::JPC_BodyInterface_SetGravityFactor(self.raw, id.0, factor) }
    }

    pub fn is_sensor(&self, id: BodyId) -> bool {
        unsafe { sys::JPC_BodyInterface_IsSensor(self.raw, id.0) }
    }

    pub fn set_is_sensor(&self, id: BodyId, is_sensor: bool) {
        unsafe { sys::JPC_BodyInterface_SetIsSensor(self.raw, id.0, is_sensor) }
    }

    pub fn user_data(&self, id: BodyId) -> u64 {
        unsafe { sys::JPC_BodyInterface_GetUserData(self.raw, id.0) }
    }

    pub fn set_user_data(&self, id: BodyId, user_data: u64) {
        unsafe { sys::JPC_BodyInterface_SetUserData(self.raw, id.0, user_data) }
    }

    /// Forces contacts with this body to be recalculated on the next update.
    pub fn invalidate_contact_cache(&self, id: BodyId) {
        unsafe { sys::JPC_BodyInterface_InvalidateContactCache(self.raw, id.0) }
    }

    pub fn collision_group(&self, id: BodyId) -> CollisionGroup {
        let mut raw = sys::JPC_CollisionGroup::default();
        unsafe { sys::JPC_BodyInterface_GetCollisionGroup(self.raw, id.0, &mut raw) };
        CollisionGroup::from_owned_sys(&raw)
    }

    pub fn set_collision_group(&self, id: BodyId, group: &CollisionGroup) {
        let raw = group.to_sys();
        unsafe { sys::JPC_BodyInterface_SetCollisionGroup(self.raw, id.0, &raw) }
    }

    pub fn world_space_bounds(&self, id: BodyId) -> AABox {
        unsafe { sys::JPC_BodyInterface_GetWorldSpaceBounds(self.raw, id.0) }.into()
    }

    /// `None` for static and kinematic bodies, which have infinite mass.
    pub fn mass(&self, id: BodyId) -> Option<f32> {
        let inv = unsafe { sys::JPC_BodyInterface_GetInverseMass(self.raw, id.0) };
        (inv > 0.0).then(|| 1.0 / inv)
    }

    /// World space inverse inertia tensor.
    pub fn inverse_inertia(&self, id: BodyId) -> Mat44 {
        unsafe { sys::JPC_BodyInterface_GetInverseInertia(self.raw, id.0) }.into()
    }

    pub fn max_linear_velocity(&self, id: BodyId) -> f32 {
        unsafe { sys::JPC_BodyInterface_GetMaxLinearVelocity(self.raw, id.0) }
    }

    pub fn set_max_linear_velocity(&self, id: BodyId, velocity: f32) {
        unsafe { sys::JPC_BodyInterface_SetMaxLinearVelocity(self.raw, id.0, velocity) }
    }

    pub fn max_angular_velocity(&self, id: BodyId) -> f32 {
        unsafe { sys::JPC_BodyInterface_GetMaxAngularVelocity(self.raw, id.0) }
    }

    pub fn set_max_angular_velocity(&self, id: BodyId, velocity: f32) {
        unsafe { sys::JPC_BodyInterface_SetMaxAngularVelocity(self.raw, id.0, velocity) }
    }

    pub fn use_manifold_reduction(&self, id: BodyId) -> bool {
        unsafe { sys::JPC_BodyInterface_GetUseManifoldReduction(self.raw, id.0) }
    }

    pub fn set_use_manifold_reduction(&self, id: BodyId, use_reduction: bool) {
        unsafe { sys::JPC_BodyInterface_SetUseManifoldReduction(self.raw, id.0, use_reduction) }
    }

    /// Applies buoyancy and drag for a fluid whose surface passes through `surface_position` with normal `surface_normal`.
    /// `buoyancy` is the fluid density relative to the body (1 = neutral). Returns false if the body is not dynamic,
    /// not in the world or not submerged.
    #[allow(clippy::too_many_arguments)]
    pub fn apply_buoyancy_impulse(
        &self,
        id: BodyId,
        surface_position: Vec3,
        surface_normal: Vec3,
        buoyancy: f32,
        linear_drag: f32,
        angular_drag: f32,
        fluid_velocity: Vec3,
        gravity: Vec3,
        delta_time: f32,
    ) -> bool {
        unsafe {
            sys::JPC_BodyInterface_ApplyBuoyancyImpulse(
                self.raw,
                id.0,
                surface_position.into(),
                surface_normal.into(),
                buoyancy,
                linear_drag,
                angular_drag,
                fluid_velocity.into(),
                gravity.into(),
                delta_time,
            )
        }
    }
}

/// Returned by [`BodyInterface::create_and_add_bodies`] when a body could not be created.
#[derive(Clone, Debug)]
pub struct BatchError {
    pub error: Error,
    /// Bodies created before the failure. They exist but are not in the world.
    pub created: Vec<BodyId>,
}

impl fmt::Display for BatchError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} (after creating {} bodies)", self.error, self.created.len())
    }
}

impl std::error::Error for BatchError {}

/// Read-only view of a body, handed to listener callbacks.
#[repr(transparent)]
pub struct Body(sys::JPC_Body);

impl Body {
    /// # Safety
    /// `raw` must point to a live body for the returned lifetime.
    pub(crate) unsafe fn from_raw<'a>(raw: *const sys::JPC_Body) -> &'a Self {
        unsafe { &*raw.cast::<Self>() }
    }

    fn raw(&self) -> *const sys::JPC_Body {
        (self as *const Self).cast()
    }

    pub fn id(&self) -> BodyId {
        BodyId(unsafe { sys::JPC_Body_GetID(self.raw()) })
    }

    pub fn user_data(&self) -> u64 {
        unsafe { sys::JPC_Body_GetUserData(self.raw()) }
    }

    pub fn object_layer(&self) -> ObjectLayer {
        ObjectLayer(unsafe { sys::JPC_Body_GetObjectLayer(self.raw()) })
    }

    pub fn motion_type(&self) -> MotionType {
        MotionType::from_sys(unsafe { sys::JPC_Body_GetMotionType(self.raw()) })
    }

    pub fn is_sensor(&self) -> bool {
        unsafe { sys::JPC_Body_IsSensor(self.raw()) }
    }

    pub fn is_active(&self) -> bool {
        unsafe { sys::JPC_Body_IsActive(self.raw()) }
    }

    pub fn position(&self) -> Vec3 {
        unsafe { sys::JPC_Body_GetPosition(self.raw()) }.into()
    }

    pub fn rotation(&self) -> Quat {
        unsafe { sys::JPC_Body_GetRotation(self.raw()) }.into()
    }

    pub fn center_of_mass_position(&self) -> Vec3 {
        unsafe { sys::JPC_Body_GetCenterOfMassPosition(self.raw()) }.into()
    }

    pub fn linear_velocity(&self) -> Vec3 {
        unsafe { sys::JPC_Body_GetLinearVelocity(self.raw()) }.into()
    }

    pub fn angular_velocity(&self) -> Vec3 {
        unsafe { sys::JPC_Body_GetAngularVelocity(self.raw()) }.into()
    }

    pub fn friction(&self) -> f32 {
        unsafe { sys::JPC_Body_GetFriction(self.raw()) }
    }

    pub fn restitution(&self) -> f32 {
        unsafe { sys::JPC_Body_GetRestitution(self.raw()) }
    }

    pub fn world_space_bounds(&self) -> AABox {
        unsafe { sys::JPC_Body_GetWorldSpaceBounds(self.raw()) }.into()
    }

    /// A new handle to the body's current shape.
    pub fn shape(&self) -> Shape {
        let raw = unsafe { sys::JPC_Body_GetShape(self.raw()) };
        unsafe { sys::JPC_Shape_AddRef(raw) };
        // SAFETY: we just took a reference for the new handle.
        unsafe { Shape::from_raw(std::ptr::NonNull::new_unchecked(raw.cast_mut())) }
    }
}

impl fmt::Debug for Body {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Body").field("id", &self.id()).field("motion_type", &self.motion_type()).finish()
    }
}
