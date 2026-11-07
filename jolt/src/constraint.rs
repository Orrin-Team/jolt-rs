use std::collections::HashMap;
use std::marker::PhantomData;
use std::ptr::NonNull;

use jolt_sys as sys;

use crate::{BodyId, Quat, Vec3};

/// Identifies a constraint owned by a [`PhysicsSystem`](crate::PhysicsSystem).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct ConstraintId(u64);

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ConstraintKind {
    Fixed,
    Point,
    Distance,
    Hinge,
    Slider,
    Cone,
    SwingTwist,
    SixDof,
    Gear,
    RackAndPinion,
    Pulley,
    Path,
    /// See [`PhysicsSystem::vehicle`](crate::PhysicsSystem::vehicle).
    Vehicle,
}

/// Shape of the swing limit of swing-twist and six DOF constraints.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SwingType {
    /// Symmetric cone limits; cheap but deforms for large angles.
    #[default]
    Cone,
    /// Supports asymmetric limits.
    Pyramid,
}

impl SwingType {
    fn to_sys(self) -> sys::JPC_SwingType {
        match self {
            Self::Cone => sys::JPC_SWING_TYPE_CONE,
            Self::Pyramid => sys::JPC_SWING_TYPE_PYRAMID,
        }
    }

    fn from_sys(raw: sys::JPC_SwingType) -> Self {
        if raw == sys::JPC_SWING_TYPE_PYRAMID { Self::Pyramid } else { Self::Cone }
    }
}

/// An axis of a six DOF constraint, in constraint space.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum SixDofAxis {
    TranslationX = 0,
    TranslationY = 1,
    TranslationZ = 2,
    RotationX = 3,
    RotationY = 4,
    RotationZ = 5,
}

impl SixDofAxis {
    pub const ALL: [Self; 6] =
        [Self::TranslationX, Self::TranslationY, Self::TranslationZ, Self::RotationX, Self::RotationY, Self::RotationZ];
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ConstraintSpace {
    /// Points and axes are relative to each body's center of mass.
    LocalToBodyCom,
    #[default]
    WorldSpace,
}

impl ConstraintSpace {
    fn to_sys(self) -> sys::JPC_ConstraintSpace {
        match self {
            Self::LocalToBodyCom => sys::JPC_CONSTRAINT_SPACE_LOCAL_TO_BODY_COM,
            Self::WorldSpace => sys::JPC_CONSTRAINT_SPACE_WORLD_SPACE,
        }
    }

    fn from_sys(raw: sys::JPC_ConstraintSpace) -> Self {
        if raw == sys::JPC_CONSTRAINT_SPACE_LOCAL_TO_BODY_COM { Self::LocalToBodyCom } else { Self::WorldSpace }
    }
}

/// A soft limit or motor spring. A frequency or stiffness of 0 makes the limit hard.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SpringSettings {
    FrequencyAndDamping { frequency: f32, damping: f32 },
    StiffnessAndDamping { stiffness: f32, damping: f32 },
}

impl Default for SpringSettings {
    fn default() -> Self {
        Self::FrequencyAndDamping { frequency: 0.0, damping: 0.0 }
    }
}

impl SpringSettings {
    pub(crate) fn to_sys(self) -> sys::JPC_SpringSettings {
        match self {
            Self::FrequencyAndDamping { frequency, damping } => sys::JPC_SpringSettings {
                mode: sys::JPC_SPRING_MODE_FREQUENCY_AND_DAMPING,
                frequency_or_stiffness: frequency,
                damping,
            },
            Self::StiffnessAndDamping { stiffness, damping } => sys::JPC_SpringSettings {
                mode: sys::JPC_SPRING_MODE_STIFFNESS_AND_DAMPING,
                frequency_or_stiffness: stiffness,
                damping,
            },
        }
    }

    pub(crate) fn from_sys(raw: sys::JPC_SpringSettings) -> Self {
        if raw.mode == sys::JPC_SPRING_MODE_STIFFNESS_AND_DAMPING {
            Self::StiffnessAndDamping { stiffness: raw.frequency_or_stiffness, damping: raw.damping }
        } else {
            Self::FrequencyAndDamping { frequency: raw.frequency_or_stiffness, damping: raw.damping }
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorSettings {
    pub spring: SpringSettings,
    pub min_force_limit: f32,
    pub max_force_limit: f32,
    pub min_torque_limit: f32,
    pub max_torque_limit: f32,
}

impl MotorSettings {
    fn to_sys(self) -> sys::JPC_MotorSettings {
        sys::JPC_MotorSettings {
            spring: self.spring.to_sys(),
            min_force_limit: self.min_force_limit,
            max_force_limit: self.max_force_limit,
            min_torque_limit: self.min_torque_limit,
            max_torque_limit: self.max_torque_limit,
        }
    }

    fn from_sys(raw: sys::JPC_MotorSettings) -> Self {
        Self {
            spring: SpringSettings::from_sys(raw.spring),
            min_force_limit: raw.min_force_limit,
            max_force_limit: raw.max_force_limit,
            min_torque_limit: raw.min_torque_limit,
            max_torque_limit: raw.max_torque_limit,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum MotorState {
    #[default]
    Off,
    Velocity,
    Position,
    PositionAndVelocity,
}

impl MotorState {
    fn to_sys(self) -> sys::JPC_MotorState {
        match self {
            Self::Off => sys::JPC_MOTOR_STATE_OFF,
            Self::Velocity => sys::JPC_MOTOR_STATE_VELOCITY,
            Self::Position => sys::JPC_MOTOR_STATE_POSITION,
            Self::PositionAndVelocity => sys::JPC_MOTOR_STATE_POSITION_AND_VELOCITY,
        }
    }
}

/// Settings every constraint has.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ConstraintCommon {
    pub enabled: bool,
    /// Higher priority constraints are solved later, so they win when constraints fight.
    pub priority: u32,
    pub num_velocity_steps_override: u32,
    pub num_position_steps_override: u32,
    pub user_data: u64,
}

impl ConstraintCommon {
    pub(crate) fn to_sys(self) -> sys::JPC_ConstraintSettings {
        sys::JPC_ConstraintSettings {
            enabled: self.enabled,
            constraint_priority: self.priority,
            num_velocity_steps_override: self.num_velocity_steps_override,
            num_position_steps_override: self.num_position_steps_override,
            user_data: self.user_data,
        }
    }

    pub(crate) fn from_sys(raw: sys::JPC_ConstraintSettings) -> Self {
        Self {
            enabled: raw.enabled,
            priority: raw.constraint_priority,
            num_velocity_steps_override: raw.num_velocity_steps_override,
            num_position_steps_override: raw.num_position_steps_override,
            user_data: raw.user_data,
        }
    }
}

pub(crate) mod private {
    pub trait Sealed {}
}

/// Implemented by the settings types that can create a constraint.
pub trait ConstraintSettings: private::Sealed {
    #[doc(hidden)]
    fn kind(&self) -> ConstraintKind;

    #[doc(hidden)]
    /// # Safety
    /// `system` must be a live physics system.
    unsafe fn create(&self, system: *mut sys::JPC_PhysicsSystem, body1: u32, body2: u32) -> *mut sys::JPC_Constraint;
}

macro_rules! constraint_settings {
    ($name:ident, $kind:ident, $sys:ident, $default_fn:ident, $create_fn:ident, { $($(#[$meta:meta])* $field:ident : $ty:ty),* $(,)? }) => {
        #[derive(Clone, Copy, Debug, PartialEq)]
        pub struct $name {
            pub common: ConstraintCommon,
            pub space: ConstraintSpace,
            $($(#[$meta])* pub $field: $ty,)*
        }

        impl Default for $name {
            fn default() -> Self {
                let mut raw = sys::$sys::default();
                unsafe { sys::$default_fn(&mut raw) };
                Self::from_sys(&raw)
            }
        }

        impl private::Sealed for $name {}

        impl ConstraintSettings for $name {
            fn kind(&self) -> ConstraintKind {
                ConstraintKind::$kind
            }

            unsafe fn create(&self, system: *mut sys::JPC_PhysicsSystem, body1: u32, body2: u32) -> *mut sys::JPC_Constraint {
                let raw = self.to_sys();
                unsafe { sys::$create_fn(system, &raw, body1, body2) }
            }
        }
    };
}

constraint_settings!(FixedConstraintSettings, Fixed, JPC_FixedConstraintSettings, JPC_FixedConstraintSettings_SetDefault, JPC_PhysicsSystem_CreateFixedConstraint, {
    /// Use the bodies' current relative position instead of the points and axes below.
    auto_detect_point: bool,
    point1: Vec3,
    axis_x1: Vec3,
    axis_y1: Vec3,
    point2: Vec3,
    axis_x2: Vec3,
    axis_y2: Vec3,
});

constraint_settings!(PointConstraintSettings, Point, JPC_PointConstraintSettings, JPC_PointConstraintSettings_SetDefault, JPC_PhysicsSystem_CreatePointConstraint, {
    point1: Vec3,
    point2: Vec3,
});

constraint_settings!(DistanceConstraintSettings, Distance, JPC_DistanceConstraintSettings, JPC_DistanceConstraintSettings_SetDefault, JPC_PhysicsSystem_CreateDistanceConstraint, {
    point1: Vec3,
    point2: Vec3,
    /// Negative means "the initial distance".
    min_distance: f32,
    /// Negative means "the initial distance".
    max_distance: f32,
    limits_spring: SpringSettings,
});

constraint_settings!(HingeConstraintSettings, Hinge, JPC_HingeConstraintSettings, JPC_HingeConstraintSettings_SetDefault, JPC_PhysicsSystem_CreateHingeConstraint, {
    point1: Vec3,
    hinge_axis1: Vec3,
    normal_axis1: Vec3,
    point2: Vec3,
    hinge_axis2: Vec3,
    normal_axis2: Vec3,
    limits_min: f32,
    limits_max: f32,
    limits_spring: SpringSettings,
    max_friction_torque: f32,
    motor: MotorSettings,
});

constraint_settings!(SliderConstraintSettings, Slider, JPC_SliderConstraintSettings, JPC_SliderConstraintSettings_SetDefault, JPC_PhysicsSystem_CreateSliderConstraint, {
    auto_detect_point: bool,
    point1: Vec3,
    slider_axis1: Vec3,
    normal_axis1: Vec3,
    point2: Vec3,
    slider_axis2: Vec3,
    normal_axis2: Vec3,
    limits_min: f32,
    limits_max: f32,
    limits_spring: SpringSettings,
    max_friction_force: f32,
    motor: MotorSettings,
});

constraint_settings!(ConeConstraintSettings, Cone, JPC_ConeConstraintSettings, JPC_ConeConstraintSettings_SetDefault, JPC_PhysicsSystem_CreateConeConstraint, {
    point1: Vec3,
    twist_axis1: Vec3,
    point2: Vec3,
    twist_axis2: Vec3,
    half_cone_angle: f32,
});

constraint_settings!(SwingTwistConstraintSettings, SwingTwist, JPC_SwingTwistConstraintSettings, JPC_SwingTwistConstraintSettings_SetDefault, JPC_PhysicsSystem_CreateSwingTwistConstraint, {
    position1: Vec3,
    twist_axis1: Vec3,
    plane_axis1: Vec3,
    position2: Vec3,
    twist_axis2: Vec3,
    plane_axis2: Vec3,
    swing_type: SwingType,
    normal_half_cone_angle: f32,
    plane_half_cone_angle: f32,
    twist_min_angle: f32,
    twist_max_angle: f32,
    max_friction_torque: f32,
    swing_motor: MotorSettings,
    twist_motor: MotorSettings,
});

constraint_settings!(SixDofConstraintSettings, SixDof, JPC_SixDOFConstraintSettings, JPC_SixDOFConstraintSettings_SetDefault, JPC_PhysicsSystem_CreateSixDOFConstraint, {
    position1: Vec3,
    axis_x1: Vec3,
    axis_y1: Vec3,
    position2: Vec3,
    axis_x2: Vec3,
    axis_y2: Vec3,
    /// Indexed by [`SixDofAxis`].
    max_friction: [f32; 6],
    swing_type: SwingType,
    /// Indexed by [`SixDofAxis`]. `min > max` fixes the axis, `-f32::MAX..f32::MAX` frees it.
    limit_min: [f32; 6],
    limit_max: [f32; 6],
    /// Translation axes only.
    limits_spring: [SpringSettings; 3],
    motor: [MotorSettings; 6],
});

constraint_settings!(GearConstraintSettings, Gear, JPC_GearConstraintSettings, JPC_GearConstraintSettings_SetDefault, JPC_PhysicsSystem_CreateGearConstraint, {
    hinge_axis1: Vec3,
    hinge_axis2: Vec3,
    /// Teeth of gear 2 / teeth of gear 1: body 2 turns `1 / ratio` times as fast as body 1, the other way.
    ratio: f32,
});

constraint_settings!(RackAndPinionConstraintSettings, RackAndPinion, JPC_RackAndPinionConstraintSettings, JPC_RackAndPinionConstraintSettings_SetDefault, JPC_PhysicsSystem_CreateRackAndPinionConstraint, {
    /// Rotation axis of the pinion (body 1).
    hinge_axis: Vec3,
    /// Sliding axis of the rack (body 2).
    slider_axis: Vec3,
    /// Pinion rotation (radians) per meter of rack travel.
    ratio: f32,
});

constraint_settings!(PulleyConstraintSettings, Pulley, JPC_PulleyConstraintSettings, JPC_PulleyConstraintSettings_SetDefault, JPC_PhysicsSystem_CreatePulleyConstraint, {
    body_point1: Vec3,
    fixed_point1: Vec3,
    body_point2: Vec3,
    fixed_point2: Vec3,
    /// Length on side 1 + ratio * length on side 2 stays within the limits.
    ratio: f32,
    min_length: f32,
    /// Negative means "the initial length".
    max_length: f32,
});

impl GearConstraintSettings {
    /// Gears turning around the given world space axes.
    pub fn new(hinge_axis1: Vec3, hinge_axis2: Vec3, ratio: f32) -> Self {
        Self { hinge_axis1, hinge_axis2, ratio, ..Default::default() }
    }

    /// Ratio from tooth counts.
    pub fn with_teeth(hinge_axis1: Vec3, hinge_axis2: Vec3, teeth1: u32, teeth2: u32) -> Self {
        Self::new(hinge_axis1, hinge_axis2, teeth2 as f32 / teeth1 as f32)
    }

    fn to_sys(self) -> sys::JPC_GearConstraintSettings {
        sys::JPC_GearConstraintSettings {
            base: self.common.to_sys(),
            space: self.space.to_sys(),
            hinge_axis1: self.hinge_axis1.into(),
            hinge_axis2: self.hinge_axis2.into(),
            ratio: self.ratio,
        }
    }

    fn from_sys(r: &sys::JPC_GearConstraintSettings) -> Self {
        Self {
            common: ConstraintCommon::from_sys(r.base),
            space: ConstraintSpace::from_sys(r.space),
            hinge_axis1: r.hinge_axis1.into(),
            hinge_axis2: r.hinge_axis2.into(),
            ratio: r.ratio,
        }
    }
}

impl RackAndPinionConstraintSettings {
    pub fn new(hinge_axis: Vec3, slider_axis: Vec3, ratio: f32) -> Self {
        Self { hinge_axis, slider_axis, ratio, ..Default::default() }
    }

    /// Ratio from tooth counts: `rack_teeth` teeth over `rack_length` meters meshing with a `pinion_teeth` pinion.
    pub fn with_teeth(hinge_axis: Vec3, slider_axis: Vec3, rack_teeth: u32, rack_length: f32, pinion_teeth: u32) -> Self {
        Self::new(hinge_axis, slider_axis, std::f32::consts::TAU * rack_teeth as f32 / (rack_length * pinion_teeth as f32))
    }

    fn to_sys(self) -> sys::JPC_RackAndPinionConstraintSettings {
        sys::JPC_RackAndPinionConstraintSettings {
            base: self.common.to_sys(),
            space: self.space.to_sys(),
            hinge_axis: self.hinge_axis.into(),
            slider_axis: self.slider_axis.into(),
            ratio: self.ratio,
        }
    }

    fn from_sys(r: &sys::JPC_RackAndPinionConstraintSettings) -> Self {
        Self {
            common: ConstraintCommon::from_sys(r.base),
            space: ConstraintSpace::from_sys(r.space),
            hinge_axis: r.hinge_axis.into(),
            slider_axis: r.slider_axis.into(),
            ratio: r.ratio,
        }
    }
}

impl PulleyConstraintSettings {
    /// A rope from `body_point1` over the fixed points to `body_point2` (world space).
    pub fn new(body_point1: Vec3, fixed_point1: Vec3, body_point2: Vec3, fixed_point2: Vec3) -> Self {
        Self { body_point1, fixed_point1, body_point2, fixed_point2, ..Default::default() }
    }

    fn to_sys(self) -> sys::JPC_PulleyConstraintSettings {
        sys::JPC_PulleyConstraintSettings {
            base: self.common.to_sys(),
            space: self.space.to_sys(),
            body_point1: self.body_point1.into(),
            fixed_point1: self.fixed_point1.into(),
            body_point2: self.body_point2.into(),
            fixed_point2: self.fixed_point2.into(),
            ratio: self.ratio,
            min_length: self.min_length,
            max_length: self.max_length,
        }
    }

    fn from_sys(r: &sys::JPC_PulleyConstraintSettings) -> Self {
        Self {
            common: ConstraintCommon::from_sys(r.base),
            space: ConstraintSpace::from_sys(r.space),
            body_point1: r.body_point1.into(),
            fixed_point1: r.fixed_point1.into(),
            body_point2: r.body_point2.into(),
            fixed_point2: r.fixed_point2.into(),
            ratio: r.ratio,
            min_length: r.min_length,
            max_length: r.max_length,
        }
    }
}

/// A smooth (Hermite) path that a [`PathConstraintSettings`] keeps a body on. Immutable and cheap to clone.
pub struct PathConstraintPath {
    raw: NonNull<sys::JPC_PathConstraintPath>,
}

// SAFETY: immutable after creation, atomic reference count.
unsafe impl Send for PathConstraintPath {}
unsafe impl Sync for PathConstraintPath {}

/// A control point of a [`PathConstraintPath`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PathPoint {
    pub position: Vec3,
    pub tangent: Vec3,
    /// Perpendicular to the tangent; orients the body when its rotation follows the path.
    pub normal: Vec3,
}

impl PathConstraintPath {
    /// A path through at least 2 points, in the space of body 1.
    pub fn hermite(points: &[PathPoint], looping: bool) -> crate::Result<Self> {
        crate::ensure_init();
        let raw_points: Vec<sys::JPC_PathPoint> = points
            .iter()
            .map(|p| sys::JPC_PathPoint { position: p.position.into(), tangent: p.tangent.into(), normal: p.normal.into() })
            .collect();
        let count = u32::try_from(raw_points.len()).map_err(|_| crate::Error::new("too many points"))?;
        let raw = unsafe { sys::JPC_PathConstraintPathHermite_Create(raw_points.as_ptr(), count, looping) };
        NonNull::new(raw).map(|raw| Self { raw }).ok_or_else(crate::Error::last)
    }

    /// Fractions along the path run from 0 to this value.
    pub fn max_fraction(&self) -> f32 {
        unsafe { sys::JPC_PathConstraintPath_GetMaxFraction(self.raw.as_ptr()) }
    }
}

impl Clone for PathConstraintPath {
    fn clone(&self) -> Self {
        unsafe { sys::JPC_PathConstraintPath_AddRef(self.raw.as_ptr()) };
        Self { raw: self.raw }
    }
}

impl Drop for PathConstraintPath {
    fn drop(&mut self) {
        unsafe { sys::JPC_PathConstraintPath_Release(self.raw.as_ptr()) }
    }
}

impl std::fmt::Debug for PathConstraintPath {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PathConstraintPath").field("max_fraction", &self.max_fraction()).finish()
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum PathRotationConstraint {
    /// Rotation is unconstrained.
    #[default]
    Free,
    ConstrainAroundTangent,
    ConstrainAroundNormal,
    ConstrainAroundBinormal,
    /// Rotation follows the path's tangent and normal.
    ConstrainToPath,
    /// Rotation is locked relative to body 1.
    FullyConstrained,
}

/// Keeps body 2 on a path attached to body 1 (or the world).
#[derive(Clone, Debug)]
pub struct PathConstraintSettings {
    pub common: ConstraintCommon,
    pub path: PathConstraintPath,
    /// Placement of the path relative to body 1.
    pub path_position: Vec3,
    pub path_rotation: Quat,
    /// Where on the path body 2 starts.
    pub path_fraction: f32,
    pub max_friction_force: f32,
    pub position_motor: MotorSettings,
    pub rotation_constraint: PathRotationConstraint,
}

impl PathConstraintSettings {
    pub fn new(path: PathConstraintPath) -> Self {
        let mut d = sys::JPC_PathConstraintSettings::default();
        unsafe { sys::JPC_PathConstraintSettings_SetDefault(&mut d) };
        Self {
            common: ConstraintCommon::from_sys(d.base),
            path,
            path_position: d.path_position.into(),
            path_rotation: d.path_rotation.into(),
            path_fraction: d.path_fraction,
            max_friction_force: d.max_friction_force,
            position_motor: MotorSettings::from_sys(d.position_motor_settings),
            rotation_constraint: PathRotationConstraint::Free,
        }
    }
}

impl private::Sealed for PathConstraintSettings {}

impl ConstraintSettings for PathConstraintSettings {
    fn kind(&self) -> ConstraintKind {
        ConstraintKind::Path
    }

    unsafe fn create(&self, system: *mut sys::JPC_PhysicsSystem, body1: u32, body2: u32) -> *mut sys::JPC_Constraint {
        let raw = sys::JPC_PathConstraintSettings {
            base: self.common.to_sys(),
            path: self.path.raw.as_ptr(),
            path_position: self.path_position.into(),
            path_rotation: self.path_rotation.into(),
            path_fraction: self.path_fraction,
            max_friction_force: self.max_friction_force,
            position_motor_settings: self.position_motor.to_sys(),
            rotation_constraint_type: self.rotation_constraint as sys::JPC_PathRotationConstraintType,
        };
        unsafe { sys::JPC_PhysicsSystem_CreatePathConstraint(system, &raw, body1, body2) }
    }
}

impl SwingTwistConstraintSettings {
    /// A shoulder-like joint at a world space point. The twist axis points along the limb; the plane axis is
    /// perpendicular to it and picks which way `plane_half_cone_angle` applies.
    pub fn at(point: Vec3, twist_axis: Vec3, plane_axis: Vec3) -> Self {
        Self {
            position1: point,
            twist_axis1: twist_axis,
            plane_axis1: plane_axis,
            position2: point,
            twist_axis2: twist_axis,
            plane_axis2: plane_axis,
            ..Default::default()
        }
    }

    pub(crate) fn to_sys(self) -> sys::JPC_SwingTwistConstraintSettings {
        sys::JPC_SwingTwistConstraintSettings {
            base: self.common.to_sys(),
            space: self.space.to_sys(),
            position1: self.position1.into(),
            twist_axis1: self.twist_axis1.into(),
            plane_axis1: self.plane_axis1.into(),
            position2: self.position2.into(),
            twist_axis2: self.twist_axis2.into(),
            plane_axis2: self.plane_axis2.into(),
            swing_type: self.swing_type.to_sys(),
            normal_half_cone_angle: self.normal_half_cone_angle,
            plane_half_cone_angle: self.plane_half_cone_angle,
            twist_min_angle: self.twist_min_angle,
            twist_max_angle: self.twist_max_angle,
            max_friction_torque: self.max_friction_torque,
            swing_motor_settings: self.swing_motor.to_sys(),
            twist_motor_settings: self.twist_motor.to_sys(),
        }
    }

    fn from_sys(r: &sys::JPC_SwingTwistConstraintSettings) -> Self {
        Self {
            common: ConstraintCommon::from_sys(r.base),
            space: ConstraintSpace::from_sys(r.space),
            position1: r.position1.into(),
            twist_axis1: r.twist_axis1.into(),
            plane_axis1: r.plane_axis1.into(),
            position2: r.position2.into(),
            twist_axis2: r.twist_axis2.into(),
            plane_axis2: r.plane_axis2.into(),
            swing_type: SwingType::from_sys(r.swing_type),
            normal_half_cone_angle: r.normal_half_cone_angle,
            plane_half_cone_angle: r.plane_half_cone_angle,
            twist_min_angle: r.twist_min_angle,
            twist_max_angle: r.twist_max_angle,
            max_friction_torque: r.max_friction_torque,
            swing_motor: MotorSettings::from_sys(r.swing_motor_settings),
            twist_motor: MotorSettings::from_sys(r.twist_motor_settings),
        }
    }
}

impl SixDofConstraintSettings {
    /// A constraint frame at a world space point; every axis starts free.
    pub fn at(point: Vec3, axis_x: Vec3, axis_y: Vec3) -> Self {
        Self { position1: point, axis_x1: axis_x, axis_y1: axis_y, position2: point, axis_x2: axis_x, axis_y2: axis_y, ..Default::default() }
    }

    pub fn make_free_axis(&mut self, axis: SixDofAxis) -> &mut Self {
        self.limit_min[axis as usize] = -f32::MAX;
        self.limit_max[axis as usize] = f32::MAX;
        self
    }

    pub fn make_fixed_axis(&mut self, axis: SixDofAxis) -> &mut Self {
        self.limit_min[axis as usize] = f32::MAX;
        self.limit_max[axis as usize] = -f32::MAX;
        self
    }

    pub fn set_limited_axis(&mut self, axis: SixDofAxis, min: f32, max: f32) -> &mut Self {
        self.limit_min[axis as usize] = min;
        self.limit_max[axis as usize] = max;
        self
    }

    pub(crate) fn to_sys(self) -> sys::JPC_SixDOFConstraintSettings {
        sys::JPC_SixDOFConstraintSettings {
            base: self.common.to_sys(),
            space: self.space.to_sys(),
            position1: self.position1.into(),
            axis_x1: self.axis_x1.into(),
            axis_y1: self.axis_y1.into(),
            position2: self.position2.into(),
            axis_x2: self.axis_x2.into(),
            axis_y2: self.axis_y2.into(),
            max_friction: self.max_friction,
            swing_type: self.swing_type.to_sys(),
            limit_min: self.limit_min,
            limit_max: self.limit_max,
            limits_spring_settings: self.limits_spring.map(SpringSettings::to_sys),
            motor_settings: self.motor.map(MotorSettings::to_sys),
        }
    }

    fn from_sys(r: &sys::JPC_SixDOFConstraintSettings) -> Self {
        Self {
            common: ConstraintCommon::from_sys(r.base),
            space: ConstraintSpace::from_sys(r.space),
            position1: r.position1.into(),
            axis_x1: r.axis_x1.into(),
            axis_y1: r.axis_y1.into(),
            position2: r.position2.into(),
            axis_x2: r.axis_x2.into(),
            axis_y2: r.axis_y2.into(),
            max_friction: r.max_friction,
            swing_type: SwingType::from_sys(r.swing_type),
            limit_min: r.limit_min,
            limit_max: r.limit_max,
            limits_spring: r.limits_spring_settings.map(SpringSettings::from_sys),
            motor: r.motor_settings.map(MotorSettings::from_sys),
        }
    }
}

impl FixedConstraintSettings {
    /// Welds two bodies together where they are now.
    pub fn auto_detect() -> Self {
        Self { auto_detect_point: true, ..Default::default() }
    }

    pub(crate) fn to_sys(self) -> sys::JPC_FixedConstraintSettings {
        sys::JPC_FixedConstraintSettings {
            base: self.common.to_sys(),
            space: self.space.to_sys(),
            auto_detect_point: self.auto_detect_point,
            point1: self.point1.into(),
            axis_x1: self.axis_x1.into(),
            axis_y1: self.axis_y1.into(),
            point2: self.point2.into(),
            axis_x2: self.axis_x2.into(),
            axis_y2: self.axis_y2.into(),
        }
    }

    fn from_sys(r: &sys::JPC_FixedConstraintSettings) -> Self {
        Self {
            common: ConstraintCommon::from_sys(r.base),
            space: ConstraintSpace::from_sys(r.space),
            auto_detect_point: r.auto_detect_point,
            point1: r.point1.into(),
            axis_x1: r.axis_x1.into(),
            axis_y1: r.axis_y1.into(),
            point2: r.point2.into(),
            axis_x2: r.axis_x2.into(),
            axis_y2: r.axis_y2.into(),
        }
    }
}

impl PointConstraintSettings {
    /// A ball joint at a world space point.
    pub fn at(point: Vec3) -> Self {
        Self { point1: point, point2: point, ..Default::default() }
    }

    pub(crate) fn to_sys(self) -> sys::JPC_PointConstraintSettings {
        sys::JPC_PointConstraintSettings {
            base: self.common.to_sys(),
            space: self.space.to_sys(),
            point1: self.point1.into(),
            point2: self.point2.into(),
        }
    }

    fn from_sys(r: &sys::JPC_PointConstraintSettings) -> Self {
        Self {
            common: ConstraintCommon::from_sys(r.base),
            space: ConstraintSpace::from_sys(r.space),
            point1: r.point1.into(),
            point2: r.point2.into(),
        }
    }
}

impl DistanceConstraintSettings {
    /// Keeps two world space points at their current distance.
    pub fn between(point1: Vec3, point2: Vec3) -> Self {
        Self { point1, point2, ..Default::default() }
    }

    pub(crate) fn to_sys(self) -> sys::JPC_DistanceConstraintSettings {
        sys::JPC_DistanceConstraintSettings {
            base: self.common.to_sys(),
            space: self.space.to_sys(),
            point1: self.point1.into(),
            point2: self.point2.into(),
            min_distance: self.min_distance,
            max_distance: self.max_distance,
            limits_spring_settings: self.limits_spring.to_sys(),
        }
    }

    fn from_sys(r: &sys::JPC_DistanceConstraintSettings) -> Self {
        Self {
            common: ConstraintCommon::from_sys(r.base),
            space: ConstraintSpace::from_sys(r.space),
            point1: r.point1.into(),
            point2: r.point2.into(),
            min_distance: r.min_distance,
            max_distance: r.max_distance,
            limits_spring: SpringSettings::from_sys(r.limits_spring_settings),
        }
    }
}

impl HingeConstraintSettings {
    /// A hinge at a world space point. `normal_axis` must be perpendicular to `hinge_axis`; angles are measured from it.
    pub fn at(point: Vec3, hinge_axis: Vec3, normal_axis: Vec3) -> Self {
        Self {
            point1: point,
            hinge_axis1: hinge_axis,
            normal_axis1: normal_axis,
            point2: point,
            hinge_axis2: hinge_axis,
            normal_axis2: normal_axis,
            ..Default::default()
        }
    }

    pub(crate) fn to_sys(self) -> sys::JPC_HingeConstraintSettings {
        sys::JPC_HingeConstraintSettings {
            base: self.common.to_sys(),
            space: self.space.to_sys(),
            point1: self.point1.into(),
            hinge_axis1: self.hinge_axis1.into(),
            normal_axis1: self.normal_axis1.into(),
            point2: self.point2.into(),
            hinge_axis2: self.hinge_axis2.into(),
            normal_axis2: self.normal_axis2.into(),
            limits_min: self.limits_min,
            limits_max: self.limits_max,
            limits_spring_settings: self.limits_spring.to_sys(),
            max_friction_torque: self.max_friction_torque,
            motor_settings: self.motor.to_sys(),
        }
    }

    fn from_sys(r: &sys::JPC_HingeConstraintSettings) -> Self {
        Self {
            common: ConstraintCommon::from_sys(r.base),
            space: ConstraintSpace::from_sys(r.space),
            point1: r.point1.into(),
            hinge_axis1: r.hinge_axis1.into(),
            normal_axis1: r.normal_axis1.into(),
            point2: r.point2.into(),
            hinge_axis2: r.hinge_axis2.into(),
            normal_axis2: r.normal_axis2.into(),
            limits_min: r.limits_min,
            limits_max: r.limits_max,
            limits_spring: SpringSettings::from_sys(r.limits_spring_settings),
            max_friction_torque: r.max_friction_torque,
            motor: MotorSettings::from_sys(r.motor_settings),
        }
    }
}

impl SliderConstraintSettings {
    /// A slider through a world space point. `normal_axis` must be perpendicular to `slider_axis`.
    pub fn at(point: Vec3, slider_axis: Vec3, normal_axis: Vec3) -> Self {
        Self {
            point1: point,
            slider_axis1: slider_axis,
            normal_axis1: normal_axis,
            point2: point,
            slider_axis2: slider_axis,
            normal_axis2: normal_axis,
            ..Default::default()
        }
    }

    pub(crate) fn to_sys(self) -> sys::JPC_SliderConstraintSettings {
        sys::JPC_SliderConstraintSettings {
            base: self.common.to_sys(),
            space: self.space.to_sys(),
            auto_detect_point: self.auto_detect_point,
            point1: self.point1.into(),
            slider_axis1: self.slider_axis1.into(),
            normal_axis1: self.normal_axis1.into(),
            point2: self.point2.into(),
            slider_axis2: self.slider_axis2.into(),
            normal_axis2: self.normal_axis2.into(),
            limits_min: self.limits_min,
            limits_max: self.limits_max,
            limits_spring_settings: self.limits_spring.to_sys(),
            max_friction_force: self.max_friction_force,
            motor_settings: self.motor.to_sys(),
        }
    }

    fn from_sys(r: &sys::JPC_SliderConstraintSettings) -> Self {
        Self {
            common: ConstraintCommon::from_sys(r.base),
            space: ConstraintSpace::from_sys(r.space),
            auto_detect_point: r.auto_detect_point,
            point1: r.point1.into(),
            slider_axis1: r.slider_axis1.into(),
            normal_axis1: r.normal_axis1.into(),
            point2: r.point2.into(),
            slider_axis2: r.slider_axis2.into(),
            normal_axis2: r.normal_axis2.into(),
            limits_min: r.limits_min,
            limits_max: r.limits_max,
            limits_spring: SpringSettings::from_sys(r.limits_spring_settings),
            max_friction_force: r.max_friction_force,
            motor: MotorSettings::from_sys(r.motor_settings),
        }
    }
}

impl ConeConstraintSettings {
    /// A ball joint at a world space point whose twist axis may deviate at most `half_cone_angle` radians.
    pub fn at(point: Vec3, twist_axis: Vec3, half_cone_angle: f32) -> Self {
        Self { point1: point, twist_axis1: twist_axis, point2: point, twist_axis2: twist_axis, half_cone_angle, ..Default::default() }
    }

    pub(crate) fn to_sys(self) -> sys::JPC_ConeConstraintSettings {
        sys::JPC_ConeConstraintSettings {
            base: self.common.to_sys(),
            space: self.space.to_sys(),
            point1: self.point1.into(),
            twist_axis1: self.twist_axis1.into(),
            point2: self.point2.into(),
            twist_axis2: self.twist_axis2.into(),
            half_cone_angle: self.half_cone_angle,
        }
    }

    fn from_sys(r: &sys::JPC_ConeConstraintSettings) -> Self {
        Self {
            common: ConstraintCommon::from_sys(r.base),
            space: ConstraintSpace::from_sys(r.space),
            point1: r.point1.into(),
            twist_axis1: r.twist_axis1.into(),
            point2: r.point2.into(),
            twist_axis2: r.twist_axis2.into(),
            half_cone_angle: r.half_cone_angle,
        }
    }
}

pub(crate) struct ConstraintEntry {
    pub(crate) raw: NonNull<sys::JPC_Constraint>,
    pub(crate) kind: ConstraintKind,
    pub(crate) body1: BodyId,
    pub(crate) body2: BodyId,
}

// SAFETY: entries are only touched through the owning system, under its mutex or with `&mut` access.
unsafe impl Send for ConstraintEntry {}

/// Constraints created through a system. Each entry holds one reference and is added to the system.
#[derive(Default)]
pub(crate) struct ConstraintRegistry {
    next_id: u64,
    pub(crate) entries: HashMap<ConstraintId, ConstraintEntry>,
}

impl ConstraintRegistry {
    pub(crate) fn insert(&mut self, entry: ConstraintEntry) -> ConstraintId {
        let id = ConstraintId(self.next_id);
        self.next_id += 1;
        self.entries.insert(id, entry);
        id
    }

    /// Removes `entry` from the system and drops our reference.
    ///
    /// # Safety
    /// `system` must be the live system the constraint was added to.
    pub(crate) unsafe fn release(system: *mut sys::JPC_PhysicsSystem, entry: ConstraintEntry) {
        unsafe {
            sys::JPC_PhysicsSystem_RemoveConstraint(system, entry.raw.as_ptr());
            sys::JPC_Constraint_Release(entry.raw.as_ptr());
        }
    }
}

/// Mutable access to a constraint, obtained from [`PhysicsSystem::constraint`](crate::PhysicsSystem::constraint).
/// Setters wake the constrained bodies so the change takes effect.
pub struct ConstraintMut<'a> {
    raw: NonNull<sys::JPC_Constraint>,
    kind: ConstraintKind,
    body_interface: *mut sys::JPC_BodyInterface,
    _system: PhantomData<&'a mut ()>,
}

impl<'a> ConstraintMut<'a> {
    pub(crate) fn new(entry: &ConstraintEntry, body_interface: *mut sys::JPC_BodyInterface) -> Self {
        Self { raw: entry.raw, kind: entry.kind, body_interface, _system: PhantomData }
    }

    /// For constraints owned by something else (ragdolls). `None` for kinds this crate does not model.
    pub(crate) fn from_raw(raw: NonNull<sys::JPC_Constraint>, body_interface: *mut sys::JPC_BodyInterface) -> Option<Self> {
        let kind = match unsafe { sys::JPC_Constraint_GetType(raw.as_ptr()) } {
            sys::JPC_CONSTRAINT_TYPE_FIXED => ConstraintKind::Fixed,
            sys::JPC_CONSTRAINT_TYPE_POINT => ConstraintKind::Point,
            sys::JPC_CONSTRAINT_TYPE_DISTANCE => ConstraintKind::Distance,
            sys::JPC_CONSTRAINT_TYPE_HINGE => ConstraintKind::Hinge,
            sys::JPC_CONSTRAINT_TYPE_SLIDER => ConstraintKind::Slider,
            sys::JPC_CONSTRAINT_TYPE_CONE => ConstraintKind::Cone,
            sys::JPC_CONSTRAINT_TYPE_SWING_TWIST => ConstraintKind::SwingTwist,
            sys::JPC_CONSTRAINT_TYPE_SIX_DOF => ConstraintKind::SixDof,
            _ => return None,
        };
        Some(Self { raw, kind, body_interface, _system: PhantomData })
    }

    fn ptr(&self) -> *mut sys::JPC_Constraint {
        self.raw.as_ptr()
    }

    fn wake(&self) {
        unsafe { sys::JPC_BodyInterface_ActivateConstraint(self.body_interface, self.ptr()) }
    }

    pub fn kind(&self) -> ConstraintKind {
        self.kind
    }

    /// The constrained bodies; `None` stands for the static world.
    pub fn bodies(&self) -> (Option<BodyId>, Option<BodyId>) {
        unsafe {
            (BodyId::from_sys(sys::JPC_Constraint_GetBody1(self.ptr())), BodyId::from_sys(sys::JPC_Constraint_GetBody2(self.ptr())))
        }
    }

    pub fn is_enabled(&self) -> bool {
        unsafe { sys::JPC_Constraint_GetEnabled(self.ptr()) }
    }

    pub fn set_enabled(&mut self, enabled: bool) {
        unsafe { sys::JPC_Constraint_SetEnabled(self.ptr(), enabled) };
        self.wake();
    }

    pub fn user_data(&self) -> u64 {
        unsafe { sys::JPC_Constraint_GetUserData(self.ptr()) }
    }

    pub fn set_user_data(&mut self, user_data: u64) {
        unsafe { sys::JPC_Constraint_SetUserData(self.ptr(), user_data) }
    }

    /// Current hinge angle in radians, if this is a hinge.
    pub fn hinge_angle(&self) -> Option<f32> {
        (self.kind == ConstraintKind::Hinge).then(|| unsafe { sys::JPC_HingeConstraint_GetCurrentAngle(self.ptr()) })
    }

    /// Current slider position, if this is a slider.
    pub fn slider_position(&self) -> Option<f32> {
        (self.kind == ConstraintKind::Slider).then(|| unsafe { sys::JPC_SliderConstraint_GetCurrentPosition(self.ptr()) })
    }

    /// Sets hinge angle limits (radians) or slider position limits. Returns false for other kinds.
    pub fn set_limits(&mut self, min: f32, max: f32) -> bool {
        match self.kind {
            ConstraintKind::Hinge => unsafe { sys::JPC_HingeConstraint_SetLimits(self.ptr(), min, max) },
            ConstraintKind::Slider => unsafe { sys::JPC_SliderConstraint_SetLimits(self.ptr(), min, max) },
            _ => return false,
        }
        self.wake();
        true
    }

    /// Hinge or swing-twist friction torque, or slider friction force. Returns false for other kinds.
    pub fn set_max_friction(&mut self, value: f32) -> bool {
        match self.kind {
            ConstraintKind::Hinge => unsafe { sys::JPC_HingeConstraint_SetMaxFrictionTorque(self.ptr(), value) },
            ConstraintKind::Slider => unsafe { sys::JPC_SliderConstraint_SetMaxFrictionForce(self.ptr(), value) },
            ConstraintKind::SwingTwist => unsafe { sys::JPC_SwingTwistConstraint_SetMaxFrictionTorque(self.ptr(), value) },
            _ => return false,
        }
        true
    }

    /// Hinge and slider only. Returns false for other kinds.
    pub fn set_motor_state(&mut self, state: MotorState) -> bool {
        match self.kind {
            ConstraintKind::Hinge => unsafe { sys::JPC_HingeConstraint_SetMotorState(self.ptr(), state.to_sys()) },
            ConstraintKind::Slider => unsafe { sys::JPC_SliderConstraint_SetMotorState(self.ptr(), state.to_sys()) },
            _ => return false,
        }
        self.wake();
        true
    }

    /// Target angular velocity (rad/s) for a hinge motor or linear velocity for a slider motor.
    pub fn set_motor_target_velocity(&mut self, velocity: f32) -> bool {
        match self.kind {
            ConstraintKind::Hinge => unsafe { sys::JPC_HingeConstraint_SetTargetAngularVelocity(self.ptr(), velocity) },
            ConstraintKind::Slider => unsafe { sys::JPC_SliderConstraint_SetTargetVelocity(self.ptr(), velocity) },
            _ => return false,
        }
        self.wake();
        true
    }

    /// Target angle (radians) for a hinge motor or position for a slider motor.
    pub fn set_motor_target_position(&mut self, position: f32) -> bool {
        match self.kind {
            ConstraintKind::Hinge => unsafe { sys::JPC_HingeConstraint_SetTargetAngle(self.ptr(), position) },
            ConstraintKind::Slider => unsafe { sys::JPC_SliderConstraint_SetTargetPosition(self.ptr(), position) },
            _ => return false,
        }
        self.wake();
        true
    }

    /// Distance constraints only. Returns false for other kinds.
    pub fn set_distance(&mut self, min: f32, max: f32) -> bool {
        if self.kind != ConstraintKind::Distance || min > max {
            return false;
        }
        unsafe { sys::JPC_DistanceConstraint_SetDistance(self.ptr(), min, max) };
        self.wake();
        true
    }

    /// Swing-twist limits in radians. Returns false for other kinds.
    pub fn set_swing_twist_limits(&mut self, normal_half_cone_angle: f32, plane_half_cone_angle: f32, twist_min: f32, twist_max: f32) -> bool {
        if self.kind != ConstraintKind::SwingTwist || twist_min > twist_max {
            return false;
        }
        unsafe {
            sys::JPC_SwingTwistConstraint_SetNormalHalfConeAngle(self.ptr(), normal_half_cone_angle);
            sys::JPC_SwingTwistConstraint_SetPlaneHalfConeAngle(self.ptr(), plane_half_cone_angle);
            sys::JPC_SwingTwistConstraint_SetTwistMinAngle(self.ptr(), twist_min);
            sys::JPC_SwingTwistConstraint_SetTwistMaxAngle(self.ptr(), twist_max);
        }
        self.wake();
        true
    }

    /// Swing-twist only. Returns false for other kinds.
    pub fn set_swing_twist_motor_states(&mut self, swing: MotorState, twist: MotorState) -> bool {
        if self.kind != ConstraintKind::SwingTwist {
            return false;
        }
        unsafe {
            sys::JPC_SwingTwistConstraint_SetSwingMotorState(self.ptr(), swing.to_sys());
            sys::JPC_SwingTwistConstraint_SetTwistMotorState(self.ptr(), twist.to_sys());
        }
        self.wake();
        true
    }

    /// Motor target orientation of body 2 relative to body 1's constraint frame. Swing-twist and six DOF only.
    pub fn set_target_orientation_cs(&mut self, orientation: Quat) -> bool {
        match self.kind {
            ConstraintKind::SwingTwist => unsafe { sys::JPC_SwingTwistConstraint_SetTargetOrientationCS(self.ptr(), orientation.into()) },
            ConstraintKind::SixDof => unsafe { sys::JPC_SixDOFConstraint_SetTargetOrientationCS(self.ptr(), orientation.into()) },
            _ => return false,
        }
        self.wake();
        true
    }

    /// Motor target angular velocity in constraint space. Swing-twist and six DOF only.
    pub fn set_target_angular_velocity_cs(&mut self, angular_velocity: Vec3) -> bool {
        match self.kind {
            ConstraintKind::SwingTwist => unsafe { sys::JPC_SwingTwistConstraint_SetTargetAngularVelocityCS(self.ptr(), angular_velocity.into()) },
            ConstraintKind::SixDof => unsafe { sys::JPC_SixDOFConstraint_SetTargetAngularVelocityCS(self.ptr(), angular_velocity.into()) },
            _ => return false,
        }
        self.wake();
        true
    }

    /// Current rotation of body 2 relative to body 1's constraint frame. Swing-twist and six DOF only.
    pub fn rotation_in_constraint_space(&self) -> Option<Quat> {
        match self.kind {
            ConstraintKind::SwingTwist => Some(unsafe { sys::JPC_SwingTwistConstraint_GetRotationInConstraintSpace(self.ptr()) }.into()),
            ConstraintKind::SixDof => Some(unsafe { sys::JPC_SixDOFConstraint_GetRotationInConstraintSpace(self.ptr()) }.into()),
            _ => None,
        }
    }

    /// Six DOF only. Returns false for other kinds.
    pub fn set_six_dof_translation_limits(&mut self, min: Vec3, max: Vec3) -> bool {
        if self.kind != ConstraintKind::SixDof {
            return false;
        }
        unsafe { sys::JPC_SixDOFConstraint_SetTranslationLimits(self.ptr(), min.into(), max.into()) };
        self.wake();
        true
    }

    /// Six DOF only; angles in radians. Returns false for other kinds.
    pub fn set_six_dof_rotation_limits(&mut self, min: Vec3, max: Vec3) -> bool {
        if self.kind != ConstraintKind::SixDof {
            return false;
        }
        unsafe { sys::JPC_SixDOFConstraint_SetRotationLimits(self.ptr(), min.into(), max.into()) };
        self.wake();
        true
    }

    /// Six DOF only. Returns false for other kinds.
    pub fn set_six_dof_max_friction(&mut self, axis: SixDofAxis, friction: f32) -> bool {
        if self.kind != ConstraintKind::SixDof {
            return false;
        }
        unsafe { sys::JPC_SixDOFConstraint_SetMaxFriction(self.ptr(), axis as sys::JPC_SixDOFAxis, friction) };
        true
    }

    /// Six DOF only. Returns false for other kinds.
    pub fn set_six_dof_motor_state(&mut self, axis: SixDofAxis, state: MotorState) -> bool {
        if self.kind != ConstraintKind::SixDof {
            return false;
        }
        unsafe { sys::JPC_SixDOFConstraint_SetMotorState(self.ptr(), axis as sys::JPC_SixDOFAxis, state.to_sys()) };
        self.wake();
        true
    }

    /// Six DOF only. Returns false for other kinds.
    pub fn set_six_dof_target_velocity_cs(&mut self, velocity: Vec3) -> bool {
        if self.kind != ConstraintKind::SixDof {
            return false;
        }
        unsafe { sys::JPC_SixDOFConstraint_SetTargetVelocityCS(self.ptr(), velocity.into()) };
        self.wake();
        true
    }

    /// Six DOF only. Returns false for other kinds.
    pub fn set_six_dof_target_position_cs(&mut self, position: Vec3) -> bool {
        if self.kind != ConstraintKind::SixDof {
            return false;
        }
        unsafe { sys::JPC_SixDOFConstraint_SetTargetPositionCS(self.ptr(), position.into()) };
        self.wake();
        true
    }

    /// Current rope length (side 1 + ratio * side 2), if this is a pulley.
    pub fn pulley_length(&self) -> Option<f32> {
        (self.kind == ConstraintKind::Pulley).then(|| unsafe { sys::JPC_PulleyConstraint_GetCurrentLength(self.ptr()) })
    }

    /// Pulleys only; needs `0 <= min <= max`. Returns false otherwise.
    pub fn set_pulley_length(&mut self, min: f32, max: f32) -> bool {
        if self.kind != ConstraintKind::Pulley || min < 0.0 || min > max {
            return false;
        }
        unsafe { sys::JPC_PulleyConstraint_SetLength(self.ptr(), min, max) };
        self.wake();
        true
    }

    /// Where body 2 is along the path, if this is a path constraint.
    pub fn path_fraction(&self) -> Option<f32> {
        (self.kind == ConstraintKind::Path).then(|| unsafe { sys::JPC_PathConstraint_GetPathFraction(self.ptr()) })
    }

    /// Path constraints only. Returns false for other kinds.
    pub fn set_path_motor_state(&mut self, state: MotorState) -> bool {
        if self.kind != ConstraintKind::Path {
            return false;
        }
        unsafe { sys::JPC_PathConstraint_SetPositionMotorState(self.ptr(), state.to_sys()) };
        self.wake();
        true
    }

    /// Target speed along the path (velocity motor). Path constraints only.
    pub fn set_path_target_velocity(&mut self, velocity: f32) -> bool {
        if self.kind != ConstraintKind::Path {
            return false;
        }
        unsafe { sys::JPC_PathConstraint_SetTargetVelocity(self.ptr(), velocity) };
        self.wake();
        true
    }

    /// Target fraction along the path (position motor), clamped unless the path loops. Path constraints only.
    pub fn set_path_target_fraction(&mut self, fraction: f32) -> bool {
        if self.kind != ConstraintKind::Path {
            return false;
        }
        unsafe { sys::JPC_PathConstraint_SetTargetPathFraction(self.ptr(), fraction) };
        self.wake();
        true
    }

    /// Path constraints only. Returns false for other kinds.
    pub fn set_path_max_friction_force(&mut self, force: f32) -> bool {
        if self.kind != ConstraintKind::Path {
            return false;
        }
        unsafe { sys::JPC_PathConstraint_SetMaxFrictionForce(self.ptr(), force) };
        true
    }

    /// Cone constraints only; the angle must be in `0..=PI`. Returns false otherwise.
    pub fn set_half_cone_angle(&mut self, angle: f32) -> bool {
        if self.kind != ConstraintKind::Cone || !(0.0..=std::f32::consts::PI).contains(&angle) {
            return false;
        }
        unsafe { sys::JPC_ConeConstraint_SetHalfConeAngle(self.ptr(), angle) };
        self.wake();
        true
    }
}
