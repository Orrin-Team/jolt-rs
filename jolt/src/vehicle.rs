//! Wheeled vehicles: a dynamic body with suspension, tires, an engine, a transmission and differentials.
//!
//! ```no_run
//! # use jolt::*;
//! # fn demo(system: &mut PhysicsSystem, car_body: BodyId, layer: ObjectLayer) -> Result<()> {
//! let mut settings = WheeledVehicleSettings::default();
//! for (x, z) in [(-0.9, 1.2), (0.9, 1.2), (-0.9, -1.2), (0.9, -1.2)] {
//!     settings.wheels.push(WheelSettings::at(Vec3::new(x, -0.3, z)));
//! }
//! settings.wheels[2].max_steer_angle = 0.0; // rear wheels don't steer
//! settings.wheels[3].max_steer_angle = 0.0;
//! settings.differentials.push(VehicleDifferentialSettings::new(Some(0), Some(1))); // front wheel drive
//!
//! let tester = VehicleCollisionTester::ray(system, layer, Vec3::Y, 80f32.to_radians())?;
//! let car = system.add_vehicle(&settings, car_body, &tester)?;
//! system.vehicle(car).unwrap().set_driver_input(1.0, 0.0, 0.0, 0.0);
//! # Ok(()) }
//! ```

use std::marker::PhantomData;
use std::ptr::NonNull;

use jolt_sys as sys;

use crate::constraint::{ConstraintCommon, ConstraintKind, ConstraintSettings, SpringSettings, private};
use crate::{BodyId, ConstraintId, Error, Mat44, ObjectLayer, PhysicsSystem, Result, SubShapeId, Vec3};

/// How wheels find the ground. Reference counted; one tester can be shared by many vehicles.
pub struct VehicleCollisionTester {
    raw: NonNull<sys::JPC_VehicleCollisionTester>,
}

// SAFETY: testers are immutable after creation and atomically reference counted.
unsafe impl Send for VehicleCollisionTester {}
unsafe impl Sync for VehicleCollisionTester {}

impl VehicleCollisionTester {
    fn wrap(raw: *mut sys::JPC_VehicleCollisionTester) -> Result<Self> {
        NonNull::new(raw).map(|raw| Self { raw }).ok_or_else(Error::last)
    }

    /// A ray per wheel; cheapest. Ground steeper than `max_slope_angle` (radians) is ignored.
    /// `layer` is the layer the wheels collide as.
    pub fn ray(system: &PhysicsSystem, layer: ObjectLayer, up: Vec3, max_slope_angle: f32) -> Result<Self> {
        Self::wrap(unsafe { sys::JPC_VehicleCollisionTesterRay_Create(system.raw(), layer.0, up.into(), max_slope_angle) })
    }

    /// A swept sphere per wheel; smoother over small bumps.
    pub fn cast_sphere(system: &PhysicsSystem, layer: ObjectLayer, radius: f32, up: Vec3, max_slope_angle: f32) -> Result<Self> {
        Self::wrap(unsafe { sys::JPC_VehicleCollisionTesterCastSphere_Create(system.raw(), layer.0, radius, up.into(), max_slope_angle) })
    }

    /// A swept cylinder matching each wheel's radius and width; most accurate.
    pub fn cast_cylinder(system: &PhysicsSystem, layer: ObjectLayer, convex_radius_fraction: f32) -> Result<Self> {
        Self::wrap(unsafe { sys::JPC_VehicleCollisionTesterCastCylinder_Create(system.raw(), layer.0, convex_radius_fraction) })
    }

    fn as_raw(&self) -> *const sys::JPC_VehicleCollisionTester {
        self.raw.as_ptr()
    }
}

impl Clone for VehicleCollisionTester {
    fn clone(&self) -> Self {
        unsafe { sys::JPC_VehicleCollisionTester_AddRef(self.as_raw()) };
        Self { raw: self.raw }
    }
}

impl Drop for VehicleCollisionTester {
    fn drop(&mut self) {
        unsafe { sys::JPC_VehicleCollisionTester_Release(self.as_raw()) }
    }
}

impl std::fmt::Debug for VehicleCollisionTester {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("VehicleCollisionTester").finish_non_exhaustive()
    }
}

/// One wheel. Positions and directions are in the vehicle body's local space.
#[derive(Clone, Debug, PartialEq)]
pub struct WheelSettings {
    /// Attachment point of the suspension (at its most compressed).
    pub position: Vec3,
    pub suspension_force_point: Vec3,
    pub suspension_direction: Vec3,
    pub steering_axis: Vec3,
    pub wheel_up: Vec3,
    pub wheel_forward: Vec3,
    pub suspension_min_length: f32,
    pub suspension_max_length: f32,
    pub suspension_preload_length: f32,
    pub suspension_spring: SpringSettings,
    pub radius: f32,
    pub width: f32,
    pub enable_suspension_force_point: bool,
    pub inertia: f32,
    pub angular_damping: f32,
    /// Radians; 0 for wheels that don't steer.
    pub max_steer_angle: f32,
    /// Friction against slip ratio as `(slip, friction)` points; `None` keeps Jolt's default curve.
    pub longitudinal_friction: Option<Vec<(f32, f32)>>,
    /// Friction against slip angle in degrees; `None` keeps Jolt's default curve.
    pub lateral_friction: Option<Vec<(f32, f32)>>,
    pub max_brake_torque: f32,
    pub max_hand_brake_torque: f32,
}

impl Default for WheelSettings {
    fn default() -> Self {
        let mut d = sys::JPC_WheelSettingsWV::default();
        unsafe { sys::JPC_WheelSettingsWV_SetDefault(&mut d) };
        Self {
            position: d.position.into(),
            suspension_force_point: d.suspension_force_point.into(),
            suspension_direction: d.suspension_direction.into(),
            steering_axis: d.steering_axis.into(),
            wheel_up: d.wheel_up.into(),
            wheel_forward: d.wheel_forward.into(),
            suspension_min_length: d.suspension_min_length,
            suspension_max_length: d.suspension_max_length,
            suspension_preload_length: d.suspension_preload_length,
            suspension_spring: SpringSettings::from_sys(d.suspension_spring),
            radius: d.radius,
            width: d.width,
            enable_suspension_force_point: d.enable_suspension_force_point,
            inertia: d.inertia,
            angular_damping: d.angular_damping,
            max_steer_angle: d.max_steer_angle,
            longitudinal_friction: None,
            lateral_friction: None,
            max_brake_torque: d.max_brake_torque,
            max_hand_brake_torque: d.max_hand_brake_torque,
        }
    }
}

impl WheelSettings {
    pub fn at(position: Vec3) -> Self {
        Self { position, ..Default::default() }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct VehicleEngineSettings {
    /// N m.
    pub max_torque: f32,
    pub min_rpm: f32,
    pub max_rpm: f32,
    /// Torque fraction against RPM fraction; `None` keeps Jolt's default curve.
    pub normalized_torque: Option<Vec<(f32, f32)>>,
    pub inertia: f32,
    pub angular_damping: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum TransmissionMode {
    #[default]
    Auto,
    /// Gears change only through [`VehicleMut::set_transmission`].
    Manual,
}

#[derive(Clone, Debug, PartialEq)]
pub struct VehicleTransmissionSettings {
    pub mode: TransmissionMode,
    /// At least one.
    pub gear_ratios: Vec<f32>,
    /// At least one; negative.
    pub reverse_gear_ratios: Vec<f32>,
    pub switch_time: f32,
    pub clutch_release_time: f32,
    pub switch_latency: f32,
    pub shift_up_rpm: f32,
    pub shift_down_rpm: f32,
    pub clutch_strength: f32,
}

/// Splits engine torque between two wheels.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VehicleDifferentialSettings {
    pub left_wheel: Option<u32>,
    pub right_wheel: Option<u32>,
    pub differential_ratio: f32,
    /// 0 sends everything left, 1 everything right.
    pub left_right_split: f32,
    pub limited_slip_ratio: f32,
    /// Share of the engine torque this differential gets.
    pub engine_torque_ratio: f32,
}

impl Default for VehicleDifferentialSettings {
    fn default() -> Self {
        let mut d = sys::JPC_VehicleDifferentialSettings::default();
        unsafe { sys::JPC_VehicleDifferentialSettings_SetDefault(&mut d) };
        Self {
            left_wheel: u32::try_from(d.left_wheel).ok(),
            right_wheel: u32::try_from(d.right_wheel).ok(),
            differential_ratio: d.differential_ratio,
            left_right_split: d.left_right_split,
            limited_slip_ratio: d.limited_slip_ratio,
            engine_torque_ratio: d.engine_torque_ratio,
        }
    }
}

impl VehicleDifferentialSettings {
    pub fn new(left_wheel: Option<u32>, right_wheel: Option<u32>) -> Self {
        Self { left_wheel, right_wheel, ..Default::default() }
    }
}

/// Resists body roll by coupling the suspension of two wheels.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AntiRollBar {
    pub left_wheel: u32,
    pub right_wheel: u32,
    pub stiffness: f32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct WheeledVehicleSettings {
    pub common: ConstraintCommon,
    /// In the body's local space.
    pub up: Vec3,
    pub forward: Vec3,
    /// The vehicle is kept from tipping further than this (radians).
    pub max_pitch_roll_angle: f32,
    pub wheels: Vec<WheelSettings>,
    pub anti_roll_bars: Vec<AntiRollBar>,
    pub engine: VehicleEngineSettings,
    pub transmission: VehicleTransmissionSettings,
    /// Without any, the engine drives nothing.
    pub differentials: Vec<VehicleDifferentialSettings>,
    pub differential_limited_slip_ratio: f32,
}

impl Default for WheeledVehicleSettings {
    fn default() -> Self {
        let mut d = sys::JPC_WheeledVehicleSettings::default();
        unsafe { sys::JPC_WheeledVehicleSettings_SetDefault(&mut d) };
        let t = &d.transmission;
        // SAFETY: the defaults point at static arrays owned by joltc.
        let (gears, reverse) = unsafe {
            (
                std::slice::from_raw_parts(t.gear_ratios, t.num_gear_ratios as usize).to_vec(),
                std::slice::from_raw_parts(t.reverse_gear_ratios, t.num_reverse_gear_ratios as usize).to_vec(),
            )
        };
        Self {
            common: ConstraintCommon::from_sys(d.base),
            up: d.up.into(),
            forward: d.forward.into(),
            max_pitch_roll_angle: d.max_pitch_roll_angle,
            wheels: Vec::new(),
            anti_roll_bars: Vec::new(),
            engine: VehicleEngineSettings {
                max_torque: d.engine.max_torque,
                min_rpm: d.engine.min_rpm,
                max_rpm: d.engine.max_rpm,
                normalized_torque: None,
                inertia: d.engine.inertia,
                angular_damping: d.engine.angular_damping,
            },
            transmission: VehicleTransmissionSettings {
                mode: if t.mode == sys::JPC_TRANSMISSION_MODE_MANUAL { TransmissionMode::Manual } else { TransmissionMode::Auto },
                gear_ratios: gears,
                reverse_gear_ratios: reverse,
                switch_time: t.switch_time,
                clutch_release_time: t.clutch_release_time,
                switch_latency: t.switch_latency,
                shift_up_rpm: t.shift_up_rpm,
                shift_down_rpm: t.shift_down_rpm,
                clutch_strength: t.clutch_strength,
            },
            differentials: Vec::new(),
            differential_limited_slip_ratio: d.differential_limited_slip_ratio,
        }
    }
}

fn curve(points: &Option<Vec<(f32, f32)>>) -> Option<Vec<sys::JPC_LinearCurvePoint>> {
    points.as_ref().map(|p| p.iter().map(|&(x, y)| sys::JPC_LinearCurvePoint { x, y }).collect())
}

fn curve_ptr(points: &Option<Vec<sys::JPC_LinearCurvePoint>>) -> (*const sys::JPC_LinearCurvePoint, u32) {
    match points {
        // An empty curve still has to replace the default, so it needs a non-null pointer.
        Some(p) => (if p.is_empty() { NonNull::dangling().as_ptr() } else { p.as_ptr() }, p.len() as u32),
        None => (std::ptr::null(), 0),
    }
}

fn wheel_index(index: Option<u32>) -> i32 {
    index.map_or(-1, |i| i32::try_from(i).unwrap_or(i32::MAX))
}

/// Settings plus tester, so vehicle creation can go through the system's constraint registry.
struct VehicleCreation<'a> {
    settings: &'a WheeledVehicleSettings,
    tester: &'a VehicleCollisionTester,
}

impl private::Sealed for VehicleCreation<'_> {}

impl ConstraintSettings for VehicleCreation<'_> {
    fn kind(&self) -> ConstraintKind {
        ConstraintKind::Vehicle
    }

    unsafe fn create(&self, system: *mut sys::JPC_PhysicsSystem, body1: u32, body2: u32) -> *mut sys::JPC_Constraint {
        debug_assert_eq!(body1, sys::JPC_BODY_ID_INVALID);
        let s = self.settings;
        let curves: Vec<_> = s.wheels.iter().map(|w| (curve(&w.longitudinal_friction), curve(&w.lateral_friction))).collect();
        let wheels: Vec<sys::JPC_WheelSettingsWV> = s
            .wheels
            .iter()
            .zip(&curves)
            .map(|(w, (long, lat))| {
                let (long_ptr, long_len) = curve_ptr(long);
                let (lat_ptr, lat_len) = curve_ptr(lat);
                sys::JPC_WheelSettingsWV {
                    position: w.position.into(),
                    suspension_force_point: w.suspension_force_point.into(),
                    suspension_direction: w.suspension_direction.into(),
                    steering_axis: w.steering_axis.into(),
                    wheel_up: w.wheel_up.into(),
                    wheel_forward: w.wheel_forward.into(),
                    suspension_min_length: w.suspension_min_length,
                    suspension_max_length: w.suspension_max_length,
                    suspension_preload_length: w.suspension_preload_length,
                    suspension_spring: w.suspension_spring.to_sys(),
                    radius: w.radius,
                    width: w.width,
                    enable_suspension_force_point: w.enable_suspension_force_point,
                    inertia: w.inertia,
                    angular_damping: w.angular_damping,
                    max_steer_angle: w.max_steer_angle,
                    longitudinal_friction: long_ptr,
                    num_longitudinal_friction: long_len,
                    lateral_friction: lat_ptr,
                    num_lateral_friction: lat_len,
                    max_brake_torque: w.max_brake_torque,
                    max_hand_brake_torque: w.max_hand_brake_torque,
                }
            })
            .collect();
        let bars: Vec<sys::JPC_VehicleAntiRollBar> = s
            .anti_roll_bars
            .iter()
            .map(|b| sys::JPC_VehicleAntiRollBar {
                left_wheel: wheel_index(Some(b.left_wheel)),
                right_wheel: wheel_index(Some(b.right_wheel)),
                stiffness: b.stiffness,
            })
            .collect();
        let differentials: Vec<sys::JPC_VehicleDifferentialSettings> = s
            .differentials
            .iter()
            .map(|d| sys::JPC_VehicleDifferentialSettings {
                left_wheel: wheel_index(d.left_wheel),
                right_wheel: wheel_index(d.right_wheel),
                differential_ratio: d.differential_ratio,
                left_right_split: d.left_right_split,
                limited_slip_ratio: d.limited_slip_ratio,
                engine_torque_ratio: d.engine_torque_ratio,
            })
            .collect();
        let torque = curve(&s.engine.normalized_torque);
        let (torque_ptr, torque_len) = curve_ptr(&torque);
        let t = &s.transmission;
        let raw = sys::JPC_WheeledVehicleSettings {
            base: s.common.to_sys(),
            up: s.up.into(),
            forward: s.forward.into(),
            max_pitch_roll_angle: s.max_pitch_roll_angle,
            wheels: wheels.as_ptr(),
            num_wheels: wheels.len() as u32,
            anti_roll_bars: bars.as_ptr(),
            num_anti_roll_bars: bars.len() as u32,
            engine: sys::JPC_VehicleEngineSettings {
                max_torque: s.engine.max_torque,
                min_rpm: s.engine.min_rpm,
                max_rpm: s.engine.max_rpm,
                normalized_torque: torque_ptr,
                num_normalized_torque: torque_len,
                inertia: s.engine.inertia,
                angular_damping: s.engine.angular_damping,
            },
            transmission: sys::JPC_VehicleTransmissionSettings {
                mode: match t.mode {
                    TransmissionMode::Auto => sys::JPC_TRANSMISSION_MODE_AUTO,
                    TransmissionMode::Manual => sys::JPC_TRANSMISSION_MODE_MANUAL,
                },
                gear_ratios: t.gear_ratios.as_ptr(),
                num_gear_ratios: t.gear_ratios.len() as u32,
                reverse_gear_ratios: t.reverse_gear_ratios.as_ptr(),
                num_reverse_gear_ratios: t.reverse_gear_ratios.len() as u32,
                switch_time: t.switch_time,
                clutch_release_time: t.clutch_release_time,
                switch_latency: t.switch_latency,
                shift_up_rpm: t.shift_up_rpm,
                shift_down_rpm: t.shift_down_rpm,
                clutch_strength: t.clutch_strength,
            },
            differentials: differentials.as_ptr(),
            num_differentials: differentials.len() as u32,
            differential_limited_slip_ratio: s.differential_limited_slip_ratio,
        };
        unsafe { sys::JPC_PhysicsSystem_CreateWheeledVehicle(system, &raw, body2, self.tester.as_raw()) }
    }
}

/// State of one wheel after the last update.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WheelState {
    /// The body the wheel stands on, if any.
    pub contact_body: Option<BodyId>,
    pub contact_sub_shape_id: SubShapeId,
    pub contact_position: Vec3,
    pub contact_normal: Vec3,
    pub contact_point_velocity: Vec3,
    pub suspension_length: f32,
    /// rad/s.
    pub angular_velocity: f32,
    pub rotation_angle: f32,
    pub steer_angle: f32,
    pub longitudinal_slip: f32,
    pub lateral_slip: f32,
}

/// Access to a vehicle, from [`PhysicsSystem::vehicle`].
pub struct VehicleMut<'a> {
    raw: NonNull<sys::JPC_Constraint>,
    body_interface: *mut sys::JPC_BodyInterface,
    _system: PhantomData<&'a mut PhysicsSystem>,
}

impl VehicleMut<'_> {
    fn ptr(&self) -> *mut sys::JPC_Constraint {
        self.raw.as_ptr()
    }

    pub fn body(&self) -> BodyId {
        BodyId(unsafe { sys::JPC_Vehicle_GetBody(self.ptr()) })
    }

    /// `forward` and `right` in -1..=1, `brake` and `hand_brake` in 0..=1 (clamped). Wakes the vehicle when any
    /// input is non-zero.
    pub fn set_driver_input(&mut self, forward: f32, right: f32, brake: f32, hand_brake: f32) {
        unsafe { sys::JPC_Vehicle_SetDriverInput(self.ptr(), forward, right, brake, hand_brake) };
        if forward != 0.0 || right != 0.0 || brake != 0.0 || hand_brake != 0.0 {
            unsafe { sys::JPC_BodyInterface_ActivateConstraint(self.body_interface, self.ptr()) };
        }
    }

    pub fn wheel_count(&self) -> usize {
        unsafe { sys::JPC_Vehicle_GetNumWheels(self.ptr()) as usize }
    }

    pub fn wheel(&self, index: usize) -> Option<WheelState> {
        let index = u32::try_from(index).ok()?;
        let mut raw = sys::JPC_WheelState::default();
        if !unsafe { sys::JPC_Vehicle_GetWheelState(self.ptr(), index, &mut raw) } {
            return None;
        }
        Some(WheelState {
            contact_body: if raw.has_contact { BodyId::from_sys(raw.contact_body_id) } else { None },
            contact_sub_shape_id: SubShapeId(raw.contact_sub_shape_id),
            contact_position: raw.contact_position.into(),
            contact_normal: raw.contact_normal.into(),
            contact_point_velocity: raw.contact_point_velocity.into(),
            suspension_length: raw.suspension_length,
            angular_velocity: raw.angular_velocity,
            rotation_angle: raw.rotation_angle,
            steer_angle: raw.steer_angle,
            longitudinal_slip: raw.longitudinal_slip,
            lateral_slip: raw.lateral_slip,
        })
    }

    /// World transform for a wheel model whose axle points along `wheel_right` and top along `wheel_up` in model space.
    pub fn wheel_world_transform(&self, index: usize, wheel_right: Vec3, wheel_up: Vec3) -> Option<Mat44> {
        let index = u32::try_from(index).ok()?;
        let mut raw = sys::JPC_Mat44::default();
        unsafe { sys::JPC_Vehicle_GetWheelWorldTransform(self.ptr(), index, wheel_right.into(), wheel_up.into(), &mut raw) }.then(|| raw.into())
    }

    pub fn engine_rpm(&self) -> f32 {
        unsafe { sys::JPC_Vehicle_GetEngineRPM(self.ptr()) }
    }

    /// Positive for forward gears, negative for reverse, 0 for neutral.
    pub fn current_gear(&self) -> i32 {
        unsafe { sys::JPC_Vehicle_GetCurrentGear(self.ptr()) }
    }

    pub fn clutch_friction(&self) -> f32 {
        unsafe { sys::JPC_Vehicle_GetClutchFriction(self.ptr()) }
    }

    pub fn is_switching_gear(&self) -> bool {
        unsafe { sys::JPC_Vehicle_IsSwitchingGear(self.ptr()) }
    }

    /// Manual transmissions only: pick a gear in `-reverse_gears..=forward_gears` and a clutch friction in 0..=1.
    pub fn set_transmission(&mut self, gear: i32, clutch_friction: f32) -> Result<()> {
        if unsafe { sys::JPC_Vehicle_SetTransmission(self.ptr(), gear, clutch_friction) } { Ok(()) } else { Err(Error::last()) }
    }

    pub fn set_max_pitch_roll_angle(&mut self, angle: f32) {
        unsafe { sys::JPC_Vehicle_SetMaxPitchRollAngle(self.ptr(), angle) }
    }

    /// Test wheel contacts only every `active` steps while awake (`inactive` while asleep). Both at least 1.
    pub fn set_collision_test_interval(&mut self, active: u32, inactive: u32) {
        unsafe { sys::JPC_Vehicle_SetNumStepsBetweenCollisionTests(self.ptr(), active, inactive) }
    }

    /// `None` goes back to the system's gravity.
    pub fn set_gravity_override(&mut self, gravity: Option<Vec3>) {
        match gravity {
            Some(g) => unsafe { sys::JPC_Vehicle_OverrideGravity(self.ptr(), g.into()) },
            None => unsafe { sys::JPC_Vehicle_ResetGravityOverride(self.ptr()) },
        }
    }

    pub fn set_collision_tester(&mut self, tester: &VehicleCollisionTester) {
        unsafe { sys::JPC_Vehicle_SetCollisionTester(self.ptr(), tester.as_raw()) }
    }
}

impl PhysicsSystem {
    /// Turns a dynamic body into a vehicle. The vehicle is a constraint: remove it with
    /// [`remove_constraint`](Self::remove_constraint); it goes away with its body.
    pub fn add_vehicle(&mut self, settings: &WheeledVehicleSettings, body: BodyId, tester: &VehicleCollisionTester) -> Result<ConstraintId> {
        self.add_constraint_to_world(&VehicleCreation { settings, tester }, body)
    }

    /// `None` if the id is gone or not a vehicle.
    pub fn vehicle(&mut self, id: ConstraintId) -> Option<VehicleMut<'_>> {
        let body_interface = self.body_interface().as_raw();
        let raw = self.constraint_raw(id, ConstraintKind::Vehicle)?;
        Some(VehicleMut { raw, body_interface, _system: PhantomData })
    }
}
