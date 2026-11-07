#include "joltc_internal.h"

#include <Jolt/Physics/Vehicle/VehicleCollisionTester.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>

// Jolt's defaults, so SetDefault can hand out pointers that stay valid
static const float sDefaultGearRatios[] = { 2.66f, 1.78f, 1.3f, 1.0f, 0.74f };
static const float sDefaultReverseGearRatios[] = { -2.90f };

static void ToJph(const JPC_LinearCurvePoint *inPoints, uint32_t inCount, LinearCurve &outCurve)
{
	if (inPoints == nullptr)
		return;
	outCurve.Clear();
	outCurve.Reserve(inCount);
	for (uint32_t i = 0; i < inCount; ++i)
		outCurve.AddPoint(inPoints[i].x, inPoints[i].y);
	outCurve.Sort();
}

void JPC_WheelSettingsWV_SetDefault(JPC_WheelSettingsWV *out)
{
	WheelSettingsWV s;
	out->position = ToC(s.mPosition);
	out->suspension_force_point = ToC(s.mSuspensionForcePoint);
	out->suspension_direction = ToC(s.mSuspensionDirection);
	out->steering_axis = ToC(s.mSteeringAxis);
	out->wheel_up = ToC(s.mWheelUp);
	out->wheel_forward = ToC(s.mWheelForward);
	out->suspension_min_length = s.mSuspensionMinLength;
	out->suspension_max_length = s.mSuspensionMaxLength;
	out->suspension_preload_length = s.mSuspensionPreloadLength;
	out->suspension_spring = { (JPC_SpringMode)s.mSuspensionSpring.mMode, s.mSuspensionSpring.mFrequency, s.mSuspensionSpring.mDamping };
	out->radius = s.mRadius;
	out->width = s.mWidth;
	out->enable_suspension_force_point = s.mEnableSuspensionForcePoint;
	out->inertia = s.mInertia;
	out->angular_damping = s.mAngularDamping;
	out->max_steer_angle = s.mMaxSteerAngle;
	out->longitudinal_friction = nullptr;
	out->num_longitudinal_friction = 0;
	out->lateral_friction = nullptr;
	out->num_lateral_friction = 0;
	out->max_brake_torque = s.mMaxBrakeTorque;
	out->max_hand_brake_torque = s.mMaxHandBrakeTorque;
}

void JPC_VehicleDifferentialSettings_SetDefault(JPC_VehicleDifferentialSettings *out)
{
	VehicleDifferentialSettings s;
	out->left_wheel = s.mLeftWheel;
	out->right_wheel = s.mRightWheel;
	out->differential_ratio = s.mDifferentialRatio;
	out->left_right_split = s.mLeftRightSplit;
	out->limited_slip_ratio = s.mLimitedSlipRatio;
	out->engine_torque_ratio = s.mEngineTorqueRatio;
}

void JPC_WheeledVehicleSettings_SetDefault(JPC_WheeledVehicleSettings *out)
{
	VehicleConstraintSettings s;
	WheeledVehicleControllerSettings c;
	out->base = { s.mEnabled, s.mConstraintPriority, s.mNumVelocityStepsOverride, s.mNumPositionStepsOverride, s.mUserData };
	out->up = ToC(s.mUp);
	out->forward = ToC(s.mForward);
	out->max_pitch_roll_angle = s.mMaxPitchRollAngle;
	out->wheels = nullptr;
	out->num_wheels = 0;
	out->anti_roll_bars = nullptr;
	out->num_anti_roll_bars = 0;

	const VehicleEngineSettings &e = c.mEngine;
	out->engine = { e.mMaxTorque, e.mMinRPM, e.mMaxRPM, nullptr, 0, e.mInertia, e.mAngularDamping };

	const VehicleTransmissionSettings &t = c.mTransmission;
	out->transmission.mode = (JPC_TransmissionMode)t.mMode;
	out->transmission.gear_ratios = sDefaultGearRatios;
	out->transmission.num_gear_ratios = uint32_t(std::size(sDefaultGearRatios));
	out->transmission.reverse_gear_ratios = sDefaultReverseGearRatios;
	out->transmission.num_reverse_gear_ratios = uint32_t(std::size(sDefaultReverseGearRatios));
	out->transmission.switch_time = t.mSwitchTime;
	out->transmission.clutch_release_time = t.mClutchReleaseTime;
	out->transmission.switch_latency = t.mSwitchLatency;
	out->transmission.shift_up_rpm = t.mShiftUpRPM;
	out->transmission.shift_down_rpm = t.mShiftDownRPM;
	out->transmission.clutch_strength = t.mClutchStrength;

	out->differentials = nullptr;
	out->num_differentials = 0;
	out->differential_limited_slip_ratio = c.mDifferentialLimitedSlipRatio;
}

// Jolt indexes wheels and gears from these settings without bounds checks, and only asserts the rest
static const char *ValidateVehicle(const JPC_WheeledVehicleSettings &in)
{
	int32_t num_wheels = int32_t(in.num_wheels);
	if (in.num_wheels == 0 || in.wheels == nullptr)
		return "vehicle needs at least one wheel";
	for (uint32_t i = 0; i < in.num_wheels; ++i)
	{
		const JPC_WheelSettingsWV &w = in.wheels[i];
		if (w.radius <= 0.0f || w.suspension_min_length < 0.0f || w.suspension_min_length > w.suspension_max_length)
			return "wheel needs a positive radius and 0 <= suspension_min_length <= suspension_max_length";
		if (w.inertia < 0.0f || w.angular_damping < 0.0f || w.max_brake_torque < 0.0f || w.max_hand_brake_torque < 0.0f)
			return "wheel inertia, damping and brake torques must not be negative";
		if (abs(w.max_steer_angle) > 0.5f * JPH_PI)
			return "wheel max_steer_angle must be at most 90 degrees";
		if ((w.num_longitudinal_friction > 0 && w.longitudinal_friction == nullptr) || (w.num_lateral_friction > 0 && w.lateral_friction == nullptr))
			return "wheel friction curve is null";
	}
	for (uint32_t i = 0; i < in.num_anti_roll_bars; ++i)
	{
		const JPC_VehicleAntiRollBar &b = in.anti_roll_bars[i];
		if (b.left_wheel < 0 || b.left_wheel >= num_wheels || b.right_wheel < 0 || b.right_wheel >= num_wheels)
			return "anti roll bar references a wheel that does not exist";
	}
	for (uint32_t i = 0; i < in.num_differentials; ++i)
	{
		const JPC_VehicleDifferentialSettings &d = in.differentials[i];
		if (d.left_wheel < -1 || d.left_wheel >= num_wheels || d.right_wheel < -1 || d.right_wheel >= num_wheels)
			return "differential references a wheel that does not exist";
		if (d.differential_ratio <= 0.0f || d.left_right_split < 0.0f || d.left_right_split > 1.0f || d.engine_torque_ratio < 0.0f || d.limited_slip_ratio <= 1.0f)
			return "differential needs ratio > 0, split in [0, 1], engine_torque_ratio >= 0 and limited_slip_ratio > 1";
	}
	if (in.differential_limited_slip_ratio <= 1.0f)
		return "differential_limited_slip_ratio must be greater than 1";
	const JPC_VehicleTransmissionSettings &t = in.transmission;
	if (t.num_gear_ratios == 0 || t.gear_ratios == nullptr || t.num_reverse_gear_ratios == 0 || t.reverse_gear_ratios == nullptr)
		return "transmission needs at least one forward and one reverse gear";
	for (uint32_t i = 0; i < t.num_gear_ratios; ++i)
		if (t.gear_ratios[i] <= 0.0f)
			return "forward gear ratios must be positive";
	for (uint32_t i = 0; i < t.num_reverse_gear_ratios; ++i)
		if (t.reverse_gear_ratios[i] >= 0.0f)
			return "reverse gear ratios must be negative";
	if (t.switch_time < 0.0f || t.clutch_strength <= 0.0f || t.shift_down_rpm <= 0.0f || t.shift_up_rpm <= t.shift_down_rpm)
		return "transmission needs switch_time >= 0, clutch_strength > 0 and 0 < shift_down_rpm < shift_up_rpm";
	if (t.mode == JPC_TRANSMISSION_MODE_AUTO && t.shift_up_rpm >= in.engine.max_rpm)
		return "automatic transmission needs shift_up_rpm below the engine's max_rpm";
	if (in.engine.max_rpm <= in.engine.min_rpm || in.engine.min_rpm < 0.0f)
		return "engine needs 0 <= min_rpm < max_rpm";
	if (in.engine.num_normalized_torque > 0 && in.engine.normalized_torque == nullptr)
		return "engine torque curve is null";
	return nullptr;
}

static VehicleConstraint *AsVehicle(JPC_Constraint *inConstraint)
{
	Constraint *c = reinterpret_cast<Constraint *>(inConstraint);
	return c->GetSubType() == EConstraintSubType::Vehicle ? static_cast<VehicleConstraint *>(c) : nullptr;
}

static const VehicleConstraint *AsVehicle(const JPC_Constraint *inConstraint)
{
	const Constraint *c = reinterpret_cast<const Constraint *>(inConstraint);
	return c->GetSubType() == EConstraintSubType::Vehicle ? static_cast<const VehicleConstraint *>(c) : nullptr;
}

static inline const VehicleCollisionTester *ToJph(const JPC_VehicleCollisionTester *t) { return reinterpret_cast<const VehicleCollisionTester *>(t); }

static JPC_VehicleCollisionTester *FinishTester(VehicleCollisionTester *inTester)
{
	inTester->AddRef();
	return reinterpret_cast<JPC_VehicleCollisionTester *>(inTester);
}

static bool CheckTesterLayer(const JPC_PhysicsSystem *inSystem, JPC_ObjectLayer inLayer)
{
	if (!inSystem->IsValidLayer(inLayer))
		JPC_SetError("Object layer does not exist in this physics system");
	return inSystem->IsValidLayer(inLayer);
}

JPC_VehicleCollisionTester *JPC_VehicleCollisionTesterRay_Create(const JPC_PhysicsSystem *system, JPC_ObjectLayer layer, JPC_Vec3 up, float max_slope_angle)
{
	return CheckTesterLayer(system, layer) ? FinishTester(new VehicleCollisionTesterRay(layer, ToJph(up), max_slope_angle)) : nullptr;
}

JPC_VehicleCollisionTester *JPC_VehicleCollisionTesterCastSphere_Create(const JPC_PhysicsSystem *system, JPC_ObjectLayer layer, float radius, JPC_Vec3 up, float max_slope_angle)
{
	if (radius <= 0.0f)
	{
		JPC_SetError("Cast sphere radius must be positive");
		return nullptr;
	}
	return CheckTesterLayer(system, layer) ? FinishTester(new VehicleCollisionTesterCastSphere(layer, radius, ToJph(up), max_slope_angle)) : nullptr;
}

JPC_VehicleCollisionTester *JPC_VehicleCollisionTesterCastCylinder_Create(const JPC_PhysicsSystem *system, JPC_ObjectLayer layer, float convex_radius_fraction)
{
	return CheckTesterLayer(system, layer) ? FinishTester(new VehicleCollisionTesterCastCylinder(layer, Clamp(convex_radius_fraction, 0.0f, 1.0f))) : nullptr;
}

void JPC_VehicleCollisionTester_AddRef(const JPC_VehicleCollisionTester *tester) { ToJph(tester)->AddRef(); }
void JPC_VehicleCollisionTester_Release(const JPC_VehicleCollisionTester *tester) { ToJph(tester)->Release(); }

JPC_Constraint *JPC_PhysicsSystem_CreateWheeledVehicle(JPC_PhysicsSystem *system, const JPC_WheeledVehicleSettings *in, JPC_BodyID body_id,
													   const JPC_VehicleCollisionTester *tester)
{
	if (tester == nullptr)
	{
		JPC_SetError("Vehicle needs a collision tester");
		return nullptr;
	}
	if (const char *error = ValidateVehicle(*in))
	{
		JPC_SetError(error);
		return nullptr;
	}

	VehicleConstraintSettings s;
	s.mEnabled = in->base.enabled;
	s.mConstraintPriority = in->base.constraint_priority;
	s.mNumVelocityStepsOverride = in->base.num_velocity_steps_override;
	s.mNumPositionStepsOverride = in->base.num_position_steps_override;
	s.mUserData = in->base.user_data;
	s.mUp = ToJph(in->up).NormalizedOr(Vec3::sAxisY());
	s.mForward = ToJph(in->forward).NormalizedOr(Vec3::sAxisZ());
	s.mMaxPitchRollAngle = in->max_pitch_roll_angle;

	for (uint32_t i = 0; i < in->num_wheels; ++i)
	{
		const JPC_WheelSettingsWV &w = in->wheels[i];
		Ref<WheelSettingsWV> ws = new WheelSettingsWV;
		ws->mPosition = ToJph(w.position);
		ws->mSuspensionForcePoint = ToJph(w.suspension_force_point);
		ws->mSuspensionDirection = ToJph(w.suspension_direction).NormalizedOr(-Vec3::sAxisY());
		ws->mSteeringAxis = ToJph(w.steering_axis).NormalizedOr(Vec3::sAxisY());
		ws->mWheelUp = ToJph(w.wheel_up).NormalizedOr(Vec3::sAxisY());
		ws->mWheelForward = ToJph(w.wheel_forward).NormalizedOr(Vec3::sAxisZ());
		ws->mSuspensionMinLength = w.suspension_min_length;
		ws->mSuspensionMaxLength = w.suspension_max_length;
		ws->mSuspensionPreloadLength = w.suspension_preload_length;
		ws->mSuspensionSpring = SpringSettings((ESpringMode)w.suspension_spring.mode, w.suspension_spring.frequency_or_stiffness, w.suspension_spring.damping);
		ws->mRadius = w.radius;
		ws->mWidth = w.width;
		ws->mEnableSuspensionForcePoint = w.enable_suspension_force_point;
		ws->mInertia = w.inertia;
		ws->mAngularDamping = w.angular_damping;
		ws->mMaxSteerAngle = w.max_steer_angle;
		ToJph(w.longitudinal_friction, w.num_longitudinal_friction, ws->mLongitudinalFriction);
		ToJph(w.lateral_friction, w.num_lateral_friction, ws->mLateralFriction);
		ws->mMaxBrakeTorque = w.max_brake_torque;
		ws->mMaxHandBrakeTorque = w.max_hand_brake_torque;
		s.mWheels.push_back(ws.GetPtr());
	}

	for (uint32_t i = 0; i < in->num_anti_roll_bars; ++i)
	{
		VehicleAntiRollBar bar;
		bar.mLeftWheel = in->anti_roll_bars[i].left_wheel;
		bar.mRightWheel = in->anti_roll_bars[i].right_wheel;
		bar.mStiffness = in->anti_roll_bars[i].stiffness;
		s.mAntiRollBars.push_back(bar);
	}

	Ref<WheeledVehicleControllerSettings> controller = new WheeledVehicleControllerSettings;
	VehicleEngineSettings &e = controller->mEngine;
	e.mMaxTorque = in->engine.max_torque;
	e.mMinRPM = in->engine.min_rpm;
	e.mMaxRPM = in->engine.max_rpm;
	ToJph(in->engine.normalized_torque, in->engine.num_normalized_torque, e.mNormalizedTorque);
	e.mInertia = in->engine.inertia;
	e.mAngularDamping = in->engine.angular_damping;

	VehicleTransmissionSettings &t = controller->mTransmission;
	t.mMode = (ETransmissionMode)in->transmission.mode;
	t.mGearRatios.assign(in->transmission.gear_ratios, in->transmission.gear_ratios + in->transmission.num_gear_ratios);
	t.mReverseGearRatios.assign(in->transmission.reverse_gear_ratios, in->transmission.reverse_gear_ratios + in->transmission.num_reverse_gear_ratios);
	t.mSwitchTime = in->transmission.switch_time;
	t.mClutchReleaseTime = in->transmission.clutch_release_time;
	t.mSwitchLatency = in->transmission.switch_latency;
	t.mShiftUpRPM = in->transmission.shift_up_rpm;
	t.mShiftDownRPM = in->transmission.shift_down_rpm;
	t.mClutchStrength = in->transmission.clutch_strength;

	controller->mDifferentials.clear();
	for (uint32_t i = 0; i < in->num_differentials; ++i)
	{
		const JPC_VehicleDifferentialSettings &d = in->differentials[i];
		VehicleDifferentialSettings ds;
		ds.mLeftWheel = d.left_wheel;
		ds.mRightWheel = d.right_wheel;
		ds.mDifferentialRatio = d.differential_ratio;
		ds.mLeftRightSplit = d.left_right_split;
		ds.mLimitedSlipRatio = d.limited_slip_ratio;
		ds.mEngineTorqueRatio = d.engine_torque_ratio;
		controller->mDifferentials.push_back(ds);
	}
	controller->mDifferentialLimitedSlipRatio = in->differential_limited_slip_ratio;
	s.mController = controller;

	std::shared_lock state_lock(system->mBodyStateMutex);
	if (system->mProtectedBodies.find(body_id) != system->mProtectedBodies.end())
	{
		JPC_SetError("Vehicles cannot use bodies owned by a character");
		return nullptr;
	}
	BodyLockWrite lock(system->mSystem.GetBodyLockInterface(), ToJph(body_id));
	if (!lock.Succeeded())
	{
		JPC_SetError("Vehicle body does not exist");
		return nullptr;
	}
	if (!lock.GetBody().IsDynamic())
	{
		JPC_SetError("Vehicle body must be dynamic");
		return nullptr;
	}

	VehicleConstraint *vehicle = new VehicleConstraint(lock.GetBody(), s);
	vehicle->SetVehicleCollisionTester(ToJph(tester));
	vehicle->AddRef();
	return reinterpret_cast<JPC_Constraint *>(static_cast<Constraint *>(vehicle));
}

static WheeledVehicleController *Controller(VehicleConstraint *inVehicle) { return static_cast<WheeledVehicleController *>(inVehicle->GetController()); }
static const WheeledVehicleController *Controller(const VehicleConstraint *inVehicle) { return static_cast<const WheeledVehicleController *>(inVehicle->GetController()); }

void JPC_Vehicle_SetCollisionTester(JPC_Constraint *vehicle, const JPC_VehicleCollisionTester *tester)
{
	if (VehicleConstraint *v = AsVehicle(vehicle); v != nullptr && tester != nullptr)
		v->SetVehicleCollisionTester(ToJph(tester));
}

void JPC_Vehicle_SetDriverInput(JPC_Constraint *vehicle, float forward, float right, float brake, float hand_brake)
{
	if (VehicleConstraint *v = AsVehicle(vehicle))
		Controller(v)->SetDriverInput(Clamp(forward, -1.0f, 1.0f), Clamp(right, -1.0f, 1.0f), Clamp(brake, 0.0f, 1.0f), Clamp(hand_brake, 0.0f, 1.0f));
}

uint32_t JPC_Vehicle_GetNumWheels(const JPC_Constraint *vehicle)
{
	const VehicleConstraint *v = AsVehicle(vehicle);
	return v != nullptr ? uint32_t(v->GetWheels().size()) : 0;
}

bool JPC_Vehicle_GetWheelState(const JPC_Constraint *vehicle, uint32_t wheel, JPC_WheelState *out)
{
	const VehicleConstraint *v = AsVehicle(vehicle);
	if (v == nullptr || wheel >= v->GetWheels().size())
		return false;
	const WheelWV *w = static_cast<const WheelWV *>(v->GetWheel(wheel));
	*out = { };
	out->has_contact = w->HasContact();
	out->contact_body_id = ToC(w->GetContactBodyID());
	out->contact_sub_shape_id = ToC(w->GetContactSubShapeID());
	if (w->HasContact())
	{
		out->contact_position = ToC(w->GetContactPosition());
		out->contact_normal = ToC(w->GetContactNormal());
		out->contact_point_velocity = ToC(w->GetContactPointVelocity());
	}
	out->suspension_length = w->GetSuspensionLength();
	out->angular_velocity = w->GetAngularVelocity();
	out->rotation_angle = w->GetRotationAngle();
	out->steer_angle = w->GetSteerAngle();
	out->longitudinal_slip = w->mLongitudinalSlip;
	out->lateral_slip = w->mLateralSlip;
	return true;
}

bool JPC_Vehicle_GetWheelWorldTransform(const JPC_Constraint *vehicle, uint32_t wheel, JPC_Vec3 wheel_right, JPC_Vec3 wheel_up, JPC_RMat44 *out)
{
	const VehicleConstraint *v = AsVehicle(vehicle);
	if (v == nullptr || wheel >= v->GetWheels().size())
		return false;
	*out = ToC(v->GetWheelWorldTransform(wheel, ToJph(wheel_right), ToJph(wheel_up)));
	return true;
}

float JPC_Vehicle_GetEngineRPM(const JPC_Constraint *vehicle) { const VehicleConstraint *v = AsVehicle(vehicle); return v ? Controller(v)->GetEngine().GetCurrentRPM() : 0.0f; }
int32_t JPC_Vehicle_GetCurrentGear(const JPC_Constraint *vehicle) { const VehicleConstraint *v = AsVehicle(vehicle); return v ? Controller(v)->GetTransmission().GetCurrentGear() : 0; }
float JPC_Vehicle_GetClutchFriction(const JPC_Constraint *vehicle) { const VehicleConstraint *v = AsVehicle(vehicle); return v ? Controller(v)->GetTransmission().GetClutchFriction() : 0.0f; }
bool JPC_Vehicle_IsSwitchingGear(const JPC_Constraint *vehicle) { const VehicleConstraint *v = AsVehicle(vehicle); return v ? Controller(v)->GetTransmission().IsSwitchingGear() : false; }

bool JPC_Vehicle_SetTransmission(JPC_Constraint *vehicle, int32_t gear, float clutch_friction)
{
	VehicleConstraint *v = AsVehicle(vehicle);
	if (v == nullptr)
		return false;
	VehicleTransmission &t = Controller(v)->GetTransmission();
	// GetCurrentRatio indexes the gear arrays with the current gear
	if (t.mMode != ETransmissionMode::Manual || gear > int32_t(t.mGearRatios.size()) || gear < -int32_t(t.mReverseGearRatios.size()))
	{
		JPC_SetError("Gear out of range or transmission is not manual");
		return false;
	}
	t.Set(gear, Clamp(clutch_friction, 0.0f, 1.0f));
	return true;
}

void JPC_Vehicle_SetMaxPitchRollAngle(JPC_Constraint *vehicle, float angle) { if (VehicleConstraint *v = AsVehicle(vehicle)) v->SetMaxPitchRollAngle(angle); }

void JPC_Vehicle_SetNumStepsBetweenCollisionTests(JPC_Constraint *vehicle, uint32_t active, uint32_t inactive)
{
	if (VehicleConstraint *v = AsVehicle(vehicle))
	{
		v->SetNumStepsBetweenCollisionTestActive(max(active, 1u));
		v->SetNumStepsBetweenCollisionTestInactive(max(inactive, 1u));
	}
}

void JPC_Vehicle_OverrideGravity(JPC_Constraint *vehicle, JPC_Vec3 gravity) { if (VehicleConstraint *v = AsVehicle(vehicle)) v->OverrideGravity(ToJph(gravity)); }
void JPC_Vehicle_ResetGravityOverride(JPC_Constraint *vehicle) { if (VehicleConstraint *v = AsVehicle(vehicle)) v->ResetGravityOverride(); }
JPC_BodyID JPC_Vehicle_GetBody(const JPC_Constraint *vehicle) { const VehicleConstraint *v = AsVehicle(vehicle); return v ? ToC(v->GetVehicleBody()->GetID()) : JPC_BODY_ID_INVALID; }
