#include "joltc_internal.h"

#include <Jolt/Physics/Vehicle/MotorcycleController.h>
#include <Jolt/Physics/Vehicle/TrackedVehicleController.h>
#include <Jolt/Physics/Vehicle/VehicleCollisionTester.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>

// Jolt's defaults, so SetDefault can hand out pointers that stay valid
static const float sDefaultGearRatios[] = { 2.66f, 1.78f, 1.3f, 1.0f, 0.74f };
static const float sDefaultReverseGearRatios[] = { -2.90f };
static const float sDefaultTrackedGearRatios[] = { 4.0f, 3.0f, 2.0f, 1.0f };
static const float sDefaultTrackedReverseGearRatios[] = { -4.0f, -3.0f };

// Wheels must not hit their own vehicle (Jolt's default when no filter is set) nor soft bodies: Jolt would link a soft
// body's index into the rigid island builder and corrupt the islands
class VehicleWheelBodyFilter final : public BodyFilter
{
public:
	virtual bool			ShouldCollide(const BodyID &inBodyID) const override		{ return inBodyID != mVehicleBody; }
	virtual bool			ShouldCollideLocked(const Body &inBody) const override		{ return !inBody.IsSoftBody(); }

	BodyID					mVehicleBody;
};

// Jolt has no way to ask which controller a vehicle has (and is built without C++ RTTI), so every vehicle joltc
// creates records it
class JPC_VehicleConstraint final : public VehicleConstraint
{
public:
							JPC_VehicleConstraint(Body &inBody, const VehicleConstraintSettings &inSettings, JPC_VehicleKind inKind) :
								VehicleConstraint(inBody, inSettings), mKind(inKind) { }

	JPC_VehicleKind			mKind;
	// Used by the vehicle's own tester; lives as long as the vehicle
	VehicleWheelBodyFilter	mWheelFilter;
	// Testers can come from another system, whose layers this one may not have
	const JPC_PhysicsSystem *mSystem = nullptr;
};

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

template <class WheelSettings>
static const char *ValidateWheelBase(const WheelSettings &w)
{
	if (w.radius <= 0.0f || w.suspension_min_length < 0.0f || w.suspension_min_length > w.suspension_max_length)
		return "wheel needs a positive radius and 0 <= suspension_min_length <= suspension_max_length";
	return nullptr;
}

static const char *ValidateAntiRollBars(const JPC_VehicleAntiRollBar *inBars, uint32_t inCount, uint32_t inNumWheels)
{
	if (inCount > 0 && inBars == nullptr)
		return "anti_roll_bars is null";
	for (uint32_t i = 0; i < inCount; ++i)
	{
		const JPC_VehicleAntiRollBar &b = inBars[i];
		if (b.left_wheel < 0 || uint32_t(b.left_wheel) >= inNumWheels || b.right_wheel < 0 || uint32_t(b.right_wheel) >= inNumWheels)
			return "anti roll bar references a wheel that does not exist";
	}
	return nullptr;
}

static const char *ValidateDrivetrain(const JPC_VehicleEngineSettings &e, const JPC_VehicleTransmissionSettings &t)
{
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
	if (t.mode == JPC_TRANSMISSION_MODE_AUTO && t.shift_up_rpm >= e.max_rpm)
		return "automatic transmission needs shift_up_rpm below the engine's max_rpm";
	if (e.max_rpm <= e.min_rpm || e.min_rpm < 0.0f)
		return "engine needs 0 <= min_rpm < max_rpm";
	if (e.num_normalized_torque > 0 && e.normalized_torque == nullptr)
		return "engine torque curve is null";
	return nullptr;
}

static const char *ValidateVehicle(const JPC_WheeledVehicleSettings &in)
{
	int32_t num_wheels = int32_t(in.num_wheels);
	if (in.num_wheels == 0 || in.wheels == nullptr)
		return "vehicle needs at least one wheel";
	for (uint32_t i = 0; i < in.num_wheels; ++i)
	{
		const JPC_WheelSettingsWV &w = in.wheels[i];
		if (const char *error = ValidateWheelBase(w))
			return error;
		if (w.inertia < 0.0f || w.angular_damping < 0.0f || w.max_brake_torque < 0.0f || w.max_hand_brake_torque < 0.0f)
			return "wheel inertia, damping and brake torques must not be negative";
		if (abs(w.max_steer_angle) > 0.5f * JPH_PI)
			return "wheel max_steer_angle must be at most 90 degrees";
		if ((w.num_longitudinal_friction > 0 && w.longitudinal_friction == nullptr) || (w.num_lateral_friction > 0 && w.lateral_friction == nullptr))
			return "wheel friction curve is null";
	}
	if (const char *error = ValidateAntiRollBars(in.anti_roll_bars, in.num_anti_roll_bars, in.num_wheels))
		return error;
	if (in.num_differentials > 0 && in.differentials == nullptr)
		return "differentials is null";
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
	return ValidateDrivetrain(in.engine, in.transmission);
}

static const char *ValidateTrackedVehicle(const JPC_TrackedVehicleSettings &in)
{
	if (in.num_wheels == 0 || in.wheels == nullptr)
		return "vehicle needs at least one wheel";
	for (uint32_t i = 0; i < in.num_wheels; ++i)
	{
		const JPC_WheelSettingsTV &w = in.wheels[i];
		if (const char *error = ValidateWheelBase(w))
			return error;
		if (w.longitudinal_friction < 0.0f || w.lateral_friction < 0.0f)
			return "wheel friction must not be negative";
	}
	if (const char *error = ValidateAntiRollBars(in.anti_roll_bars, in.num_anti_roll_bars, in.num_wheels))
		return error;

	// Jolt looks up each wheel's track and each track's driven wheel without checks
	Array<uint32> tracks_per_wheel(in.num_wheels, 0);
	for (const JPC_VehicleTrackSettings &t : in.tracks)
	{
		if (t.num_wheels == 0 || t.wheels == nullptr)
			return "every track needs at least one wheel";
		bool has_driven_wheel = false;
		for (uint32_t i = 0; i < t.num_wheels; ++i)
		{
			if (t.wheels[i] >= in.num_wheels)
				return "track references a wheel that does not exist";
			++tracks_per_wheel[t.wheels[i]];
			has_driven_wheel |= t.wheels[i] == t.driven_wheel;
		}
		if (!has_driven_wheel)
			return "a track's driven_wheel must be one of its wheels";
		if (t.inertia <= 0.0f || t.angular_damping < 0.0f || t.max_brake_torque < 0.0f || t.differential_ratio <= 0.0f)
			return "track needs inertia > 0, angular_damping >= 0, max_brake_torque >= 0 and differential_ratio > 0";
	}
	for (uint32 count : tracks_per_wheel)
		if (count != 1)
			return "every wheel must be in exactly one track";
	return ValidateDrivetrain(in.engine, in.transmission);
}

static JPC_VehicleConstraint *AsVehicle(JPC_Constraint *inConstraint)
{
	Constraint *c = reinterpret_cast<Constraint *>(inConstraint);
	return c->GetSubType() == EConstraintSubType::Vehicle ? static_cast<JPC_VehicleConstraint *>(static_cast<VehicleConstraint *>(c)) : nullptr;
}

static const JPC_VehicleConstraint *AsVehicle(const JPC_Constraint *inConstraint)
{
	const Constraint *c = reinterpret_cast<const Constraint *>(inConstraint);
	return c->GetSubType() == EConstraintSubType::Vehicle ? static_cast<const JPC_VehicleConstraint *>(static_cast<const VehicleConstraint *>(c)) : nullptr;
}


// A tester as the user describes it. Each vehicle builds its own Jolt tester from it, because the tester's body filter
// has to know the vehicle's body and one description can be shared by many vehicles.
struct JPC_VehicleCollisionTester
{
	JPH_OVERRIDE_NEW_DELETE

	enum class EKind { Ray, CastSphere, CastCylinder };

	std::atomic<uint32>		mRefCount { 1 };
	EKind					mKind = EKind::Ray;
	ObjectLayer				mLayer = 0;
	Vec3					mUp = Vec3::sAxisY();
	float					mMaxSlopeAngle = 0.0f;
	float					mRadius = 0.0f;
	float					mConvexRadiusFraction = 0.0f;
};

static Ref<VehicleCollisionTester> Instantiate(const JPC_VehicleCollisionTester &inTester, const BodyFilter *inFilter)
{
	Ref<VehicleCollisionTester> tester;
	switch (inTester.mKind)
	{
	case JPC_VehicleCollisionTester::EKind::Ray:			tester = new VehicleCollisionTesterRay(inTester.mLayer, inTester.mUp, inTester.mMaxSlopeAngle); break;
	case JPC_VehicleCollisionTester::EKind::CastSphere:		tester = new VehicleCollisionTesterCastSphere(inTester.mLayer, inTester.mRadius, inTester.mUp, inTester.mMaxSlopeAngle); break;
	case JPC_VehicleCollisionTester::EKind::CastCylinder:	tester = new VehicleCollisionTesterCastCylinder(inTester.mLayer, inTester.mConvexRadiusFraction); break;
	}
	tester->SetBodyFilter(inFilter);
	return tester;
}

static JPC_VehicleCollisionTester *MakeTester(const JPC_PhysicsSystem *inSystem, JPC_ObjectLayer inLayer, JPC_VehicleCollisionTester::EKind inKind)
{
	if (!inSystem->IsValidLayer(inLayer))
	{
		JPC_SetError("Object layer does not exist in this physics system");
		return nullptr;
	}
	JPC_VehicleCollisionTester *tester = new JPC_VehicleCollisionTester;
	tester->mKind = inKind;
	tester->mLayer = inLayer;
	return tester;
}

JPC_VehicleCollisionTester *JPC_VehicleCollisionTesterRay_Create(const JPC_PhysicsSystem *system, JPC_ObjectLayer layer, JPC_Vec3 up, float max_slope_angle)
{
	JPC_VehicleCollisionTester *tester = MakeTester(system, layer, JPC_VehicleCollisionTester::EKind::Ray);
	if (tester != nullptr)
	{
		tester->mUp = ToJph(up);
		tester->mMaxSlopeAngle = max_slope_angle;
	}
	return tester;
}

JPC_VehicleCollisionTester *JPC_VehicleCollisionTesterCastSphere_Create(const JPC_PhysicsSystem *system, JPC_ObjectLayer layer, float radius, JPC_Vec3 up, float max_slope_angle)
{
	if (radius <= 0.0f)
	{
		JPC_SetError("Cast sphere radius must be positive");
		return nullptr;
	}
	JPC_VehicleCollisionTester *tester = MakeTester(system, layer, JPC_VehicleCollisionTester::EKind::CastSphere);
	if (tester != nullptr)
	{
		tester->mRadius = radius;
		tester->mUp = ToJph(up);
		tester->mMaxSlopeAngle = max_slope_angle;
	}
	return tester;
}

JPC_VehicleCollisionTester *JPC_VehicleCollisionTesterCastCylinder_Create(const JPC_PhysicsSystem *system, JPC_ObjectLayer layer, float convex_radius_fraction)
{
	JPC_VehicleCollisionTester *tester = MakeTester(system, layer, JPC_VehicleCollisionTester::EKind::CastCylinder);
	if (tester != nullptr)
		tester->mConvexRadiusFraction = Clamp(convex_radius_fraction, 0.0f, 1.0f);
	return tester;
}

void JPC_VehicleCollisionTester_AddRef(const JPC_VehicleCollisionTester *tester)
{
	const_cast<JPC_VehicleCollisionTester *>(tester)->mRefCount.fetch_add(1, std::memory_order_relaxed);
}

void JPC_VehicleCollisionTester_Release(const JPC_VehicleCollisionTester *tester)
{
	JPC_VehicleCollisionTester *t = const_cast<JPC_VehicleCollisionTester *>(tester);
	if (t->mRefCount.fetch_sub(1, std::memory_order_acq_rel) == 1)
		delete t;
}

static void ToJphBase(const JPC_ConstraintSettings &inBase, JPC_Vec3 inUp, JPC_Vec3 inForward, float inMaxPitchRollAngle,
					  const JPC_VehicleAntiRollBar *inBars, uint32_t inNumBars, VehicleConstraintSettings &outSettings)
{
	outSettings.mEnabled = inBase.enabled;
	outSettings.mConstraintPriority = inBase.constraint_priority;
	outSettings.mNumVelocityStepsOverride = inBase.num_velocity_steps_override;
	outSettings.mNumPositionStepsOverride = inBase.num_position_steps_override;
	outSettings.mUserData = inBase.user_data;
	outSettings.mUp = ToJph(inUp).NormalizedOr(Vec3::sAxisY());
	outSettings.mForward = ToJph(inForward).NormalizedOr(Vec3::sAxisZ());
	outSettings.mMaxPitchRollAngle = inMaxPitchRollAngle;
	for (uint32_t i = 0; i < inNumBars; ++i)
	{
		VehicleAntiRollBar bar;
		bar.mLeftWheel = inBars[i].left_wheel;
		bar.mRightWheel = inBars[i].right_wheel;
		bar.mStiffness = inBars[i].stiffness;
		outSettings.mAntiRollBars.push_back(bar);
	}
}

template <class WheelSettings>
static void ToJphWheelBase(const WheelSettings &w, JPH::WheelSettings &out)
{
	out.mPosition = ToJph(w.position);
	out.mSuspensionForcePoint = ToJph(w.suspension_force_point);
	out.mSuspensionDirection = ToJph(w.suspension_direction).NormalizedOr(-Vec3::sAxisY());
	out.mSteeringAxis = ToJph(w.steering_axis).NormalizedOr(Vec3::sAxisY());
	out.mWheelUp = ToJph(w.wheel_up).NormalizedOr(Vec3::sAxisY());
	out.mWheelForward = ToJph(w.wheel_forward).NormalizedOr(Vec3::sAxisZ());
	out.mSuspensionMinLength = w.suspension_min_length;
	out.mSuspensionMaxLength = w.suspension_max_length;
	out.mSuspensionPreloadLength = w.suspension_preload_length;
	out.mSuspensionSpring = SpringSettings((ESpringMode)w.suspension_spring.mode, w.suspension_spring.frequency_or_stiffness, w.suspension_spring.damping);
	out.mRadius = w.radius;
	out.mWidth = w.width;
	out.mEnableSuspensionForcePoint = w.enable_suspension_force_point;
}

template <class WheelSettings>
static void FromJphWheelBase(const JPH::WheelSettings &s, WheelSettings &out)
{
	out.position = ToC(s.mPosition);
	out.suspension_force_point = ToC(s.mSuspensionForcePoint);
	out.suspension_direction = ToC(s.mSuspensionDirection);
	out.steering_axis = ToC(s.mSteeringAxis);
	out.wheel_up = ToC(s.mWheelUp);
	out.wheel_forward = ToC(s.mWheelForward);
	out.suspension_min_length = s.mSuspensionMinLength;
	out.suspension_max_length = s.mSuspensionMaxLength;
	out.suspension_preload_length = s.mSuspensionPreloadLength;
	out.suspension_spring = { (JPC_SpringMode)s.mSuspensionSpring.mMode, s.mSuspensionSpring.mFrequency, s.mSuspensionSpring.mDamping };
	out.radius = s.mRadius;
	out.width = s.mWidth;
	out.enable_suspension_force_point = s.mEnableSuspensionForcePoint;
}

static void ToJph(const JPC_VehicleEngineSettings &in, VehicleEngineSettings &e)
{
	e.mMaxTorque = in.max_torque;
	e.mMinRPM = in.min_rpm;
	e.mMaxRPM = in.max_rpm;
	ToJph(in.normalized_torque, in.num_normalized_torque, e.mNormalizedTorque);
	e.mInertia = in.inertia;
	e.mAngularDamping = in.angular_damping;
}

static void ToJph(const JPC_VehicleTransmissionSettings &in, VehicleTransmissionSettings &t)
{
	t.mMode = (ETransmissionMode)in.mode;
	t.mGearRatios.assign(in.gear_ratios, in.gear_ratios + in.num_gear_ratios);
	t.mReverseGearRatios.assign(in.reverse_gear_ratios, in.reverse_gear_ratios + in.num_reverse_gear_ratios);
	t.mSwitchTime = in.switch_time;
	t.mClutchReleaseTime = in.clutch_release_time;
	t.mSwitchLatency = in.switch_latency;
	t.mShiftUpRPM = in.shift_up_rpm;
	t.mShiftDownRPM = in.shift_down_rpm;
	t.mClutchStrength = in.clutch_strength;
}

static void FromJph(const VehicleTransmissionSettings &t, const float *inGears, uint32_t inNumGears, const float *inReverse, uint32_t inNumReverse,
					JPC_VehicleTransmissionSettings &out)
{
	out.mode = (JPC_TransmissionMode)t.mMode;
	out.gear_ratios = inGears;
	out.num_gear_ratios = inNumGears;
	out.reverse_gear_ratios = inReverse;
	out.num_reverse_gear_ratios = inNumReverse;
	out.switch_time = t.mSwitchTime;
	out.clutch_release_time = t.mClutchReleaseTime;
	out.switch_latency = t.mSwitchLatency;
	out.shift_up_rpm = t.mShiftUpRPM;
	out.shift_down_rpm = t.mShiftDownRPM;
	out.clutch_strength = t.mClutchStrength;
}

// Builds the settings of a wheeled vehicle or motorcycle; the controller settings object decides which
static void ToJphWheeled(const JPC_WheeledVehicleSettings &in, VehicleConstraintSettings &s, WheeledVehicleControllerSettings &controller)
{
	ToJphBase(in.base, in.up, in.forward, in.max_pitch_roll_angle, in.anti_roll_bars, in.num_anti_roll_bars, s);
	for (uint32_t i = 0; i < in.num_wheels; ++i)
	{
		const JPC_WheelSettingsWV &w = in.wheels[i];
		Ref<WheelSettingsWV> ws = new WheelSettingsWV;
		ToJphWheelBase(w, *ws);
		ws->mInertia = w.inertia;
		ws->mAngularDamping = w.angular_damping;
		ws->mMaxSteerAngle = w.max_steer_angle;
		ToJph(w.longitudinal_friction, w.num_longitudinal_friction, ws->mLongitudinalFriction);
		ToJph(w.lateral_friction, w.num_lateral_friction, ws->mLateralFriction);
		ws->mMaxBrakeTorque = w.max_brake_torque;
		ws->mMaxHandBrakeTorque = w.max_hand_brake_torque;
		s.mWheels.push_back(ws.GetPtr());
	}

	ToJph(in.engine, controller.mEngine);
	ToJph(in.transmission, controller.mTransmission);
	controller.mDifferentials.clear();
	for (uint32_t i = 0; i < in.num_differentials; ++i)
	{
		const JPC_VehicleDifferentialSettings &d = in.differentials[i];
		VehicleDifferentialSettings ds;
		ds.mLeftWheel = d.left_wheel;
		ds.mRightWheel = d.right_wheel;
		ds.mDifferentialRatio = d.differential_ratio;
		ds.mLeftRightSplit = d.left_right_split;
		ds.mLimitedSlipRatio = d.limited_slip_ratio;
		ds.mEngineTorqueRatio = d.engine_torque_ratio;
		controller.mDifferentials.push_back(ds);
	}
	controller.mDifferentialLimitedSlipRatio = in.differential_limited_slip_ratio;
}

// Jolt indexes its layer tables with the tester's layer without checks
static bool CheckTesterLayer(const JPC_PhysicsSystem *inSystem, const JPC_VehicleCollisionTester &inTester)
{
	if (inSystem->IsValidLayer(inTester.mLayer))
		return true;
	JPC_SetError("Collision tester layer does not exist in this physics system");
	return false;
}

static JPC_Constraint *CreateVehicle(JPC_PhysicsSystem *system, const VehicleConstraintSettings &inSettings, JPC_BodyID body_id,
									 const JPC_VehicleCollisionTester *tester, JPC_VehicleKind inKind)
{
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

	if (lock.GetBody().IsSoftBody())
	{
		JPC_SetError("Vehicle body must be a rigid body");
		return nullptr;
	}
	if (!CheckTesterLayer(system, *tester))
		return nullptr;

	JPC_VehicleConstraint *vehicle = new JPC_VehicleConstraint(lock.GetBody(), inSettings, inKind);
	vehicle->mWheelFilter.mVehicleBody = lock.GetBody().GetID();
	vehicle->mSystem = system;
	vehicle->SetVehicleCollisionTester(Instantiate(*tester, &vehicle->mWheelFilter));
	vehicle->AddRef();
	return reinterpret_cast<JPC_Constraint *>(static_cast<Constraint *>(vehicle));
}

static bool CheckTester(const JPC_VehicleCollisionTester *tester)
{
	if (tester == nullptr)
		JPC_SetError("Vehicle needs a collision tester");
	return tester != nullptr;
}


JPC_Constraint *JPC_PhysicsSystem_CreateWheeledVehicle(JPC_PhysicsSystem *system, const JPC_WheeledVehicleSettings *in, JPC_BodyID body_id,
													   const JPC_VehicleCollisionTester *tester)
{
	if (!CheckTester(tester))
		return nullptr;
	if (const char *error = ValidateVehicle(*in))
	{
		JPC_SetError(error);
		return nullptr;
	}
	VehicleConstraintSettings s;
	Ref<WheeledVehicleControllerSettings> controller = new WheeledVehicleControllerSettings;
	ToJphWheeled(*in, s, *controller);
	s.mController = controller;
	return CreateVehicle(system, s, body_id, tester, JPC_VEHICLE_KIND_WHEELED);
}

void JPC_MotorcycleSettings_SetDefault(JPC_MotorcycleSettings *out)
{
	JPC_WheeledVehicleSettings_SetDefault(&out->vehicle);
	MotorcycleControllerSettings m;
	out->max_lean_angle = m.mMaxLeanAngle;
	out->lean_spring_constant = m.mLeanSpringConstant;
	out->lean_spring_damping = m.mLeanSpringDamping;
	out->lean_spring_integration_coefficient = m.mLeanSpringIntegrationCoefficient;
	out->lean_spring_integration_coefficient_decay = m.mLeanSpringIntegrationCoefficientDecay;
	out->lean_smoothing_factor = m.mLeanSmoothingFactor;
}

static const char *ValidateLean(float inSpringConstant, float inDamping, float inIntegration, float inDecay, float inSmoothing)
{
	if (inSpringConstant < 0.0f || inDamping < 0.0f || inIntegration < 0.0f || inDecay < 0.0f || inSmoothing < 0.0f || inSmoothing >= 1.0f)
		return "lean spring settings must not be negative and lean_smoothing_factor must be in [0, 1)";
	return nullptr;
}

JPC_Constraint *JPC_PhysicsSystem_CreateMotorcycle(JPC_PhysicsSystem *system, const JPC_MotorcycleSettings *in, JPC_BodyID body_id,
												   const JPC_VehicleCollisionTester *tester)
{
	if (!CheckTester(tester))
		return nullptr;
	const char *error = ValidateVehicle(in->vehicle);
	if (error == nullptr && in->vehicle.num_wheels < 2)
		error = "motorcycle needs at least two wheels";
	if (error == nullptr && (in->max_lean_angle < 0.0f || in->max_lean_angle > 0.5f * JPH_PI))
		error = "motorcycle max_lean_angle must be between 0 and 90 degrees";
	if (error == nullptr)
		error = ValidateLean(in->lean_spring_constant, in->lean_spring_damping, in->lean_spring_integration_coefficient,
							 in->lean_spring_integration_coefficient_decay, in->lean_smoothing_factor);
	if (error != nullptr)
	{
		JPC_SetError(error);
		return nullptr;
	}
	VehicleConstraintSettings s;
	Ref<MotorcycleControllerSettings> controller = new MotorcycleControllerSettings;
	ToJphWheeled(in->vehicle, s, *controller);
	controller->mMaxLeanAngle = in->max_lean_angle;
	controller->mLeanSpringConstant = in->lean_spring_constant;
	controller->mLeanSpringDamping = in->lean_spring_damping;
	controller->mLeanSpringIntegrationCoefficient = in->lean_spring_integration_coefficient;
	controller->mLeanSpringIntegrationCoefficientDecay = in->lean_spring_integration_coefficient_decay;
	controller->mLeanSmoothingFactor = in->lean_smoothing_factor;
	s.mController = controller;
	return CreateVehicle(system, s, body_id, tester, JPC_VEHICLE_KIND_MOTORCYCLE);
}

void JPC_WheelSettingsTV_SetDefault(JPC_WheelSettingsTV *out)
{
	WheelSettingsTV s;
	FromJphWheelBase(s, *out);
	out->longitudinal_friction = s.mLongitudinalFriction;
	out->lateral_friction = s.mLateralFriction;
}

void JPC_TrackedVehicleSettings_SetDefault(JPC_TrackedVehicleSettings *out)
{
	VehicleConstraintSettings s;
	TrackedVehicleControllerSettings c;
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
	FromJph(c.mTransmission, sDefaultTrackedGearRatios, uint32_t(std::size(sDefaultTrackedGearRatios)), sDefaultTrackedReverseGearRatios,
			uint32_t(std::size(sDefaultTrackedReverseGearRatios)), out->transmission);
	for (int i = 0; i < 2; ++i)
	{
		const VehicleTrackSettings &t = c.mTracks[i];
		out->tracks[i] = { 0, nullptr, 0, t.mInertia, t.mAngularDamping, t.mMaxBrakeTorque, t.mDifferentialRatio };
	}
}

JPC_Constraint *JPC_PhysicsSystem_CreateTrackedVehicle(JPC_PhysicsSystem *system, const JPC_TrackedVehicleSettings *in, JPC_BodyID body_id,
													   const JPC_VehicleCollisionTester *tester)
{
	if (!CheckTester(tester))
		return nullptr;
	if (const char *error = ValidateTrackedVehicle(*in))
	{
		JPC_SetError(error);
		return nullptr;
	}
	VehicleConstraintSettings s;
	ToJphBase(in->base, in->up, in->forward, in->max_pitch_roll_angle, in->anti_roll_bars, in->num_anti_roll_bars, s);
	for (uint32_t i = 0; i < in->num_wheels; ++i)
	{
		Ref<WheelSettingsTV> ws = new WheelSettingsTV;
		ToJphWheelBase(in->wheels[i], *ws);
		ws->mLongitudinalFriction = in->wheels[i].longitudinal_friction;
		ws->mLateralFriction = in->wheels[i].lateral_friction;
		s.mWheels.push_back(ws.GetPtr());
	}
	Ref<TrackedVehicleControllerSettings> controller = new TrackedVehicleControllerSettings;
	ToJph(in->engine, controller->mEngine);
	ToJph(in->transmission, controller->mTransmission);
	for (int i = 0; i < 2; ++i)
	{
		const JPC_VehicleTrackSettings &t = in->tracks[i];
		VehicleTrackSettings &out = controller->mTracks[i];
		out.mDrivenWheel = t.driven_wheel;
		out.mWheels.assign(t.wheels, t.wheels + t.num_wheels);
		out.mInertia = t.inertia;
		out.mAngularDamping = t.angular_damping;
		out.mMaxBrakeTorque = t.max_brake_torque;
		out.mDifferentialRatio = t.differential_ratio;
	}
	s.mController = controller;
	return CreateVehicle(system, s, body_id, tester, JPC_VEHICLE_KIND_TRACKED);
}

// Wheeled vehicles and motorcycles share a controller class (MotorcycleController derives from WheeledVehicleController)
static WheeledVehicleController *Wheeled(JPC_VehicleConstraint *inVehicle)
{
	return inVehicle != nullptr && inVehicle->mKind != JPC_VEHICLE_KIND_TRACKED ? static_cast<WheeledVehicleController *>(inVehicle->GetController()) : nullptr;
}

static TrackedVehicleController *Tracked(JPC_VehicleConstraint *inVehicle)
{
	return inVehicle != nullptr && inVehicle->mKind == JPC_VEHICLE_KIND_TRACKED ? static_cast<TrackedVehicleController *>(inVehicle->GetController()) : nullptr;
}

static MotorcycleController *Motorcycle(JPC_VehicleConstraint *inVehicle)
{
	return inVehicle != nullptr && inVehicle->mKind == JPC_VEHICLE_KIND_MOTORCYCLE ? static_cast<MotorcycleController *>(inVehicle->GetController()) : nullptr;
}

static JPC_VehicleConstraint *AsVehicleMut(const JPC_Constraint *inConstraint) { return AsVehicle(const_cast<JPC_Constraint *>(inConstraint)); }

static VehicleEngine *Engine(JPC_VehicleConstraint *inVehicle)
{
	if (WheeledVehicleController *w = Wheeled(inVehicle))
		return &w->GetEngine();
	if (TrackedVehicleController *t = Tracked(inVehicle))
		return &t->GetEngine();
	return nullptr;
}

static VehicleTransmission *Transmission(JPC_VehicleConstraint *inVehicle)
{
	if (WheeledVehicleController *w = Wheeled(inVehicle))
		return &w->GetTransmission();
	if (TrackedVehicleController *t = Tracked(inVehicle))
		return &t->GetTransmission();
	return nullptr;
}

JPC_VehicleKind JPC_Vehicle_GetKind(const JPC_Constraint *vehicle)
{
	const JPC_VehicleConstraint *v = AsVehicle(vehicle);
	return v != nullptr ? v->mKind : JPC_VEHICLE_KIND_WHEELED;
}

bool JPC_Vehicle_SetCollisionTester(JPC_Constraint *vehicle, const JPC_VehicleCollisionTester *tester)
{
	JPC_VehicleConstraint *v = AsVehicle(vehicle);
	if (v == nullptr || !CheckTester(tester) || !CheckTesterLayer(v->mSystem, *tester))
		return false;
	v->SetVehicleCollisionTester(Instantiate(*tester, &v->mWheelFilter));
	return true;
}

void JPC_Vehicle_SetDriverInput(JPC_Constraint *vehicle, float forward, float right, float brake, float hand_brake)
{
	if (WheeledVehicleController *c = Wheeled(AsVehicle(vehicle)))
		c->SetDriverInput(Clamp(forward, -1.0f, 1.0f), Clamp(right, -1.0f, 1.0f), Clamp(brake, 0.0f, 1.0f), Clamp(hand_brake, 0.0f, 1.0f));
}

// Jolt asserts that track ratios are not zero
static float TrackRatio(float inRatio)
{
	inRatio = Clamp(inRatio, -1.0f, 1.0f);
	return abs(inRatio) < 1.0e-3f ? (inRatio < 0.0f ? -1.0e-3f : 1.0e-3f) : inRatio;
}

bool JPC_Vehicle_SetTrackInput(JPC_Constraint *vehicle, float forward, float left_ratio, float right_ratio, float brake)
{
	TrackedVehicleController *c = Tracked(AsVehicle(vehicle));
	if (c == nullptr)
		return false;
	c->SetDriverInput(Clamp(forward, -1.0f, 1.0f), TrackRatio(left_ratio), TrackRatio(right_ratio), Clamp(brake, 0.0f, 1.0f));
	return true;
}

bool JPC_Vehicle_GetTrackAngularVelocity(const JPC_Constraint *vehicle, uint32_t side, float *out_angular_velocity)
{
	TrackedVehicleController *c = Tracked(AsVehicleMut(vehicle));
	if (c == nullptr || side >= 2)
		return false;
	*out_angular_velocity = c->GetTracks()[side].mAngularVelocity;
	return true;
}

uint32_t JPC_Vehicle_GetNumWheels(const JPC_Constraint *vehicle)
{
	const VehicleConstraint *v = AsVehicle(vehicle);
	return v != nullptr ? uint32_t(v->GetWheels().size()) : 0;
}

bool JPC_Vehicle_GetWheelState(const JPC_Constraint *vehicle, uint32_t wheel, JPC_WheelState *out)
{
	const JPC_VehicleConstraint *v = AsVehicle(vehicle);
	if (v == nullptr || wheel >= v->GetWheels().size())
		return false;
	const Wheel *w = v->GetWheel(wheel);
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
	if (v->mKind != JPC_VEHICLE_KIND_TRACKED)
	{
		const WheelWV *wv = static_cast<const WheelWV *>(w);
		out->longitudinal_slip = wv->mLongitudinalSlip;
		out->lateral_slip = wv->mLateralSlip;
	}
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

float JPC_Vehicle_GetEngineRPM(const JPC_Constraint *vehicle) { const VehicleEngine *e = Engine(AsVehicleMut(vehicle)); return e ? e->GetCurrentRPM() : 0.0f; }
int32_t JPC_Vehicle_GetCurrentGear(const JPC_Constraint *vehicle) { const VehicleTransmission *t = Transmission(AsVehicleMut(vehicle)); return t ? t->GetCurrentGear() : 0; }
float JPC_Vehicle_GetClutchFriction(const JPC_Constraint *vehicle) { const VehicleTransmission *t = Transmission(AsVehicleMut(vehicle)); return t ? t->GetClutchFriction() : 0.0f; }
bool JPC_Vehicle_IsSwitchingGear(const JPC_Constraint *vehicle) { const VehicleTransmission *t = Transmission(AsVehicleMut(vehicle)); return t ? t->IsSwitchingGear() : false; }

bool JPC_Vehicle_SetTransmission(JPC_Constraint *vehicle, int32_t gear, float clutch_friction)
{
	VehicleTransmission *t = Transmission(AsVehicle(vehicle));
	if (t == nullptr)
		return false;
	// GetCurrentRatio indexes the gear arrays with the current gear
	if (t->mMode != ETransmissionMode::Manual || gear > int32_t(t->mGearRatios.size()) || gear < -int32_t(t->mReverseGearRatios.size()))
	{
		JPC_SetError("Gear out of range or transmission is not manual");
		return false;
	}
	t->Set(gear, Clamp(clutch_friction, 0.0f, 1.0f));
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

bool JPC_Motorcycle_SetLeanControllerEnabled(JPC_Constraint *vehicle, bool enabled)
{
	MotorcycleController *c = Motorcycle(AsVehicle(vehicle));
	if (c != nullptr)
		c->EnableLeanController(enabled);
	return c != nullptr;
}

bool JPC_Motorcycle_IsLeanControllerEnabled(const JPC_Constraint *vehicle)
{
	const MotorcycleController *c = Motorcycle(AsVehicleMut(vehicle));
	return c != nullptr && c->IsLeanControllerEnabled();
}

bool JPC_Motorcycle_SetLeanSteeringLimitEnabled(JPC_Constraint *vehicle, bool enabled)
{
	MotorcycleController *c = Motorcycle(AsVehicle(vehicle));
	if (c != nullptr)
		c->EnableLeanSteeringLimit(enabled);
	return c != nullptr;
}

bool JPC_Motorcycle_IsLeanSteeringLimitEnabled(const JPC_Constraint *vehicle)
{
	const MotorcycleController *c = Motorcycle(AsVehicleMut(vehicle));
	return c != nullptr && c->IsLeanSteeringLimitEnabled();
}

bool JPC_Motorcycle_SetLeanSpring(JPC_Constraint *vehicle, float spring_constant, float damping, float integration_coefficient,
								  float integration_coefficient_decay, float smoothing_factor)
{
	MotorcycleController *c = Motorcycle(AsVehicle(vehicle));
	if (c == nullptr)
		return false;
	if (const char *error = ValidateLean(spring_constant, damping, integration_coefficient, integration_coefficient_decay, smoothing_factor))
	{
		JPC_SetError(error);
		return false;
	}
	c->SetLeanSpringConstant(spring_constant);
	c->SetLeanSpringDamping(damping);
	c->SetLeanSpringIntegrationCoefficient(integration_coefficient);
	c->SetLeanSpringIntegrationCoefficientDecay(integration_coefficient_decay);
	c->SetLeanSmoothingFactor(smoothing_factor);
	return true;
}

float JPC_Motorcycle_GetWheelBase(const JPC_Constraint *vehicle)
{
	const MotorcycleController *c = Motorcycle(AsVehicleMut(vehicle));
	return c != nullptr ? c->GetWheelBase() : 0.0f;
}
