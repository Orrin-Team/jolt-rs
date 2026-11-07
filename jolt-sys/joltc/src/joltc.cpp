#include "joltc.h"

#include "joltc_internal.h"

#include <cstdarg>
#include <cstdio>
#include <string>

#include <Jolt/Physics/Collision/AABoxCast.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseQuery.h>
#include <Jolt/Physics/Constraints/GearConstraint.h>
#include <Jolt/Physics/Constraints/PathConstraint.h>
#include <Jolt/Physics/Constraints/PathConstraintPathHermite.h>
#include <Jolt/Physics/Constraints/PulleyConstraint.h>
#include <Jolt/Physics/Constraints/RackAndPinionConstraint.h>
#include <Jolt/Physics/StateRecorderImpl.h>
#include <Jolt/Geometry/OrientedBox.h>

static_assert(sizeof(JPC_ObjectLayer) == sizeof(ObjectLayer), "JPC_ObjectLayer must match JPH_OBJECT_LAYER_BITS");
static_assert(sizeof(JPC_BodyID) == sizeof(BodyID), "BodyID size mismatch");
static_assert(sizeof(JPC_SubShapeID) == sizeof(SubShapeID), "SubShapeID size mismatch");
static_assert(JPC_MAX_PHYSICS_JOBS == cMaxPhysicsJobs && JPC_MAX_PHYSICS_BARRIERS == cMaxPhysicsBarriers, "Job limits changed");
static_assert((int)EShapeSubType::Plane == JPC_SHAPE_SUB_TYPE_PLANE && (int)EShapeSubType::Empty == JPC_SHAPE_SUB_TYPE_EMPTY, "Shape sub types changed");

// ---------------------------------------------------------------------------
// Errors, tracing and global setup
// ---------------------------------------------------------------------------

static thread_local std::string sLastError;

void JPC_SetError(const char *inMessage) { sLastError = inMessage; }

const char *JPC_GetLastError(void) { return sLastError.c_str(); }

static JPC_TraceFn sTraceHandler = nullptr;
static JPC_AssertFailedFn sAssertHandler = nullptr;

static void TraceImpl(const char *inFMT, ...)
{
	char buffer[1024];
	va_list list;
	va_start(list, inFMT);
	vsnprintf(buffer, sizeof(buffer), inFMT, list);
	va_end(list);

	if (sTraceHandler != nullptr)
		sTraceHandler(buffer);
	else
		fprintf(stderr, "%s\n", buffer);
}

#ifdef JPH_ENABLE_ASSERTS
static bool AssertFailedImpl(const char *inExpression, const char *inMessage, const char *inFile, uint inLine)
{
	if (sAssertHandler != nullptr)
		return sAssertHandler(inExpression, inMessage, inFile, inLine);
	fprintf(stderr, "%s:%u: (%s) %s\n", inFile, inLine, inExpression, inMessage != nullptr ? inMessage : "");
	return true;
}
#endif

static std::mutex sInitMutex;
static bool sInitialized = false;

void JPC_Init(void)
{
	std::lock_guard lock(sInitMutex);
	if (sInitialized)
		return;
	sInitialized = true;

	RegisterDefaultAllocator();
	Trace = TraceImpl;
	JPH_IF_ENABLE_ASSERTS(AssertFailed = AssertFailedImpl;)
	Factory::sInstance = new Factory();
	RegisterTypes();
}

void JPC_Shutdown(void)
{
	std::lock_guard lock(sInitMutex);
	if (!sInitialized)
		return;
	sInitialized = false;

	UnregisterTypes();
	delete Factory::sInstance;
	Factory::sInstance = nullptr;
}

void JPC_SetTraceHandler(JPC_TraceFn handler) { sTraceHandler = handler; }
void JPC_SetAssertFailedHandler(JPC_AssertFailedFn handler) { sAssertHandler = handler; }

// ---------------------------------------------------------------------------
// Temp allocator and job systems
// ---------------------------------------------------------------------------

JPC_TempAllocator *JPC_TempAllocator_Create(uint32_t size_in_bytes)
{
	return reinterpret_cast<JPC_TempAllocator *>(static_cast<TempAllocator *>(new TempAllocatorImpl(size_in_bytes)));
}

void JPC_TempAllocator_Destroy(JPC_TempAllocator *allocator) { delete ToJph(allocator); }

// A physics update creates many jobs at once and blocks forever when the pool runs out, and the free list needs a
// power of two page size
static uint32 JobCapacity(uint32 inMaxJobs) { return GetNextPowerOf2(max(inMaxJobs, uint32(cMaxPhysicsJobs))); }

JPC_JobSystem *JPC_JobSystemThreadPool_Create(uint32_t max_jobs, uint32_t max_barriers, int32_t num_threads)
{
	uint32 barriers = max(max_barriers, 1u);
	return new JPC_JobSystem(new JobSystemThreadPool(JobCapacity(max_jobs), barriers, num_threads), barriers);
}

JPC_JobSystem *JPC_JobSystemSingleThreaded_Create(uint32_t max_jobs)
{
	// Its barrier is a dummy that can be handed out any number of times
	return new JPC_JobSystem(new JobSystemSingleThreaded(JobCapacity(max_jobs)), UINT32_MAX);
}

void JPC_JobSystem_Destroy(JPC_JobSystem *job_system) { delete job_system; }

// ---------------------------------------------------------------------------
// Shapes
// ---------------------------------------------------------------------------

static JPC_Shape *FinishShape(const ShapeSettings::ShapeResult &inResult)
{
	if (inResult.HasError())
	{
		JPC_SetError(inResult.GetError().c_str());
		return nullptr;
	}
	Shape *shape = inResult.Get().GetPtr();
	shape->AddRef();
	return ToC(shape);
}

template <class Settings>
static JPC_Shape *CreateShape(Settings &ioSettings)
{
	ioSettings.SetEmbedded();
	return FinishShape(ioSettings.Create());
}

static bool CheckInner(const JPC_Shape *inner)
{
	if (inner == nullptr)
		JPC_SetError("Inner shape is null");
	return inner != nullptr;
}

JPC_Shape *JPC_SphereShape_Create(float radius)
{
	SphereShapeSettings settings(radius);
	return CreateShape(settings);
}

JPC_Shape *JPC_BoxShape_Create(JPC_Vec3 half_extent, float convex_radius)
{
	BoxShapeSettings settings(ToJph(half_extent), convex_radius);
	return CreateShape(settings);
}

JPC_Shape *JPC_CapsuleShape_Create(float half_height_of_cylinder, float radius)
{
	CapsuleShapeSettings settings(half_height_of_cylinder, radius);
	return CreateShape(settings);
}

JPC_Shape *JPC_TaperedCapsuleShape_Create(float half_height, float top_radius, float bottom_radius)
{
	TaperedCapsuleShapeSettings settings(half_height, top_radius, bottom_radius);
	return CreateShape(settings);
}

JPC_Shape *JPC_CylinderShape_Create(float half_height, float radius, float convex_radius)
{
	CylinderShapeSettings settings(half_height, radius, convex_radius);
	return CreateShape(settings);
}

JPC_Shape *JPC_TaperedCylinderShape_Create(float half_height, float top_radius, float bottom_radius, float convex_radius)
{
	TaperedCylinderShapeSettings settings(half_height, top_radius, bottom_radius, convex_radius);
	return CreateShape(settings);
}

JPC_Shape *JPC_ConvexHullShape_Create(const JPC_Vec3 *points, uint32_t num_points, float max_convex_radius)
{
	ConvexHullShapeSettings settings;
	settings.mPoints.reserve(num_points);
	for (uint32_t i = 0; i < num_points; ++i)
		settings.mPoints.push_back(ToJph(points[i]));
	settings.mMaxConvexRadius = max_convex_radius;
	return CreateShape(settings);
}

JPC_Shape *JPC_MeshShape_Create(const JPC_Vec3 *vertices, uint32_t num_vertices, const uint32_t *indices, uint32_t num_triangles)
{
	VertexList vertex_list;
	vertex_list.reserve(num_vertices);
	for (uint32_t i = 0; i < num_vertices; ++i)
		vertex_list.push_back(Float3(vertices[i].x, vertices[i].y, vertices[i].z));

	IndexedTriangleList triangles;
	triangles.reserve(num_triangles);
	for (uint32_t i = 0; i < num_triangles; ++i)
	{
		uint32_t a = indices[3 * i], b = indices[3 * i + 1], c = indices[3 * i + 2];
		if (a >= num_vertices || b >= num_vertices || c >= num_vertices)
		{
			JPC_SetError("Mesh triangle index out of range");
			return nullptr;
		}
		triangles.push_back(IndexedTriangle(a, b, c));
	}

	MeshShapeSettings settings(std::move(vertex_list), std::move(triangles));
	return CreateShape(settings);
}

JPC_Shape *JPC_HeightFieldShape_Create(const float *samples, JPC_Vec3 offset, JPC_Vec3 scale, uint32_t sample_count)
{
	HeightFieldShapeSettings settings(samples, ToJph(offset), ToJph(scale), sample_count);
	return CreateShape(settings);
}

JPC_Shape *JPC_PlaneShape_Create(JPC_Vec3 normal, float distance, float half_extent)
{
	PlaneShapeSettings settings(Plane(ToJph(normal), distance), nullptr, half_extent);
	return CreateShape(settings);
}

JPC_Shape *JPC_EmptyShape_Create(JPC_Vec3 center_of_mass)
{
	EmptyShapeSettings settings(ToJph(center_of_mass));
	return CreateShape(settings);
}

JPC_Shape *JPC_StaticCompoundShape_Create(const JPC_CompoundSubShape *sub_shapes, uint32_t num_sub_shapes)
{
	StaticCompoundShapeSettings settings;
	for (uint32_t i = 0; i < num_sub_shapes; ++i)
	{
		const JPC_CompoundSubShape &sub = sub_shapes[i];
		if (!CheckInner(sub.shape))
			return nullptr;
		settings.AddShape(ToJph(sub.position), ToJph(sub.rotation), ToJph(sub.shape), sub.user_data);
	}
	return CreateShape(settings);
}

JPC_Shape *JPC_RotatedTranslatedShape_Create(JPC_Vec3 position, JPC_Quat rotation, const JPC_Shape *inner)
{
	if (!CheckInner(inner))
		return nullptr;
	RotatedTranslatedShapeSettings settings(ToJph(position), ToJph(rotation), ToJph(inner));
	return CreateShape(settings);
}

JPC_Shape *JPC_ScaledShape_Create(const JPC_Shape *inner, JPC_Vec3 scale)
{
	if (!CheckInner(inner))
		return nullptr;
	ScaledShapeSettings settings(ToJph(inner), ToJph(scale));
	return CreateShape(settings);
}

JPC_Shape *JPC_OffsetCenterOfMassShape_Create(JPC_Vec3 offset, const JPC_Shape *inner)
{
	if (!CheckInner(inner))
		return nullptr;
	OffsetCenterOfMassShapeSettings settings(ToJph(offset), ToJph(inner));
	return CreateShape(settings);
}

void JPC_Shape_AddRef(const JPC_Shape *shape) { ToJph(shape)->AddRef(); }
void JPC_Shape_Release(const JPC_Shape *shape) { ToJph(shape)->Release(); }
uint32_t JPC_Shape_GetRefCount(const JPC_Shape *shape) { return ToJph(shape)->GetRefCount(); }
JPC_ShapeType JPC_Shape_GetType(const JPC_Shape *shape) { return (JPC_ShapeType)ToJph(shape)->GetType(); }
JPC_ShapeSubType JPC_Shape_GetSubType(const JPC_Shape *shape) { return (JPC_ShapeSubType)ToJph(shape)->GetSubType(); }
uint64_t JPC_Shape_GetUserData(const JPC_Shape *shape) { return ToJph(shape)->GetUserData(); }
void JPC_Shape_SetUserData(JPC_Shape *shape, uint64_t user_data) { ToJph(shape)->SetUserData(user_data); }
JPC_AABox JPC_Shape_GetLocalBounds(const JPC_Shape *shape) { return ToC(ToJph(shape)->GetLocalBounds()); }
JPC_Vec3 JPC_Shape_GetCenterOfMass(const JPC_Shape *shape) { return ToC(ToJph(shape)->GetCenterOfMass()); }
float JPC_Shape_GetInnerRadius(const JPC_Shape *shape) { return ToJph(shape)->GetInnerRadius(); }
float JPC_Shape_GetVolume(const JPC_Shape *shape) { return ToJph(shape)->GetVolume(); }
JPC_MassProperties JPC_Shape_GetMassProperties(const JPC_Shape *shape) { return ToC(ToJph(shape)->GetMassProperties()); }

void JPC_ConvexShape_SetDensity(JPC_Shape *shape, float density)
{
	Shape *s = ToJph(shape);
	if (s->GetType() == EShapeType::Convex)
		static_cast<ConvexShape *>(s)->SetDensity(density);
}

float JPC_ConvexShape_GetDensity(const JPC_Shape *shape)
{
	const Shape *s = ToJph(shape);
	return s->GetType() == EShapeType::Convex ? static_cast<const ConvexShape *>(s)->GetDensity() : 0.0f;
}


static const char *ValidateSettings(const JPC_PhysicsSystemSettings *s)
{
	if (s == nullptr)
		return "Settings are null";
	if (s->max_bodies == 0 || s->max_bodies > BodyID::cMaxBodyIndex + 1)
		return "max_bodies must be between 1 and 8388608";
	if (s->num_object_layers == 0 || s->num_object_layers > (1u << (8 * sizeof(ObjectLayer))) - 1)
		return "num_object_layers out of range";
	if (s->num_broad_phase_layers == 0 || s->num_broad_phase_layers > 255)
		return "num_broad_phase_layers out of range";
	if (s->object_to_broad_phase_layer == nullptr)
		return "object_to_broad_phase_layer is null";
	for (uint32_t i = 0; i < s->num_object_layers; ++i)
		if (s->object_to_broad_phase_layer[i] >= s->num_broad_phase_layers)
			return "object_to_broad_phase_layer maps to a broad phase layer that does not exist";
	if (s->num_colliding_pairs > 0 && s->colliding_pairs == nullptr)
		return "colliding_pairs is null";
	for (uint32_t i = 0; i < s->num_colliding_pairs; ++i)
		if (s->colliding_pairs[i].layer1 >= s->num_object_layers || s->colliding_pairs[i].layer2 >= s->num_object_layers)
			return "colliding_pairs references an object layer that does not exist";
	return nullptr;
}

JPC_PhysicsSystem *JPC_PhysicsSystem_Create(const JPC_PhysicsSystemSettings *settings)
{
	if (const char *error = ValidateSettings(settings))
	{
		JPC_SetError(error);
		return nullptr;
	}
	return new JPC_PhysicsSystem(*settings);
}

void JPC_PhysicsSystem_Destroy(JPC_PhysicsSystem *system) { delete system; }

JPC_PhysicsUpdateError JPC_PhysicsSystem_Update(JPC_PhysicsSystem *system, float delta_time, int32_t collision_steps, JPC_TempAllocator *temp_allocator, JPC_JobSystem *job_system)
{
	if (collision_steps < 1)
		collision_steps = 1;
	job_system->AcquireUpdateSlot();
	EPhysicsUpdateError error = system->mSystem.Update(delta_time, collision_steps, ToJph(temp_allocator), ToJph(job_system));
	job_system->ReleaseUpdateSlot();
	return (JPC_PhysicsUpdateError)error;
}

void JPC_PhysicsSystem_OptimizeBroadPhase(JPC_PhysicsSystem *system) { system->mSystem.OptimizeBroadPhase(); }
void JPC_PhysicsSystem_SetGravity(JPC_PhysicsSystem *system, JPC_Vec3 gravity) { system->mSystem.SetGravity(ToJph(gravity)); }
JPC_Vec3 JPC_PhysicsSystem_GetGravity(const JPC_PhysicsSystem *system) { return ToC(system->mSystem.GetGravity()); }

void JPC_PhysicsSystem_GetPhysicsSettings(const JPC_PhysicsSystem *system, JPC_PhysicsSettings *out)
{
	const PhysicsSettings &s = system->mSystem.GetPhysicsSettings();
	out->max_in_flight_body_pairs = s.mMaxInFlightBodyPairs;
	out->step_listeners_batch_size = s.mStepListenersBatchSize;
	out->step_listener_batches_per_job = s.mStepListenerBatchesPerJob;
	out->baumgarte = s.mBaumgarte;
	out->speculative_contact_distance = s.mSpeculativeContactDistance;
	out->penetration_slop = s.mPenetrationSlop;
	out->linear_cast_threshold = s.mLinearCastThreshold;
	out->linear_cast_max_penetration = s.mLinearCastMaxPenetration;
	out->manifold_tolerance = s.mManifoldTolerance;
	out->max_penetration_distance = s.mMaxPenetrationDistance;
	out->body_pair_cache_max_delta_position_sq = s.mBodyPairCacheMaxDeltaPositionSq;
	out->body_pair_cache_cos_max_delta_rotation_div2 = s.mBodyPairCacheCosMaxDeltaRotationDiv2;
	out->contact_normal_cos_max_delta_rotation = s.mContactNormalCosMaxDeltaRotation;
	out->contact_point_preserve_lambda_max_dist_sq = s.mContactPointPreserveLambdaMaxDistSq;
	out->num_velocity_steps = s.mNumVelocitySteps;
	out->num_position_steps = s.mNumPositionSteps;
	out->min_velocity_for_restitution = s.mMinVelocityForRestitution;
	out->time_before_sleep = s.mTimeBeforeSleep;
	out->point_velocity_sleep_threshold = s.mPointVelocitySleepThreshold;
	out->deterministic_simulation = s.mDeterministicSimulation;
	out->constraint_warm_start = s.mConstraintWarmStart;
	out->use_body_pair_contact_cache = s.mUseBodyPairContactCache;
	out->use_manifold_reduction = s.mUseManifoldReduction;
	out->use_large_island_splitter = s.mUseLargeIslandSplitter;
	out->allow_sleeping = s.mAllowSleeping;
	out->check_active_edges = s.mCheckActiveEdges;
}

static const char *ValidatePhysicsSettings(const JPC_PhysicsSettings &in)
{
	// Zero batch sizes divide by zero or spin forever in Update; too few body pairs gives every queue zero capacity
	if (in.step_listeners_batch_size < 1 || in.step_listener_batches_per_job < 1)
		return "step_listeners_batch_size and step_listener_batches_per_job must be at least 1";
	if (in.max_in_flight_body_pairs < 256)
		return "max_in_flight_body_pairs must be at least 256";
	if (in.num_velocity_steps < 1)
		return "num_velocity_steps must be at least 1";
	return nullptr;
}

bool JPC_PhysicsSystem_SetPhysicsSettings(JPC_PhysicsSystem *system, const JPC_PhysicsSettings *in)
{
	if (const char *error = ValidatePhysicsSettings(*in))
	{
		JPC_SetError(error);
		return false;
	}

	PhysicsSettings s = system->mSystem.GetPhysicsSettings();
	s.mMaxInFlightBodyPairs = in->max_in_flight_body_pairs;
	s.mStepListenersBatchSize = in->step_listeners_batch_size;
	s.mStepListenerBatchesPerJob = in->step_listener_batches_per_job;
	s.mBaumgarte = in->baumgarte;
	s.mSpeculativeContactDistance = in->speculative_contact_distance;
	s.mPenetrationSlop = in->penetration_slop;
	s.mLinearCastThreshold = in->linear_cast_threshold;
	s.mLinearCastMaxPenetration = in->linear_cast_max_penetration;
	s.mManifoldTolerance = in->manifold_tolerance;
	s.mMaxPenetrationDistance = in->max_penetration_distance;
	s.mBodyPairCacheMaxDeltaPositionSq = in->body_pair_cache_max_delta_position_sq;
	s.mBodyPairCacheCosMaxDeltaRotationDiv2 = in->body_pair_cache_cos_max_delta_rotation_div2;
	s.mContactNormalCosMaxDeltaRotation = in->contact_normal_cos_max_delta_rotation;
	s.mContactPointPreserveLambdaMaxDistSq = in->contact_point_preserve_lambda_max_dist_sq;
	s.mNumVelocitySteps = in->num_velocity_steps;
	s.mNumPositionSteps = in->num_position_steps;
	s.mMinVelocityForRestitution = in->min_velocity_for_restitution;
	s.mTimeBeforeSleep = in->time_before_sleep;
	s.mPointVelocitySleepThreshold = in->point_velocity_sleep_threshold;
	s.mDeterministicSimulation = in->deterministic_simulation;
	s.mConstraintWarmStart = in->constraint_warm_start;
	s.mUseBodyPairContactCache = in->use_body_pair_contact_cache;
	s.mUseManifoldReduction = in->use_manifold_reduction;
	s.mUseLargeIslandSplitter = in->use_large_island_splitter;
	s.mAllowSleeping = in->allow_sleeping;
	s.mCheckActiveEdges = in->check_active_edges;
	system->mSystem.SetPhysicsSettings(s);
	return true;
}

uint32_t JPC_PhysicsSystem_GetNumBodies(const JPC_PhysicsSystem *system) { return system->mSystem.GetNumBodies(); }
uint32_t JPC_PhysicsSystem_GetNumActiveBodies(const JPC_PhysicsSystem *system) { return system->mSystem.GetNumActiveBodies(EBodyType::RigidBody); }
uint32_t JPC_PhysicsSystem_GetMaxBodies(const JPC_PhysicsSystem *system) { return system->mSystem.GetMaxBodies(); }

static uint32_t CopyIDs(const BodyIDVector &inIDs, JPC_BodyID *outIDs, uint32_t inCapacity)
{
	uint32_t count = (uint32_t)inIDs.size();
	for (uint32_t i = 0; i < count && i < inCapacity; ++i)
		outIDs[i] = ToC(inIDs[i]);
	return count;
}

uint32_t JPC_PhysicsSystem_GetBodies(const JPC_PhysicsSystem *system, JPC_BodyID *out_ids, uint32_t capacity)
{
	BodyIDVector ids;
	system->mSystem.GetBodies(ids);
	return CopyIDs(ids, out_ids, capacity);
}

uint32_t JPC_PhysicsSystem_GetActiveBodies(const JPC_PhysicsSystem *system, JPC_BodyID *out_ids, uint32_t capacity)
{
	BodyIDVector ids;
	system->mSystem.GetActiveBodies(EBodyType::RigidBody, ids);
	return CopyIDs(ids, out_ids, capacity);
}

JPC_BodyInterface *JPC_PhysicsSystem_GetBodyInterface(JPC_PhysicsSystem *system) { return &system->mBodyInterface; }
JPC_BodyInterface *JPC_PhysicsSystem_GetBodyInterfaceNoLock(JPC_PhysicsSystem *system) { return &system->mBodyInterfaceNoLock; }

void JPC_PhysicsSystem_AddConstraint(JPC_PhysicsSystem *system, JPC_Constraint *constraint)
{
	Constraint *c = ToJph(constraint);
	system->mSystem.AddConstraint(c);
	if (c->GetSubType() == EConstraintSubType::Vehicle)
		system->AddStepListener(static_cast<VehicleConstraint *>(c));
	system->BumpStructure();
}

void JPC_PhysicsSystem_RemoveConstraint(JPC_PhysicsSystem *system, JPC_Constraint *constraint)
{
	Constraint *c = ToJph(constraint);
	if (c->GetSubType() == EConstraintSubType::Vehicle)
		system->RemoveStepListener(static_cast<VehicleConstraint *>(c));
	system->mSystem.RemoveConstraint(c);
	system->BumpStructure();
}

void JPC_PhysicsSystem_SetContactListener(JPC_PhysicsSystem *system, const JPC_ContactListenerFns *fns, void *user_data)
{
	if (fns == nullptr)
	{
		system->mSystem.SetContactListener(nullptr);
		system->mContactListener.mFns = { };
		system->mContactListener.mUserData = nullptr;
		return;
	}
	system->mContactListener.mFns = *fns;
	system->mContactListener.mUserData = user_data;
	system->mSystem.SetContactListener(&system->mContactListener);
}

void JPC_PhysicsSystem_SetBodyActivationListener(JPC_PhysicsSystem *system, const JPC_BodyActivationListenerFns *fns, void *user_data)
{
	if (fns == nullptr)
	{
		system->mSystem.SetBodyActivationListener(nullptr);
		system->mActivationListener.mFns = { };
		system->mActivationListener.mUserData = nullptr;
		return;
	}
	system->mActivationListener.mFns = *fns;
	system->mActivationListener.mUserData = user_data;
	system->mSystem.SetBodyActivationListener(&system->mActivationListener);
}

// ---------------------------------------------------------------------------
// Bodies
// ---------------------------------------------------------------------------

void JPC_BodyCreationSettings_SetDefault(JPC_BodyCreationSettings *out)
{
	BodyCreationSettings s;
	out->position = ToC(s.mPosition);
	out->rotation = ToC(s.mRotation);
	out->linear_velocity = ToC(s.mLinearVelocity);
	out->angular_velocity = ToC(s.mAngularVelocity);
	out->user_data = s.mUserData;
	out->object_layer = s.mObjectLayer;
	out->motion_type = (JPC_MotionType)s.mMotionType;
	out->allowed_dofs = (JPC_AllowedDOFs)s.mAllowedDOFs;
	out->allow_dynamic_or_kinematic = s.mAllowDynamicOrKinematic;
	out->is_sensor = s.mIsSensor;
	out->collide_kinematic_vs_non_dynamic = s.mCollideKinematicVsNonDynamic;
	out->use_manifold_reduction = s.mUseManifoldReduction;
	out->apply_gyroscopic_force = s.mApplyGyroscopicForce;
	out->motion_quality = (JPC_MotionQuality)s.mMotionQuality;
	out->enhanced_internal_edge_removal = s.mEnhancedInternalEdgeRemoval;
	out->allow_sleeping = s.mAllowSleeping;
	out->friction = s.mFriction;
	out->restitution = s.mRestitution;
	out->linear_damping = s.mLinearDamping;
	out->angular_damping = s.mAngularDamping;
	out->max_linear_velocity = s.mMaxLinearVelocity;
	out->max_angular_velocity = s.mMaxAngularVelocity;
	out->gravity_factor = s.mGravityFactor;
	out->num_velocity_steps_override = s.mNumVelocityStepsOverride;
	out->num_position_steps_override = s.mNumPositionStepsOverride;
	out->override_mass_properties = (JPC_OverrideMassProperties)s.mOverrideMassProperties;
	out->inertia_multiplier = s.mInertiaMultiplier;
	out->mass_properties_override = ToC(s.mMassPropertiesOverride);
	out->collision_group = ToC(s.mCollisionGroup);
	out->shape = nullptr;
}

static BodyCreationSettings ToJph(const JPC_BodyCreationSettings &in)
{
	BodyCreationSettings s;
	s.mPosition = ToJph(in.position);
	s.mRotation = ToJph(in.rotation);
	s.mLinearVelocity = ToJph(in.linear_velocity);
	s.mAngularVelocity = ToJph(in.angular_velocity);
	s.mUserData = in.user_data;
	s.mObjectLayer = in.object_layer;
	s.mMotionType = (EMotionType)in.motion_type;
	s.mAllowedDOFs = (EAllowedDOFs)in.allowed_dofs;
	s.mAllowDynamicOrKinematic = in.allow_dynamic_or_kinematic;
	s.mIsSensor = in.is_sensor;
	s.mCollideKinematicVsNonDynamic = in.collide_kinematic_vs_non_dynamic;
	s.mUseManifoldReduction = in.use_manifold_reduction;
	s.mApplyGyroscopicForce = in.apply_gyroscopic_force;
	s.mMotionQuality = (EMotionQuality)in.motion_quality;
	s.mEnhancedInternalEdgeRemoval = in.enhanced_internal_edge_removal;
	s.mAllowSleeping = in.allow_sleeping;
	s.mFriction = in.friction;
	s.mRestitution = in.restitution;
	s.mLinearDamping = in.linear_damping;
	s.mAngularDamping = in.angular_damping;
	s.mMaxLinearVelocity = in.max_linear_velocity;
	s.mMaxAngularVelocity = in.max_angular_velocity;
	s.mGravityFactor = in.gravity_factor;
	s.mNumVelocityStepsOverride = in.num_velocity_steps_override;
	s.mNumPositionStepsOverride = in.num_position_steps_override;
	s.mOverrideMassProperties = (EOverrideMassProperties)in.override_mass_properties;
	s.mInertiaMultiplier = in.inertia_multiplier;
	s.mMassPropertiesOverride = ToJph(in.mass_properties_override);
	s.mCollisionGroup = ToJph(in.collision_group);
	s.SetShape(ToJph(in.shape));
	return s;
}

static bool CheckBodySettings(const JPC_BodyInterface *bi, const JPC_BodyCreationSettings *settings)
{
	if (!bi->mOwner->IsValidLayer(settings->object_layer))
	{
		JPC_SetError("Object layer does not exist in this physics system");
		return false;
	}
	if (settings->shape == nullptr)
	{
		JPC_SetError("Body creation settings have no shape");
		return false;
	}
	if (settings->allowed_dofs == JPC_ALLOWED_DOFS_NONE && settings->motion_type != JPC_MOTION_TYPE_STATIC)
	{
		JPC_SetError("allowed_dofs must not be NONE for a non-static body");
		return false;
	}
	return true;
}

JPC_BodyID JPC_BodyInterface_CreateBody(JPC_BodyInterface *bi, const JPC_BodyCreationSettings *settings)
{
	if (!CheckBodySettings(bi, settings))
		return JPC_BODY_ID_INVALID;
	Body *body = ToJph(bi)->CreateBody(ToJph(*settings));
	if (body == nullptr)
	{
		JPC_SetError("Out of bodies (max_bodies reached)");
		return JPC_BODY_ID_INVALID;
	}
	return ToC(body->GetID());
}

JPC_BodyID JPC_BodyInterface_CreateAndAddBody(JPC_BodyInterface *bi, const JPC_BodyCreationSettings *settings, JPC_Activation activation)
{
	if (!CheckBodySettings(bi, settings))
		return JPC_BODY_ID_INVALID;
	BodyID id = ToJph(bi)->CreateAndAddBody(ToJph(*settings), ToJph(activation));
	if (id.IsInvalid())
		JPC_SetError("Out of bodies (max_bodies reached)");
	bi->mOwner->BumpStructure();
	return ToC(id);
}

// Jolt only guards these with asserts, so check the body first to turn misuse into a no-op instead of a crash
static bool IsMovable(const JPC_BodyInterface *bi, JPC_BodyID id)
{
	BodyLockRead lock(*bi->mLockInterface, ToJph(id));
	return lock.Succeeded() && !lock.GetBody().IsStatic();
}

static bool HasMotionProperties(const JPC_BodyInterface *bi, JPC_BodyID id)
{
	BodyLockRead lock(*bi->mLockInterface, ToJph(id));
	return lock.Succeeded() && lock.GetBody().GetMotionPropertiesUnchecked() != nullptr;
}

void JPC_BodyInterface_DestroyBody(JPC_BodyInterface *bi, JPC_BodyID id)
{
	BodyID body_id = ToJph(id);
	std::unique_lock lock(bi->mOwner->mBodyStateMutex);
	if (bi->mOwner->mProtectedBodies.find(id) != bi->mOwner->mProtectedBodies.end())
	{
		JPC_SetError("Body is owned by a character; destroy the character instead");
		return;
	}
	{
		// Jolt's DestroyBody does not check the sequence number, so a stale id would free whatever body reuses the slot
		BodyLockRead body_lock(*bi->mLockInterface, body_id);
		if (!body_lock.Succeeded())
			return;
	}
	ToJph(bi)->RemoveBody(body_id);
	ToJph(bi)->DestroyBody(body_id);
	bi->mOwner->BumpStructure();
}

void JPC_BodyInterface_AddBody(JPC_BodyInterface *bi, JPC_BodyID id, JPC_Activation activation)
{
	std::unique_lock lock(bi->mOwner->mBodyStateMutex);
	if (!ToJph(bi)->IsAdded(ToJph(id)))
	{
		ToJph(bi)->AddBody(ToJph(id), ToJph(activation));
		bi->mOwner->BumpStructure();
	}
}

void JPC_BodyInterface_RemoveBody(JPC_BodyInterface *bi, JPC_BodyID id)
{
	std::unique_lock lock(bi->mOwner->mBodyStateMutex);
	ToJph(bi)->RemoveBody(ToJph(id));
	bi->mOwner->BumpStructure();
}

// Collects valid, unique ids whose "added" state equals inAdded
static Array<BodyID> FilterBodies(const JPC_BodyInterface *bi, const JPC_BodyID *inIDs, uint32_t inCount, bool inAdded)
{
	Array<BodyID> out;
	out.reserve(inCount);
	UnorderedSet<uint32> seen;
	for (uint32_t i = 0; i < inCount; ++i)
	{
		if (!seen.insert(inIDs[i]).second)
			continue;
		BodyLockRead lock(*bi->mLockInterface, ToJph(inIDs[i]));
		if (lock.Succeeded() && lock.GetBody().IsInBroadPhase() == inAdded)
			out.push_back(ToJph(inIDs[i]));
	}
	return out;
}

void JPC_BodyInterface_AddBodies(JPC_BodyInterface *bi, const JPC_BodyID *ids, uint32_t count, JPC_Activation activation)
{
	std::unique_lock lock(bi->mOwner->mBodyStateMutex);
	Array<BodyID> bodies = FilterBodies(bi, ids, count, false);
	if (bodies.empty())
		return;
	BodyInterface::AddState state = ToJph(bi)->AddBodiesPrepare(bodies.data(), (int)bodies.size());
	ToJph(bi)->AddBodiesFinalize(bodies.data(), (int)bodies.size(), state, ToJph(activation));
	bi->mOwner->BumpStructure();
}

void JPC_BodyInterface_RemoveBodies(JPC_BodyInterface *bi, const JPC_BodyID *ids, uint32_t count)
{
	std::unique_lock lock(bi->mOwner->mBodyStateMutex);
	Array<BodyID> bodies = FilterBodies(bi, ids, count, true);
	if (!bodies.empty())
	{
		ToJph(bi)->RemoveBodies(bodies.data(), (int)bodies.size());
		bi->mOwner->BumpStructure();
	}
}

// For calls whose Jolt implementation activates the body without checking that it is in the world.
// Must be called with mBodyStateMutex held (shared).
static bool IsAddedAndMovable(const JPC_BodyInterface *bi, JPC_BodyID id)
{
	BodyLockRead lock(*bi->mLockInterface, ToJph(id));
	return lock.Succeeded() && lock.GetBody().IsInBroadPhase() && !lock.GetBody().IsStatic();
}

bool JPC_BodyInterface_IsAdded(const JPC_BodyInterface *bi, JPC_BodyID id) { return ToJph(bi)->IsAdded(ToJph(id)); }

void JPC_BodyInterface_ActivateBody(JPC_BodyInterface *bi, JPC_BodyID id)
{
	std::shared_lock lock(bi->mOwner->mBodyStateMutex);
	if (IsAddedAndMovable(bi, id))
		ToJph(bi)->ActivateBody(ToJph(id));
}
void JPC_BodyInterface_DeactivateBody(JPC_BodyInterface *bi, JPC_BodyID id) { ToJph(bi)->DeactivateBody(ToJph(id)); }
bool JPC_BodyInterface_IsActive(const JPC_BodyInterface *bi, JPC_BodyID id) { return ToJph(bi)->IsActive(ToJph(id)); }
void JPC_BodyInterface_ResetSleepTimer(JPC_BodyInterface *bi, JPC_BodyID id) { if (IsMovable(bi, id)) ToJph(bi)->ResetSleepTimer(ToJph(id)); }

JPC_Shape *JPC_BodyInterface_GetShape(const JPC_BodyInterface *bi, JPC_BodyID id)
{
	RefConst<Shape> shape = ToJph(bi)->GetShape(ToJph(id));
	if (shape == nullptr)
		return nullptr;
	shape->AddRef();
	return ToC(const_cast<Shape *>(shape.GetPtr()));
}

void JPC_BodyInterface_SetShape(JPC_BodyInterface *bi, JPC_BodyID id, const JPC_Shape *shape, bool update_mass_properties, JPC_Activation activation)
{
	ToJph(bi)->SetShape(ToJph(id), ToJph(shape), update_mass_properties, ToJph(activation));
	bi->mOwner->BumpStructure();
}

JPC_ObjectLayer JPC_BodyInterface_GetObjectLayer(const JPC_BodyInterface *bi, JPC_BodyID id) { return ToJph(bi)->GetObjectLayer(ToJph(id)); }
bool JPC_BodyInterface_SetObjectLayer(JPC_BodyInterface *bi, JPC_BodyID id, JPC_ObjectLayer layer)
{
	if (!bi->mOwner->IsValidLayer(layer))
	{
		JPC_SetError("Object layer does not exist in this physics system");
		return false;
	}
	ToJph(bi)->SetObjectLayer(ToJph(id), layer);
	return true;
}

JPC_RVec3 JPC_BodyInterface_GetPosition(const JPC_BodyInterface *bi, JPC_BodyID id) { return ToC(ToJph(bi)->GetPosition(ToJph(id))); }
void JPC_BodyInterface_SetPosition(JPC_BodyInterface *bi, JPC_BodyID id, JPC_RVec3 position, JPC_Activation activation) { ToJph(bi)->SetPosition(ToJph(id), ToJph(position), ToJph(activation)); }
JPC_RVec3 JPC_BodyInterface_GetCenterOfMassPosition(const JPC_BodyInterface *bi, JPC_BodyID id) { return ToC(ToJph(bi)->GetCenterOfMassPosition(ToJph(id))); }
JPC_Quat JPC_BodyInterface_GetRotation(const JPC_BodyInterface *bi, JPC_BodyID id) { return ToC(ToJph(bi)->GetRotation(ToJph(id))); }
void JPC_BodyInterface_SetRotation(JPC_BodyInterface *bi, JPC_BodyID id, JPC_Quat rotation, JPC_Activation activation) { ToJph(bi)->SetRotation(ToJph(id), ToJph(rotation), ToJph(activation)); }

void JPC_BodyInterface_SetPositionAndRotation(JPC_BodyInterface *bi, JPC_BodyID id, JPC_RVec3 position, JPC_Quat rotation, JPC_Activation activation)
{
	ToJph(bi)->SetPositionAndRotation(ToJph(id), ToJph(position), ToJph(rotation), ToJph(activation));
}

void JPC_BodyInterface_SetPositionAndRotationWhenChanged(JPC_BodyInterface *bi, JPC_BodyID id, JPC_RVec3 position, JPC_Quat rotation, JPC_Activation activation)
{
	ToJph(bi)->SetPositionAndRotationWhenChanged(ToJph(id), ToJph(position), ToJph(rotation), ToJph(activation));
}

void JPC_BodyInterface_GetPositionAndRotation(const JPC_BodyInterface *bi, JPC_BodyID id, JPC_RVec3 *out_position, JPC_Quat *out_rotation)
{
	RVec3 position;
	Quat rotation;
	ToJph(bi)->GetPositionAndRotation(ToJph(id), position, rotation);
	*out_position = ToC(position);
	*out_rotation = ToC(rotation);
}

JPC_RMat44 JPC_BodyInterface_GetWorldTransform(const JPC_BodyInterface *bi, JPC_BodyID id) { return ToC(ToJph(bi)->GetWorldTransform(ToJph(id))); }
JPC_RMat44 JPC_BodyInterface_GetCenterOfMassTransform(const JPC_BodyInterface *bi, JPC_BodyID id) { return ToC(ToJph(bi)->GetCenterOfMassTransform(ToJph(id))); }

void JPC_BodyInterface_MoveKinematic(JPC_BodyInterface *bi, JPC_BodyID id, JPC_RVec3 target_position, JPC_Quat target_rotation, float delta_time)
{
	if (IsMovable(bi, id))
		ToJph(bi)->MoveKinematic(ToJph(id), ToJph(target_position), ToJph(target_rotation), delta_time);
}

JPC_Vec3 JPC_BodyInterface_GetLinearVelocity(const JPC_BodyInterface *bi, JPC_BodyID id) { return ToC(ToJph(bi)->GetLinearVelocity(ToJph(id))); }
void JPC_BodyInterface_SetLinearVelocity(JPC_BodyInterface *bi, JPC_BodyID id, JPC_Vec3 velocity) { ToJph(bi)->SetLinearVelocity(ToJph(id), ToJph(velocity)); }
void JPC_BodyInterface_AddLinearVelocity(JPC_BodyInterface *bi, JPC_BodyID id, JPC_Vec3 velocity) { ToJph(bi)->AddLinearVelocity(ToJph(id), ToJph(velocity)); }
JPC_Vec3 JPC_BodyInterface_GetAngularVelocity(const JPC_BodyInterface *bi, JPC_BodyID id) { return ToC(ToJph(bi)->GetAngularVelocity(ToJph(id))); }
void JPC_BodyInterface_SetAngularVelocity(JPC_BodyInterface *bi, JPC_BodyID id, JPC_Vec3 velocity) { ToJph(bi)->SetAngularVelocity(ToJph(id), ToJph(velocity)); }
void JPC_BodyInterface_SetLinearAndAngularVelocity(JPC_BodyInterface *bi, JPC_BodyID id, JPC_Vec3 linear, JPC_Vec3 angular) { ToJph(bi)->SetLinearAndAngularVelocity(ToJph(id), ToJph(linear), ToJph(angular)); }
JPC_Vec3 JPC_BodyInterface_GetPointVelocity(const JPC_BodyInterface *bi, JPC_BodyID id, JPC_RVec3 point) { return ToC(ToJph(bi)->GetPointVelocity(ToJph(id), ToJph(point))); }

// Forces and impulses on bodies outside the world are dropped: Jolt would activate them, which corrupts the active list.
#define JPC_GUARD_ACTIVATION(bi, id) \
	std::shared_lock body_state_lock(bi->mOwner->mBodyStateMutex); \
	if (!IsAddedAndMovable(bi, id)) \
		return

void JPC_BodyInterface_AddForce(JPC_BodyInterface *bi, JPC_BodyID id, JPC_Vec3 force, JPC_Activation activation)
{
	JPC_GUARD_ACTIVATION(bi, id);
	ToJph(bi)->AddForce(ToJph(id), ToJph(force), ToJph(activation));
}

void JPC_BodyInterface_AddForceAtPoint(JPC_BodyInterface *bi, JPC_BodyID id, JPC_Vec3 force, JPC_RVec3 point, JPC_Activation activation)
{
	JPC_GUARD_ACTIVATION(bi, id);
	ToJph(bi)->AddForce(ToJph(id), ToJph(force), ToJph(point), ToJph(activation));
}

void JPC_BodyInterface_AddTorque(JPC_BodyInterface *bi, JPC_BodyID id, JPC_Vec3 torque, JPC_Activation activation)
{
	JPC_GUARD_ACTIVATION(bi, id);
	ToJph(bi)->AddTorque(ToJph(id), ToJph(torque), ToJph(activation));
}

void JPC_BodyInterface_AddImpulse(JPC_BodyInterface *bi, JPC_BodyID id, JPC_Vec3 impulse)
{
	JPC_GUARD_ACTIVATION(bi, id);
	ToJph(bi)->AddImpulse(ToJph(id), ToJph(impulse));
}

void JPC_BodyInterface_AddImpulseAtPoint(JPC_BodyInterface *bi, JPC_BodyID id, JPC_Vec3 impulse, JPC_RVec3 point)
{
	JPC_GUARD_ACTIVATION(bi, id);
	ToJph(bi)->AddImpulse(ToJph(id), ToJph(impulse), ToJph(point));
}

void JPC_BodyInterface_AddAngularImpulse(JPC_BodyInterface *bi, JPC_BodyID id, JPC_Vec3 angular_impulse)
{
	JPC_GUARD_ACTIVATION(bi, id);
	ToJph(bi)->AddAngularImpulse(ToJph(id), ToJph(angular_impulse));
}

bool JPC_BodyInterface_ApplyBuoyancyImpulse(JPC_BodyInterface *bi, JPC_BodyID id, JPC_RVec3 surface_position, JPC_Vec3 surface_normal, float buoyancy,
											float linear_drag, float angular_drag, JPC_Vec3 fluid_velocity, JPC_Vec3 gravity, float delta_time)
{
	std::shared_lock body_state_lock(bi->mOwner->mBodyStateMutex);
	if (!IsAddedAndMovable(bi, id))
		return false;
	return ToJph(bi)->ApplyBuoyancyImpulse(ToJph(id), ToJph(surface_position), ToJph(surface_normal), buoyancy, linear_drag, angular_drag,
										   ToJph(fluid_velocity), ToJph(gravity), delta_time);
}

void JPC_BodyInterface_GetCollisionGroup(const JPC_BodyInterface *bi, JPC_BodyID id, JPC_CollisionGroup *out_group)
{
	BodyLockRead lock(*bi->mLockInterface, ToJph(id));
	*out_group = lock.Succeeded() ? ToC(lock.GetBody().GetCollisionGroup()) : ToC(CollisionGroup());
	// The body may drop its filter as soon as the lock is released, so the caller gets its own reference
	if (out_group->group_filter != nullptr)
		ToJph(out_group->group_filter)->AddRef();
}

void JPC_BodyInterface_SetCollisionGroup(JPC_BodyInterface *bi, JPC_BodyID id, const JPC_CollisionGroup *group)
{
	ToJph(bi)->SetCollisionGroup(ToJph(id), ToJph(*group));
}

JPC_AABox JPC_BodyInterface_GetWorldSpaceBounds(const JPC_BodyInterface *bi, JPC_BodyID id)
{
	BodyLockRead lock(*bi->mLockInterface, ToJph(id));
	return lock.Succeeded() ? ToC(lock.GetBody().GetWorldSpaceBounds()) : JPC_AABox { };
}

float JPC_BodyInterface_GetInverseMass(const JPC_BodyInterface *bi, JPC_BodyID id)
{
	BodyLockRead lock(*bi->mLockInterface, ToJph(id));
	if (!lock.Succeeded() || lock.GetBody().GetMotionPropertiesUnchecked() == nullptr || !lock.GetBody().IsDynamic())
		return 0.0f;
	return lock.GetBody().GetMotionPropertiesUnchecked()->GetInverseMassUnchecked();
}

JPC_Mat44 JPC_BodyInterface_GetInverseInertia(const JPC_BodyInterface *bi, JPC_BodyID id) { return ToC(ToJph(bi)->GetInverseInertia(ToJph(id))); }
float JPC_BodyInterface_GetMaxLinearVelocity(const JPC_BodyInterface *bi, JPC_BodyID id) { return ToJph(bi)->GetMaxLinearVelocity(ToJph(id)); }
void JPC_BodyInterface_SetMaxLinearVelocity(JPC_BodyInterface *bi, JPC_BodyID id, float velocity) { ToJph(bi)->SetMaxLinearVelocity(ToJph(id), velocity); }
float JPC_BodyInterface_GetMaxAngularVelocity(const JPC_BodyInterface *bi, JPC_BodyID id) { return ToJph(bi)->GetMaxAngularVelocity(ToJph(id)); }
void JPC_BodyInterface_SetMaxAngularVelocity(JPC_BodyInterface *bi, JPC_BodyID id, float velocity) { ToJph(bi)->SetMaxAngularVelocity(ToJph(id), velocity); }
bool JPC_BodyInterface_GetUseManifoldReduction(const JPC_BodyInterface *bi, JPC_BodyID id) { return ToJph(bi)->GetUseManifoldReduction(ToJph(id)); }
void JPC_BodyInterface_SetUseManifoldReduction(JPC_BodyInterface *bi, JPC_BodyID id, bool use) { ToJph(bi)->SetUseManifoldReduction(ToJph(id), use); }

JPC_MotionType JPC_BodyInterface_GetMotionType(const JPC_BodyInterface *bi, JPC_BodyID id) { return (JPC_MotionType)ToJph(bi)->GetMotionType(ToJph(id)); }
bool JPC_BodyInterface_SetMotionType(JPC_BodyInterface *bi, JPC_BodyID id, JPC_MotionType motion_type, JPC_Activation activation)
{
	if (motion_type != JPC_MOTION_TYPE_STATIC && !HasMotionProperties(bi, id))
	{
		JPC_SetError("Body must be created with allow_dynamic_or_kinematic to become kinematic or dynamic");
		return false;
	}
	ToJph(bi)->SetMotionType(ToJph(id), (EMotionType)motion_type, ToJph(activation));
	bi->mOwner->BumpStructure();
	return true;
}
JPC_MotionQuality JPC_BodyInterface_GetMotionQuality(const JPC_BodyInterface *bi, JPC_BodyID id) { return (JPC_MotionQuality)ToJph(bi)->GetMotionQuality(ToJph(id)); }
void JPC_BodyInterface_SetMotionQuality(JPC_BodyInterface *bi, JPC_BodyID id, JPC_MotionQuality quality) { if (HasMotionProperties(bi, id)) ToJph(bi)->SetMotionQuality(ToJph(id), (EMotionQuality)quality); }

float JPC_BodyInterface_GetFriction(const JPC_BodyInterface *bi, JPC_BodyID id) { return ToJph(bi)->GetFriction(ToJph(id)); }
void JPC_BodyInterface_SetFriction(JPC_BodyInterface *bi, JPC_BodyID id, float friction) { ToJph(bi)->SetFriction(ToJph(id), friction); }
float JPC_BodyInterface_GetRestitution(const JPC_BodyInterface *bi, JPC_BodyID id) { return ToJph(bi)->GetRestitution(ToJph(id)); }
void JPC_BodyInterface_SetRestitution(JPC_BodyInterface *bi, JPC_BodyID id, float restitution) { ToJph(bi)->SetRestitution(ToJph(id), restitution); }
float JPC_BodyInterface_GetGravityFactor(const JPC_BodyInterface *bi, JPC_BodyID id) { return ToJph(bi)->GetGravityFactor(ToJph(id)); }
void JPC_BodyInterface_SetGravityFactor(JPC_BodyInterface *bi, JPC_BodyID id, float factor) { ToJph(bi)->SetGravityFactor(ToJph(id), factor); }
bool JPC_BodyInterface_IsSensor(const JPC_BodyInterface *bi, JPC_BodyID id) { return ToJph(bi)->IsSensor(ToJph(id)); }
void JPC_BodyInterface_SetIsSensor(JPC_BodyInterface *bi, JPC_BodyID id, bool is_sensor) { ToJph(bi)->SetIsSensor(ToJph(id), is_sensor); }
uint64_t JPC_BodyInterface_GetUserData(const JPC_BodyInterface *bi, JPC_BodyID id) { return ToJph(bi)->GetUserData(ToJph(id)); }
void JPC_BodyInterface_SetUserData(JPC_BodyInterface *bi, JPC_BodyID id, uint64_t user_data) { ToJph(bi)->SetUserData(ToJph(id), user_data); }
void JPC_BodyInterface_InvalidateContactCache(JPC_BodyInterface *bi, JPC_BodyID id) { ToJph(bi)->InvalidateContactCache(ToJph(id)); }

JPC_BodyID JPC_Body_GetID(const JPC_Body *body) { return ToC(ToJph(body)->GetID()); }
uint64_t JPC_Body_GetUserData(const JPC_Body *body) { return ToJph(body)->GetUserData(); }
JPC_ObjectLayer JPC_Body_GetObjectLayer(const JPC_Body *body) { return ToJph(body)->GetObjectLayer(); }
JPC_MotionType JPC_Body_GetMotionType(const JPC_Body *body) { return (JPC_MotionType)ToJph(body)->GetMotionType(); }
bool JPC_Body_IsSensor(const JPC_Body *body) { return ToJph(body)->IsSensor(); }
bool JPC_Body_IsActive(const JPC_Body *body) { return ToJph(body)->IsActive(); }
JPC_RVec3 JPC_Body_GetPosition(const JPC_Body *body) { return ToC(ToJph(body)->GetPosition()); }
JPC_Quat JPC_Body_GetRotation(const JPC_Body *body) { return ToC(ToJph(body)->GetRotation()); }
JPC_RVec3 JPC_Body_GetCenterOfMassPosition(const JPC_Body *body) { return ToC(ToJph(body)->GetCenterOfMassPosition()); }
JPC_Vec3 JPC_Body_GetLinearVelocity(const JPC_Body *body) { return ToC(ToJph(body)->GetLinearVelocity()); }
JPC_Vec3 JPC_Body_GetAngularVelocity(const JPC_Body *body) { return ToC(ToJph(body)->GetAngularVelocity()); }
float JPC_Body_GetFriction(const JPC_Body *body) { return ToJph(body)->GetFriction(); }
float JPC_Body_GetRestitution(const JPC_Body *body) { return ToJph(body)->GetRestitution(); }
const JPC_Shape *JPC_Body_GetShape(const JPC_Body *body) { return reinterpret_cast<const JPC_Shape *>(ToJph(body)->GetShape()); }
JPC_AABox JPC_Body_GetWorldSpaceBounds(const JPC_Body *body) { return ToC(ToJph(body)->GetWorldSpaceBounds()); }

// ---------------------------------------------------------------------------
// Queries
// ---------------------------------------------------------------------------

void JPC_QueryFilter_SetDefault(JPC_QueryFilter *filter)
{
	filter->use_object_layer = false;
	filter->object_layer = 0;
	filter->ignore_body = JPC_BODY_ID_INVALID;
	filter->body_filter = nullptr;
	filter->body_filter_user_data = nullptr;
}

void JPC_RayCastSettings_SetDefault(JPC_RayCastSettings *out)
{
	RayCastSettings s;
	out->back_face_mode_triangles = (JPC_BackFaceMode)s.mBackFaceModeTriangles;
	out->back_face_mode_convex = (JPC_BackFaceMode)s.mBackFaceModeConvex;
	out->treat_convex_as_solid = s.mTreatConvexAsSolid;
}

namespace {

JPC_RayCastHit MakeRayHit(const CastRayCollector &inCollector, const RRayCast &inRay, const RayCastResult &inResult)
{
	RVec3 point = inRay.GetPointOnRay(inResult.mFraction);
	JPC_RayCastHit hit;
	hit.body_id = ToC(inResult.mBodyID);
	hit.sub_shape_id = ToC(inResult.mSubShapeID2);
	hit.fraction = inResult.mFraction;
	hit.point = ToC(point);
	const TransformedShape *ts = inCollector.GetContext();
	hit.normal = ts != nullptr ? ToC(ts->GetWorldSpaceSurfaceNormal(inResult.mSubShapeID2, point)) : JPC_Vec3 { 0, 0, 0 };
	hit.material = ts != nullptr ? ToC(ts->GetMaterial(inResult.mSubShapeID2)) : nullptr;
	return hit;
}

class ClosestRayCollector final : public CastRayCollector
{
public:
	explicit ClosestRayCollector(const RRayCast &inRay) : mRay(inRay) { }

	virtual void			AddHit(const RayCastResult &inResult) override
	{
		if (inResult.mFraction < GetEarlyOutFraction())
		{
			UpdateEarlyOutFraction(inResult.mFraction);
			mHit = MakeRayHit(*this, mRay, inResult);
			// The hit body is locked right now; after the query its shape (and material) may be freed, so keep a reference
			mMaterial = ToJph(mHit.material);
			mHasHit = true;
		}
	}

	RefConst<PhysicsMaterial> mMaterial;

	const RRayCast &		mRay;
	JPC_RayCastHit			mHit { };
	bool					mHasHit = false;
};

class CallbackRayCollector final : public CastRayCollector
{
public:
	CallbackRayCollector(const RRayCast &inRay, JPC_RayCastCollectorFn inFn, void *inUserData) : mRay(inRay), mFn(inFn), mUserData(inUserData) { }

	virtual void			AddHit(const RayCastResult &inResult) override
	{
		JPC_RayCastHit hit = MakeRayHit(*this, mRay, inResult);
		if (!mFn(mUserData, &hit))
			ForceEarlyOut();
	}

private:
	const RRayCast &		mRay;
	JPC_RayCastCollectorFn	mFn;
	void *					mUserData;
};

class CallbackPointCollector final : public CollidePointCollector
{
public:
	CallbackPointCollector(JPC_CollidePointCollectorFn inFn, void *inUserData) : mFn(inFn), mUserData(inUserData) { }

	virtual void			AddHit(const CollidePointResult &inResult) override
	{
		JPC_CollidePointHit hit { ToC(inResult.mBodyID), ToC(inResult.mSubShapeID2) };
		if (!mFn(mUserData, &hit))
			ForceEarlyOut();
	}

private:
	JPC_CollidePointCollectorFn mFn;
	void *					mUserData;
};

JPC_ShapeHit MakeShapeHit(const TransformedShape *inContext, const CollideShapeResult &inResult, float inFraction, bool inBackFace)
{
	JPC_ShapeHit hit;
	hit.body_id = ToC(inResult.mBodyID2);
	hit.sub_shape_id1 = ToC(inResult.mSubShapeID1);
	hit.sub_shape_id2 = ToC(inResult.mSubShapeID2);
	hit.contact_point_on1 = ToC(inResult.mContactPointOn1);
	hit.contact_point_on2 = ToC(inResult.mContactPointOn2);
	hit.penetration_axis = ToC(inResult.mPenetrationAxis);
	hit.penetration_depth = inResult.mPenetrationDepth;
	hit.fraction = inFraction;
	hit.is_back_face_hit = inBackFace;
	hit.material = inContext != nullptr ? ToC(inContext->GetMaterial(inResult.mSubShapeID2)) : nullptr;
	return hit;
}

class CallbackCollideShapeCollector final : public CollideShapeCollector
{
public:
	CallbackCollideShapeCollector(JPC_ShapeCollectorFn inFn, void *inUserData) : mFn(inFn), mUserData(inUserData) { }

	virtual void			AddHit(const CollideShapeResult &inResult) override
	{
		JPC_ShapeHit hit = MakeShapeHit(GetContext(), inResult, 0.0f, false);
		if (!mFn(mUserData, &hit))
			ForceEarlyOut();
	}

private:
	JPC_ShapeCollectorFn	mFn;
	void *					mUserData;
};

class CallbackCastShapeCollector final : public CastShapeCollector
{
public:
	CallbackCastShapeCollector(JPC_ShapeCollectorFn inFn, void *inUserData) : mFn(inFn), mUserData(inUserData) { }

	virtual void			AddHit(const ShapeCastResult &inResult) override
	{
		JPC_ShapeHit hit = MakeShapeHit(GetContext(), inResult, inResult.mFraction, inResult.mIsBackFaceHit);
		if (!mFn(mUserData, &hit))
			ForceEarlyOut();
	}

private:
	JPC_ShapeCollectorFn	mFn;
	void *					mUserData;
};

} // namespace

bool JPC_PhysicsSystem_CastRay(const JPC_PhysicsSystem *system, JPC_RVec3 origin, JPC_Vec3 direction, const JPC_QueryFilter *filter, JPC_RayCastHit *out_hit)
{
	RRayCast ray { ToJph(origin), ToJph(direction) };
	QueryFilters filters(*system, filter);
	ClosestRayCollector collector(ray);
	system->mSystem.GetNarrowPhaseQuery().CastRay(ray, RayCastSettings(), collector, filters.mBroadPhase, filters.mObjectLayer, filters.mBody);
	if (collector.mHasHit && out_hit != nullptr)
	{
		*out_hit = collector.mHit;
		if (out_hit->material != nullptr)
			ToJph(out_hit->material)->AddRef();
	}
	return collector.mHasHit;
}

void JPC_PhysicsSystem_CastRayAll(const JPC_PhysicsSystem *system, JPC_RVec3 origin, JPC_Vec3 direction, const JPC_RayCastSettings *settings,
								  const JPC_QueryFilter *filter, JPC_RayCastCollectorFn collector_fn, void *user_data)
{
	RRayCast ray { ToJph(origin), ToJph(direction) };
	RayCastSettings ray_settings;
	if (settings != nullptr)
	{
		ray_settings.mBackFaceModeTriangles = (EBackFaceMode)settings->back_face_mode_triangles;
		ray_settings.mBackFaceModeConvex = (EBackFaceMode)settings->back_face_mode_convex;
		ray_settings.mTreatConvexAsSolid = settings->treat_convex_as_solid;
	}
	QueryFilters filters(*system, filter);
	CallbackRayCollector collector(ray, collector_fn, user_data);
	system->mSystem.GetNarrowPhaseQuery().CastRay(ray, ray_settings, collector, filters.mBroadPhase, filters.mObjectLayer, filters.mBody);
}

void JPC_PhysicsSystem_CollidePoint(const JPC_PhysicsSystem *system, JPC_RVec3 point, const JPC_QueryFilter *filter, JPC_CollidePointCollectorFn collector_fn, void *user_data)
{
	QueryFilters filters(*system, filter);
	CallbackPointCollector collector(collector_fn, user_data);
	system->mSystem.GetNarrowPhaseQuery().CollidePoint(ToJph(point), collector, filters.mBroadPhase, filters.mObjectLayer, filters.mBody);
}

void JPC_PhysicsSystem_CollideShape(const JPC_PhysicsSystem *system, const JPC_Shape *shape, JPC_Vec3 scale, const JPC_RMat44 *transform,
									float max_separation_distance, const JPC_QueryFilter *filter, JPC_ShapeCollectorFn collector_fn, void *user_data)
{
	CollideShapeSettings settings;
	settings.mMaxSeparationDistance = max_separation_distance;
	QueryFilters filters(*system, filter);
	CallbackCollideShapeCollector collector(collector_fn, user_data);
	system->mSystem.GetNarrowPhaseQuery().CollideShape(ToJph(shape), ToJph(scale), ToJph(*transform), settings, RVec3::sZero(), collector,
													   filters.mBroadPhase, filters.mObjectLayer, filters.mBody);
}

void JPC_PhysicsSystem_CastShape(const JPC_PhysicsSystem *system, const JPC_Shape *shape, JPC_Vec3 scale, const JPC_RMat44 *transform, JPC_Vec3 direction,
								 const JPC_QueryFilter *filter, JPC_ShapeCollectorFn collector_fn, void *user_data)
{
	RShapeCast cast(ToJph(shape), ToJph(scale), ToJph(*transform), ToJph(direction));
	ShapeCastSettings settings;
	QueryFilters filters(*system, filter);
	CallbackCastShapeCollector collector(collector_fn, user_data);
	system->mSystem.GetNarrowPhaseQuery().CastShape(cast, settings, RVec3::sZero(), collector, filters.mBroadPhase, filters.mObjectLayer, filters.mBody);
}

// ---------------------------------------------------------------------------
// Constraints
// ---------------------------------------------------------------------------

static JPC_ConstraintSettings ToC(const ConstraintSettings &in)
{
	return { in.mEnabled, in.mConstraintPriority, in.mNumVelocityStepsOverride, in.mNumPositionStepsOverride, in.mUserData };
}

static void ToJph(const JPC_ConstraintSettings &in, ConstraintSettings &out)
{
	out.mEnabled = in.enabled;
	out.mConstraintPriority = in.constraint_priority;
	out.mNumVelocityStepsOverride = in.num_velocity_steps_override;
	out.mNumPositionStepsOverride = in.num_position_steps_override;
	out.mUserData = in.user_data;
}

static JPC_SpringSettings ToC(const SpringSettings &in)
{
	return { (JPC_SpringMode)in.mMode, in.mFrequency, in.mDamping };
}

static SpringSettings ToJph(const JPC_SpringSettings &in)
{
	return SpringSettings((ESpringMode)in.mode, in.frequency_or_stiffness, in.damping);
}

static JPC_MotorSettings ToC(const MotorSettings &in)
{
	return { ToC(in.mSpringSettings), in.mMinForceLimit, in.mMaxForceLimit, in.mMinTorqueLimit, in.mMaxTorqueLimit };
}

static MotorSettings ToJph(const JPC_MotorSettings &in)
{
	MotorSettings out;
	out.mSpringSettings = ToJph(in.spring);
	out.mMinForceLimit = in.min_force_limit;
	out.mMaxForceLimit = in.max_force_limit;
	out.mMinTorqueLimit = in.min_torque_limit;
	out.mMaxTorqueLimit = in.max_torque_limit;
	return out;
}

void JPC_FixedConstraintSettings_SetDefault(JPC_FixedConstraintSettings *out)
{
	FixedConstraintSettings s;
	out->base = ToC(s);
	out->space = (JPC_ConstraintSpace)s.mSpace;
	out->auto_detect_point = s.mAutoDetectPoint;
	out->point1 = ToC(s.mPoint1);
	out->axis_x1 = ToC(s.mAxisX1);
	out->axis_y1 = ToC(s.mAxisY1);
	out->point2 = ToC(s.mPoint2);
	out->axis_x2 = ToC(s.mAxisX2);
	out->axis_y2 = ToC(s.mAxisY2);
}

void JPC_PointConstraintSettings_SetDefault(JPC_PointConstraintSettings *out)
{
	PointConstraintSettings s;
	out->base = ToC(s);
	out->space = (JPC_ConstraintSpace)s.mSpace;
	out->point1 = ToC(s.mPoint1);
	out->point2 = ToC(s.mPoint2);
}

void JPC_DistanceConstraintSettings_SetDefault(JPC_DistanceConstraintSettings *out)
{
	DistanceConstraintSettings s;
	out->base = ToC(s);
	out->space = (JPC_ConstraintSpace)s.mSpace;
	out->point1 = ToC(s.mPoint1);
	out->point2 = ToC(s.mPoint2);
	out->min_distance = s.mMinDistance;
	out->max_distance = s.mMaxDistance;
	out->limits_spring_settings = ToC(s.mLimitsSpringSettings);
}

void JPC_HingeConstraintSettings_SetDefault(JPC_HingeConstraintSettings *out)
{
	HingeConstraintSettings s;
	out->base = ToC(s);
	out->space = (JPC_ConstraintSpace)s.mSpace;
	out->point1 = ToC(s.mPoint1);
	out->hinge_axis1 = ToC(s.mHingeAxis1);
	out->normal_axis1 = ToC(s.mNormalAxis1);
	out->point2 = ToC(s.mPoint2);
	out->hinge_axis2 = ToC(s.mHingeAxis2);
	out->normal_axis2 = ToC(s.mNormalAxis2);
	out->limits_min = s.mLimitsMin;
	out->limits_max = s.mLimitsMax;
	out->limits_spring_settings = ToC(s.mLimitsSpringSettings);
	out->max_friction_torque = s.mMaxFrictionTorque;
	out->motor_settings = ToC(s.mMotorSettings);
}

void JPC_SliderConstraintSettings_SetDefault(JPC_SliderConstraintSettings *out)
{
	SliderConstraintSettings s;
	out->base = ToC(s);
	out->space = (JPC_ConstraintSpace)s.mSpace;
	out->auto_detect_point = s.mAutoDetectPoint;
	out->point1 = ToC(s.mPoint1);
	out->slider_axis1 = ToC(s.mSliderAxis1);
	out->normal_axis1 = ToC(s.mNormalAxis1);
	out->point2 = ToC(s.mPoint2);
	out->slider_axis2 = ToC(s.mSliderAxis2);
	out->normal_axis2 = ToC(s.mNormalAxis2);
	out->limits_min = s.mLimitsMin;
	out->limits_max = s.mLimitsMax;
	out->limits_spring_settings = ToC(s.mLimitsSpringSettings);
	out->max_friction_force = s.mMaxFrictionForce;
	out->motor_settings = ToC(s.mMotorSettings);
}

void JPC_ConeConstraintSettings_SetDefault(JPC_ConeConstraintSettings *out)
{
	ConeConstraintSettings s;
	out->base = ToC(s);
	out->space = (JPC_ConstraintSpace)s.mSpace;
	out->point1 = ToC(s.mPoint1);
	out->twist_axis1 = ToC(s.mTwistAxis1);
	out->point2 = ToC(s.mPoint2);
	out->twist_axis2 = ToC(s.mTwistAxis2);
	out->half_cone_angle = s.mHalfConeAngle;
}

static JPC_Constraint *CreateConstraint(JPC_PhysicsSystem *inSystem, TwoBodyConstraintSettings &ioSettings, JPC_BodyID inBody1, JPC_BodyID inBody2)
{
	if (inBody1 == inBody2)
	{
		JPC_SetError("A constraint needs two different bodies");
		return nullptr;
	}

	ioSettings.SetEmbedded();
	std::shared_lock state_lock(inSystem->mBodyStateMutex);
	for (JPC_BodyID id : { inBody1, inBody2 })
		if (inSystem->mProtectedBodies.find(id) != inSystem->mProtectedBodies.end())
		{
			// The character destroys its body without knowing about constraints, which would leave them dangling
			JPC_SetError("Constraints cannot use bodies owned by a character");
			return nullptr;
		}

	BodyID ids[] = { ToJph(inBody1), ToJph(inBody2) };
	BodyLockMultiWrite lock(inSystem->mSystem.GetBodyLockInterface(), ids, 2);
	Body *body1 = lock.GetBody(0);
	Body *body2 = lock.GetBody(1);
	if ((inBody1 != JPC_BODY_ID_INVALID && body1 == nullptr) || (inBody2 != JPC_BODY_ID_INVALID && body2 == nullptr))
	{
		JPC_SetError("Constraint body does not exist");
		return nullptr;
	}
	if (body1 == nullptr && body2 == nullptr)
	{
		JPC_SetError("A constraint needs at least one body");
		return nullptr;
	}

	Constraint *constraint = ioSettings.Create(body1 != nullptr ? *body1 : Body::sFixedToWorld, body2 != nullptr ? *body2 : Body::sFixedToWorld);
	constraint->AddRef();
	return ToC(constraint);
}

JPC_Constraint *JPC_PhysicsSystem_CreateFixedConstraint(JPC_PhysicsSystem *system, const JPC_FixedConstraintSettings *in, JPC_BodyID body1, JPC_BodyID body2)
{
	FixedConstraintSettings s;
	ToJph(in->base, s);
	s.mSpace = (EConstraintSpace)in->space;
	s.mAutoDetectPoint = in->auto_detect_point;
	s.mPoint1 = ToJph(in->point1);
	s.mAxisX1 = ToJph(in->axis_x1);
	s.mAxisY1 = ToJph(in->axis_y1);
	s.mPoint2 = ToJph(in->point2);
	s.mAxisX2 = ToJph(in->axis_x2);
	s.mAxisY2 = ToJph(in->axis_y2);
	return CreateConstraint(system, s, body1, body2);
}

JPC_Constraint *JPC_PhysicsSystem_CreatePointConstraint(JPC_PhysicsSystem *system, const JPC_PointConstraintSettings *in, JPC_BodyID body1, JPC_BodyID body2)
{
	PointConstraintSettings s;
	ToJph(in->base, s);
	s.mSpace = (EConstraintSpace)in->space;
	s.mPoint1 = ToJph(in->point1);
	s.mPoint2 = ToJph(in->point2);
	return CreateConstraint(system, s, body1, body2);
}

JPC_Constraint *JPC_PhysicsSystem_CreateDistanceConstraint(JPC_PhysicsSystem *system, const JPC_DistanceConstraintSettings *in, JPC_BodyID body1, JPC_BodyID body2)
{
	DistanceConstraintSettings s;
	ToJph(in->base, s);
	s.mSpace = (EConstraintSpace)in->space;
	s.mPoint1 = ToJph(in->point1);
	s.mPoint2 = ToJph(in->point2);
	s.mMinDistance = in->min_distance;
	s.mMaxDistance = in->max_distance;
	s.mLimitsSpringSettings = ToJph(in->limits_spring_settings);
	return CreateConstraint(system, s, body1, body2);
}

JPC_Constraint *JPC_PhysicsSystem_CreateHingeConstraint(JPC_PhysicsSystem *system, const JPC_HingeConstraintSettings *in, JPC_BodyID body1, JPC_BodyID body2)
{
	HingeConstraintSettings s;
	ToJph(in->base, s);
	s.mSpace = (EConstraintSpace)in->space;
	s.mPoint1 = ToJph(in->point1);
	s.mHingeAxis1 = ToJph(in->hinge_axis1);
	s.mNormalAxis1 = ToJph(in->normal_axis1);
	s.mPoint2 = ToJph(in->point2);
	s.mHingeAxis2 = ToJph(in->hinge_axis2);
	s.mNormalAxis2 = ToJph(in->normal_axis2);
	s.mLimitsMin = in->limits_min;
	s.mLimitsMax = in->limits_max;
	s.mLimitsSpringSettings = ToJph(in->limits_spring_settings);
	s.mMaxFrictionTorque = in->max_friction_torque;
	s.mMotorSettings = ToJph(in->motor_settings);
	return CreateConstraint(system, s, body1, body2);
}

JPC_Constraint *JPC_PhysicsSystem_CreateSliderConstraint(JPC_PhysicsSystem *system, const JPC_SliderConstraintSettings *in, JPC_BodyID body1, JPC_BodyID body2)
{
	SliderConstraintSettings s;
	ToJph(in->base, s);
	s.mSpace = (EConstraintSpace)in->space;
	s.mAutoDetectPoint = in->auto_detect_point;
	s.mPoint1 = ToJph(in->point1);
	s.mSliderAxis1 = ToJph(in->slider_axis1);
	s.mNormalAxis1 = ToJph(in->normal_axis1);
	s.mPoint2 = ToJph(in->point2);
	s.mSliderAxis2 = ToJph(in->slider_axis2);
	s.mNormalAxis2 = ToJph(in->normal_axis2);
	s.mLimitsMin = in->limits_min;
	s.mLimitsMax = in->limits_max;
	s.mLimitsSpringSettings = ToJph(in->limits_spring_settings);
	s.mMaxFrictionForce = in->max_friction_force;
	s.mMotorSettings = ToJph(in->motor_settings);
	return CreateConstraint(system, s, body1, body2);
}

JPC_Constraint *JPC_PhysicsSystem_CreateConeConstraint(JPC_PhysicsSystem *system, const JPC_ConeConstraintSettings *in, JPC_BodyID body1, JPC_BodyID body2)
{
	ConeConstraintSettings s;
	ToJph(in->base, s);
	s.mSpace = (EConstraintSpace)in->space;
	s.mPoint1 = ToJph(in->point1);
	s.mTwistAxis1 = ToJph(in->twist_axis1);
	s.mPoint2 = ToJph(in->point2);
	s.mTwistAxis2 = ToJph(in->twist_axis2);
	s.mHalfConeAngle = in->half_cone_angle;
	return CreateConstraint(system, s, body1, body2);
}

void JPC_BodyInterface_ActivateConstraint(JPC_BodyInterface *bi, const JPC_Constraint *constraint)
{
	// ActivateBodies does not check that the bodies are in the world
	std::shared_lock body_state_lock(bi->mOwner->mBodyStateMutex);
	const Constraint *constraint_base = ToJph(constraint);
	if (constraint_base->GetSubType() == EConstraintSubType::Vehicle)
	{
		BodyID id = static_cast<const VehicleConstraint *>(constraint_base)->GetVehicleBody()->GetID();
		if (IsAddedAndMovable(bi, ToC(id)))
			ToJph(bi)->ActivateBody(id);
		return;
	}
	const TwoBodyConstraint *c = static_cast<const TwoBodyConstraint *>(constraint_base);
	for (const Body *body : { c->GetBody1(), c->GetBody2() })
		if (!body->GetID().IsInvalid() && IsAddedAndMovable(bi, ToC(body->GetID())))
			ToJph(bi)->ActivateBody(body->GetID());
}

void JPC_Constraint_AddRef(const JPC_Constraint *constraint) { ToJph(constraint)->AddRef(); }
void JPC_Constraint_Release(const JPC_Constraint *constraint) { ToJph(constraint)->Release(); }
bool JPC_Constraint_GetEnabled(const JPC_Constraint *constraint) { return ToJph(constraint)->GetEnabled(); }
void JPC_Constraint_SetEnabled(JPC_Constraint *constraint, bool enabled) { ToJph(constraint)->SetEnabled(enabled); }
uint64_t JPC_Constraint_GetUserData(const JPC_Constraint *constraint) { return ToJph(constraint)->GetUserData(); }
void JPC_Constraint_SetUserData(JPC_Constraint *constraint, uint64_t user_data) { ToJph(constraint)->SetUserData(user_data); }
// Vehicles are not two body constraints: report the vehicle body as body 2 and the world as body 1
JPC_BodyID JPC_Constraint_GetBody1(const JPC_Constraint *constraint)
{
	const Constraint *c = ToJph(constraint);
	if (c->GetSubType() == EConstraintSubType::Vehicle)
		return JPC_BODY_ID_INVALID;
	return ToC(static_cast<const TwoBodyConstraint *>(c)->GetBody1()->GetID());
}

JPC_BodyID JPC_Constraint_GetBody2(const JPC_Constraint *constraint)
{
	const Constraint *c = ToJph(constraint);
	if (c->GetSubType() == EConstraintSubType::Vehicle)
		return ToC(static_cast<const VehicleConstraint *>(c)->GetVehicleBody()->GetID());
	return ToC(static_cast<const TwoBodyConstraint *>(c)->GetBody2()->GetID());
}

template <class T, EConstraintSubType SubType>
static T *As(JPC_Constraint *inConstraint)
{
	Constraint *c = ToJph(inConstraint);
	return c->GetSubType() == SubType ? static_cast<T *>(c) : nullptr;
}

template <class T, EConstraintSubType SubType>
static const T *As(const JPC_Constraint *inConstraint)
{
	const Constraint *c = ToJph(inConstraint);
	return c->GetSubType() == SubType ? static_cast<const T *>(c) : nullptr;
}

#define JPC_HINGE(c) As<HingeConstraint, EConstraintSubType::Hinge>(c)
#define JPC_SLIDER(c) As<SliderConstraint, EConstraintSubType::Slider>(c)

float JPC_HingeConstraint_GetCurrentAngle(const JPC_Constraint *constraint) { auto c = JPC_HINGE(constraint); return c ? c->GetCurrentAngle() : 0.0f; }
void JPC_HingeConstraint_SetLimits(JPC_Constraint *constraint, float min, float max) { if (auto c = JPC_HINGE(constraint)) c->SetLimits(min, max); }
void JPC_HingeConstraint_SetMaxFrictionTorque(JPC_Constraint *constraint, float torque) { if (auto c = JPC_HINGE(constraint)) c->SetMaxFrictionTorque(torque); }
void JPC_HingeConstraint_SetMotorState(JPC_Constraint *constraint, JPC_MotorState state) { if (auto c = JPC_HINGE(constraint)) c->SetMotorState((EMotorState)state); }
void JPC_HingeConstraint_SetTargetAngularVelocity(JPC_Constraint *constraint, float angular_velocity) { if (auto c = JPC_HINGE(constraint)) c->SetTargetAngularVelocity(angular_velocity); }
void JPC_HingeConstraint_SetTargetAngle(JPC_Constraint *constraint, float angle) { if (auto c = JPC_HINGE(constraint)) c->SetTargetAngle(angle); }

float JPC_SliderConstraint_GetCurrentPosition(const JPC_Constraint *constraint) { auto c = JPC_SLIDER(constraint); return c ? c->GetCurrentPosition() : 0.0f; }
void JPC_SliderConstraint_SetLimits(JPC_Constraint *constraint, float min, float max) { if (auto c = JPC_SLIDER(constraint)) c->SetLimits(min, max); }
void JPC_SliderConstraint_SetMaxFrictionForce(JPC_Constraint *constraint, float force) { if (auto c = JPC_SLIDER(constraint)) c->SetMaxFrictionForce(force); }
void JPC_SliderConstraint_SetMotorState(JPC_Constraint *constraint, JPC_MotorState state) { if (auto c = JPC_SLIDER(constraint)) c->SetMotorState((EMotorState)state); }
void JPC_SliderConstraint_SetTargetVelocity(JPC_Constraint *constraint, float velocity) { if (auto c = JPC_SLIDER(constraint)) c->SetTargetVelocity(velocity); }
void JPC_SliderConstraint_SetTargetPosition(JPC_Constraint *constraint, float position) { if (auto c = JPC_SLIDER(constraint)) c->SetTargetPosition(position); }

void JPC_DistanceConstraint_SetDistance(JPC_Constraint *constraint, float min_distance, float max_distance)
{
	if (auto c = As<DistanceConstraint, EConstraintSubType::Distance>(constraint))
		c->SetDistance(min_distance, max_distance);
}

void JPC_ConeConstraint_SetHalfConeAngle(JPC_Constraint *constraint, float half_cone_angle)
{
	if (auto c = As<ConeConstraint, EConstraintSubType::Cone>(constraint))
		c->SetHalfConeAngle(half_cone_angle);
}

template <class Settings>
static void ToJphSwingTwist(const JPC_SwingTwistConstraintSettings &in, Settings &s)
{
	ToJph(in.base, s);
	s.mSpace = (EConstraintSpace)in.space;
	s.mPosition1 = ToJph(in.position1);
	s.mTwistAxis1 = ToJph(in.twist_axis1);
	s.mPlaneAxis1 = ToJph(in.plane_axis1);
	s.mPosition2 = ToJph(in.position2);
	s.mTwistAxis2 = ToJph(in.twist_axis2);
	s.mPlaneAxis2 = ToJph(in.plane_axis2);
	s.mSwingType = (ESwingType)in.swing_type;
	s.mNormalHalfConeAngle = in.normal_half_cone_angle;
	s.mPlaneHalfConeAngle = in.plane_half_cone_angle;
	s.mTwistMinAngle = in.twist_min_angle;
	s.mTwistMaxAngle = in.twist_max_angle;
	s.mMaxFrictionTorque = in.max_friction_torque;
	s.mSwingMotorSettings = ToJph(in.swing_motor_settings);
	s.mTwistMotorSettings = ToJph(in.twist_motor_settings);
}

void JPC_SwingTwistConstraintSettings_SetDefault(JPC_SwingTwistConstraintSettings *out)
{
	SwingTwistConstraintSettings s;
	out->base = ToC(s);
	out->space = (JPC_ConstraintSpace)s.mSpace;
	out->position1 = ToC(s.mPosition1);
	out->twist_axis1 = ToC(s.mTwistAxis1);
	out->plane_axis1 = ToC(s.mPlaneAxis1);
	out->position2 = ToC(s.mPosition2);
	out->twist_axis2 = ToC(s.mTwistAxis2);
	out->plane_axis2 = ToC(s.mPlaneAxis2);
	out->swing_type = (JPC_SwingType)s.mSwingType;
	out->normal_half_cone_angle = s.mNormalHalfConeAngle;
	out->plane_half_cone_angle = s.mPlaneHalfConeAngle;
	out->twist_min_angle = s.mTwistMinAngle;
	out->twist_max_angle = s.mTwistMaxAngle;
	out->max_friction_torque = s.mMaxFrictionTorque;
	out->swing_motor_settings = ToC(s.mSwingMotorSettings);
	out->twist_motor_settings = ToC(s.mTwistMotorSettings);
}

JPC_Constraint *JPC_PhysicsSystem_CreateSwingTwistConstraint(JPC_PhysicsSystem *system, const JPC_SwingTwistConstraintSettings *in, JPC_BodyID body1, JPC_BodyID body2)
{
	SwingTwistConstraintSettings s;
	ToJphSwingTwist(*in, s);
	return CreateConstraint(system, s, body1, body2);
}

void JPC_SixDOFConstraintSettings_SetDefault(JPC_SixDOFConstraintSettings *out)
{
	SixDOFConstraintSettings s;
	out->base = ToC(s);
	out->space = (JPC_ConstraintSpace)s.mSpace;
	out->position1 = ToC(s.mPosition1);
	out->axis_x1 = ToC(s.mAxisX1);
	out->axis_y1 = ToC(s.mAxisY1);
	out->position2 = ToC(s.mPosition2);
	out->axis_x2 = ToC(s.mAxisX2);
	out->axis_y2 = ToC(s.mAxisY2);
	out->swing_type = (JPC_SwingType)s.mSwingType;
	for (int i = 0; i < SixDOFConstraintSettings::EAxis::Num; ++i)
	{
		out->max_friction[i] = s.mMaxFriction[i];
		out->limit_min[i] = s.mLimitMin[i];
		out->limit_max[i] = s.mLimitMax[i];
		out->motor_settings[i] = ToC(s.mMotorSettings[i]);
	}
	for (int i = 0; i < SixDOFConstraintSettings::EAxis::NumTranslation; ++i)
		out->limits_spring_settings[i] = ToC(s.mLimitsSpringSettings[i]);
}

JPC_Constraint *JPC_PhysicsSystem_CreateSixDOFConstraint(JPC_PhysicsSystem *system, const JPC_SixDOFConstraintSettings *in, JPC_BodyID body1, JPC_BodyID body2)
{
	SixDOFConstraintSettings s;
	ToJph(in->base, s);
	s.mSpace = (EConstraintSpace)in->space;
	s.mPosition1 = ToJph(in->position1);
	s.mAxisX1 = ToJph(in->axis_x1);
	s.mAxisY1 = ToJph(in->axis_y1);
	s.mPosition2 = ToJph(in->position2);
	s.mAxisX2 = ToJph(in->axis_x2);
	s.mAxisY2 = ToJph(in->axis_y2);
	s.mSwingType = (ESwingType)in->swing_type;
	for (int i = 0; i < SixDOFConstraintSettings::EAxis::Num; ++i)
	{
		s.mMaxFriction[i] = in->max_friction[i];
		s.mLimitMin[i] = in->limit_min[i];
		s.mLimitMax[i] = in->limit_max[i];
		s.mMotorSettings[i] = ToJph(in->motor_settings[i]);
	}
	for (int i = 0; i < SixDOFConstraintSettings::EAxis::NumTranslation; ++i)
		s.mLimitsSpringSettings[i] = ToJph(in->limits_spring_settings[i]);
	return CreateConstraint(system, s, body1, body2);
}

#define JPC_SWING_TWIST(c) As<SwingTwistConstraint, EConstraintSubType::SwingTwist>(c)
#define JPC_SIX_DOF(c) As<SixDOFConstraint, EConstraintSubType::SixDOF>(c)

void JPC_SwingTwistConstraint_SetNormalHalfConeAngle(JPC_Constraint *constraint, float angle) { if (auto c = JPC_SWING_TWIST(constraint)) c->SetNormalHalfConeAngle(angle); }
void JPC_SwingTwistConstraint_SetPlaneHalfConeAngle(JPC_Constraint *constraint, float angle) { if (auto c = JPC_SWING_TWIST(constraint)) c->SetPlaneHalfConeAngle(angle); }
void JPC_SwingTwistConstraint_SetTwistMinAngle(JPC_Constraint *constraint, float angle) { if (auto c = JPC_SWING_TWIST(constraint)) c->SetTwistMinAngle(angle); }
void JPC_SwingTwistConstraint_SetTwistMaxAngle(JPC_Constraint *constraint, float angle) { if (auto c = JPC_SWING_TWIST(constraint)) c->SetTwistMaxAngle(angle); }
void JPC_SwingTwistConstraint_SetMaxFrictionTorque(JPC_Constraint *constraint, float torque) { if (auto c = JPC_SWING_TWIST(constraint)) c->SetMaxFrictionTorque(torque); }
void JPC_SwingTwistConstraint_SetSwingMotorState(JPC_Constraint *constraint, JPC_MotorState state) { if (auto c = JPC_SWING_TWIST(constraint)) c->SetSwingMotorState((EMotorState)state); }
void JPC_SwingTwistConstraint_SetTwistMotorState(JPC_Constraint *constraint, JPC_MotorState state) { if (auto c = JPC_SWING_TWIST(constraint)) c->SetTwistMotorState((EMotorState)state); }
void JPC_SwingTwistConstraint_SetTargetAngularVelocityCS(JPC_Constraint *constraint, JPC_Vec3 angular_velocity) { if (auto c = JPC_SWING_TWIST(constraint)) c->SetTargetAngularVelocityCS(ToJph(angular_velocity)); }
void JPC_SwingTwistConstraint_SetTargetOrientationCS(JPC_Constraint *constraint, JPC_Quat orientation) { if (auto c = JPC_SWING_TWIST(constraint)) c->SetTargetOrientationCS(ToJph(orientation).Normalized()); }
JPC_Quat JPC_SwingTwistConstraint_GetRotationInConstraintSpace(const JPC_Constraint *constraint) { auto c = JPC_SWING_TWIST(constraint); return ToC(c ? c->GetRotationInConstraintSpace() : Quat::sIdentity()); }

static bool IsValidAxis(JPC_SixDOFAxis axis) { return axis >= JPC_SIX_DOF_AXIS_TRANSLATION_X && axis < JPC_SIX_DOF_AXIS_NUM; }

void JPC_SixDOFConstraint_SetTranslationLimits(JPC_Constraint *constraint, JPC_Vec3 min, JPC_Vec3 max) { if (auto c = JPC_SIX_DOF(constraint)) c->SetTranslationLimits(ToJph(min), ToJph(max)); }
void JPC_SixDOFConstraint_SetRotationLimits(JPC_Constraint *constraint, JPC_Vec3 min, JPC_Vec3 max) { if (auto c = JPC_SIX_DOF(constraint)) c->SetRotationLimits(ToJph(min), ToJph(max)); }

void JPC_SixDOFConstraint_SetMaxFriction(JPC_Constraint *constraint, JPC_SixDOFAxis axis, float friction)
{
	if (auto c = JPC_SIX_DOF(constraint); c != nullptr && IsValidAxis(axis))
		c->SetMaxFriction((SixDOFConstraintSettings::EAxis)axis, friction);
}

void JPC_SixDOFConstraint_SetMotorState(JPC_Constraint *constraint, JPC_SixDOFAxis axis, JPC_MotorState state)
{
	if (auto c = JPC_SIX_DOF(constraint); c != nullptr && IsValidAxis(axis))
		c->SetMotorState((SixDOFConstraintSettings::EAxis)axis, (EMotorState)state);
}

void JPC_SixDOFConstraint_SetTargetVelocityCS(JPC_Constraint *constraint, JPC_Vec3 velocity) { if (auto c = JPC_SIX_DOF(constraint)) c->SetTargetVelocityCS(ToJph(velocity)); }
void JPC_SixDOFConstraint_SetTargetAngularVelocityCS(JPC_Constraint *constraint, JPC_Vec3 angular_velocity) { if (auto c = JPC_SIX_DOF(constraint)) c->SetTargetAngularVelocityCS(ToJph(angular_velocity)); }
void JPC_SixDOFConstraint_SetTargetPositionCS(JPC_Constraint *constraint, JPC_Vec3 position) { if (auto c = JPC_SIX_DOF(constraint)) c->SetTargetPositionCS(ToJph(position)); }
void JPC_SixDOFConstraint_SetTargetOrientationCS(JPC_Constraint *constraint, JPC_Quat orientation) { if (auto c = JPC_SIX_DOF(constraint)) c->SetTargetOrientationCS(ToJph(orientation).Normalized()); }
JPC_Quat JPC_SixDOFConstraint_GetRotationInConstraintSpace(const JPC_Constraint *constraint) { auto c = JPC_SIX_DOF(constraint); return ToC(c ? c->GetRotationInConstraintSpace() : Quat::sIdentity()); }

// ---------------------------------------------------------------------------
// Materials
// ---------------------------------------------------------------------------

JPC_PhysicsMaterial *JPC_PhysicsMaterial_Create(uint64_t user_data)
{
	UserMaterial *material = new UserMaterial(user_data);
	material->AddRef();
	return reinterpret_cast<JPC_PhysicsMaterial *>(material);
}

void JPC_PhysicsMaterial_AddRef(const JPC_PhysicsMaterial *material) { ToJph(material)->AddRef(); }
void JPC_PhysicsMaterial_Release(const JPC_PhysicsMaterial *material) { ToJph(material)->Release(); }
uint64_t JPC_PhysicsMaterial_GetUserData(const JPC_PhysicsMaterial *material) { return ToJph(material)->mUserData; }

void JPC_ConvexShape_SetMaterial(JPC_Shape *shape, const JPC_PhysicsMaterial *material)
{
	Shape *s = ToJph(shape);
	if (s->GetType() == EShapeType::Convex)
		static_cast<ConvexShape *>(s)->SetMaterial(material != nullptr ? ToJph(material) : nullptr);
}

JPC_Shape *JPC_MeshShape_CreateWithMaterials(const JPC_Vec3 *vertices, uint32_t num_vertices, const uint32_t *indices, uint32_t num_triangles,
											 const uint32_t *material_indices, const JPC_PhysicsMaterial *const *materials, uint32_t num_materials)
{
	VertexList vertex_list;
	vertex_list.reserve(num_vertices);
	for (uint32_t i = 0; i < num_vertices; ++i)
		vertex_list.push_back(Float3(vertices[i].x, vertices[i].y, vertices[i].z));

	PhysicsMaterialList material_list;
	material_list.reserve(num_materials);
	for (uint32_t i = 0; i < num_materials; ++i)
	{
		if (materials[i] == nullptr)
		{
			JPC_SetError("Mesh material is null");
			return nullptr;
		}
		material_list.push_back(ToJph(materials[i]));
	}

	IndexedTriangleList triangles;
	triangles.reserve(num_triangles);
	for (uint32_t i = 0; i < num_triangles; ++i)
	{
		uint32_t a = indices[3 * i], b = indices[3 * i + 1], c = indices[3 * i + 2];
		if (a >= num_vertices || b >= num_vertices || c >= num_vertices)
		{
			JPC_SetError("Mesh triangle index out of range");
			return nullptr;
		}
		uint32_t material = material_indices != nullptr ? material_indices[i] : 0;
		if (num_materials > 0 ? material >= num_materials : material != 0)
		{
			JPC_SetError("Mesh material index out of range");
			return nullptr;
		}
		triangles.push_back(IndexedTriangle(a, b, c, material));
	}

	MeshShapeSettings settings(std::move(vertex_list), std::move(triangles), std::move(material_list));
	settings.SetEmbedded();
	ShapeSettings::ShapeResult result = settings.Create();
	if (result.HasError())
	{
		JPC_SetError(result.GetError().c_str());
		return nullptr;
	}
	Shape *shape = result.Get().GetPtr();
	shape->AddRef();
	return ToC(shape);
}

const JPC_PhysicsMaterial *JPC_Shape_GetMaterialUnchecked(const JPC_Shape *shape, JPC_SubShapeID sub_shape_id)
{
	SubShapeID id;
	id.SetValue(sub_shape_id);
	return ToC(ToJph(shape)->GetMaterial(id));
}

// ---------------------------------------------------------------------------
// Collision groups
// ---------------------------------------------------------------------------

JPC_GroupFilter *JPC_GroupFilterTable_Create(uint32_t num_sub_groups)
{
	SafeGroupFilterTable *filter = new SafeGroupFilterTable(num_sub_groups);
	filter->AddRef();
	return reinterpret_cast<JPC_GroupFilter *>(static_cast<GroupFilter *>(filter));
}

void JPC_GroupFilterTable_DisableCollision(JPC_GroupFilter *filter, uint32_t a, uint32_t b) { ToJphTable(filter)->SetCollision(a, b, false); }
void JPC_GroupFilterTable_EnableCollision(JPC_GroupFilter *filter, uint32_t a, uint32_t b) { ToJphTable(filter)->SetCollision(a, b, true); }
bool JPC_GroupFilterTable_IsCollisionEnabled(const JPC_GroupFilter *filter, uint32_t a, uint32_t b) { return ToJphTable(filter)->IsCollisionEnabled(a, b); }
void JPC_GroupFilter_AddRef(const JPC_GroupFilter *filter) { ToJph(filter)->AddRef(); }
void JPC_GroupFilter_Release(const JPC_GroupFilter *filter) { ToJph(filter)->Release(); }

// ---------------------------------------------------------------------------
// Step listeners
// ---------------------------------------------------------------------------

struct JPC_StepListener final : public PhysicsStepListener
{
	JPH_OVERRIDE_NEW_DELETE

	JPC_StepListener(JPC_PhysicsSystem *inSystem, JPC_StepListenerFn inFn, void *inUserData) : mSystem(inSystem), mFn(inFn), mUserData(inUserData) { }

	virtual void			OnStep(const PhysicsStepListenerContext &inContext) override
	{
		JPC_PhysicsStepContext context { inContext.mDeltaTime, inContext.mIsFirstStep, inContext.mIsLastStep };
		mFn(mUserData, mSystem, &context);
	}

	JPC_PhysicsSystem *		mSystem;
	JPC_StepListenerFn		mFn;
	void *					mUserData;
};

JPC_StepListener *JPC_PhysicsSystem_AddStepListener(JPC_PhysicsSystem *system, JPC_StepListenerFn fn, void *user_data)
{
	JPC_StepListener *listener = new JPC_StepListener(system, fn, user_data);
	system->AddStepListener(listener);
	return listener;
}

void JPC_PhysicsSystem_RemoveStepListener(JPC_PhysicsSystem *system, JPC_StepListener *listener)
{
	system->RemoveStepListener(listener);
	delete listener;
}

// ---------------------------------------------------------------------------
// Query settings
// ---------------------------------------------------------------------------

void JPC_CollideShapeSettings_SetDefault(JPC_CollideShapeSettings *out)
{
	CollideShapeSettings s;
	out->max_separation_distance = s.mMaxSeparationDistance;
	out->back_face_mode = (JPC_BackFaceMode)s.mBackFaceMode;
	out->active_edge_mode = (JPC_ActiveEdgeMode)s.mActiveEdgeMode;
	out->active_edge_movement_direction = ToC(s.mActiveEdgeMovementDirection);
	out->collision_tolerance = s.mCollisionTolerance;
	out->penetration_tolerance = s.mPenetrationTolerance;
}

void JPC_ShapeCastSettings_SetDefault(JPC_ShapeCastSettings *out)
{
	ShapeCastSettings s;
	out->back_face_mode_triangles = (JPC_BackFaceMode)s.mBackFaceModeTriangles;
	out->back_face_mode_convex = (JPC_BackFaceMode)s.mBackFaceModeConvex;
	out->use_shrunken_shape_and_convex_radius = s.mUseShrunkenShapeAndConvexRadius;
	out->return_deepest_point = s.mReturnDeepestPoint;
	out->active_edge_mode = (JPC_ActiveEdgeMode)s.mActiveEdgeMode;
	out->active_edge_movement_direction = ToC(s.mActiveEdgeMovementDirection);
	out->collision_tolerance = s.mCollisionTolerance;
	out->penetration_tolerance = s.mPenetrationTolerance;
}

static RayCastSettings ToJph(const JPC_RayCastSettings *in)
{
	RayCastSettings s;
	if (in != nullptr)
	{
		s.mBackFaceModeTriangles = (EBackFaceMode)in->back_face_mode_triangles;
		s.mBackFaceModeConvex = (EBackFaceMode)in->back_face_mode_convex;
		s.mTreatConvexAsSolid = in->treat_convex_as_solid;
	}
	return s;
}

bool JPC_PhysicsSystem_CastRayWithSettings(const JPC_PhysicsSystem *system, JPC_RVec3 origin, JPC_Vec3 direction, const JPC_RayCastSettings *settings,
										   const JPC_QueryFilter *filter, JPC_RayCastHit *out_hit)
{
	RRayCast ray { ToJph(origin), ToJph(direction) };
	QueryFilters filters(*system, filter);
	ClosestRayCollector collector(ray);
	system->mSystem.GetNarrowPhaseQuery().CastRay(ray, ToJph(settings), collector, filters.mBroadPhase, filters.mObjectLayer, filters.mBody);
	if (collector.mHasHit && out_hit != nullptr)
	{
		*out_hit = collector.mHit;
		if (out_hit->material != nullptr)
			ToJph(out_hit->material)->AddRef();
	}
	return collector.mHasHit;
}

void JPC_PhysicsSystem_CollideShapeWithSettings(const JPC_PhysicsSystem *system, const JPC_Shape *shape, JPC_Vec3 scale, const JPC_RMat44 *transform,
												const JPC_CollideShapeSettings *in, const JPC_QueryFilter *filter, JPC_ShapeCollectorFn collector_fn, void *user_data)
{
	CollideShapeSettings settings;
	if (in != nullptr)
	{
		settings.mMaxSeparationDistance = in->max_separation_distance;
		settings.mBackFaceMode = (EBackFaceMode)in->back_face_mode;
		settings.mActiveEdgeMode = (EActiveEdgeMode)in->active_edge_mode;
		settings.mActiveEdgeMovementDirection = ToJph(in->active_edge_movement_direction);
		settings.mCollisionTolerance = in->collision_tolerance;
		settings.mPenetrationTolerance = in->penetration_tolerance;
	}
	QueryFilters filters(*system, filter);
	CallbackCollideShapeCollector collector(collector_fn, user_data);
	system->mSystem.GetNarrowPhaseQuery().CollideShape(ToJph(shape), ToJph(scale), ToJph(*transform), settings, RVec3::sZero(), collector,
													   filters.mBroadPhase, filters.mObjectLayer, filters.mBody);
}

void JPC_PhysicsSystem_CastShapeWithSettings(const JPC_PhysicsSystem *system, const JPC_Shape *shape, JPC_Vec3 scale, const JPC_RMat44 *transform, JPC_Vec3 direction,
											 const JPC_ShapeCastSettings *in, const JPC_QueryFilter *filter, JPC_ShapeCollectorFn collector_fn, void *user_data)
{
	RShapeCast cast(ToJph(shape), ToJph(scale), ToJph(*transform), ToJph(direction));
	ShapeCastSettings settings;
	if (in != nullptr)
	{
		settings.mBackFaceModeTriangles = (EBackFaceMode)in->back_face_mode_triangles;
		settings.mBackFaceModeConvex = (EBackFaceMode)in->back_face_mode_convex;
		settings.mUseShrunkenShapeAndConvexRadius = in->use_shrunken_shape_and_convex_radius;
		settings.mReturnDeepestPoint = in->return_deepest_point;
		settings.mActiveEdgeMode = (EActiveEdgeMode)in->active_edge_mode;
		settings.mActiveEdgeMovementDirection = ToJph(in->active_edge_movement_direction);
		settings.mCollisionTolerance = in->collision_tolerance;
		settings.mPenetrationTolerance = in->penetration_tolerance;
	}
	QueryFilters filters(*system, filter);
	CallbackCastShapeCollector collector(collector_fn, user_data);
	system->mSystem.GetNarrowPhaseQuery().CastShape(cast, settings, RVec3::sZero(), collector, filters.mBroadPhase, filters.mObjectLayer, filters.mBody);
}

// ---------------------------------------------------------------------------
// Broad phase queries
// ---------------------------------------------------------------------------

namespace {

class BroadPhaseBodyCollector final : public CollideShapeBodyCollector
{
public:
	BroadPhaseBodyCollector(const QueryBodyFilter &inFilter, JPC_BodyCollectorFn inFn, void *inUserData) : mFilter(inFilter), mFn(inFn), mUserData(inUserData) { }

	virtual void			AddHit(const BodyID &inResult) override
	{
		// The broad phase knows nothing about body filters, so apply ours here
		if (mFilter.ShouldCollide(inResult) && !mFn(mUserData, ToC(inResult)))
			ForceEarlyOut();
	}

private:
	const QueryBodyFilter &	mFilter;
	JPC_BodyCollectorFn		mFn;
	void *					mUserData;
};

template <class Base>
class BroadPhaseCastCollector final : public Base
{
public:
	BroadPhaseCastCollector(const QueryBodyFilter &inFilter, JPC_BroadPhaseCastCollectorFn inFn, void *inUserData) : mFilter(inFilter), mFn(inFn), mUserData(inUserData) { }

	virtual void			AddHit(const BroadPhaseCastResult &inResult) override
	{
		if (!mFilter.ShouldCollide(inResult.mBodyID))
			return;
		JPC_BroadPhaseCastHit hit { ToC(inResult.mBodyID), inResult.mFraction };
		if (!mFn(mUserData, &hit))
			this->ForceEarlyOut();
	}

private:
	const QueryBodyFilter &	mFilter;
	JPC_BroadPhaseCastCollectorFn mFn;
	void *					mUserData;
};

} // namespace

#define JPC_BROAD_PHASE_QUERY(call)																	\
	QueryFilters filters(*system, filter);															\
	BroadPhaseBodyCollector collector(filters.mBody, collector_fn, user_data);						\
	system->mSystem.GetBroadPhaseQuery().call

void JPC_PhysicsSystem_BroadPhaseCollideAABox(const JPC_PhysicsSystem *system, const JPC_AABox *box, const JPC_QueryFilter *filter, JPC_BodyCollectorFn collector_fn, void *user_data)
{
	JPC_BROAD_PHASE_QUERY(CollideAABox(AABox(ToJph(box->min), ToJph(box->max)), collector, filters.mBroadPhase, filters.mObjectLayer));
}

void JPC_PhysicsSystem_BroadPhaseCollideSphere(const JPC_PhysicsSystem *system, JPC_Vec3 center, float radius, const JPC_QueryFilter *filter, JPC_BodyCollectorFn collector_fn, void *user_data)
{
	JPC_BROAD_PHASE_QUERY(CollideSphere(ToJph(center), radius, collector, filters.mBroadPhase, filters.mObjectLayer));
}

void JPC_PhysicsSystem_BroadPhaseCollidePoint(const JPC_PhysicsSystem *system, JPC_Vec3 point, const JPC_QueryFilter *filter, JPC_BodyCollectorFn collector_fn, void *user_data)
{
	JPC_BROAD_PHASE_QUERY(CollidePoint(ToJph(point), collector, filters.mBroadPhase, filters.mObjectLayer));
}

void JPC_PhysicsSystem_BroadPhaseCollideOrientedBox(const JPC_PhysicsSystem *system, const JPC_Mat44 *orientation, JPC_Vec3 half_extent, const JPC_QueryFilter *filter,
													JPC_BodyCollectorFn collector_fn, void *user_data)
{
	JPC_BROAD_PHASE_QUERY(CollideOrientedBox(OrientedBox(ToJph(*orientation), ToJph(half_extent)), collector, filters.mBroadPhase, filters.mObjectLayer));
}

void JPC_PhysicsSystem_BroadPhaseCastRay(const JPC_PhysicsSystem *system, JPC_Vec3 origin, JPC_Vec3 direction, const JPC_QueryFilter *filter,
										 JPC_BroadPhaseCastCollectorFn collector_fn, void *user_data)
{
	QueryFilters filters(*system, filter);
	BroadPhaseCastCollector<RayCastBodyCollector> collector(filters.mBody, collector_fn, user_data);
	system->mSystem.GetBroadPhaseQuery().CastRay(RayCast { ToJph(origin), ToJph(direction) }, collector, filters.mBroadPhase, filters.mObjectLayer);
}

void JPC_PhysicsSystem_BroadPhaseCastAABox(const JPC_PhysicsSystem *system, const JPC_AABox *box, JPC_Vec3 direction, const JPC_QueryFilter *filter,
										   JPC_BroadPhaseCastCollectorFn collector_fn, void *user_data)
{
	QueryFilters filters(*system, filter);
	BroadPhaseCastCollector<CastShapeBodyCollector> collector(filters.mBody, collector_fn, user_data);
	AABoxCast cast { AABox(ToJph(box->min), ToJph(box->max)), ToJph(direction) };
	system->mSystem.GetBroadPhaseQuery().CastAABox(cast, collector, filters.mBroadPhase, filters.mObjectLayer);
}

// ---------------------------------------------------------------------------
// Height field materials
// ---------------------------------------------------------------------------

JPC_Shape *JPC_HeightFieldShape_CreateWithMaterials(const float *samples, JPC_Vec3 offset, JPC_Vec3 scale, uint32_t sample_count, const uint8_t *material_indices,
													const JPC_PhysicsMaterial *const *materials, uint32_t num_materials)
{
	if (num_materials == 0 || num_materials > 256 || sample_count < 2)
	{
		JPC_SetError("Height field needs 1 to 256 materials and a sample count of at least 2");
		return nullptr;
	}
	PhysicsMaterialList material_list;
	for (uint32_t i = 0; i < num_materials; ++i)
	{
		if (materials[i] == nullptr)
		{
			JPC_SetError("Height field material is null");
			return nullptr;
		}
		material_list.push_back(ToJph(materials[i]));
	}
	uint32_t num_cells = (sample_count - 1) * (sample_count - 1);
	for (uint32_t i = 0; i < num_cells; ++i)
		if (material_indices[i] >= num_materials)
		{
			JPC_SetError("Height field material index out of range");
			return nullptr;
		}
	HeightFieldShapeSettings settings(samples, ToJph(offset), ToJph(scale), sample_count, material_indices, material_list);
	settings.SetEmbedded();
	ShapeSettings::ShapeResult result = settings.Create();
	if (result.HasError())
	{
		JPC_SetError(result.GetError().c_str());
		return nullptr;
	}
	Shape *shape = result.Get().GetPtr();
	shape->AddRef();
	return ToC(shape);
}

// ---------------------------------------------------------------------------
// Snapshots
// ---------------------------------------------------------------------------

struct JPC_PhysicsSnapshot
{
	JPH_OVERRIDE_NEW_DELETE

	std::string				mData;
	// Null for snapshots made from raw bytes, which are restored without checks
	const JPC_PhysicsSystem *mSystem = nullptr;
	uint64					mStructureVersion = 0;
};

JPC_PhysicsSnapshot *JPC_PhysicsSystem_SaveState(const JPC_PhysicsSystem *system, JPC_StateParts parts)
{
	StateRecorderImpl recorder;
	system->mSystem.SaveState(recorder, EStateRecorderState(parts & JPC_STATE_PARTS_ALL));
	JPC_PhysicsSnapshot *snapshot = new JPC_PhysicsSnapshot;
	snapshot->mData = recorder.GetData();
	snapshot->mSystem = system;
	snapshot->mStructureVersion = system->mStructureVersion;
	return snapshot;
}

bool JPC_PhysicsSystem_RestoreState(JPC_PhysicsSystem *system, const JPC_PhysicsSnapshot *snapshot)
{
	if (snapshot->mSystem != nullptr && (snapshot->mSystem != system || snapshot->mStructureVersion != system->mStructureVersion))
	{
		JPC_SetError("Snapshot was taken from a different system, or bodies, shapes or constraints changed since");
		return false;
	}
	StateRecorderImpl recorder;
	recorder.WriteBytes(snapshot->mData.data(), snapshot->mData.size());
	recorder.Rewind();
	if (!system->mSystem.RestoreState(recorder))
	{
		JPC_SetError("Snapshot does not match the world");
		return false;
	}
	return true;
}

JPC_PhysicsSnapshot *JPC_PhysicsSnapshot_CreateFromData(const void *data, size_t size)
{
	JPC_PhysicsSnapshot *snapshot = new JPC_PhysicsSnapshot;
	snapshot->mData.assign(static_cast<const char *>(data), size);
	return snapshot;
}

const void *JPC_PhysicsSnapshot_GetData(const JPC_PhysicsSnapshot *snapshot) { return snapshot->mData.data(); }
size_t JPC_PhysicsSnapshot_GetSize(const JPC_PhysicsSnapshot *snapshot) { return snapshot->mData.size(); }
void JPC_PhysicsSnapshot_Destroy(JPC_PhysicsSnapshot *snapshot) { delete snapshot; }

// ---------------------------------------------------------------------------
// Gear, rack and pinion, pulley and path constraints
// ---------------------------------------------------------------------------

void JPC_GearConstraintSettings_SetDefault(JPC_GearConstraintSettings *out)
{
	GearConstraintSettings s;
	out->base = ToC(s);
	out->space = (JPC_ConstraintSpace)s.mSpace;
	out->hinge_axis1 = ToC(s.mHingeAxis1);
	out->hinge_axis2 = ToC(s.mHingeAxis2);
	out->ratio = s.mRatio;
}

void JPC_RackAndPinionConstraintSettings_SetDefault(JPC_RackAndPinionConstraintSettings *out)
{
	RackAndPinionConstraintSettings s;
	out->base = ToC(s);
	out->space = (JPC_ConstraintSpace)s.mSpace;
	out->hinge_axis = ToC(s.mHingeAxis);
	out->slider_axis = ToC(s.mSliderAxis);
	out->ratio = s.mRatio;
}

void JPC_PulleyConstraintSettings_SetDefault(JPC_PulleyConstraintSettings *out)
{
	PulleyConstraintSettings s;
	out->base = ToC(s);
	out->space = (JPC_ConstraintSpace)s.mSpace;
	out->body_point1 = ToC(s.mBodyPoint1);
	out->fixed_point1 = ToC(s.mFixedPoint1);
	out->body_point2 = ToC(s.mBodyPoint2);
	out->fixed_point2 = ToC(s.mFixedPoint2);
	out->ratio = s.mRatio;
	out->min_length = s.mMinLength;
	out->max_length = s.mMaxLength;
}

void JPC_PathConstraintSettings_SetDefault(JPC_PathConstraintSettings *out)
{
	PathConstraintSettings s;
	out->base = ToC(s);
	out->path = nullptr;
	out->path_position = ToC(s.mPathPosition);
	out->path_rotation = ToC(s.mPathRotation);
	out->path_fraction = s.mPathFraction;
	out->max_friction_force = s.mMaxFrictionForce;
	out->position_motor_settings = ToC(s.mPositionMotorSettings);
	out->rotation_constraint_type = (JPC_PathRotationConstraintType)s.mRotationConstraintType;
}

JPC_Constraint *JPC_PhysicsSystem_CreateGearConstraint(JPC_PhysicsSystem *system, const JPC_GearConstraintSettings *in, JPC_BodyID body1, JPC_BodyID body2)
{
	GearConstraintSettings s;
	ToJph(in->base, s);
	s.mSpace = (EConstraintSpace)in->space;
	s.mHingeAxis1 = ToJph(in->hinge_axis1);
	s.mHingeAxis2 = ToJph(in->hinge_axis2);
	s.mRatio = in->ratio;
	return CreateConstraint(system, s, body1, body2);
}

JPC_Constraint *JPC_PhysicsSystem_CreateRackAndPinionConstraint(JPC_PhysicsSystem *system, const JPC_RackAndPinionConstraintSettings *in, JPC_BodyID body1, JPC_BodyID body2)
{
	RackAndPinionConstraintSettings s;
	ToJph(in->base, s);
	s.mSpace = (EConstraintSpace)in->space;
	s.mHingeAxis = ToJph(in->hinge_axis);
	s.mSliderAxis = ToJph(in->slider_axis);
	s.mRatio = in->ratio;
	return CreateConstraint(system, s, body1, body2);
}

JPC_Constraint *JPC_PhysicsSystem_CreatePulleyConstraint(JPC_PhysicsSystem *system, const JPC_PulleyConstraintSettings *in, JPC_BodyID body1, JPC_BodyID body2)
{
	if (in->ratio <= 0.0f || in->min_length < 0.0f || (in->max_length >= 0.0f && in->min_length > in->max_length))
	{
		JPC_SetError("Pulley needs a positive ratio and 0 <= min_length <= max_length");
		return nullptr;
	}
	PulleyConstraintSettings s;
	ToJph(in->base, s);
	s.mSpace = (EConstraintSpace)in->space;
	s.mBodyPoint1 = ToJph(in->body_point1);
	s.mFixedPoint1 = ToJph(in->fixed_point1);
	s.mBodyPoint2 = ToJph(in->body_point2);
	s.mFixedPoint2 = ToJph(in->fixed_point2);
	s.mRatio = in->ratio;
	s.mMinLength = in->min_length;
	s.mMaxLength = in->max_length;
	return CreateConstraint(system, s, body1, body2);
}

static inline const PathConstraintPath *ToJph(const JPC_PathConstraintPath *p) { return reinterpret_cast<const PathConstraintPath *>(p); }

JPC_PathConstraintPath *JPC_PathConstraintPathHermite_Create(const JPC_PathPoint *points, uint32_t num_points, bool looping)
{
	if (num_points < 2)
	{
		JPC_SetError("A path needs at least 2 points");
		return nullptr;
	}
	PathConstraintPathHermite *path = new PathConstraintPathHermite;
	for (uint32_t i = 0; i < num_points; ++i)
		path->AddPoint(ToJph(points[i].position), ToJph(points[i].tangent), ToJph(points[i].normal));
	path->SetIsLooping(looping);
	path->AddRef();
	return reinterpret_cast<JPC_PathConstraintPath *>(static_cast<PathConstraintPath *>(path));
}

void JPC_PathConstraintPath_AddRef(const JPC_PathConstraintPath *path) { ToJph(path)->AddRef(); }
void JPC_PathConstraintPath_Release(const JPC_PathConstraintPath *path) { ToJph(path)->Release(); }
float JPC_PathConstraintPath_GetMaxFraction(const JPC_PathConstraintPath *path) { return ToJph(path)->GetPathMaxFraction(); }

JPC_Constraint *JPC_PhysicsSystem_CreatePathConstraint(JPC_PhysicsSystem *system, const JPC_PathConstraintSettings *in, JPC_BodyID body1, JPC_BodyID body2)
{
	if (in->path == nullptr)
	{
		JPC_SetError("Path constraint needs a path");
		return nullptr;
	}
	PathConstraintSettings s;
	ToJph(in->base, s);
	s.mPath = ToJph(in->path);
	s.mPathPosition = ToJph(in->path_position);
	s.mPathRotation = ToJph(in->path_rotation).Normalized();
	s.mPathFraction = in->path_fraction;
	s.mMaxFrictionForce = in->max_friction_force;
	s.mPositionMotorSettings = ToJph(in->position_motor_settings);
	s.mRotationConstraintType = (EPathRotationConstraintType)in->rotation_constraint_type;
	return CreateConstraint(system, s, body1, body2);
}

#define JPC_PULLEY(c) As<PulleyConstraint, EConstraintSubType::Pulley>(c)
#define JPC_PATH(c) As<PathConstraint, EConstraintSubType::Path>(c)

void JPC_PulleyConstraint_SetLength(JPC_Constraint *constraint, float min_length, float max_length)
{
	if (auto c = JPC_PULLEY(constraint); c != nullptr && min_length >= 0.0f && min_length <= max_length)
		c->SetLength(min_length, max_length);
}

float JPC_PulleyConstraint_GetCurrentLength(const JPC_Constraint *constraint) { auto c = JPC_PULLEY(constraint); return c ? c->GetCurrentLength() : 0.0f; }
float JPC_PathConstraint_GetPathFraction(const JPC_Constraint *constraint) { auto c = JPC_PATH(constraint); return c ? c->GetPathFraction() : 0.0f; }
void JPC_PathConstraint_SetMaxFrictionForce(JPC_Constraint *constraint, float force) { if (auto c = JPC_PATH(constraint)) c->SetMaxFrictionForce(force); }
void JPC_PathConstraint_SetPositionMotorState(JPC_Constraint *constraint, JPC_MotorState state) { if (auto c = JPC_PATH(constraint)) c->SetPositionMotorState((EMotorState)state); }
void JPC_PathConstraint_SetTargetVelocity(JPC_Constraint *constraint, float velocity) { if (auto c = JPC_PATH(constraint)) c->SetTargetVelocity(velocity); }

void JPC_PathConstraint_SetTargetPathFraction(JPC_Constraint *constraint, float fraction)
{
	PathConstraint *c = JPC_PATH(constraint);
	if (c == nullptr)
		return;
	const PathConstraintPath *path = c->GetPath();
	if (!path->IsLooping())
		fraction = Clamp(fraction, 0.0f, path->GetPathMaxFraction());
	c->SetTargetPathFraction(fraction);
}
