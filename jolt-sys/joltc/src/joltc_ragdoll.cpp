#include "joltc_internal.h"

#include <Jolt/Physics/Collision/GroupFilterTable.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>
#include <Jolt/Skeleton/Skeleton.h>
#include <Jolt/Skeleton/SkeletonPose.h>

static inline Skeleton *ToJph(JPC_Skeleton *s) { return reinterpret_cast<Skeleton *>(s); }
static inline const Skeleton *ToJph(const JPC_Skeleton *s) { return reinterpret_cast<const Skeleton *>(s); }
static inline const RagdollSettings *ToJph(const JPC_RagdollSettings *s) { return reinterpret_cast<const RagdollSettings *>(s); }

static Array<Mat44> ToJph(const JPC_Mat44 *inMatrices, uint32_t inCount)
{
	Array<Mat44> out;
	out.reserve(inCount);
	for (uint32_t i = 0; i < inCount; ++i)
		out.push_back(ToJph(inMatrices[i]));
	return out;
}

// ---------------------------------------------------------------------------
// Skeleton
// ---------------------------------------------------------------------------

JPC_Skeleton *JPC_Skeleton_Create(void)
{
	Skeleton *skeleton = new Skeleton;
	skeleton->AddRef();
	return reinterpret_cast<JPC_Skeleton *>(skeleton);
}

int32_t JPC_Skeleton_AddJoint(JPC_Skeleton *skeleton, const char *name, int32_t parent_index)
{
	Skeleton *s = ToJph(skeleton);
	// Requiring parents to exist keeps the joints ordered parent first, which ragdolls and poses rely on
	if (parent_index < -1 || parent_index >= s->GetJointCount())
	{
		JPC_SetError("Parent joint does not exist");
		return -1;
	}
	return int32_t(s->AddJoint(name != nullptr ? name : "", parent_index));
}

uint32_t JPC_Skeleton_GetJointCount(const JPC_Skeleton *skeleton) { return uint32_t(ToJph(skeleton)->GetJointCount()); }
int32_t JPC_Skeleton_GetJointIndex(const JPC_Skeleton *skeleton, const char *name) { return ToJph(skeleton)->GetJointIndex(name != nullptr ? name : ""); }

const char *JPC_Skeleton_GetJointName(const JPC_Skeleton *skeleton, uint32_t joint)
{
	const Skeleton *s = ToJph(skeleton);
	return joint < uint32_t(s->GetJointCount()) ? s->GetJoint(joint).mName.c_str() : nullptr;
}

int32_t JPC_Skeleton_GetParentIndex(const JPC_Skeleton *skeleton, uint32_t joint)
{
	const Skeleton *s = ToJph(skeleton);
	return joint < uint32_t(s->GetJointCount()) ? s->GetJoint(joint).mParentJointIndex : -1;
}

void JPC_Skeleton_AddRef(const JPC_Skeleton *skeleton) { ToJph(skeleton)->AddRef(); }
void JPC_Skeleton_Release(const JPC_Skeleton *skeleton) { ToJph(skeleton)->Release(); }

void JPC_Skeleton_CalculateJointMatrices(const JPC_Skeleton *skeleton, const JPC_JointState *states, JPC_Mat44 *out_matrices)
{
	SkeletonPose pose;
	pose.SetSkeleton(ToJph(skeleton));
	for (uint i = 0; i < pose.GetJointCount(); ++i)
	{
		pose.GetJoint(i).mTranslation = ToJph(states[i].translation);
		pose.GetJoint(i).mRotation = ToJph(states[i].rotation).Normalized();
	}
	pose.CalculateJointMatrices();
	for (uint i = 0; i < pose.GetJointCount(); ++i)
		out_matrices[i] = ToC(pose.GetJointMatrix(i));
}

void JPC_Skeleton_CalculateJointStates(const JPC_Skeleton *skeleton, const JPC_Mat44 *matrices, JPC_JointState *out_states)
{
	SkeletonPose pose;
	pose.SetSkeleton(ToJph(skeleton));
	for (uint i = 0; i < pose.GetJointCount(); ++i)
		pose.GetJointMatrix(i) = ToJph(matrices[i]);
	pose.CalculateJointStates();
	for (uint i = 0; i < pose.GetJointCount(); ++i)
		out_states[i] = { ToC(pose.GetJoint(i).mTranslation), ToC(pose.GetJoint(i).mRotation) };
}

// ---------------------------------------------------------------------------
// Ragdoll settings
// ---------------------------------------------------------------------------

static bool IsSupportedJoint(JPC_ConstraintType inType) { return inType >= JPC_CONSTRAINT_TYPE_FIXED && inType <= JPC_CONSTRAINT_TYPE_SIX_DOF; }

static const char *ValidateRagdoll(const JPC_RagdollSettingsDesc &in)
{
	if (in.skeleton == nullptr)
		return "Ragdoll needs a skeleton";
	const Skeleton *skeleton = ToJph(in.skeleton);
	uint32_t num_joints = uint32_t(skeleton->GetJointCount());
	if (num_joints == 0 || in.num_parts != num_joints || in.parts == nullptr)
		return "Ragdoll needs exactly one part per skeleton joint";
	for (uint32_t i = 0; i < in.num_parts; ++i)
	{
		const JPC_RagdollPart &part = in.parts[i];
		if (part.body.shape == nullptr)
			return "Ragdoll part has no shape";
		if (part.body.allowed_dofs == JPC_ALLOWED_DOFS_NONE && part.body.motion_type != JPC_MOTION_TYPE_STATIC)
			return "Ragdoll part allowed_dofs must not be NONE";
		bool is_root = skeleton->GetJoint(i).mParentJointIndex < 0;
		if (part.to_parent.type != JPC_CONSTRAINT_TYPE_NONE && (is_root || !IsSupportedJoint(part.to_parent.type)))
			return "Root parts can't have a parent joint, and joints must be fixed, point, distance, hinge, slider, cone, swing-twist or six-DOF";
	}
	if (in.num_additional_constraints > 0 && in.additional_constraints == nullptr)
		return "additional_constraints is null";
	for (uint32_t i = 0; i < in.num_additional_constraints; ++i)
	{
		const JPC_RagdollConstraint &c = in.additional_constraints[i];
		if (c.body1 >= in.num_parts || c.body2 >= in.num_parts || c.body1 == c.body2 || !IsSupportedJoint(c.constraint.type))
			return "Additional ragdoll constraint needs two different existing parts and a supported joint type";
	}
	return nullptr;
}

JPC_RagdollSettings *JPC_RagdollSettings_Create(const JPC_RagdollSettingsDesc *in)
{
	if (const char *error = ValidateRagdoll(*in))
	{
		JPC_SetError(error);
		return nullptr;
	}

	Ref<RagdollSettings> settings = new RagdollSettings;
	settings->mSkeleton = const_cast<Skeleton *>(ToJph(in->skeleton));
	settings->mParts.resize(in->num_parts);
	for (uint32_t i = 0; i < in->num_parts; ++i)
	{
		RagdollSettings::Part &part = settings->mParts[i];
		static_cast<BodyCreationSettings &>(part) = JPC_ToJphBodySettings(in->parts[i].body);
		part.mToParent = JPC_MakeConstraintSettings(in->parts[i].to_parent);
	}
	for (uint32_t i = 0; i < in->num_additional_constraints; ++i)
	{
		const JPC_RagdollConstraint &c = in->additional_constraints[i];
		settings->mAdditionalConstraints.push_back(RagdollSettings::AdditionalConstraint(int(c.body1), int(c.body2), JPC_MakeConstraintSettings(c.constraint).GetPtr()));
	}

	if (in->stabilize && !settings->Stabilize())
	{
		JPC_SetError("Ragdoll could not be stabilized (are all parts dynamic with a valid mass?)");
		return nullptr;
	}

	if (in->disable_parent_child_collisions)
	{
		Array<Mat44> pose;
		if (in->pose_matrices != nullptr)
			pose = ToJph(in->pose_matrices, in->num_parts);
		settings->DisableParentChildCollisions(pose.empty() ? nullptr : pose.data(), in->min_separation_distance);

		// Jolt installs its own GroupFilterTable, which reads out of bounds for sub groups beyond its size if a user
		// reuses it for other bodies. Swap in the bounds checked table with the same contents.
		const GroupFilterTable *jolt_filter = static_cast<const GroupFilterTable *>(settings->mParts[0].mCollisionGroup.GetGroupFilter());
		Ref<SafeGroupFilterTable> safe = new SafeGroupFilterTable(in->num_parts);
		for (uint32_t a = 0; a < in->num_parts; ++a)
			for (uint32_t b = a + 1; b < in->num_parts; ++b)
				safe->SetCollision(a, b, jolt_filter->IsCollisionEnabled(a, b));
		for (RagdollSettings::Part &part : settings->mParts)
			part.mCollisionGroup.SetGroupFilter(safe);
	}

	if (in->calculate_constraint_priorities)
		settings->CalculateConstraintPriorities(in->base_priority);

	// Needed by the pose driving functions
	settings->CalculateBodyIndexToConstraintIndex();
	settings->CalculateConstraintIndexToBodyIdxPair();

	settings->AddRef();
	return reinterpret_cast<JPC_RagdollSettings *>(settings.GetPtr());
}

void JPC_RagdollSettings_AddRef(const JPC_RagdollSettings *settings) { ToJph(settings)->AddRef(); }
void JPC_RagdollSettings_Release(const JPC_RagdollSettings *settings) { ToJph(settings)->Release(); }
const JPC_Skeleton *JPC_RagdollSettings_GetSkeleton(const JPC_RagdollSettings *settings) { return reinterpret_cast<const JPC_Skeleton *>(ToJph(settings)->GetSkeleton()); }

// ---------------------------------------------------------------------------
// Ragdoll
// ---------------------------------------------------------------------------

struct JPC_Ragdoll
{
	JPH_OVERRIDE_NEW_DELETE

	JPC_PhysicsSystem *		mSystem = nullptr;
	Ref<Ragdoll>			mRagdoll;
	// Guarded by the system's mBodyStateMutex
	bool					mAdded = false;
};

JPC_Ragdoll *JPC_RagdollSettings_CreateRagdoll(const JPC_RagdollSettings *settings, JPC_PhysicsSystem *system, uint32_t group_id, uint64_t user_data)
{
	const RagdollSettings *s = ToJph(settings);
	for (const RagdollSettings::Part &part : s->mParts)
		if (!system->IsValidLayer(part.mObjectLayer))
		{
			JPC_SetError("Ragdoll part layer does not exist in this physics system");
			return nullptr;
		}

	std::unique_lock lock(system->mBodyStateMutex);
	Ragdoll *ragdoll = s->CreateRagdoll(group_id, user_data, &system->mSystem);
	if (ragdoll == nullptr)
	{
		JPC_SetError("Out of bodies (max_bodies reached)");
		return nullptr;
	}
	for (const BodyID &id : ragdoll->GetBodyIDs())
		system->mProtectedBodies.insert(id.GetIndexAndSequenceNumber());

	JPC_Ragdoll *out = new JPC_Ragdoll;
	out->mSystem = system;
	out->mRagdoll = ragdoll;
	return out;
}

void JPC_Ragdoll_Destroy(JPC_Ragdoll *ragdoll)
{
	JPC_PhysicsSystem *system = ragdoll->mSystem;
	{
		std::unique_lock lock(system->mBodyStateMutex);
		if (ragdoll->mAdded)
		{
			// Also removes the constraints, which the ragdoll owns
			ragdoll->mRagdoll->RemoveFromPhysicsSystem();
			system->BumpStructure();
		}
		for (const BodyID &id : ragdoll->mRagdoll->GetBodyIDs())
			system->Unprotect(id);
		// ~Ragdoll destroys the bodies
		ragdoll->mRagdoll = nullptr;
	}
	delete ragdoll;
}

void JPC_Ragdoll_AddToPhysicsSystem(JPC_Ragdoll *ragdoll, JPC_Activation activation)
{
	std::unique_lock lock(ragdoll->mSystem->mBodyStateMutex);
	if (ragdoll->mAdded)
		return;
	ragdoll->mRagdoll->AddToPhysicsSystem(ToJph(activation));
	ragdoll->mAdded = true;
	ragdoll->mSystem->BumpStructure();
}

void JPC_Ragdoll_RemoveFromPhysicsSystem(JPC_Ragdoll *ragdoll)
{
	std::unique_lock lock(ragdoll->mSystem->mBodyStateMutex);
	if (!ragdoll->mAdded)
		return;
	ragdoll->mRagdoll->RemoveFromPhysicsSystem();
	ragdoll->mAdded = false;
	ragdoll->mSystem->BumpStructure();
}

bool JPC_Ragdoll_IsAdded(const JPC_Ragdoll *ragdoll)
{
	std::shared_lock lock(ragdoll->mSystem->mBodyStateMutex);
	return ragdoll->mAdded;
}

// Activate and AddImpulse activate bodies without checking that they are in the world
void JPC_Ragdoll_Activate(JPC_Ragdoll *ragdoll)
{
	std::shared_lock lock(ragdoll->mSystem->mBodyStateMutex);
	if (ragdoll->mAdded)
		ragdoll->mRagdoll->Activate();
}

void JPC_Ragdoll_AddImpulse(JPC_Ragdoll *ragdoll, JPC_Vec3 impulse)
{
	std::shared_lock lock(ragdoll->mSystem->mBodyStateMutex);
	if (ragdoll->mAdded)
		ragdoll->mRagdoll->AddImpulse(ToJph(impulse));
}

bool JPC_Ragdoll_IsActive(const JPC_Ragdoll *ragdoll) { return ragdoll->mRagdoll->IsActive(); }
void JPC_Ragdoll_SetGroupID(JPC_Ragdoll *ragdoll, uint32_t group_id) { ragdoll->mRagdoll->SetGroupID(group_id); }

void JPC_Ragdoll_SetPose(JPC_Ragdoll *ragdoll, JPC_RVec3 root_offset, const JPC_Mat44 *matrices)
{
	Array<Mat44> m = ToJph(matrices, uint32_t(ragdoll->mRagdoll->GetBodyCount()));
	ragdoll->mRagdoll->SetPose(ToJph(root_offset), m.data());
}

void JPC_Ragdoll_GetPose(JPC_Ragdoll *ragdoll, JPC_RVec3 *out_root_offset, JPC_Mat44 *out_matrices)
{
	Array<Mat44> m(ragdoll->mRagdoll->GetBodyCount());
	RVec3 root;
	ragdoll->mRagdoll->GetPose(root, m.data());
	*out_root_offset = ToC(root);
	for (size_t i = 0; i < m.size(); ++i)
		out_matrices[i] = ToC(m[i]);
}

void JPC_Ragdoll_DriveToPoseUsingKinematics(JPC_Ragdoll *ragdoll, JPC_RVec3 root_offset, const JPC_Mat44 *matrices, float delta_time)
{
	if (delta_time <= 0.0f)
		return;
	Array<Mat44> m = ToJph(matrices, uint32_t(ragdoll->mRagdoll->GetBodyCount()));
	ragdoll->mRagdoll->DriveToPoseUsingKinematics(ToJph(root_offset), m.data(), delta_time);
}

void JPC_Ragdoll_DriveToPoseUsingMotors(JPC_Ragdoll *ragdoll, JPC_RVec3 root_offset, const JPC_Mat44 *matrices)
{
	SkeletonPose pose;
	pose.SetSkeleton(ragdoll->mRagdoll->GetRagdollSettings()->GetSkeleton());
	pose.SetRootOffset(ToJph(root_offset));
	for (uint i = 0; i < pose.GetJointCount(); ++i)
		pose.GetJointMatrix(i) = ToJph(matrices[i]);
	pose.CalculateJointStates();
	ragdoll->mRagdoll->DriveToPoseUsingMotors(pose);
}

void JPC_Ragdoll_ResetWarmStart(JPC_Ragdoll *ragdoll) { ragdoll->mRagdoll->ResetWarmStart(); }
void JPC_Ragdoll_SetLinearAndAngularVelocity(JPC_Ragdoll *ragdoll, JPC_Vec3 linear, JPC_Vec3 angular) { ragdoll->mRagdoll->SetLinearAndAngularVelocity(ToJph(linear), ToJph(angular)); }
void JPC_Ragdoll_AddLinearVelocity(JPC_Ragdoll *ragdoll, JPC_Vec3 velocity) { ragdoll->mRagdoll->AddLinearVelocity(ToJph(velocity)); }

void JPC_Ragdoll_GetRootTransform(const JPC_Ragdoll *ragdoll, JPC_RVec3 *out_position, JPC_Quat *out_rotation)
{
	RVec3 position;
	Quat rotation;
	ragdoll->mRagdoll->GetRootTransform(position, rotation);
	*out_position = ToC(position);
	*out_rotation = ToC(rotation);
}

JPC_AABox JPC_Ragdoll_GetWorldSpaceBounds(const JPC_Ragdoll *ragdoll) { return ToC(ragdoll->mRagdoll->GetWorldSpaceBounds()); }
uint32_t JPC_Ragdoll_GetBodyCount(const JPC_Ragdoll *ragdoll) { return uint32_t(ragdoll->mRagdoll->GetBodyCount()); }

JPC_BodyID JPC_Ragdoll_GetBodyID(const JPC_Ragdoll *ragdoll, uint32_t index)
{
	return index < ragdoll->mRagdoll->GetBodyCount() ? ToC(ragdoll->mRagdoll->GetBodyID(int(index))) : JPC_BODY_ID_INVALID;
}

uint32_t JPC_Ragdoll_GetConstraintCount(const JPC_Ragdoll *ragdoll) { return uint32_t(ragdoll->mRagdoll->GetConstraintCount()); }

JPC_Constraint *JPC_Ragdoll_GetConstraint(JPC_Ragdoll *ragdoll, uint32_t index)
{
	if (index >= ragdoll->mRagdoll->GetConstraintCount())
		return nullptr;
	return reinterpret_cast<JPC_Constraint *>(static_cast<Constraint *>(ragdoll->mRagdoll->GetConstraint(int(index))));
}

const JPC_RagdollSettings *JPC_Ragdoll_GetSettings(const JPC_Ragdoll *ragdoll) { return reinterpret_cast<const JPC_RagdollSettings *>(ragdoll->mRagdoll->GetRagdollSettings()); }
