// Skeletal animation and skeleton mapping.
#include "joltc_internal.h"

#include <Jolt/Skeleton/SkeletalAnimation.h>
#include <Jolt/Skeleton/Skeleton.h>
#include <Jolt/Skeleton/SkeletonMapper.h>
#include <Jolt/Skeleton/SkeletonPose.h>

#include <cmath>

static inline const Skeleton *ToJph(const JPC_Skeleton *s) { return reinterpret_cast<const Skeleton *>(s); }
static inline const SkeletalAnimation *ToJph(const JPC_SkeletalAnimation *a) { return reinterpret_cast<const SkeletalAnimation *>(a); }

// ---------------------------------------------------------------------------
// Animation
// ---------------------------------------------------------------------------

JPC_SkeletalAnimation *JPC_SkeletalAnimation_Create(const JPC_AnimatedJoint *joints, uint32_t num_joints, bool looping)
{
	Ref<SkeletalAnimation> animation = new SkeletalAnimation;
	animation->SetIsLooping(looping);
	SkeletalAnimation::AnimatedJointVector &out = animation->GetAnimatedJoints();
	out.reserve(num_joints);
	for (uint32_t i = 0; i < num_joints; ++i)
	{
		const JPC_AnimatedJoint &in = joints[i];
		// Sampling reads the first and last keyframe of every joint
		if (in.num_keyframes == 0 || in.keyframes == nullptr)
		{
			JPC_SetError("Every animated joint needs at least one keyframe");
			return nullptr;
		}
		SkeletalAnimation::AnimatedJoint joint;
		joint.mJointName = in.name != nullptr ? in.name : "";
		joint.mKeyframes.reserve(in.num_keyframes);
		for (uint32_t k = 0; k < in.num_keyframes; ++k)
		{
			const JPC_AnimationKeyframe &key = in.keyframes[k];
			// The keyframe lookup is a binary search over time
			if (!std::isfinite(key.time) || (k > 0 && key.time < in.keyframes[k - 1].time))
			{
				JPC_SetError("Keyframe times must be finite and in increasing order");
				return nullptr;
			}
			Quat rotation = ToJph(key.rotation);
			if (!(rotation.LengthSq() > 1.0e-12f))
			{
				JPC_SetError("Keyframe rotation has length 0");
				return nullptr;
			}
			SkeletalAnimation::Keyframe keyframe;
			keyframe.mTime = key.time;
			keyframe.mTranslation = ToJph(key.translation);
			keyframe.mRotation = rotation.Normalized();
			joint.mKeyframes.push_back(keyframe);
		}
		out.push_back(std::move(joint));
	}
	animation->AddRef();
	return reinterpret_cast<JPC_SkeletalAnimation *>(animation.GetPtr());
}

void JPC_SkeletalAnimation_AddRef(const JPC_SkeletalAnimation *animation) { ToJph(animation)->AddRef(); }
void JPC_SkeletalAnimation_Release(const JPC_SkeletalAnimation *animation) { ToJph(animation)->Release(); }
float JPC_SkeletalAnimation_GetDuration(const JPC_SkeletalAnimation *animation) { return ToJph(animation)->GetDuration(); }
bool JPC_SkeletalAnimation_IsLooping(const JPC_SkeletalAnimation *animation) { return ToJph(animation)->IsLooping(); }

bool JPC_SkeletalAnimation_Sample(const JPC_SkeletalAnimation *animation, const JPC_Skeleton *skeleton, float time, JPC_JointState *io_states)
{
	const SkeletalAnimation *a = ToJph(animation);
	const Skeleton *s = ToJph(skeleton);
	// Jolt looks joints up by name and indexes the pose with the result, -1 included
	for (const SkeletalAnimation::AnimatedJoint &joint : a->GetAnimatedJoints())
		if (s->GetJointIndex(joint.mJointName) < 0)
		{
			JPC_SetError("Animation animates a joint the skeleton does not have");
			return false;
		}

	SkeletonPose pose;
	pose.SetSkeleton(s);
	for (uint i = 0; i < pose.GetJointCount(); ++i)
	{
		pose.GetJoint(i).mTranslation = ToJph(io_states[i].translation);
		pose.GetJoint(i).mRotation = ToJph(io_states[i].rotation);
	}
	a->Sample(time > 0.0f ? time : 0.0f, pose);
	for (uint i = 0; i < pose.GetJointCount(); ++i)
	{
		io_states[i].translation = ToC(pose.GetJoint(i).mTranslation);
		io_states[i].rotation = ToC(pose.GetJoint(i).mRotation);
	}
	return true;
}

// ---------------------------------------------------------------------------
// Skeleton mapper
// ---------------------------------------------------------------------------

struct JPC_SkeletonMapper
{
	JPH_OVERRIDE_NEW_DELETE

	std::atomic<uint32>		mRefCount { 1 };
	SkeletonMapper			mMapper;
	uint32					mJointCount1 = 0;
	uint32					mJointCount2 = 0;
};

static Array<Mat44> ToJph(const JPC_Mat44 *inMatrices, uint32 inCount)
{
	Array<Mat44> out;
	out.reserve(inCount);
	for (uint32 i = 0; i < inCount; ++i)
		out.push_back(ToJph(inMatrices[i]));
	return out;
}

static void ToC(const Array<Mat44> &inMatrices, JPC_Mat44 *outMatrices)
{
	for (size_t i = 0; i < inMatrices.size(); ++i)
		outMatrices[i] = ToC(inMatrices[i]);
}

JPC_SkeletonMapper *JPC_SkeletonMapper_Create(const JPC_Skeleton *skeleton1, const JPC_Mat44 *neutral_pose1, const JPC_Skeleton *skeleton2,
											  const JPC_Mat44 *neutral_pose2, const bool *locked_translations, bool lock_all_translations)
{
	const Skeleton *s1 = ToJph(skeleton1), *s2 = ToJph(skeleton2);
	uint32 n1 = uint32(s1->GetJointCount()), n2 = uint32(s2->GetJointCount());
	if (n1 > n2)
	{
		JPC_SetError("Skeleton 1 must be the low detail skeleton (with no more joints than skeleton 2)");
		return nullptr;
	}
	if (locked_translations != nullptr && lock_all_translations)
	{
		JPC_SetError("Pass either locked_translations or lock_all_translations");
		return nullptr;
	}

	Array<Mat44> pose1 = ToJph(neutral_pose1, n1), pose2 = ToJph(neutral_pose2, n2);
	JPC_SkeletonMapper *mapper = new JPC_SkeletonMapper;
	mapper->mJointCount1 = n1;
	mapper->mJointCount2 = n2;
	mapper->mMapper.Initialize(s1, pose1.data(), s2, pose2.data());
	if (locked_translations != nullptr)
	{
		// Map sets a locked joint's translation from its parent, so a root (without one) can't be locked
		std::unique_ptr<bool[]> locked(new bool[n2]);
		for (uint32 i = 0; i < n2; ++i)
			locked[i] = locked_translations[i] && s2->GetJoint(i).mParentJointIndex >= 0;
		mapper->mMapper.LockTranslations(s2, locked.get(), pose2.data());
	}
	// Locking all starts from the first mapping
	else if (lock_all_translations && !mapper->mMapper.GetMappings().empty())
		mapper->mMapper.LockAllTranslations(s2, pose2.data());
	return mapper;
}

void JPC_SkeletonMapper_AddRef(const JPC_SkeletonMapper *mapper) { const_cast<JPC_SkeletonMapper *>(mapper)->mRefCount.fetch_add(1, std::memory_order_relaxed); }

void JPC_SkeletonMapper_Release(const JPC_SkeletonMapper *mapper)
{
	JPC_SkeletonMapper *m = const_cast<JPC_SkeletonMapper *>(mapper);
	if (m->mRefCount.fetch_sub(1, std::memory_order_acq_rel) == 1)
		delete m;
}

void JPC_SkeletonMapper_GetJointCounts(const JPC_SkeletonMapper *mapper, uint32_t *out_count1, uint32_t *out_count2)
{
	*out_count1 = mapper->mJointCount1;
	*out_count2 = mapper->mJointCount2;
}

void JPC_SkeletonMapper_Map(const JPC_SkeletonMapper *mapper, const JPC_Mat44 *pose1_model_space, const JPC_Mat44 *pose2_local_space, JPC_Mat44 *out_pose2_model_space)
{
	Array<Mat44> pose1 = ToJph(pose1_model_space, mapper->mJointCount1);
	Array<Mat44> local2 = ToJph(pose2_local_space, mapper->mJointCount2);
	// Joints the mapping doesn't reach keep what the caller passed in
	Array<Mat44> out2 = ToJph(out_pose2_model_space, mapper->mJointCount2);
	mapper->mMapper.Map(pose1.data(), local2.data(), out2.data());
	ToC(out2, out_pose2_model_space);
}

void JPC_SkeletonMapper_MapReverse(const JPC_SkeletonMapper *mapper, const JPC_Mat44 *pose2_model_space, JPC_Mat44 *out_pose1_model_space)
{
	Array<Mat44> pose2 = ToJph(pose2_model_space, mapper->mJointCount2);
	Array<Mat44> out1 = ToJph(out_pose1_model_space, mapper->mJointCount1);
	mapper->mMapper.MapReverse(pose2.data(), out1.data());
	ToC(out1, out_pose1_model_space);
}

int32_t JPC_SkeletonMapper_GetMappedJointIndex(const JPC_SkeletonMapper *mapper, uint32_t joint1)
{
	return joint1 < mapper->mJointCount1 ? mapper->mMapper.GetMappedJointIdx(int(joint1)) : -1;
}

bool JPC_SkeletonMapper_IsJointTranslationLocked(const JPC_SkeletonMapper *mapper, uint32_t joint2)
{
	return joint2 < mapper->mJointCount2 && mapper->mMapper.IsJointTranslationLocked(int(joint2));
}
