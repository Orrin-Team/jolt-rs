//! Keyframed skeletal animation, and mapping poses between a ragdoll's skeleton and a more detailed animation
//! skeleton.

use std::ffi::CString;
use std::fmt;
use std::ptr::NonNull;

use jolt_sys as sys;

use crate::{Error, JointState, Mat44, Quat, Result, Skeleton, Vec3};

/// A joint's local transform at a point in time.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Keyframe {
    /// Seconds.
    pub time: f32,
    pub translation: Vec3,
    pub rotation: Quat,
}

/// The keyframes of one joint, matched by name against the skeleton when sampling.
#[derive(Clone, Debug, PartialEq)]
pub struct AnimatedJoint {
    pub name: String,
    /// At least one, in time order.
    pub keyframes: Vec<Keyframe>,
}

/// A keyframed animation. Immutable and reference counted.
pub struct SkeletalAnimation {
    raw: NonNull<sys::JPC_SkeletalAnimation>,
}

// SAFETY: immutable after creation, with an atomic reference count.
unsafe impl Send for SkeletalAnimation {}
unsafe impl Sync for SkeletalAnimation {}

impl SkeletalAnimation {
    /// Fails for a joint without keyframes, keyframes out of time order or a rotation of length 0. Rotations are
    /// normalized.
    pub fn new(joints: &[AnimatedJoint], looping: bool) -> Result<Self> {
        crate::ensure_init();
        let names: Vec<CString> = joints
            .iter()
            .map(|j| CString::new(j.name.as_str()).map_err(|_| Error::new("joint name contains a NUL byte")))
            .collect::<Result<_>>()?;
        let keyframes: Vec<Vec<sys::JPC_AnimationKeyframe>> = joints
            .iter()
            .map(|j| {
                j.keyframes
                    .iter()
                    .map(|k| sys::JPC_AnimationKeyframe { time: k.time, translation: k.translation.into(), rotation: k.rotation.into() })
                    .collect()
            })
            .collect();
        let raw_joints: Vec<sys::JPC_AnimatedJoint> = names
            .iter()
            .zip(&keyframes)
            .map(|(name, keys)| sys::JPC_AnimatedJoint {
                name: name.as_ptr(),
                keyframes: keys.as_ptr(),
                num_keyframes: u32::try_from(keys.len()).unwrap_or(u32::MAX),
            })
            .collect();
        let count = u32::try_from(raw_joints.len()).map_err(|_| Error::new("too many joints"))?;
        let raw = unsafe { sys::JPC_SkeletalAnimation_Create(raw_joints.as_ptr(), count, looping) };
        NonNull::new(raw).map(|raw| Self { raw }).ok_or_else(Error::last)
    }

    fn as_raw(&self) -> *const sys::JPC_SkeletalAnimation {
        self.raw.as_ptr()
    }

    /// Time of the last keyframe of the first joint.
    pub fn duration(&self) -> f32 {
        unsafe { sys::JPC_SkeletalAnimation_GetDuration(self.as_raw()) }
    }

    pub fn is_looping(&self) -> bool {
        unsafe { sys::JPC_SkeletalAnimation_IsLooping(self.as_raw()) }
    }

    /// Overwrites the local states (one per skeleton joint) of the joints this animation animates, interpolating
    /// between keyframes. Looping animations wrap `time` around the duration; negative times count as 0. Fails
    /// (changing nothing) if the animation animates a joint the skeleton doesn't have. Panics unless there is one
    /// state per joint.
    pub fn sample(&self, skeleton: &Skeleton, time: f32, states: &mut [JointState]) -> Result<()> {
        assert_eq!(states.len(), skeleton.joint_count(), "one state per joint");
        let mut raw: Vec<sys::JPC_JointState> =
            states.iter().map(|s| sys::JPC_JointState { translation: s.translation.into(), rotation: s.rotation.into() }).collect();
        if !unsafe { sys::JPC_SkeletalAnimation_Sample(self.as_raw(), skeleton.as_raw(), time, raw.as_mut_ptr()) } {
            return Err(Error::last());
        }
        for (state, raw) in states.iter_mut().zip(raw) {
            *state = JointState { translation: raw.translation.into(), rotation: raw.rotation.into() };
        }
        Ok(())
    }
}

impl Clone for SkeletalAnimation {
    fn clone(&self) -> Self {
        unsafe { sys::JPC_SkeletalAnimation_AddRef(self.as_raw()) };
        Self { raw: self.raw }
    }
}

impl Drop for SkeletalAnimation {
    fn drop(&mut self) {
        unsafe { sys::JPC_SkeletalAnimation_Release(self.as_raw()) }
    }
}

impl fmt::Debug for SkeletalAnimation {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SkeletalAnimation").field("duration", &self.duration()).field("looping", &self.is_looping()).finish()
    }
}

/// Which joint translations of the detailed skeleton a [`SkeletonMapper`] locks to their neutral offset from their
/// parent (so animation can't stretch the character).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub enum LockedTranslations {
    #[default]
    None,
    /// Every joint below the first mapped one.
    All,
    /// One flag per joint of the detailed skeleton. Root joints can't be locked and are ignored.
    Joints(Vec<bool>),
}

/// Maps poses between a low detail skeleton 1 (e.g. a ragdoll's) and a high detail skeleton 2 (e.g. an animated
/// character's), matching joints by name. Joints of 2 between mapped joints follow as chains; other joints of 2 keep
/// their animated local transform. Immutable and reference counted.
pub struct SkeletonMapper {
    raw: NonNull<sys::JPC_SkeletonMapper>,
    counts: (usize, usize),
}

// SAFETY: immutable after creation, with an atomic reference count.
unsafe impl Send for SkeletonMapper {}
unsafe impl Sync for SkeletonMapper {}

impl SkeletonMapper {
    /// The neutral poses are model space matrices, one per joint, with both skeletons in the same pose. Fails if
    /// skeleton 1 has more joints than skeleton 2. Panics if a pose or the locked flags have the wrong length.
    pub fn new(skeleton1: &Skeleton, neutral_pose1: &[Mat44], skeleton2: &Skeleton, neutral_pose2: &[Mat44], locked: &LockedTranslations) -> Result<Self> {
        let (n1, n2) = (skeleton1.joint_count(), skeleton2.joint_count());
        assert_eq!(neutral_pose1.len(), n1, "one matrix per joint of skeleton 1");
        assert_eq!(neutral_pose2.len(), n2, "one matrix per joint of skeleton 2");
        let pose1: Vec<sys::JPC_Mat44> = neutral_pose1.iter().map(|&m| m.into()).collect();
        let pose2: Vec<sys::JPC_Mat44> = neutral_pose2.iter().map(|&m| m.into()).collect();
        let (flags, all) = match locked {
            LockedTranslations::None => (None, false),
            LockedTranslations::All => (None, true),
            LockedTranslations::Joints(flags) => {
                assert_eq!(flags.len(), n2, "one locked flag per joint of skeleton 2");
                (Some(flags.as_slice()), false)
            }
        };
        let raw = unsafe {
            sys::JPC_SkeletonMapper_Create(
                skeleton1.as_raw(),
                pose1.as_ptr(),
                skeleton2.as_raw(),
                pose2.as_ptr(),
                flags.map_or(std::ptr::null(), |f| f.as_ptr()),
                all,
            )
        };
        NonNull::new(raw).map(|raw| Self { raw, counts: (n1, n2) }).ok_or_else(Error::last)
    }

    fn as_raw(&self) -> *const sys::JPC_SkeletonMapper {
        self.raw.as_ptr()
    }

    /// Joint counts of skeleton 1 and 2.
    pub fn joint_counts(&self) -> (usize, usize) {
        self.counts
    }

    /// Poses skeleton 2 (e.g. for rendering) from a model space pose of skeleton 1 (e.g. from a ragdoll) and the
    /// local space animated pose of skeleton 2. Joints the mapping doesn't reach keep their value in `out_pose2`.
    /// Panics if a slice has the wrong length.
    pub fn map(&self, pose1_model_space: &[Mat44], pose2_local_space: &[Mat44], out_pose2_model_space: &mut [Mat44]) {
        let (n1, n2) = self.counts;
        assert_eq!(pose1_model_space.len(), n1, "one matrix per joint of skeleton 1");
        assert_eq!(pose2_local_space.len(), n2, "one matrix per joint of skeleton 2");
        assert_eq!(out_pose2_model_space.len(), n2, "one matrix per joint of skeleton 2");
        let p1: Vec<sys::JPC_Mat44> = pose1_model_space.iter().map(|&m| m.into()).collect();
        let l2: Vec<sys::JPC_Mat44> = pose2_local_space.iter().map(|&m| m.into()).collect();
        let mut out: Vec<sys::JPC_Mat44> = out_pose2_model_space.iter().map(|&m| m.into()).collect();
        unsafe { sys::JPC_SkeletonMapper_Map(self.as_raw(), p1.as_ptr(), l2.as_ptr(), out.as_mut_ptr()) };
        for (dst, src) in out_pose2_model_space.iter_mut().zip(out) {
            *dst = src.into();
        }
    }

    /// Poses skeleton 1 (e.g. to drive a ragdoll) from a model space pose of skeleton 2. Only the joints of 1 that
    /// map directly are written. Panics if a slice has the wrong length.
    pub fn map_reverse(&self, pose2_model_space: &[Mat44], out_pose1_model_space: &mut [Mat44]) {
        let (n1, n2) = self.counts;
        assert_eq!(pose2_model_space.len(), n2, "one matrix per joint of skeleton 2");
        assert_eq!(out_pose1_model_space.len(), n1, "one matrix per joint of skeleton 1");
        let p2: Vec<sys::JPC_Mat44> = pose2_model_space.iter().map(|&m| m.into()).collect();
        let mut out: Vec<sys::JPC_Mat44> = out_pose1_model_space.iter().map(|&m| m.into()).collect();
        unsafe { sys::JPC_SkeletonMapper_MapReverse(self.as_raw(), p2.as_ptr(), out.as_mut_ptr()) };
        for (dst, src) in out_pose1_model_space.iter_mut().zip(out) {
            *dst = src.into();
        }
    }

    /// The joint of skeleton 2 that joint `joint1` of skeleton 1 maps to directly.
    pub fn mapped_joint(&self, joint1: usize) -> Option<usize> {
        let joint1 = u32::try_from(joint1).ok()?;
        usize::try_from(unsafe { sys::JPC_SkeletonMapper_GetMappedJointIndex(self.as_raw(), joint1) }).ok()
    }

    pub fn is_translation_locked(&self, joint2: usize) -> bool {
        u32::try_from(joint2).is_ok_and(|j| unsafe { sys::JPC_SkeletonMapper_IsJointTranslationLocked(self.as_raw(), j) })
    }
}

impl Clone for SkeletonMapper {
    fn clone(&self) -> Self {
        unsafe { sys::JPC_SkeletonMapper_AddRef(self.as_raw()) };
        Self { raw: self.raw, counts: self.counts }
    }
}

impl Drop for SkeletonMapper {
    fn drop(&mut self) {
        unsafe { sys::JPC_SkeletonMapper_Release(self.as_raw()) }
    }
}

impl fmt::Debug for SkeletonMapper {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SkeletonMapper").field("joint_counts", &self.counts).finish()
    }
}
