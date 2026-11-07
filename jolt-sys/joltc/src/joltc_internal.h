// Shared internals of joltc. Not part of the public API.
#pragma once

#include "joltc.h"

#include <Jolt/Jolt.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Body/BodyLockMulti.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.h>
#include <Jolt/Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollidePointResult.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/ObjectLayerPairFilterTable.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/EmptyShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/PlaneShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/TaperedCapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/TaperedCylinderShape.h>
#include <Jolt/Physics/Constraints/ConeConstraint.h>
#include <Jolt/Physics/Constraints/DistanceConstraint.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/PointConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Mutex.h>
#include <Jolt/Core/UnorderedSet.h>
#include <Jolt/Physics/Character/Character.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>
#include <Jolt/Physics/Collision/GroupFilter.h>
#include <Jolt/Physics/Collision/PhysicsMaterial.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/PhysicsStepListener.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>

#include <atomic>

#include <condition_variable>
#include <memory>
#include <mutex>
#include <shared_mutex>

#ifdef JPH_DOUBLE_PRECISION
#error "joltc currently only supports single precision builds of Jolt"
#endif

using namespace JPH;

// Records the error returned by JPC_GetLastError on this thread.
void JPC_SetError(const char *inMessage);

// ---------------------------------------------------------------------------
// Conversions
// ---------------------------------------------------------------------------

inline Vec3 ToJph(JPC_Vec3 v) { return Vec3(v.x, v.y, v.z); }
inline Quat ToJph(JPC_Quat q) { return Quat(q.x, q.y, q.z, q.w); }
inline Vec4 ToJph(JPC_Vec4 v) { return Vec4(v.x, v.y, v.z, v.w); }
inline Mat44 ToJph(const JPC_Mat44 &m) { return Mat44(ToJph(m.columns[0]), ToJph(m.columns[1]), ToJph(m.columns[2]), ToJph(m.columns[3])); }
inline BodyID ToJph(JPC_BodyID id) { return BodyID(id); }

inline JPC_Vec3 ToC(Vec3Arg v) { return { v.GetX(), v.GetY(), v.GetZ() }; }
inline JPC_Quat ToC(QuatArg q) { return { q.GetX(), q.GetY(), q.GetZ(), q.GetW() }; }
inline JPC_Vec4 ToC(Vec4Arg v) { return { v.GetX(), v.GetY(), v.GetZ(), v.GetW() }; }
inline JPC_Mat44 ToC(Mat44Arg m) { return { { ToC(m.GetColumn4(0)), ToC(m.GetColumn4(1)), ToC(m.GetColumn4(2)), ToC(m.GetColumn4(3)) } }; }
inline JPC_BodyID ToC(const BodyID &id) { return id.GetIndexAndSequenceNumber(); }
inline JPC_SubShapeID ToC(const SubShapeID &id) { return id.GetValue(); }
inline JPC_AABox ToC(const AABox &b) { return { ToC(b.mMin), ToC(b.mMax) }; }
inline JPC_MassProperties ToC(const MassProperties &m) { return { m.mMass, ToC(m.mInertia) }; }

inline MassProperties ToJph(const JPC_MassProperties &m)
{
	MassProperties out;
	out.mMass = m.mass;
	out.mInertia = ToJph(m.inertia);
	return out;
}

inline EActivation ToJph(JPC_Activation a) { return a == JPC_ACTIVATION_ACTIVATE ? EActivation::Activate : EActivation::DontActivate; }

inline const Shape *ToJph(const JPC_Shape *s) { return reinterpret_cast<const Shape *>(s); }
inline Shape *ToJph(JPC_Shape *s) { return reinterpret_cast<Shape *>(s); }
inline JPC_Shape *ToC(Shape *s) { return reinterpret_cast<JPC_Shape *>(s); }

inline const Body *ToJph(const JPC_Body *b) { return reinterpret_cast<const Body *>(b); }
inline const JPC_Body *ToC(const Body *b) { return reinterpret_cast<const JPC_Body *>(b); }

struct JPC_BodyInterface
{
	BodyInterface *			mInterface;
	const BodyLockInterface *mLockInterface;
	JPC_PhysicsSystem *		mOwner;
};

inline BodyInterface *ToJph(JPC_BodyInterface *bi) { return bi->mInterface; }
inline const BodyInterface *ToJph(const JPC_BodyInterface *bi) { return bi->mInterface; }

inline Constraint *ToJph(JPC_Constraint *c) { return reinterpret_cast<Constraint *>(c); }
inline const Constraint *ToJph(const JPC_Constraint *c) { return reinterpret_cast<const Constraint *>(c); }
inline JPC_Constraint *ToC(Constraint *c) { return reinterpret_cast<JPC_Constraint *>(c); }

// Jolt's thread pool hands out a fixed number of barriers and PhysicsSystem::Update dereferences a failed
// CreateBarrier, so concurrent updates on one job system are limited to the number of barriers.
struct JPC_JobSystem
{
	JPH_OVERRIDE_NEW_DELETE

	JPC_JobSystem(JobSystem *inJobSystem, uint32 inMaxConcurrentUpdates) : mJobSystem(inJobSystem), mMaxConcurrentUpdates(inMaxConcurrentUpdates) { }

	void					AcquireUpdateSlot()
	{
		std::unique_lock lock(mMutex);
		mCondition.wait(lock, [this] { return mActiveUpdates < mMaxConcurrentUpdates; });
		++mActiveUpdates;
	}

	void					ReleaseUpdateSlot()
	{
		{
			std::lock_guard lock(mMutex);
			--mActiveUpdates;
		}
		mCondition.notify_one();
	}

	std::unique_ptr<JobSystem> mJobSystem;
	uint32					mMaxConcurrentUpdates;
	uint32					mActiveUpdates = 0;
	std::mutex				mMutex;
	std::condition_variable	mCondition;
};

inline TempAllocator *ToJph(JPC_TempAllocator *a) { return reinterpret_cast<TempAllocator *>(a); }
inline TempAllocator &ToJphRef(JPC_TempAllocator *a) { return *reinterpret_cast<TempAllocator *>(a); }
inline JobSystem *ToJph(JPC_JobSystem *j) { return j->mJobSystem.get(); }

// ---------------------------------------------------------------------------
// Materials and group filters
// ---------------------------------------------------------------------------

// Every material handed out by joltc is a UserMaterial, so any material that is not Jolt's default is one.
class UserMaterial final : public PhysicsMaterial
{
public:
	explicit				UserMaterial(uint64 inUserData) : mUserData(inUserData) { }

	uint64					mUserData;
};

inline const UserMaterial *ToJph(const JPC_PhysicsMaterial *m) { return reinterpret_cast<const UserMaterial *>(m); }

inline const JPC_PhysicsMaterial *ToC(const PhysicsMaterial *m)
{
	if (m == nullptr || m == PhysicsMaterial::sDefault.GetPtr())
		return nullptr;
	return reinterpret_cast<const JPC_PhysicsMaterial *>(static_cast<const UserMaterial *>(m));
}

// Same semantics as Jolt's GroupFilterTable, but out of range sub groups collide instead of reading out of bounds.
class SafeGroupFilterTable final : public GroupFilter
{
public:
	explicit				SafeGroupFilterTable(uint32 inNumSubGroups) : mNumSubGroups(inNumSubGroups)
	{
		uint64 bits = uint64(inNumSubGroups) * (inNumSubGroups > 0 ? inNumSubGroups - 1 : 0) / 2;
		mTable.resize(size_t((bits + 7) / 8), 0xff);
	}

	void					SetCollision(uint32 inA, uint32 inB, bool inEnabled)
	{
		uint64 bit;
		if (!GetBit(inA, inB, bit))
			return;
		if (inEnabled)
			mTable[size_t(bit >> 3)] |= uint8(1 << (bit & 7));
		else
			mTable[size_t(bit >> 3)] &= uint8(~(1 << (bit & 7)));
	}

	bool					IsCollisionEnabled(uint32 inA, uint32 inB) const
	{
		if (inA == inB)
			return false;
		uint64 bit;
		if (!GetBit(inA, inB, bit))
			return true;
		return (mTable[size_t(bit >> 3)] & (1 << (bit & 7))) != 0;
	}

	virtual bool			CanCollide(const CollisionGroup &inGroup1, const CollisionGroup &inGroup2) const override
	{
		if (inGroup1.GetGroupID() == CollisionGroup::cInvalidGroup || inGroup1.GetGroupID() != inGroup2.GetGroupID())
			return true;
		if (inGroup1.GetGroupFilter() != inGroup2.GetGroupFilter())
			return false;
		return IsCollisionEnabled(inGroup1.GetSubGroupID(), inGroup2.GetSubGroupID());
	}

private:
	bool					GetBit(uint32 inA, uint32 inB, uint64 &outBit) const
	{
		if (inA == inB || inA >= mNumSubGroups || inB >= mNumSubGroups)
			return false;
		if (inA > inB)
			std::swap(inA, inB);
		outBit = uint64(inB) * (inB - 1) / 2 + inA;
		return true;
	}

	uint32					mNumSubGroups;
	Array<uint8>			mTable;
};

inline const GroupFilter *ToJph(const JPC_GroupFilter *f) { return reinterpret_cast<const GroupFilter *>(f); }
inline SafeGroupFilterTable *ToJphTable(JPC_GroupFilter *f) { return static_cast<SafeGroupFilterTable *>(reinterpret_cast<GroupFilter *>(f)); }
inline const SafeGroupFilterTable *ToJphTable(const JPC_GroupFilter *f) { return static_cast<const SafeGroupFilterTable *>(reinterpret_cast<const GroupFilter *>(f)); }

inline CollisionGroup ToJph(const JPC_CollisionGroup &g) { return CollisionGroup(ToJph(g.group_filter), g.group_id, g.sub_group_id); }

inline JPC_CollisionGroup ToC(const CollisionGroup &g)
{
	return { reinterpret_cast<const JPC_GroupFilter *>(g.GetGroupFilter()), g.GetGroupID(), g.GetSubGroupID() };
}

// ---------------------------------------------------------------------------
// Listener bridges
// ---------------------------------------------------------------------------

inline JPC_ContactManifold ToC(const ContactManifold &inManifold, JPC_Vec3 *outPoints1, JPC_Vec3 *outPoints2)
{
	uint num_points = (uint)inManifold.mRelativeContactPointsOn1.size();
	for (uint i = 0; i < num_points; ++i)
	{
		outPoints1[i] = ToC(inManifold.mRelativeContactPointsOn1[i]);
		outPoints2[i] = ToC(inManifold.mRelativeContactPointsOn2[i]);
	}

	JPC_ContactManifold out;
	out.base_offset = ToC(inManifold.mBaseOffset);
	out.world_space_normal = ToC(inManifold.mWorldSpaceNormal);
	out.penetration_depth = inManifold.mPenetrationDepth;
	out.sub_shape_id1 = ToC(inManifold.mSubShapeID1);
	out.sub_shape_id2 = ToC(inManifold.mSubShapeID2);
	out.num_points = num_points;
	out.relative_points_on1 = outPoints1;
	out.relative_points_on2 = outPoints2;
	out.material1 = nullptr;
	out.material2 = nullptr;
	return out;
}

inline JPC_ContactSettings ToC(const ContactSettings &inSettings)
{
	JPC_ContactSettings out;
	out.combined_friction = inSettings.mCombinedFriction;
	out.combined_restitution = inSettings.mCombinedRestitution;
	out.inv_mass_scale1 = inSettings.mInvMassScale1;
	out.inv_inertia_scale1 = inSettings.mInvInertiaScale1;
	out.inv_mass_scale2 = inSettings.mInvMassScale2;
	out.inv_inertia_scale2 = inSettings.mInvInertiaScale2;
	out.is_sensor = inSettings.mIsSensor;
	out.relative_linear_surface_velocity = ToC(inSettings.mRelativeLinearSurfaceVelocity);
	out.relative_angular_surface_velocity = ToC(inSettings.mRelativeAngularSurfaceVelocity);
	return out;
}

inline void CopyBack(const JPC_ContactSettings &inSettings, ContactSettings &outSettings)
{
	outSettings.mCombinedFriction = inSettings.combined_friction;
	outSettings.mCombinedRestitution = inSettings.combined_restitution;
	outSettings.mInvMassScale1 = inSettings.inv_mass_scale1;
	outSettings.mInvInertiaScale1 = inSettings.inv_inertia_scale1;
	outSettings.mInvMassScale2 = inSettings.inv_mass_scale2;
	outSettings.mInvInertiaScale2 = inSettings.inv_inertia_scale2;
	outSettings.mIsSensor = inSettings.is_sensor;
	outSettings.mRelativeLinearSurfaceVelocity = ToJph(inSettings.relative_linear_surface_velocity);
	outSettings.mRelativeAngularSurfaceVelocity = ToJph(inSettings.relative_angular_surface_velocity);
}

class ContactListenerBridge final : public ContactListener
{
public:
	JPC_ContactListenerFns	mFns { };
	void *					mUserData = nullptr;

	virtual ValidateResult	OnContactValidate(const Body &inBody1, const Body &inBody2, RVec3Arg inBaseOffset, const CollideShapeResult &inResult) override
	{
		if (mFns.on_contact_validate == nullptr)
			return ValidateResult::AcceptAllContactsForThisBodyPair;

		JPC_RVec3 base_offset = ToC(inBaseOffset);
		JPC_CollideShapeResult result;
		result.contact_point_on1 = ToC(inResult.mContactPointOn1);
		result.contact_point_on2 = ToC(inResult.mContactPointOn2);
		result.penetration_axis = ToC(inResult.mPenetrationAxis);
		result.penetration_depth = inResult.mPenetrationDepth;
		result.sub_shape_id1 = ToC(inResult.mSubShapeID1);
		result.sub_shape_id2 = ToC(inResult.mSubShapeID2);
		result.body_id2 = ToC(inResult.mBodyID2);
		return (ValidateResult)mFns.on_contact_validate(mUserData, ToC(&inBody1), ToC(&inBody2), &base_offset, &result);
	}

	virtual void			OnContactAdded(const Body &inBody1, const Body &inBody2, const ContactManifold &inManifold, ContactSettings &ioSettings) override
	{
		if (mFns.on_contact_added != nullptr)
			Forward(mFns.on_contact_added, inBody1, inBody2, inManifold, ioSettings);
	}

	virtual void			OnContactPersisted(const Body &inBody1, const Body &inBody2, const ContactManifold &inManifold, ContactSettings &ioSettings) override
	{
		if (mFns.on_contact_persisted != nullptr)
			Forward(mFns.on_contact_persisted, inBody1, inBody2, inManifold, ioSettings);
	}

	virtual void			OnContactRemoved(const SubShapeIDPair &inPair) override
	{
		if (mFns.on_contact_removed == nullptr)
			return;

		JPC_SubShapeIDPair pair { ToC(inPair.GetBody1ID()), ToC(inPair.GetSubShapeID1()), ToC(inPair.GetBody2ID()), ToC(inPair.GetSubShapeID2()) };
		mFns.on_contact_removed(mUserData, &pair);
	}

private:
	using ContactFn = void (*)(void *, const JPC_Body *, const JPC_Body *, const JPC_ContactManifold *, JPC_ContactSettings *);

	void					Forward(ContactFn inFn, const Body &inBody1, const Body &inBody2, const ContactManifold &inManifold, ContactSettings &ioSettings)
	{
		JPC_Vec3 points1[ContactPoints::Capacity], points2[ContactPoints::Capacity];
		JPC_ContactManifold manifold = ToC(inManifold, points1, points2);
		manifold.material1 = ToC(inBody1.GetShape()->GetMaterial(inManifold.mSubShapeID1));
		manifold.material2 = ToC(inBody2.GetShape()->GetMaterial(inManifold.mSubShapeID2));
		JPC_ContactSettings settings = ToC(ioSettings);
		inFn(mUserData, ToC(&inBody1), ToC(&inBody2), &manifold, &settings);
		CopyBack(settings, ioSettings);
	}
};

class BodyActivationListenerBridge final : public BodyActivationListener
{
public:
	JPC_BodyActivationListenerFns mFns { };
	void *					mUserData = nullptr;

	virtual void			OnBodyActivated(const BodyID &inBodyID, uint64 inBodyUserData) override
	{
		if (mFns.on_body_activated != nullptr)
			mFns.on_body_activated(mUserData, ToC(inBodyID), inBodyUserData);
	}

	virtual void			OnBodyDeactivated(const BodyID &inBodyID, uint64 inBodyUserData) override
	{
		if (mFns.on_body_deactivated != nullptr)
			mFns.on_body_deactivated(mUserData, ToC(inBodyID), inBodyUserData);
	}
};

// Jolt may run separate step listeners in parallel, so a Rust listener could race a vehicle touching the same
// bodies. Every listener and vehicle of a system therefore runs from this single listener, one after another.
class CombinedStepListener final : public PhysicsStepListener
{
public:
	virtual void			OnStep(const PhysicsStepListenerContext &inContext) override
	{
		for (PhysicsStepListener *listener : mListeners)
			listener->OnStep(inContext);
	}

	Array<PhysicsStepListener *> mListeners;
};

// ---------------------------------------------------------------------------
// Physics system
// ---------------------------------------------------------------------------

struct JPC_PhysicsSystem
{
	JPH_OVERRIDE_NEW_DELETE

	explicit JPC_PhysicsSystem(const JPC_PhysicsSystemSettings &inSettings) :
		mNumObjectLayers(inSettings.num_object_layers),
		mBroadPhaseLayerInterface(inSettings.num_object_layers, inSettings.num_broad_phase_layers),
		mObjectLayerPairFilter(inSettings.num_object_layers)
	{
		for (uint32_t i = 0; i < inSettings.num_object_layers; ++i)
			mBroadPhaseLayerInterface.MapObjectToBroadPhaseLayer(ObjectLayer(i), BroadPhaseLayer(inSettings.object_to_broad_phase_layer[i]));
		for (uint32_t i = 0; i < inSettings.num_colliding_pairs; ++i)
			mObjectLayerPairFilter.EnableCollision(inSettings.colliding_pairs[i].layer1, inSettings.colliding_pairs[i].layer2);

		// Built from the two tables above, so it can only be created once they are filled in
		mObjectVsBroadPhaseLayerFilter = std::make_unique<ObjectVsBroadPhaseLayerFilterTable>(mBroadPhaseLayerInterface, inSettings.num_broad_phase_layers, mObjectLayerPairFilter, inSettings.num_object_layers);

		mSystem.Init(inSettings.max_bodies, inSettings.num_body_mutexes, inSettings.max_body_pairs, inSettings.max_contact_constraints,
					 mBroadPhaseLayerInterface, *mObjectVsBroadPhaseLayerFilter, mObjectLayerPairFilter);

		mBodyInterface = { &mSystem.GetBodyInterface(), &mSystem.GetBodyLockInterface(), this };
		mBodyInterfaceNoLock = { &mSystem.GetBodyInterfaceNoLock(), &mSystem.GetBodyLockInterfaceNoLock(), this };
	}

	// Jolt indexes its layer tables without bounds checks, so every layer coming in through the API is checked against this
	uint32								mNumObjectLayers;
	bool								IsValidLayer(uint32 inLayer) const { return inLayer < mNumObjectLayers; }

	BroadPhaseLayerInterfaceTable		mBroadPhaseLayerInterface;
	ObjectLayerPairFilterTable			mObjectLayerPairFilter;
	std::unique_ptr<ObjectVsBroadPhaseLayerFilterTable> mObjectVsBroadPhaseLayerFilter;
	ContactListenerBridge				mContactListener;
	BodyActivationListenerBridge		mActivationListener;
	PhysicsSystem						mSystem;
	JPC_BodyInterface					mBodyInterface;
	JPC_BodyInterface					mBodyInterfaceNoLock;

	// Taken exclusively while bodies enter or leave the world, and shared by calls whose Jolt implementation
	// activates a body without checking that it is in the world. This turns "check, then act" into one step.
	SharedMutex							mBodyStateMutex;

	// Bumped whenever bodies, shapes, motion types or constraints change, so snapshots can refuse to restore onto
	// a world they don't match (Jolt restores without validating).
	std::atomic<uint64>					mStructureVersion { 0 };
	void								BumpStructure() { ++mStructureVersion; }

	CombinedStepListener				mStepListeners;

	void								AddStepListener(PhysicsStepListener *inListener)
	{
		if (mStepListeners.mListeners.empty())
			mSystem.AddStepListener(&mStepListeners);
		mStepListeners.mListeners.push_back(inListener);
	}

	void								RemoveStepListener(PhysicsStepListener *inListener)
	{
		Array<PhysicsStepListener *> &listeners = mStepListeners.mListeners;
		Array<PhysicsStepListener *>::iterator it = std::find(listeners.begin(), listeners.end(), inListener);
		if (it == listeners.end())
			return;
		listeners.erase(it);
		if (listeners.empty())
			mSystem.RemoveStepListener(&mStepListeners);
	}

	// Bodies owned by characters. Guarded by mBodyStateMutex.
	UnorderedSet<uint32>				mProtectedBodies;

	void								Protect(BodyID inBody)
	{
		if (inBody.IsInvalid())
			return;
		std::unique_lock lock(mBodyStateMutex);
		mProtectedBodies.insert(inBody.GetIndexAndSequenceNumber());
	}

	void								Unprotect(BodyID inBody)
	{
		mProtectedBodies.erase(inBody.GetIndexAndSequenceNumber());
	}
};

class QueryBroadPhaseLayerFilter final : public BroadPhaseLayerFilter
{
public:
	QueryBroadPhaseLayerFilter(const JPC_PhysicsSystem &inSystem, const JPC_QueryFilter *inFilter) :
		mFilter(*inSystem.mObjectVsBroadPhaseLayerFilter), mEnabled(inFilter != nullptr && inFilter->use_object_layer), mLayer(mEnabled ? inFilter->object_layer : 0),
		mValid(!mEnabled || inSystem.IsValidLayer(mLayer)) { }

	// An unknown layer collides with nothing
	virtual bool			ShouldCollide(BroadPhaseLayer inLayer) const override { return !mEnabled || (mValid && mFilter.ShouldCollide(mLayer, inLayer)); }

private:
	const ObjectVsBroadPhaseLayerFilter &mFilter;
	bool					mEnabled;
	ObjectLayer				mLayer;
	bool					mValid;
};

class QueryObjectLayerFilter final : public ObjectLayerFilter
{
public:
	QueryObjectLayerFilter(const JPC_PhysicsSystem &inSystem, const JPC_QueryFilter *inFilter) :
		mFilter(inSystem.mObjectLayerPairFilter), mEnabled(inFilter != nullptr && inFilter->use_object_layer), mLayer(mEnabled ? inFilter->object_layer : 0),
		mValid(!mEnabled || inSystem.IsValidLayer(mLayer)) { }

	virtual bool			ShouldCollide(ObjectLayer inLayer) const override { return !mEnabled || (mValid && mFilter.ShouldCollide(mLayer, inLayer)); }

private:
	const ObjectLayerPairFilter &mFilter;
	bool					mEnabled;
	ObjectLayer				mLayer;
	bool					mValid;
};

class QueryBodyFilter final : public BodyFilter
{
public:
	explicit QueryBodyFilter(const JPC_QueryFilter *inFilter) : mFilter(inFilter) { }

	virtual bool			ShouldCollide(const BodyID &inBodyID) const override
	{
		if (mFilter == nullptr)
			return true;
		JPC_BodyID id = ToC(inBodyID);
		if (id == mFilter->ignore_body)
			return false;
		return mFilter->body_filter == nullptr || mFilter->body_filter(mFilter->body_filter_user_data, id);
	}

private:
	const JPC_QueryFilter *	mFilter;
};

struct QueryFilters
{
	QueryFilters(const JPC_PhysicsSystem &inSystem, const JPC_QueryFilter *inFilter) : mBroadPhase(inSystem, inFilter), mObjectLayer(inSystem, inFilter), mBody(inFilter) { }

	QueryBroadPhaseLayerFilter mBroadPhase;
	QueryObjectLayerFilter	mObjectLayer;
	QueryBodyFilter			mBody;
};

