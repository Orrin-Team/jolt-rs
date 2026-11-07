#include "joltc_internal.h"

#include <cfloat>

// ---------------------------------------------------------------------------
// Shared helpers
// ---------------------------------------------------------------------------

static void ToJphBase(const JPC_Vec3 &inUp, const JPC_Vec3 &inSupportNormal, float inSupportConstant, float inMaxSlopeAngle,
					  bool inEnhancedInternalEdgeRemoval, const JPC_Shape *inShape, CharacterBaseSettings &outSettings)
{
	outSettings.mUp = ToJph(inUp);
	outSettings.mSupportingVolume = Plane(ToJph(inSupportNormal), inSupportConstant);
	outSettings.mMaxSlopeAngle = inMaxSlopeAngle;
	outSettings.mEnhancedInternalEdgeRemoval = inEnhancedInternalEdgeRemoval;
	outSettings.mShape = ToJph(inShape);
}

static void GetGround(const CharacterBase &inCharacter, JPC_CharacterGround *outGround)
{
	outGround->state = (JPC_GroundState)inCharacter.GetGroundState();
	outGround->position = ToC(inCharacter.GetGroundPosition());
	outGround->normal = ToC(inCharacter.GetGroundNormal());
	outGround->velocity = ToC(inCharacter.GetGroundVelocity());
	outGround->body_id = ToC(inCharacter.GetGroundBodyID());
	outGround->sub_shape_id = ToC(inCharacter.GetGroundSubShapeID());
	outGround->user_data = inCharacter.GetGroundUserData();
	outGround->material = ToC(inCharacter.GetGroundMaterial());
}

static JPC_CharacterContact ToC(const CharacterContact &inContact)
{
	JPC_CharacterContact out;
	out.body_id = ToC(inContact.mBodyB);
	out.character_id = inContact.mCharacterIDB.GetValue();
	out.sub_shape_id = ToC(inContact.mSubShapeIDB);
	out.position = ToC(inContact.mPosition);
	out.linear_velocity = ToC(inContact.mLinearVelocity);
	out.contact_normal = ToC(inContact.mContactNormal);
	out.surface_normal = ToC(inContact.mSurfaceNormal);
	out.distance = inContact.mDistance;
	out.fraction = inContact.mFraction;
	out.motion_type = (JPC_MotionType)inContact.mMotionTypeB;
	out.is_sensor = inContact.mIsSensorB;
	out.user_data = inContact.mUserData;
	out.material = ToC(inContact.mMaterial);
	out.had_collision = inContact.mHadCollision;
	out.was_discarded = inContact.mWasDiscarded;
	out.can_push_character = inContact.mCanPushCharacter;
	out.is_back_facing = inContact.mIsBackFacingContact;
	return out;
}

// A contact as reported to a solve callback, which only gets a subset of the fields
static JPC_CharacterContact SolveContact(JPC_BodyID inBody, uint32 inCharacter, const SubShapeID &inSubShape, RVec3Arg inPosition, Vec3Arg inNormal,
										 Vec3Arg inVelocity, const PhysicsMaterial *inMaterial)
{
	JPC_CharacterContact out { };
	out.body_id = inBody;
	out.character_id = inCharacter;
	out.sub_shape_id = ToC(inSubShape);
	out.position = ToC(inPosition);
	out.linear_velocity = ToC(inVelocity);
	out.contact_normal = ToC(inNormal);
	out.surface_normal = ToC(inNormal);
	out.material = ToC(inMaterial);
	out.had_collision = true;
	out.can_push_character = true;
	return out;
}

// ---------------------------------------------------------------------------
// CharacterVirtual
// ---------------------------------------------------------------------------

class CharacterContactListenerBridge final : public CharacterContactListener
{
public:
	JPC_CharacterVirtual *	mOwner = nullptr;
	JPC_CharacterContactListenerFns mFns { };
	void *					mUserData = nullptr;

	virtual void			OnAdjustBodyVelocity(const CharacterVirtual *, const Body &inBody2, Vec3 &ioLinearVelocity, Vec3 &ioAngularVelocity) override
	{
		if (mFns.on_adjust_body_velocity == nullptr)
			return;
		JPC_Vec3 linear = ToC(ioLinearVelocity), angular = ToC(ioAngularVelocity);
		mFns.on_adjust_body_velocity(mUserData, mOwner, ToC(&inBody2), &linear, &angular);
		ioLinearVelocity = ToJph(linear);
		ioAngularVelocity = ToJph(angular);
	}

	virtual bool			OnContactValidate(const CharacterVirtual *, const CharacterContact &inContact) override
	{
		if (mFns.on_contact_validate == nullptr)
			return true;
		JPC_CharacterContact contact = ToC(inContact);
		return mFns.on_contact_validate(mUserData, mOwner, &contact);
	}

	virtual void			OnContactAdded(const CharacterVirtual *, const CharacterContact &inContact, CharacterContactSettings &ioSettings) override
	{
		Forward(mFns.on_contact_added, inContact, ioSettings);
	}

	virtual void			OnContactPersisted(const CharacterVirtual *, const CharacterContact &inContact, CharacterContactSettings &ioSettings) override
	{
		Forward(mFns.on_contact_persisted, inContact, ioSettings);
	}

	virtual void			OnContactRemoved(const CharacterVirtual *, const BodyID &inBodyID2, const SubShapeID &inSubShapeID2) override
	{
		if (mFns.on_contact_removed != nullptr)
			mFns.on_contact_removed(mUserData, mOwner, ToC(inBodyID2), ToC(inSubShapeID2));
	}

	virtual bool			OnCharacterContactValidate(const CharacterVirtual *, const CharacterContact &inContact) override
	{
		if (mFns.on_character_contact_validate == nullptr)
			return true;
		JPC_CharacterContact contact = ToC(inContact);
		return mFns.on_character_contact_validate(mUserData, mOwner, &contact);
	}

	virtual void			OnCharacterContactAdded(const CharacterVirtual *, const CharacterContact &inContact, CharacterContactSettings &ioSettings) override
	{
		Forward(mFns.on_character_contact_added, inContact, ioSettings);
	}

	virtual void			OnCharacterContactPersisted(const CharacterVirtual *, const CharacterContact &inContact, CharacterContactSettings &ioSettings) override
	{
		Forward(mFns.on_character_contact_persisted, inContact, ioSettings);
	}

	virtual void			OnCharacterContactRemoved(const CharacterVirtual *, const CharacterID &inOtherCharacterID, const SubShapeID &inSubShapeID2) override
	{
		if (mFns.on_character_contact_removed != nullptr)
			mFns.on_character_contact_removed(mUserData, mOwner, inOtherCharacterID.GetValue(), ToC(inSubShapeID2));
	}

	virtual void			OnContactSolve(const CharacterVirtual *, const BodyID &inBodyID2, const SubShapeID &inSubShapeID2, RVec3Arg inContactPosition,
										   Vec3Arg inContactNormal, Vec3Arg inContactVelocity, const PhysicsMaterial *inContactMaterial,
										   Vec3Arg inCharacterVelocity, Vec3 &ioNewCharacterVelocity) override
	{
		JPC_CharacterContact contact = SolveContact(ToC(inBodyID2), JPC_CHARACTER_ID_INVALID, inSubShapeID2, inContactPosition, inContactNormal, inContactVelocity, inContactMaterial);
		Solve(contact, inCharacterVelocity, ioNewCharacterVelocity);
	}

	virtual void			OnCharacterContactSolve(const CharacterVirtual *, const CharacterVirtual *inOtherCharacter, const SubShapeID &inSubShapeID2,
													RVec3Arg inContactPosition, Vec3Arg inContactNormal, Vec3Arg inContactVelocity,
													const PhysicsMaterial *inContactMaterial, Vec3Arg inCharacterVelocity, Vec3 &ioNewCharacterVelocity) override
	{
		JPC_CharacterContact contact = SolveContact(JPC_BODY_ID_INVALID, inOtherCharacter->GetID().GetValue(), inSubShapeID2, inContactPosition,
													inContactNormal, inContactVelocity, inContactMaterial);
		Solve(contact, inCharacterVelocity, ioNewCharacterVelocity);
	}

private:
	using ContactFn = void (*)(void *, const JPC_CharacterVirtual *, const JPC_CharacterContact *, JPC_CharacterContactSettings *);

	void					Forward(ContactFn inFn, const CharacterContact &inContact, CharacterContactSettings &ioSettings)
	{
		if (inFn == nullptr)
			return;
		JPC_CharacterContact contact = ToC(inContact);
		JPC_CharacterContactSettings settings { ioSettings.mCanPushCharacter, ioSettings.mCanReceiveImpulses };
		inFn(mUserData, mOwner, &contact, &settings);
		ioSettings.mCanPushCharacter = settings.can_push_character;
		ioSettings.mCanReceiveImpulses = settings.can_receive_impulses;
	}

	void					Solve(const JPC_CharacterContact &inContact, Vec3Arg inCharacterVelocity, Vec3 &ioNewCharacterVelocity)
	{
		if (mFns.on_contact_solve == nullptr)
			return;
		JPC_Vec3 character_velocity = ToC(inCharacterVelocity), new_velocity = ToC(ioNewCharacterVelocity);
		mFns.on_contact_solve(mUserData, mOwner, &inContact, &character_velocity, &new_velocity);
		ioNewCharacterVelocity = ToJph(new_velocity);
	}
};

struct JPC_CharacterVsCharacterCollision
{
	JPH_OVERRIDE_NEW_DELETE

	CharacterVsCharacterCollisionSimple mCollision;
};

struct JPC_CharacterVirtual
{
	JPH_OVERRIDE_NEW_DELETE

	JPC_PhysicsSystem *		mSystem = nullptr;
	Ref<CharacterVirtual>	mCharacter;
	CharacterContactListenerBridge mListener;
	JPC_CharacterVsCharacterCollision *mCollision = nullptr;

	// Jolt stores the ground and contact materials as raw pointers, which dangle once the touched shape is freed.
	// These references keep them alive until the next call that recomputes them.
	Array<RefConst<PhysicsMaterial>> mMaterials;

	void					PinMaterials()
	{
		mMaterials.clear();
		mMaterials.push_back(mCharacter->GetGroundMaterial());
		for (const CharacterContact &c : mCharacter->GetActiveContacts())
			mMaterials.push_back(c.mMaterial);
	}
};

void JPC_CharacterVirtualSettings_SetDefault(JPC_CharacterVirtualSettings *out)
{
	CharacterVirtualSettings s;
	out->up = ToC(s.mUp);
	out->supporting_volume_normal = ToC(s.mSupportingVolume.GetNormal());
	out->supporting_volume_constant = s.mSupportingVolume.GetConstant();
	out->max_slope_angle = s.mMaxSlopeAngle;
	out->enhanced_internal_edge_removal = s.mEnhancedInternalEdgeRemoval;
	out->shape = nullptr;
	out->mass = s.mMass;
	out->max_strength = s.mMaxStrength;
	out->shape_offset = ToC(s.mShapeOffset);
	out->back_face_mode = (JPC_BackFaceMode)s.mBackFaceMode;
	out->predictive_contact_distance = s.mPredictiveContactDistance;
	out->max_collision_iterations = s.mMaxCollisionIterations;
	out->max_constraint_iterations = s.mMaxConstraintIterations;
	out->min_time_remaining = s.mMinTimeRemaining;
	out->collision_tolerance = s.mCollisionTolerance;
	out->character_padding = s.mCharacterPadding;
	out->max_num_hits = s.mMaxNumHits;
	out->hit_reduction_cos_max_angle = s.mHitReductionCosMaxAngle;
	out->penetration_recovery_speed = s.mPenetrationRecoverySpeed;
	out->inner_body_shape = nullptr;
	out->inner_body_layer = s.mInnerBodyLayer;
}

void JPC_ExtendedUpdateSettings_SetDefault(JPC_ExtendedUpdateSettings *out)
{
	CharacterVirtual::ExtendedUpdateSettings s;
	out->stick_to_floor_step_down = ToC(s.mStickToFloorStepDown);
	out->walk_stairs_step_up = ToC(s.mWalkStairsStepUp);
	out->walk_stairs_min_step_forward = s.mWalkStairsMinStepForward;
	out->walk_stairs_step_forward_test = s.mWalkStairsStepForwardTest;
	out->walk_stairs_cos_angle_forward_contact = s.mWalkStairsCosAngleForwardContact;
	out->walk_stairs_step_down_extra = ToC(s.mWalkStairsStepDownExtra);
}

JPC_CharacterVirtual *JPC_CharacterVirtual_Create(const JPC_CharacterVirtualSettings *in, JPC_RVec3 position, JPC_Quat rotation, uint64_t user_data,
												  JPC_PhysicsSystem *system)
{
	if (in->shape == nullptr)
	{
		JPC_SetError("Character shape is null");
		return nullptr;
	}
	if (in->inner_body_shape != nullptr && !system->IsValidLayer(in->inner_body_layer))
	{
		JPC_SetError("Inner body layer does not exist in this physics system");
		return nullptr;
	}

	Ref<CharacterVirtualSettings> s = new CharacterVirtualSettings;
	ToJphBase(in->up, in->supporting_volume_normal, in->supporting_volume_constant, in->max_slope_angle, in->enhanced_internal_edge_removal, in->shape, *s);
	s->mMass = in->mass;
	s->mMaxStrength = in->max_strength;
	s->mShapeOffset = ToJph(in->shape_offset);
	s->mBackFaceMode = (EBackFaceMode)in->back_face_mode;
	s->mPredictiveContactDistance = in->predictive_contact_distance;
	s->mMaxCollisionIterations = in->max_collision_iterations;
	s->mMaxConstraintIterations = in->max_constraint_iterations;
	s->mMinTimeRemaining = in->min_time_remaining;
	s->mCollisionTolerance = in->collision_tolerance;
	s->mCharacterPadding = in->character_padding;
	s->mMaxNumHits = in->max_num_hits;
	s->mHitReductionCosMaxAngle = in->hit_reduction_cos_max_angle;
	s->mPenetrationRecoverySpeed = in->penetration_recovery_speed;
	s->mInnerBodyShape = ToJph(in->inner_body_shape);
	s->mInnerBodyLayer = in->inner_body_layer;

	JPC_CharacterVirtual *character = new JPC_CharacterVirtual;
	character->mSystem = system;
	character->mListener.mOwner = character;
	{
		// The constructor adds the inner body to the world
		std::unique_lock lock(system->mBodyStateMutex);
		character->mCharacter = new CharacterVirtual(s, ToJph(position), ToJph(rotation), user_data, &system->mSystem);
		BodyID inner = character->mCharacter->GetInnerBodyID();
		if (!inner.IsInvalid())
			system->mProtectedBodies.insert(inner.GetIndexAndSequenceNumber());
	}
	if (in->inner_body_shape != nullptr && character->mCharacter->GetInnerBodyID().IsInvalid())
	{
		character->mCharacter = nullptr;
		delete character;
		JPC_SetError("Out of bodies (max_bodies reached) while creating the inner body");
		return nullptr;
	}
	character->mCharacter->SetListener(&character->mListener);
	return character;
}

void JPC_CharacterVirtual_Destroy(JPC_CharacterVirtual *character)
{
	if (character->mCollision != nullptr)
		JPC_CharacterVsCharacterCollision_Remove(character->mCollision, character);

	JPC_PhysicsSystem *system = character->mSystem;
	{
		// The destructor removes and destroys the inner body
		std::unique_lock lock(system->mBodyStateMutex);
		system->Unprotect(character->mCharacter->GetInnerBodyID());
		character->mCharacter = nullptr;
	}
	delete character;
}

uint32_t JPC_CharacterVirtual_GetID(const JPC_CharacterVirtual *character) { return character->mCharacter->GetID().GetValue(); }

void JPC_CharacterVirtual_SetListener(JPC_CharacterVirtual *character, const JPC_CharacterContactListenerFns *fns, void *user_data)
{
	character->mListener.mFns = fns != nullptr ? *fns : JPC_CharacterContactListenerFns { };
	character->mListener.mUserData = fns != nullptr ? user_data : nullptr;
}

void JPC_CharacterVirtual_Update(JPC_CharacterVirtual *character, float delta_time, JPC_Vec3 gravity, const JPC_QueryFilter *filter, JPC_TempAllocator *temp_allocator)
{
	QueryFilters f(*character->mSystem, filter);
	character->mCharacter->Update(delta_time, ToJph(gravity), f.mBroadPhase, f.mObjectLayer, f.mBody, { }, ToJphRef(temp_allocator));
	character->PinMaterials();
}

void JPC_CharacterVirtual_ExtendedUpdate(JPC_CharacterVirtual *character, float delta_time, JPC_Vec3 gravity, const JPC_ExtendedUpdateSettings *settings,
										 const JPC_QueryFilter *filter, JPC_TempAllocator *temp_allocator)
{
	CharacterVirtual::ExtendedUpdateSettings s;
	if (settings != nullptr)
	{
		s.mStickToFloorStepDown = ToJph(settings->stick_to_floor_step_down);
		s.mWalkStairsStepUp = ToJph(settings->walk_stairs_step_up);
		s.mWalkStairsMinStepForward = settings->walk_stairs_min_step_forward;
		s.mWalkStairsStepForwardTest = settings->walk_stairs_step_forward_test;
		s.mWalkStairsCosAngleForwardContact = settings->walk_stairs_cos_angle_forward_contact;
		s.mWalkStairsStepDownExtra = ToJph(settings->walk_stairs_step_down_extra);
	}
	QueryFilters f(*character->mSystem, filter);
	character->mCharacter->ExtendedUpdate(delta_time, ToJph(gravity), s, f.mBroadPhase, f.mObjectLayer, f.mBody, { }, ToJphRef(temp_allocator));
	character->PinMaterials();
}

bool JPC_CharacterVirtual_CanWalkStairs(const JPC_CharacterVirtual *character, JPC_Vec3 linear_velocity)
{
	return character->mCharacter->CanWalkStairs(ToJph(linear_velocity));
}

bool JPC_CharacterVirtual_WalkStairs(JPC_CharacterVirtual *character, float delta_time, JPC_Vec3 step_up, JPC_Vec3 step_forward, JPC_Vec3 step_forward_test,
									 JPC_Vec3 step_down_extra, const JPC_QueryFilter *filter, JPC_TempAllocator *temp_allocator)
{
	QueryFilters f(*character->mSystem, filter);
	bool result = character->mCharacter->WalkStairs(delta_time, ToJph(step_up), ToJph(step_forward), ToJph(step_forward_test), ToJph(step_down_extra),
											 f.mBroadPhase, f.mObjectLayer, f.mBody, { }, ToJphRef(temp_allocator));
	character->PinMaterials();
	return result;
}

bool JPC_CharacterVirtual_StickToFloor(JPC_CharacterVirtual *character, JPC_Vec3 step_down, const JPC_QueryFilter *filter, JPC_TempAllocator *temp_allocator)
{
	QueryFilters f(*character->mSystem, filter);
	bool result = character->mCharacter->StickToFloor(ToJph(step_down), f.mBroadPhase, f.mObjectLayer, f.mBody, { }, ToJphRef(temp_allocator));
	character->PinMaterials();
	return result;
}

void JPC_CharacterVirtual_RefreshContacts(JPC_CharacterVirtual *character, const JPC_QueryFilter *filter, JPC_TempAllocator *temp_allocator)
{
	QueryFilters f(*character->mSystem, filter);
	character->mCharacter->RefreshContacts(f.mBroadPhase, f.mObjectLayer, f.mBody, { }, ToJphRef(temp_allocator));
	character->PinMaterials();
}

void JPC_CharacterVirtual_UpdateGroundVelocity(JPC_CharacterVirtual *character) { character->mCharacter->UpdateGroundVelocity(); }

bool JPC_CharacterVirtual_SetShape(JPC_CharacterVirtual *character, const JPC_Shape *shape, float max_penetration_depth, const JPC_QueryFilter *filter,
								   JPC_TempAllocator *temp_allocator)
{
	if (shape == nullptr)
		return false;
	QueryFilters f(*character->mSystem, filter);
	bool result = character->mCharacter->SetShape(ToJph(shape), max_penetration_depth, f.mBroadPhase, f.mObjectLayer, f.mBody, { }, ToJphRef(temp_allocator));
	character->PinMaterials();
	return result;
}

void JPC_CharacterVirtual_SetInnerBodyShape(JPC_CharacterVirtual *character, const JPC_Shape *shape)
{
	if (shape != nullptr && !character->mCharacter->GetInnerBodyID().IsInvalid())
		character->mCharacter->SetInnerBodyShape(ToJph(shape));
}

JPC_Vec3 JPC_CharacterVirtual_CancelVelocityTowardsSteepSlopes(const JPC_CharacterVirtual *character, JPC_Vec3 desired_velocity)
{
	return ToC(character->mCharacter->CancelVelocityTowardsSteepSlopes(ToJph(desired_velocity)));
}

JPC_Vec3 JPC_CharacterVirtual_GetLinearVelocity(const JPC_CharacterVirtual *c) { return ToC(c->mCharacter->GetLinearVelocity()); }
void JPC_CharacterVirtual_SetLinearVelocity(JPC_CharacterVirtual *c, JPC_Vec3 velocity) { c->mCharacter->SetLinearVelocity(ToJph(velocity)); }
JPC_RVec3 JPC_CharacterVirtual_GetPosition(const JPC_CharacterVirtual *c) { return ToC(c->mCharacter->GetPosition()); }
void JPC_CharacterVirtual_SetPosition(JPC_CharacterVirtual *c, JPC_RVec3 position) { c->mCharacter->SetPosition(ToJph(position)); }
JPC_Quat JPC_CharacterVirtual_GetRotation(const JPC_CharacterVirtual *c) { return ToC(c->mCharacter->GetRotation()); }
void JPC_CharacterVirtual_SetRotation(JPC_CharacterVirtual *c, JPC_Quat rotation) { c->mCharacter->SetRotation(ToJph(rotation).Normalized()); }
JPC_RVec3 JPC_CharacterVirtual_GetCenterOfMassPosition(const JPC_CharacterVirtual *c) { return ToC(c->mCharacter->GetCenterOfMassPosition()); }
JPC_RMat44 JPC_CharacterVirtual_GetWorldTransform(const JPC_CharacterVirtual *c) { return ToC(c->mCharacter->GetWorldTransform()); }
JPC_Vec3 JPC_CharacterVirtual_GetUp(const JPC_CharacterVirtual *c) { return ToC(c->mCharacter->GetUp()); }
void JPC_CharacterVirtual_SetUp(JPC_CharacterVirtual *c, JPC_Vec3 up) { c->mCharacter->SetUp(ToJph(up)); }
void JPC_CharacterVirtual_SetMaxSlopeAngle(JPC_CharacterVirtual *c, float angle) { c->mCharacter->SetMaxSlopeAngle(angle); }
float JPC_CharacterVirtual_GetCosMaxSlopeAngle(const JPC_CharacterVirtual *c) { return c->mCharacter->GetCosMaxSlopeAngle(); }
bool JPC_CharacterVirtual_IsSlopeTooSteep(const JPC_CharacterVirtual *c, JPC_Vec3 normal) { return c->mCharacter->IsSlopeTooSteep(ToJph(normal)); }
float JPC_CharacterVirtual_GetMass(const JPC_CharacterVirtual *c) { return c->mCharacter->GetMass(); }
void JPC_CharacterVirtual_SetMass(JPC_CharacterVirtual *c, float mass) { c->mCharacter->SetMass(mass); }
float JPC_CharacterVirtual_GetMaxStrength(const JPC_CharacterVirtual *c) { return c->mCharacter->GetMaxStrength(); }
void JPC_CharacterVirtual_SetMaxStrength(JPC_CharacterVirtual *c, float strength) { c->mCharacter->SetMaxStrength(strength); }
float JPC_CharacterVirtual_GetPenetrationRecoverySpeed(const JPC_CharacterVirtual *c) { return c->mCharacter->GetPenetrationRecoverySpeed(); }
void JPC_CharacterVirtual_SetPenetrationRecoverySpeed(JPC_CharacterVirtual *c, float speed) { c->mCharacter->SetPenetrationRecoverySpeed(speed); }
JPC_Vec3 JPC_CharacterVirtual_GetShapeOffset(const JPC_CharacterVirtual *c) { return ToC(c->mCharacter->GetShapeOffset()); }
void JPC_CharacterVirtual_SetShapeOffset(JPC_CharacterVirtual *c, JPC_Vec3 offset) { c->mCharacter->SetShapeOffset(ToJph(offset)); }
uint64_t JPC_CharacterVirtual_GetUserData(const JPC_CharacterVirtual *c) { return c->mCharacter->GetUserData(); }
void JPC_CharacterVirtual_SetUserData(JPC_CharacterVirtual *c, uint64_t user_data) { c->mCharacter->SetUserData(user_data); }
bool JPC_CharacterVirtual_GetEnhancedInternalEdgeRemoval(const JPC_CharacterVirtual *c) { return c->mCharacter->GetEnhancedInternalEdgeRemoval(); }
void JPC_CharacterVirtual_SetEnhancedInternalEdgeRemoval(JPC_CharacterVirtual *c, bool enabled) { c->mCharacter->SetEnhancedInternalEdgeRemoval(enabled); }
const JPC_Shape *JPC_CharacterVirtual_GetShape(const JPC_CharacterVirtual *c) { return reinterpret_cast<const JPC_Shape *>(c->mCharacter->GetShape()); }
JPC_BodyID JPC_CharacterVirtual_GetInnerBodyID(const JPC_CharacterVirtual *c) { return ToC(c->mCharacter->GetInnerBodyID()); }
void JPC_CharacterVirtual_GetGround(const JPC_CharacterVirtual *c, JPC_CharacterGround *out_ground) { GetGround(*c->mCharacter, out_ground); }
bool JPC_CharacterVirtual_IsSupported(const JPC_CharacterVirtual *c) { return c->mCharacter->IsSupported(); }
bool JPC_CharacterVirtual_HasCollidedWith(const JPC_CharacterVirtual *c, JPC_BodyID body_id) { return c->mCharacter->HasCollidedWith(ToJph(body_id)); }
bool JPC_CharacterVirtual_GetMaxHitsExceeded(const JPC_CharacterVirtual *c) { return c->mCharacter->GetMaxHitsExceeded(); }

uint32_t JPC_CharacterVirtual_GetActiveContacts(const JPC_CharacterVirtual *c, JPC_CharacterContact *out_contacts, uint32_t capacity)
{
	const CharacterVirtual::ContactList &contacts = c->mCharacter->GetActiveContacts();
	uint32_t count = (uint32_t)contacts.size();
	for (uint32_t i = 0; i < count && i < capacity; ++i)
		out_contacts[i] = ToC(contacts[i]);
	return count;
}

JPC_CharacterVsCharacterCollision *JPC_CharacterVsCharacterCollision_Create(void) { return new JPC_CharacterVsCharacterCollision; }
void JPC_CharacterVsCharacterCollision_Destroy(JPC_CharacterVsCharacterCollision *collision) { delete collision; }

void JPC_CharacterVsCharacterCollision_Add(JPC_CharacterVsCharacterCollision *collision, JPC_CharacterVirtual *character)
{
	if (character->mCollision == collision)
		return;
	if (character->mCollision != nullptr)
		JPC_CharacterVsCharacterCollision_Remove(character->mCollision, character);
	collision->mCollision.Add(character->mCharacter);
	character->mCharacter->SetCharacterVsCharacterCollision(&collision->mCollision);
	character->mCollision = collision;
}

void JPC_CharacterVsCharacterCollision_Remove(JPC_CharacterVsCharacterCollision *collision, JPC_CharacterVirtual *character)
{
	if (character->mCollision != collision)
		return;
	collision->mCollision.Remove(character->mCharacter);
	character->mCharacter->SetCharacterVsCharacterCollision(nullptr);
	character->mCollision = nullptr;
}

// ---------------------------------------------------------------------------
// Character (rigid body)
// ---------------------------------------------------------------------------

struct JPC_Character
{
	JPH_OVERRIDE_NEW_DELETE

	JPC_PhysicsSystem *		mSystem = nullptr;
	Ref<Character>			mCharacter;
	// See JPC_CharacterVirtual::mMaterials
	RefConst<PhysicsMaterial> mGroundMaterial;
};

void JPC_CharacterSettings_SetDefault(JPC_CharacterSettings *out)
{
	CharacterSettings s;
	out->up = ToC(s.mUp);
	out->supporting_volume_normal = ToC(s.mSupportingVolume.GetNormal());
	out->supporting_volume_constant = s.mSupportingVolume.GetConstant();
	out->max_slope_angle = s.mMaxSlopeAngle;
	out->enhanced_internal_edge_removal = s.mEnhancedInternalEdgeRemoval;
	out->shape = nullptr;
	out->layer = s.mLayer;
	out->mass = s.mMass;
	out->friction = s.mFriction;
	out->gravity_factor = s.mGravityFactor;
	out->allowed_dofs = (JPC_AllowedDOFs)s.mAllowedDOFs;
}

JPC_Character *JPC_Character_Create(const JPC_CharacterSettings *in, JPC_RVec3 position, JPC_Quat rotation, uint64_t user_data, JPC_PhysicsSystem *system)
{
	if (in->shape == nullptr)
	{
		JPC_SetError("Character shape is null");
		return nullptr;
	}
	if (in->allowed_dofs == JPC_ALLOWED_DOFS_NONE)
	{
		JPC_SetError("allowed_dofs must not be NONE");
		return nullptr;
	}
	if (!system->IsValidLayer(in->layer))
	{
		JPC_SetError("Object layer does not exist in this physics system");
		return nullptr;
	}
	if (system->mSystem.GetNumBodies() >= system->mSystem.GetMaxBodies())
	{
		JPC_SetError("Out of bodies (max_bodies reached)");
		return nullptr;
	}

	Ref<CharacterSettings> s = new CharacterSettings;
	ToJphBase(in->up, in->supporting_volume_normal, in->supporting_volume_constant, in->max_slope_angle, in->enhanced_internal_edge_removal, in->shape, *s);
	s->mLayer = in->layer;
	s->mMass = in->mass;
	s->mFriction = in->friction;
	s->mGravityFactor = in->gravity_factor;
	s->mAllowedDOFs = (EAllowedDOFs)in->allowed_dofs;

	Ref<Character> character = new Character(s, ToJph(position), ToJph(rotation), user_data, &system->mSystem);
	if (character->GetBodyID().IsInvalid())
	{
		// ~Character destroys its body without checking that it exists, which corrupts the body manager.
		// Only reachable when another thread took the last body slot after the check above: leak this bodiless character.
		character->AddRef();
		JPC_SetError("Out of bodies (max_bodies reached)");
		return nullptr;
	}
	system->Protect(character->GetBodyID());

	JPC_Character *out = new JPC_Character;
	out->mSystem = system;
	out->mCharacter = character;
	return out;
}

void JPC_Character_Destroy(JPC_Character *character)
{
	JPC_PhysicsSystem *system = character->mSystem;
	{
		std::unique_lock lock(system->mBodyStateMutex);
		system->Unprotect(character->mCharacter->GetBodyID());
		// ~Character destroys the body, which must not be in the world anymore
		character->mCharacter->RemoveFromPhysicsSystem();
		character->mCharacter = nullptr;
	}
	delete character;
}

JPC_BodyID JPC_Character_GetBodyID(const JPC_Character *c) { return ToC(c->mCharacter->GetBodyID()); }

void JPC_Character_AddToPhysicsSystem(JPC_Character *c, JPC_Activation activation)
{
	std::unique_lock lock(c->mSystem->mBodyStateMutex);
	if (!c->mSystem->mSystem.GetBodyInterface().IsAdded(c->mCharacter->GetBodyID()))
		c->mCharacter->AddToPhysicsSystem(ToJph(activation));
}

void JPC_Character_RemoveFromPhysicsSystem(JPC_Character *c)
{
	std::unique_lock lock(c->mSystem->mBodyStateMutex);
	c->mCharacter->RemoveFromPhysicsSystem();
}

void JPC_Character_Activate(JPC_Character *c)
{
	std::shared_lock lock(c->mSystem->mBodyStateMutex);
	if (c->mSystem->mSystem.GetBodyInterface().IsAdded(c->mCharacter->GetBodyID()))
		c->mCharacter->Activate();
}

void JPC_Character_PostSimulation(JPC_Character *c, float max_separation_distance)
{
	c->mCharacter->PostSimulation(max_separation_distance);
	c->mGroundMaterial = c->mCharacter->GetGroundMaterial();
}
JPC_Vec3 JPC_Character_GetLinearVelocity(const JPC_Character *c) { return ToC(c->mCharacter->GetLinearVelocity()); }
void JPC_Character_SetLinearVelocity(JPC_Character *c, JPC_Vec3 velocity) { c->mCharacter->SetLinearVelocity(ToJph(velocity)); }
void JPC_Character_AddLinearVelocity(JPC_Character *c, JPC_Vec3 velocity) { c->mCharacter->AddLinearVelocity(ToJph(velocity)); }
void JPC_Character_SetLinearAndAngularVelocity(JPC_Character *c, JPC_Vec3 linear, JPC_Vec3 angular) { c->mCharacter->SetLinearAndAngularVelocity(ToJph(linear), ToJph(angular)); }

void JPC_Character_AddImpulse(JPC_Character *c, JPC_Vec3 impulse)
{
	// AddImpulse activates the body without checking that it is in the world
	std::shared_lock lock(c->mSystem->mBodyStateMutex);
	if (c->mSystem->mSystem.GetBodyInterface().IsAdded(c->mCharacter->GetBodyID()))
		c->mCharacter->AddImpulse(ToJph(impulse));
}

JPC_RVec3 JPC_Character_GetPosition(const JPC_Character *c) { return ToC(c->mCharacter->GetPosition()); }
void JPC_Character_SetPosition(JPC_Character *c, JPC_RVec3 position, JPC_Activation activation) { c->mCharacter->SetPosition(ToJph(position), ToJph(activation)); }
JPC_Quat JPC_Character_GetRotation(const JPC_Character *c) { return ToC(c->mCharacter->GetRotation()); }
void JPC_Character_SetRotation(JPC_Character *c, JPC_Quat rotation, JPC_Activation activation) { c->mCharacter->SetRotation(ToJph(rotation).Normalized(), ToJph(activation)); }
JPC_RVec3 JPC_Character_GetCenterOfMassPosition(const JPC_Character *c) { return ToC(c->mCharacter->GetCenterOfMassPosition()); }
JPC_RMat44 JPC_Character_GetWorldTransform(const JPC_Character *c) { return ToC(c->mCharacter->GetWorldTransform()); }
JPC_ObjectLayer JPC_Character_GetLayer(const JPC_Character *c) { return c->mCharacter->GetLayer(); }
bool JPC_Character_SetLayer(JPC_Character *c, JPC_ObjectLayer layer)
{
	if (!c->mSystem->IsValidLayer(layer))
	{
		JPC_SetError("Object layer does not exist in this physics system");
		return false;
	}
	c->mCharacter->SetLayer(layer);
	return true;
}

bool JPC_Character_SetShape(JPC_Character *c, const JPC_Shape *shape, float max_penetration_depth)
{
	return shape != nullptr && c->mCharacter->SetShape(ToJph(shape), max_penetration_depth);
}

const JPC_Shape *JPC_Character_GetShape(const JPC_Character *c) { return reinterpret_cast<const JPC_Shape *>(c->mCharacter->GetShape()); }
JPC_Vec3 JPC_Character_GetUp(const JPC_Character *c) { return ToC(c->mCharacter->GetUp()); }
void JPC_Character_SetUp(JPC_Character *c, JPC_Vec3 up) { c->mCharacter->SetUp(ToJph(up)); }
void JPC_Character_SetMaxSlopeAngle(JPC_Character *c, float angle) { c->mCharacter->SetMaxSlopeAngle(angle); }
void JPC_Character_GetGround(const JPC_Character *c, JPC_CharacterGround *out_ground) { GetGround(*c->mCharacter, out_ground); }
bool JPC_Character_IsSupported(const JPC_Character *c) { return c->mCharacter->IsSupported(); }
