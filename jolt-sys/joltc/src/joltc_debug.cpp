// Debug rendering and collision response estimation.
#include "joltc_internal.h"

#include <Jolt/Physics/Collision/EstimateCollisionResponse.h>
#include <Jolt/Skeleton/Skeleton.h>
#include <Jolt/Skeleton/SkeletonPose.h>

#ifdef JPH_DEBUG_RENDERER
#include <Jolt/Physics/Body/BodyFilter.h>
#include <Jolt/Renderer/DebugRendererSimple.h>
#endif

// ---------------------------------------------------------------------------
// Collision response estimation
// ---------------------------------------------------------------------------

bool JPC_EstimateCollisionResponse(const JPC_Body *body1, const JPC_Body *body2, const JPC_ContactManifold *manifold, const JPC_ContactSettings *settings,
								   float min_velocity_for_restitution, uint32_t num_iterations, JPC_CollisionEstimation *out)
{
	if (manifold->num_points > ContactPoints::Capacity)
	{
		JPC_SetError("Manifold has too many contact points");
		return false;
	}
	ContactManifold m;
	m.mBaseOffset = ToJph(manifold->base_offset);
	m.mWorldSpaceNormal = ToJph(manifold->world_space_normal);
	m.mPenetrationDepth = manifold->penetration_depth;
	m.mSubShapeID1.SetValue(manifold->sub_shape_id1);
	m.mSubShapeID2.SetValue(manifold->sub_shape_id2);
	for (uint32_t i = 0; i < manifold->num_points; ++i)
	{
		m.mRelativeContactPointsOn1.push_back(ToJph(manifold->relative_points_on1[i]));
		m.mRelativeContactPointsOn2.push_back(ToJph(manifold->relative_points_on2[i]));
	}

	ContactSettings s;
	CopyBack(*settings, s);
	CollisionEstimationResult result;
	EstimateCollisionResponse(*ToJph(body1), *ToJph(body2), m, result, s, min_velocity_for_restitution, max(num_iterations, 1u));

	*out = { };
	out->linear_velocity1 = ToC(result.mLinearVelocity1);
	out->angular_velocity1 = ToC(result.mAngularVelocity1);
	out->linear_velocity2 = ToC(result.mLinearVelocity2);
	out->angular_velocity2 = ToC(result.mAngularVelocity2);
	out->friction_point = ToC(result.mFrictionPoint);
	out->friction_impulse = ToC(result.mFrictionImpulse);
	out->angular_friction_impulse = result.mAngularFrictionImpulse;
	out->num_contact_impulses = uint32_t(result.mContactImpulse.size());
	for (uint i = 0; i < result.mContactImpulse.size(); ++i)
		out->contact_impulses[i] = result.mContactImpulse[i];
	return true;
}

// ---------------------------------------------------------------------------
// Debug rendering
// ---------------------------------------------------------------------------

void JPC_BodyDrawSettings_SetDefault(JPC_BodyDrawSettings *out)
{
	*out = { };
	out->draw_shape = true;
	out->shape_color = JPC_SHAPE_COLOR_MOTION_TYPE;
	out->soft_body_constraint_color = JPC_SOFT_BODY_CONSTRAINT_COLOR_TYPE;
}

#ifdef JPH_DEBUG_RENDERER

static inline JPC_Color ToC(ColorArg inColor) { return { inColor.r, inColor.g, inColor.b, inColor.a }; }

class CallbackDebugRenderer final : public DebugRendererSimple
{
public:
	JPH_OVERRIDE_NEW_DELETE

	virtual void			DrawLine(RVec3Arg inFrom, RVec3Arg inTo, ColorArg inColor) override
	{
		if (mFns.draw_line != nullptr)
			mFns.draw_line(mUserData, ToC(inFrom), ToC(inTo), ToC(inColor));
	}

	virtual void			DrawTriangle(RVec3Arg inV1, RVec3Arg inV2, RVec3Arg inV3, ColorArg inColor, ECastShadow inCastShadow) override
	{
		if (mFns.draw_triangle != nullptr)
			mFns.draw_triangle(mUserData, ToC(inV1), ToC(inV2), ToC(inV3), ToC(inColor), inCastShadow == ECastShadow::On);
		else
			DebugRendererSimple::DrawTriangle(inV1, inV2, inV3, inColor, inCastShadow);
	}

	virtual void			DrawText3D(RVec3Arg inPosition, const string_view &inString, ColorArg inColor, float inHeight) override
	{
		if (mFns.draw_text != nullptr)
			mFns.draw_text(mUserData, ToC(inPosition), inString.data(), inString.size(), ToC(inColor), inHeight);
	}

	JPC_DebugRendererFns	mFns { };
	void *					mUserData = nullptr;
};

class CallbackBodyDrawFilter final : public BodyDrawFilter
{
public:
							CallbackBodyDrawFilter(JPC_BodyDrawFilterFn inFn, void *inUserData) : mFn(inFn), mUserData(inUserData) { }

	virtual bool			ShouldDraw(const Body &inBody) const override			{ return mFn(mUserData, ToC(inBody.GetID())); }

private:
	JPC_BodyDrawFilterFn	mFn;
	void *					mUserData;
};

struct JPC_DebugRenderer
{
	JPH_OVERRIDE_NEW_DELETE

	CallbackDebugRenderer	mRenderer;
};

static inline DebugRenderer *ToJph(JPC_DebugRenderer *r) { return &r->mRenderer; }

// The base class registers itself as Jolt's single DebugRenderer::sInstance and only asserts that there is no other
static std::mutex sRendererMutex;

bool JPC_DebugRenderer_IsAvailable(void) { return true; }

JPC_DebugRenderer *JPC_DebugRenderer_Create(const JPC_DebugRendererFns *fns, void *user_data)
{
	std::lock_guard lock(sRendererMutex);
	if (DebugRenderer::sInstance != nullptr)
	{
		JPC_SetError("Only one debug renderer can exist at a time");
		return nullptr;
	}
	JPC_DebugRenderer *renderer = new JPC_DebugRenderer;
	JPC_DebugRenderer_SetFns(renderer, fns, user_data);
	return renderer;
}

void JPC_DebugRenderer_Destroy(JPC_DebugRenderer *renderer)
{
	std::lock_guard lock(sRendererMutex);
	delete renderer;
}

void JPC_DebugRenderer_SetFns(JPC_DebugRenderer *renderer, const JPC_DebugRendererFns *fns, void *user_data)
{
	renderer->mRenderer.mFns = fns != nullptr ? *fns : JPC_DebugRendererFns { };
	renderer->mRenderer.mUserData = fns != nullptr ? user_data : nullptr;
}

void JPC_DebugRenderer_SetCameraPosition(JPC_DebugRenderer *renderer, JPC_RVec3 position) { renderer->mRenderer.SetCameraPos(ToJph(position)); }

void JPC_PhysicsSystem_DrawBodies(JPC_PhysicsSystem *system, const JPC_BodyDrawSettings *in, JPC_DebugRenderer *renderer, JPC_BodyDrawFilterFn filter,
								  void *filter_user_data)
{
	BodyManager::DrawSettings s;
	s.mDrawGetSupportFunction = in->draw_get_support_function;
	s.mDrawSupportDirection = in->draw_support_direction;
	s.mDrawGetSupportingFace = in->draw_get_supporting_face;
	s.mDrawShape = in->draw_shape;
	s.mDrawShapeWireframe = in->draw_shape_wireframe;
	s.mDrawShapeColor = in->shape_color <= JPC_SHAPE_COLOR_MATERIAL ? (BodyManager::EShapeColor)in->shape_color : BodyManager::EShapeColor::MotionTypeColor;
	s.mDrawBoundingBox = in->draw_bounding_box;
	s.mDrawCenterOfMassTransform = in->draw_center_of_mass_transform;
	s.mDrawWorldTransform = in->draw_world_transform;
	s.mDrawVelocity = in->draw_velocity;
	s.mDrawMassAndInertia = in->draw_mass_and_inertia;
	s.mDrawSleepStats = in->draw_sleep_stats;
	s.mDrawSoftBodyVertices = in->draw_soft_body_vertices;
	s.mDrawSoftBodyVertexVelocities = in->draw_soft_body_vertex_velocities;
	s.mDrawSoftBodyEdgeConstraints = in->draw_soft_body_edge_constraints;
	s.mDrawSoftBodyBendConstraints = in->draw_soft_body_bend_constraints;
	s.mDrawSoftBodyVolumeConstraints = in->draw_soft_body_volume_constraints;
	s.mDrawSoftBodySkinConstraints = in->draw_soft_body_skin_constraints;
	s.mDrawSoftBodyLRAConstraints = in->draw_soft_body_lra_constraints;
	s.mDrawSoftBodyRods = in->draw_soft_body_rods;
	s.mDrawSoftBodyRodStates = in->draw_soft_body_rod_states;
	s.mDrawSoftBodyRodBendTwistConstraints = in->draw_soft_body_rod_bend_twist_constraints;
	s.mDrawSoftBodyPredictedBounds = in->draw_soft_body_predicted_bounds;
	s.mDrawSoftBodyConstraintColor = in->soft_body_constraint_color <= JPC_SOFT_BODY_CONSTRAINT_COLOR_ORDER
		? (ESoftBodyConstraintColor)in->soft_body_constraint_color : ESoftBodyConstraintColor::ConstraintType;
	if (filter != nullptr)
	{
		CallbackBodyDrawFilter body_filter(filter, filter_user_data);
		system->mSystem.DrawBodies(s, ToJph(renderer), &body_filter);
	}
	else
		system->mSystem.DrawBodies(s, ToJph(renderer));
}

void JPC_PhysicsSystem_DrawConstraints(JPC_PhysicsSystem *system, JPC_DebugRenderer *renderer) { system->mSystem.DrawConstraints(ToJph(renderer)); }
void JPC_PhysicsSystem_DrawConstraintLimits(JPC_PhysicsSystem *system, JPC_DebugRenderer *renderer) { system->mSystem.DrawConstraintLimits(ToJph(renderer)); }
void JPC_PhysicsSystem_DrawConstraintReferenceFrames(JPC_PhysicsSystem *system, JPC_DebugRenderer *renderer) { system->mSystem.DrawConstraintReferenceFrame(ToJph(renderer)); }

void JPC_Skeleton_DrawPose(const JPC_Skeleton *skeleton, const JPC_Mat44 *joint_matrices, JPC_RVec3 root_offset, bool draw_joint_names, JPC_DebugRenderer *renderer)
{
	SkeletonPose pose;
	pose.SetSkeleton(reinterpret_cast<const Skeleton *>(skeleton));
	for (uint i = 0; i < pose.GetJointCount(); ++i)
		pose.GetJointMatrix(i) = ToJph(joint_matrices[i]);
	SkeletonPose::DrawSettings settings;
	settings.mDrawJointNames = draw_joint_names;
	pose.Draw(settings, ToJph(renderer), RMat44::sTranslation(ToJph(root_offset)));
}

#else // JPH_DEBUG_RENDERER

// Never created, so the functions taking one are never called with a valid pointer
struct JPC_DebugRenderer { };

bool JPC_DebugRenderer_IsAvailable(void) { return false; }

JPC_DebugRenderer *JPC_DebugRenderer_Create(const JPC_DebugRendererFns *, void *)
{
	JPC_SetError("Jolt was built without its debug renderer (enable the debug-renderer feature)");
	return nullptr;
}

void JPC_DebugRenderer_Destroy(JPC_DebugRenderer *) { }
void JPC_DebugRenderer_SetFns(JPC_DebugRenderer *, const JPC_DebugRendererFns *, void *) { }
void JPC_DebugRenderer_SetCameraPosition(JPC_DebugRenderer *, JPC_RVec3) { }
void JPC_PhysicsSystem_DrawBodies(JPC_PhysicsSystem *, const JPC_BodyDrawSettings *, JPC_DebugRenderer *, JPC_BodyDrawFilterFn, void *) { }
void JPC_PhysicsSystem_DrawConstraints(JPC_PhysicsSystem *, JPC_DebugRenderer *) { }
void JPC_PhysicsSystem_DrawConstraintLimits(JPC_PhysicsSystem *, JPC_DebugRenderer *) { }
void JPC_PhysicsSystem_DrawConstraintReferenceFrames(JPC_PhysicsSystem *, JPC_DebugRenderer *) { }
void JPC_Skeleton_DrawPose(const JPC_Skeleton *, const JPC_Mat44 *, JPC_RVec3, bool, JPC_DebugRenderer *) { }

#endif // JPH_DEBUG_RENDERER
