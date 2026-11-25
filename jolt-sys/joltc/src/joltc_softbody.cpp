// Soft bodies.
#include "joltc_internal.h"

#include <Jolt/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodyMotionProperties.h>
#include <Jolt/Physics/SoftBody/SoftBodySharedSettings.h>
#include <Jolt/Core/UnorderedMap.h>

#include <cfloat>
#include <cmath>

static inline const SoftBodySharedSettings *ToJph(const JPC_SoftBodySharedSettings *s) { return reinterpret_cast<const SoftBodySharedSettings *>(s); }

static JPC_SoftBodySharedSettings *ToC(Ref<SoftBodySharedSettings> &inSettings)
{
	inSettings->AddRef();
	return reinterpret_cast<JPC_SoftBodySharedSettings *>(inSettings.GetPtr());
}

// ---------------------------------------------------------------------------
// Shared settings
// ---------------------------------------------------------------------------

void JPC_SoftBodyVertexAttributes_SetDefault(JPC_SoftBodyVertexAttributes *out)
{
	SoftBodySharedSettings::VertexAttributes a;
	out->compliance = a.mCompliance;
	out->shear_compliance = a.mShearCompliance;
	out->bend_compliance = a.mBendCompliance;
	out->lra_type = (JPC_SoftBodyLRAType)a.mLRAType;
	out->lra_max_distance_multiplier = a.mLRAMaxDistanceMultiplier;
}

void JPC_SoftBodySharedSettingsDesc_SetDefault(JPC_SoftBodySharedSettingsDesc *out)
{
	*out = { };
	out->bend_type = JPC_SOFT_BODY_BEND_TYPE_DISTANCE;
	out->angle_tolerance = DegreesToRadians(8.0f);
}

namespace {

// Jolt indexes vertices with every constraint and only asserts that they are distinct and in range
template <size_t N>
bool CheckVertices(const uint32_t (&inVertices)[N], uint32_t inNumVertices)
{
	for (size_t i = 0; i < N; ++i)
	{
		if (inVertices[i] >= inNumVertices)
			return false;
		for (size_t j = 0; j < i; ++j)
			if (inVertices[i] == inVertices[j])
				return false;
	}
	return true;
}

bool IsCompliance(float inCompliance) { return inCompliance >= 0.0f; } // FLT_MAX (disabled) included, NaN excluded

template <class T>
bool CheckArray(const T *inArray, uint32_t inCount) { return inCount == 0 || inArray != nullptr; }

bool SamePosition(const JPC_SoftBodySharedSettingsDesc &d, uint32_t inA, uint32_t inB)
{
	const JPC_Vec3 &a = d.vertices[inA].position, &b = d.vertices[inB].position;
	return a.x == b.x && a.y == b.y && a.z == b.z;
}

// Jolt asserts while generating that no edge has length 0. It creates edges along every face edge and, for two faces
// that share an edge, between their opposite corners (shear and bend edges).
bool FacesMakeZeroLengthEdges(const JPC_SoftBodySharedSettingsDesc &d)
{
	UnorderedMap<uint64, Array<uint32>> opposite;
	for (uint32_t i = 0; i < d.num_faces; ++i)
	{
		const uint32_t *v = d.faces[i].vertices;
		for (int e = 0; e < 3; ++e)
		{
			uint32_t a = v[e], b = v[(e + 1) % 3], c = v[(e + 2) % 3];
			if (SamePosition(d, a, b))
				return true;
			opposite[(uint64(min(a, b)) << 32) | max(a, b)].push_back(c);
		}
	}
	for (const auto &edge : opposite)
		for (size_t i = 0; i < edge.second.size(); ++i)
			for (size_t j = 0; j < i; ++j)
				if (edge.second[i] != edge.second[j] && SamePosition(d, edge.second[i], edge.second[j]))
					return true;
	return false;
}

const char *Validate(const JPC_SoftBodySharedSettingsDesc &d)
{
	if (d.num_vertices == 0 || d.vertices == nullptr)
		return "soft body needs at least one vertex";
	if (!CheckArray(d.faces, d.num_faces) || !CheckArray(d.materials, d.num_materials) || !CheckArray(d.vertex_attributes, d.num_vertex_attributes)
		|| !CheckArray(d.edges, d.num_edges) || !CheckArray(d.dihedral_bends, d.num_dihedral_bends) || !CheckArray(d.volumes, d.num_volumes)
		|| !CheckArray(d.lras, d.num_lras))
		return "soft body array is null";
	for (uint32_t i = 0; i < d.num_vertices; ++i)
	{
		const JPC_SoftBodyVertex &v = d.vertices[i];
		if (!(v.inv_mass >= 0.0f) || !std::isfinite(v.inv_mass)
			|| !std::isfinite(v.position.x) || !std::isfinite(v.position.y) || !std::isfinite(v.position.z))
			return "vertex positions and inverse masses must be finite, inverse masses not negative";
	}
	for (uint32_t i = 0; i < d.num_materials; ++i)
		if (d.materials[i] == nullptr)
			return "soft body material is null";
	uint32_t num_materials = max(d.num_materials, 1u);
	for (uint32_t i = 0; i < d.num_faces; ++i)
	{
		if (!CheckVertices(d.faces[i].vertices, d.num_vertices))
			return "face needs 3 different vertices that exist";
		if (d.faces[i].material_index >= num_materials)
			return "face material index out of range";
	}
	for (uint32_t i = 0; i < d.num_vertex_attributes; ++i)
	{
		const JPC_SoftBodyVertexAttributes &a = d.vertex_attributes[i];
		if (!IsCompliance(a.compliance) || !IsCompliance(a.shear_compliance) || !IsCompliance(a.bend_compliance) || !(a.lra_max_distance_multiplier > 0.0f)
			|| uint32_t(a.lra_type) > JPC_SOFT_BODY_LRA_TYPE_GEODESIC_DISTANCE)
			return "vertex attributes need compliances >= 0, a known LRA type and an LRA multiplier > 0";
	}
	if (uint32_t(d.bend_type) > JPC_SOFT_BODY_BEND_TYPE_DIHEDRAL)
		return "unknown bend type";
	if (d.num_vertex_attributes > 0 && FacesMakeZeroLengthEdges(d))
		return "faces put two connected vertices at the same position";
	for (uint32_t i = 0; i < d.num_edges; ++i)
	{
		if (!CheckVertices(d.edges[i].vertices, d.num_vertices) || !IsCompliance(d.edges[i].compliance))
			return "edge needs 2 different vertices that exist and a compliance >= 0";
		if (SamePosition(d, d.edges[i].vertices[0], d.edges[i].vertices[1]))
			return "edge connects two vertices at the same position";
	}
	for (uint32_t i = 0; i < d.num_dihedral_bends; ++i)
		if (!CheckVertices(d.dihedral_bends[i].vertices, d.num_vertices) || !IsCompliance(d.dihedral_bends[i].compliance))
			return "dihedral bend needs 4 different vertices that exist and a compliance >= 0";
	for (uint32_t i = 0; i < d.num_volumes; ++i)
		if (!CheckVertices(d.volumes[i].vertices, d.num_vertices) || !IsCompliance(d.volumes[i].compliance))
			return "volume constraint needs 4 different vertices that exist and a compliance >= 0";
	if (!CheckArray(d.inv_binds, d.num_inv_binds) || !CheckArray(d.skinned, d.num_skinned) || !CheckArray(d.rods, d.num_rods)
		|| !CheckArray(d.rod_bend_twists, d.num_rod_bend_twists))
		return "soft body array is null";
	if (d.num_skinned > 0)
	{
		// Jolt packs the faces around a skinned vertex into 8 bits of count and 24 bits of offset
		Array<uint32> faces_per_vertex(d.num_vertices, 0);
		for (uint32_t i = 0; i < d.num_faces; ++i)
			for (uint32_t v : d.faces[i].vertices)
				++faces_per_vertex[v];
		Array<bool> skinned(d.num_vertices, false);
		uint64 total_faces = 0;
		for (uint32_t i = 0; i < d.num_skinned; ++i)
		{
			const JPC_SoftBodySkinned &k = d.skinned[i];
			if (k.vertex >= d.num_vertices || skinned[k.vertex])
				return "skinned vertex does not exist or is skinned twice";
			skinned[k.vertex] = true;
			if (faces_per_vertex[k.vertex] >= 256)
				return "a skinned vertex touches 256 or more faces";
			total_faces += faces_per_vertex[k.vertex];
			if (!(k.max_distance >= 0.0f) || !(k.back_stop_distance >= -FLT_MAX) || !(k.back_stop_radius > 0.0f))
				return "skinned vertex needs max_distance >= 0 and back_stop_radius > 0";
			for (const JPC_SoftBodySkinWeight &w : k.weights)
			{
				if (w.weight == 0.0f)
					break;
				if (!(w.weight > 0.0f) || !std::isfinite(w.weight) || w.inv_bind_index >= d.num_inv_binds)
					return "skin weights must be positive and refer to an inverse bind matrix that exists";
			}
		}
		if (total_faces >= (1u << 24))
			return "too many faces around skinned vertices";
	}
	for (uint32_t i = 0; i < d.num_rods; ++i)
	{
		if (!CheckVertices(d.rods[i].vertices, d.num_vertices) || !IsCompliance(d.rods[i].compliance))
			return "rod needs 2 different vertices that exist and a compliance >= 0";
		if (SamePosition(d, d.rods[i].vertices[0], d.rods[i].vertices[1]))
			return "rod connects two vertices at the same position";
	}
	for (uint32_t i = 0; i < d.num_rod_bend_twists; ++i)
	{
		const JPC_SoftBodyRodBendTwist &b = d.rod_bend_twists[i];
		if (b.rods[0] >= d.num_rods || b.rods[1] >= d.num_rods || b.rods[0] == b.rods[1] || !IsCompliance(b.compliance))
			return "rod bend twist needs 2 different rods that exist and a compliance >= 0";
	}
	for (uint32_t i = 0; i < d.num_lras; ++i)
	{
		if (!CheckVertices(d.lras[i].vertices, d.num_vertices) || !(d.lras[i].max_distance_multiplier > 0.0f))
			return "LRA needs 2 different vertices that exist and a multiplier > 0";
		if (SamePosition(d, d.lras[i].vertices[0], d.lras[i].vertices[1]))
			return "LRA connects two vertices at the same position";
	}
	return nullptr;
}

// Lengths of zero make Jolt divide by zero while solving
const char *ValidateLengths(const SoftBodySharedSettings &inSettings)
{
	for (const SoftBodySharedSettings::Edge &e : inSettings.mEdgeConstraints)
		if (!(e.mRestLength > 0.0f))
			return "edge connects two vertices at the same position";
	for (const SoftBodySharedSettings::LRA &l : inSettings.mLRAConstraints)
		if (!(l.mMaxDistance > 0.0f))
			return "LRA connects two vertices at the same position";
	return nullptr;
}

} // namespace

JPC_SoftBodySharedSettings *JPC_SoftBodySharedSettings_Create(const JPC_SoftBodySharedSettingsDesc *desc)
{
	const JPC_SoftBodySharedSettingsDesc &d = *desc;
	if (const char *error = Validate(d))
	{
		JPC_SetError(error);
		return nullptr;
	}

	Ref<SoftBodySharedSettings> s = new SoftBodySharedSettings;
	s->mVertices.reserve(d.num_vertices);
	for (uint32_t i = 0; i < d.num_vertices; ++i)
	{
		const JPC_SoftBodyVertex &v = d.vertices[i];
		s->mVertices.emplace_back(Float3(v.position.x, v.position.y, v.position.z), Float3(v.velocity.x, v.velocity.y, v.velocity.z), v.inv_mass);
	}
	if (d.num_materials > 0)
	{
		s->mMaterials.clear();
		for (uint32_t i = 0; i < d.num_materials; ++i)
			s->mMaterials.push_back(ToJph(d.materials[i]));
	}
	for (uint32_t i = 0; i < d.num_faces; ++i)
	{
		const JPC_SoftBodyFace &f = d.faces[i];
		s->AddFace(SoftBodySharedSettings::Face(f.vertices[0], f.vertices[1], f.vertices[2], f.material_index));
	}

	// Generated constraints first, so the explicit ones come on top
	if (d.num_vertex_attributes > 0)
	{
		Array<SoftBodySharedSettings::VertexAttributes> attributes;
		attributes.reserve(d.num_vertex_attributes);
		for (uint32_t i = 0; i < d.num_vertex_attributes; ++i)
		{
			const JPC_SoftBodyVertexAttributes &a = d.vertex_attributes[i];
			attributes.emplace_back(a.compliance, a.shear_compliance, a.bend_compliance, (SoftBodySharedSettings::ELRAType)a.lra_type,
									a.lra_max_distance_multiplier);
		}
		s->CreateConstraints(attributes.data(), uint(attributes.size()), (SoftBodySharedSettings::EBendType)d.bend_type, d.angle_tolerance);
	}

	for (uint32_t i = 0; i < d.num_edges; ++i)
		s->mEdgeConstraints.emplace_back(d.edges[i].vertices[0], d.edges[i].vertices[1], d.edges[i].compliance);
	for (uint32_t i = 0; i < d.num_dihedral_bends; ++i)
	{
		const JPC_SoftBodyDihedralBend &b = d.dihedral_bends[i];
		s->mDihedralBendConstraints.emplace_back(b.vertices[0], b.vertices[1], b.vertices[2], b.vertices[3], b.compliance);
	}
	for (uint32_t i = 0; i < d.num_volumes; ++i)
	{
		const JPC_SoftBodyVolume &v = d.volumes[i];
		s->mVolumeConstraints.emplace_back(v.vertices[0], v.vertices[1], v.vertices[2], v.vertices[3], v.compliance);
	}
	for (uint32_t i = 0; i < d.num_lras; ++i)
	{
		// CalculateLRALengths would apply one multiplier to every LRA, generated ones included
		const JPC_SoftBodyLRA &l = d.lras[i];
		float distance = (Vec3(s->mVertices[l.vertices[1]].mPosition) - Vec3(s->mVertices[l.vertices[0]].mPosition)).Length();
		s->mLRAConstraints.emplace_back(l.vertices[0], l.vertices[1], distance * l.max_distance_multiplier);
	}

	for (uint32_t i = 0; i < d.num_inv_binds; ++i)
		s->mInvBindMatrices.emplace_back(d.inv_binds[i].joint_index, ToJph(d.inv_binds[i].inv_bind));
	for (uint32_t i = 0; i < d.num_skinned; ++i)
	{
		const JPC_SoftBodySkinned &k = d.skinned[i];
		SoftBodySharedSettings::Skinned skinned(k.vertex, k.max_distance, k.back_stop_distance, k.back_stop_radius);
		for (int w = 0; w < 4 && k.weights[w].weight != 0.0f; ++w)
			skinned.mWeights[w] = SoftBodySharedSettings::SkinWeight(k.weights[w].inv_bind_index, k.weights[w].weight);
		skinned.NormalizeWeights();
		s->mSkinnedConstraints.push_back(skinned);
	}
	for (uint32_t i = 0; i < d.num_rods; ++i)
		s->mRodStretchShearConstraints.emplace_back(d.rods[i].vertices[0], d.rods[i].vertices[1], d.rods[i].compliance);
	for (uint32_t i = 0; i < d.num_rod_bend_twists; ++i)
		s->mRodBendTwistConstraints.emplace_back(d.rod_bend_twists[i].rods[0], d.rod_bend_twists[i].rods[1], d.rod_bend_twists[i].compliance);

	// All of these derive from the rest pose, so recomputing the generated constraints changes nothing
	s->CalculateEdgeLengths();
	s->CalculateRodProperties();
	s->CalculateSkinnedConstraintNormals();
	s->CalculateBendConstraintConstants();
	s->CalculateVolumeConstraintVolumes();
	if (const char *error = ValidateLengths(*s))
	{
		JPC_SetError(error);
		return nullptr;
	}
	// Builds the groups the solver runs; without them nothing is solved
	s->Optimize();
	return ToC(s);
}

JPC_SoftBodySharedSettings *JPC_SoftBodySharedSettings_CreateCube(uint32_t grid_size, float spacing)
{
	if (grid_size < 2 || grid_size > 64 || !(spacing > 0.0f) || !std::isfinite(spacing))
	{
		JPC_SetError("Cube needs 2 to 64 vertices per side and a positive spacing");
		return nullptr;
	}
	Ref<SoftBodySharedSettings> s = SoftBodySharedSettings::sCreateCube(grid_size, spacing);
	return ToC(s);
}

// Number of joint matrices skinning needs: Jolt indexes them with the inverse bind matrices' joint indices
static uint32 SkinJointCount(const SoftBodySharedSettings &inSettings)
{
	uint32 count = 0;
	for (const SoftBodySharedSettings::InvBind &b : inSettings.mInvBindMatrices)
		count = max(count, b.mJointIndex + 1);
	return count;
}

void JPC_SoftBodySharedSettings_AddRef(const JPC_SoftBodySharedSettings *settings) { ToJph(settings)->AddRef(); }
void JPC_SoftBodySharedSettings_Release(const JPC_SoftBodySharedSettings *settings) { ToJph(settings)->Release(); }

void JPC_SoftBodySharedSettings_GetCounts(const JPC_SoftBodySharedSettings *settings, JPC_SoftBodySharedSettingsCounts *out)
{
	const SoftBodySharedSettings *s = ToJph(settings);
	out->vertices = uint32_t(s->mVertices.size());
	out->faces = uint32_t(s->mFaces.size());
	out->edges = uint32_t(s->mEdgeConstraints.size());
	out->dihedral_bends = uint32_t(s->mDihedralBendConstraints.size());
	out->volumes = uint32_t(s->mVolumeConstraints.size());
	out->lras = uint32_t(s->mLRAConstraints.size());
	out->skinned = uint32_t(s->mSkinnedConstraints.size());
	out->rods = uint32_t(s->mRodStretchShearConstraints.size());
	out->rod_bend_twists = uint32_t(s->mRodBendTwistConstraints.size());
	out->skin_joints = SkinJointCount(*s);
}

uint32_t JPC_SoftBodySharedSettings_GetRods(const JPC_SoftBodySharedSettings *settings, JPC_SoftBodyRod *out_rods, uint32_t capacity)
{
	const Array<SoftBodySharedSettings::RodStretchShear> &rods = ToJph(settings)->mRodStretchShearConstraints;
	for (uint32_t i = 0; i < capacity && i < rods.size(); ++i)
		out_rods[i] = { { rods[i].mVertex[0], rods[i].mVertex[1] }, rods[i].mCompliance };
	return uint32_t(rods.size());
}

uint32_t JPC_SoftBodySharedSettings_GetFaces(const JPC_SoftBodySharedSettings *settings, JPC_SoftBodyFace *out_faces, uint32_t capacity)
{
	const Array<SoftBodySharedSettings::Face> &faces = ToJph(settings)->mFaces;
	for (uint32_t i = 0; i < capacity && i < faces.size(); ++i)
	{
		const SoftBodySharedSettings::Face &f = faces[i];
		out_faces[i] = { { f.mVertex[0], f.mVertex[1], f.mVertex[2] }, f.mMaterialIndex };
	}
	return uint32_t(faces.size());
}

// ---------------------------------------------------------------------------
// Bodies
// ---------------------------------------------------------------------------

void JPC_SoftBodyCreationSettings_SetDefault(JPC_SoftBodyCreationSettings *out)
{
	SoftBodyCreationSettings s;
	out->settings = nullptr;
	out->position = ToC(s.mPosition);
	out->rotation = ToC(s.mRotation);
	out->user_data = s.mUserData;
	out->object_layer = s.mObjectLayer;
	out->collision_group = { nullptr, JPC_COLLISION_GROUP_INVALID, JPC_COLLISION_GROUP_INVALID };
	out->num_iterations = s.mNumIterations;
	out->linear_damping = s.mLinearDamping;
	out->max_linear_velocity = s.mMaxLinearVelocity;
	out->restitution = s.mRestitution;
	out->friction = s.mFriction;
	out->pressure = s.mPressure;
	out->gravity_factor = s.mGravityFactor;
	out->vertex_radius = s.mVertexRadius;
	out->update_position = s.mUpdatePosition;
	out->make_rotation_identity = s.mMakeRotationIdentity;
	out->allow_sleeping = s.mAllowSleeping;
	out->faces_double_sided = s.mFacesDoubleSided;
}

static bool ToJph(const JPC_BodyInterface *bi, const JPC_SoftBodyCreationSettings &in, SoftBodyCreationSettings &out)
{
	if (in.settings == nullptr)
	{
		JPC_SetError("Soft body creation settings have no shared settings");
		return false;
	}
	if (!bi->mOwner->IsValidLayer(in.object_layer))
	{
		JPC_SetError("Object layer does not exist in this physics system");
		return false;
	}
	if (!(in.vertex_radius >= 0.0f) || !(in.max_linear_velocity >= 0.0f) || !(in.linear_damping >= 0.0f))
	{
		JPC_SetError("vertex_radius, max_linear_velocity and linear_damping must not be negative");
		return false;
	}
	out.mSettings = ToJph(in.settings);
	out.mPosition = ToJph(in.position);
	Quat rotation = ToJph(in.rotation);
	out.mRotation = rotation.LengthSq() > 1.0e-12f ? rotation.Normalized() : Quat::sIdentity();
	out.mUserData = in.user_data;
	out.mObjectLayer = in.object_layer;
	out.mCollisionGroup = ToJph(in.collision_group);
	out.mNumIterations = in.num_iterations;
	out.mLinearDamping = in.linear_damping;
	out.mMaxLinearVelocity = in.max_linear_velocity;
	out.mRestitution = in.restitution;
	out.mFriction = in.friction;
	out.mPressure = in.pressure;
	out.mGravityFactor = in.gravity_factor;
	out.mVertexRadius = in.vertex_radius;
	out.mUpdatePosition = in.update_position;
	out.mMakeRotationIdentity = in.make_rotation_identity;
	out.mAllowSleeping = in.allow_sleeping;
	out.mFacesDoubleSided = in.faces_double_sided;
	return true;
}

// Jolt starts skinned positions as NaN, which the skin constraints would spread into the vertices and the body's bounds
// if the body is updated before it is first skinned. Skinning twice with identity joints makes the current and previous
// positions finite (but not the bind pose: skin the body before its first update).
static void SeedSkinState(Body &ioBody)
{
	SoftBodyMotionProperties *mp = static_cast<SoftBodyMotionProperties *>(ioBody.GetMotionProperties());
	const SoftBodySharedSettings &settings = *mp->GetSettings();
	if (settings.mSkinnedConstraints.empty())
		return;
	Array<Mat44> joints(max(SkinJointCount(settings), 1u), Mat44::sIdentity());
	TempAllocatorMalloc temp;
	for (int i = 0; i < 2; ++i)
		mp->SkinVertices(ioBody.GetCenterOfMassTransform(), joints.data(), uint(joints.size()), false, temp);
}

static Body *CreateSoftBody(JPC_BodyInterface *bi, const JPC_SoftBodyCreationSettings &inSettings)
{
	SoftBodyCreationSettings s;
	if (!ToJph(bi, inSettings, s))
		return nullptr;
	Body *body = ToJph(bi)->CreateSoftBody(s);
	if (body == nullptr)
	{
		JPC_SetError("Out of bodies (max_bodies reached)");
		return nullptr;
	}
	BodyLockWrite lock(*bi->mLockInterface, body->GetID());
	SeedSkinState(lock.GetBody());
	return body;
}

JPC_BodyID JPC_BodyInterface_CreateSoftBody(JPC_BodyInterface *bi, const JPC_SoftBodyCreationSettings *settings)
{
	Body *body = CreateSoftBody(bi, *settings);
	return body != nullptr ? ToC(body->GetID()) : JPC_BODY_ID_INVALID;
}

JPC_BodyID JPC_BodyInterface_CreateAndAddSoftBody(JPC_BodyInterface *bi, const JPC_SoftBodyCreationSettings *settings, JPC_Activation activation)
{
	Body *body = CreateSoftBody(bi, *settings);
	if (body == nullptr)
		return JPC_BODY_ID_INVALID;
	BodyID id = body->GetID();
	std::unique_lock lock(bi->mOwner->mBodyStateMutex);
	ToJph(bi)->AddBody(id, ToJph(activation));
	bi->mOwner->BumpStructure();
	return ToC(id);
}

bool JPC_BodyInterface_IsSoftBody(const JPC_BodyInterface *bi, JPC_BodyID id)
{
	BodyLockRead lock(*bi->mLockInterface, ToJph(id));
	return lock.Succeeded() && lock.GetBody().IsSoftBody();
}

JPC_SoftBodySharedSettings *JPC_BodyInterface_GetSoftBodySharedSettings(const JPC_BodyInterface *bi, JPC_BodyID id)
{
	BodyLockRead lock(*bi->mLockInterface, ToJph(id));
	if (!lock.Succeeded() || !lock.GetBody().IsSoftBody())
		return nullptr;
	const SoftBodySharedSettings *settings = static_cast<const SoftBodyMotionProperties *>(lock.GetBody().GetMotionProperties())->GetSettings();
	settings->AddRef();
	return reinterpret_cast<JPC_SoftBodySharedSettings *>(const_cast<SoftBodySharedSettings *>(settings));
}

static const SoftBodyMotionProperties *SoftBody(const Body &inBody)
{
	return inBody.IsSoftBody() ? static_cast<const SoftBodyMotionProperties *>(inBody.GetMotionProperties()) : nullptr;
}

static SoftBodyMotionProperties *SoftBody(Body &inBody)
{
	return inBody.IsSoftBody() ? static_cast<SoftBodyMotionProperties *>(inBody.GetMotionProperties()) : nullptr;
}

uint32_t JPC_BodyInterface_GetSoftBodyVertices(const JPC_BodyInterface *bi, JPC_BodyID id, uint32_t first, JPC_SoftBodyVertexState *out_vertices, uint32_t capacity,
											   bool world_space)
{
	BodyLockRead lock(*bi->mLockInterface, ToJph(id));
	if (!lock.Succeeded())
		return 0;
	const SoftBodyMotionProperties *mp = SoftBody(lock.GetBody());
	if (mp == nullptr)
		return 0;
	const Array<SoftBodyVertex> &vertices = mp->GetVertices();
	RMat44 transform = world_space ? lock.GetBody().GetCenterOfMassTransform() : RMat44::sIdentity();
	for (uint64 i = first, o = 0; i < vertices.size() && o < capacity; ++i, ++o)
	{
		const SoftBodyVertex &v = vertices[size_t(i)];
		// mHasContact is reset at the end of every update, so it means nothing here
		out_vertices[o] = { ToC(transform * v.mPosition), ToC(transform.Multiply3x3(v.mVelocity)), v.mInvMass };
	}
	return uint32_t(vertices.size());
}

bool JPC_BodyInterface_SetSoftBodyVertices(JPC_BodyInterface *bi, JPC_BodyID id, uint32_t first, const JPC_SoftBodyVertexState *vertices, uint32_t count,
										   bool world_space, JPC_Activation activation)
{
	{
		BodyLockWrite lock(*bi->mLockInterface, ToJph(id));
		SoftBodyMotionProperties *mp = lock.Succeeded() ? SoftBody(lock.GetBody()) : nullptr;
		if (mp == nullptr)
		{
			JPC_SetError("Body is not a soft body");
			return false;
		}
		Array<SoftBodyVertex> &out = mp->GetVertices();
		if (uint64(first) + count > out.size())
		{
			JPC_SetError("Vertex range out of bounds");
			return false;
		}
		for (uint32_t i = 0; i < count; ++i)
			if (!(vertices[i].inv_mass >= 0.0f) || !std::isfinite(vertices[i].inv_mass))
			{
				JPC_SetError("Inverse masses must be finite and not negative");
				return false;
			}

		RMat44 transform = world_space ? lock.GetBody().GetInverseCenterOfMassTransform() : RMat44::sIdentity();
		bool mass_changed = false;
		for (uint32_t i = 0; i < count; ++i)
		{
			const JPC_SoftBodyVertexState &in = vertices[i];
			SoftBodyVertex &v = out[first + i];
			// Relative to the center of mass, so small enough for single precision
			v.mPosition = Vec3(transform * ToJph(in.position));
			v.mVelocity = transform.Multiply3x3(ToJph(in.velocity));
			mass_changed |= v.mInvMass != in.inv_mass;
			v.mInvMass = in.inv_mass;
		}
		if (mass_changed)
			mp->CalculateMassAndInertia();
	}
	if (activation == JPC_ACTIVATION_ACTIVATE)
		JPC_BodyInterface_ActivateBody(bi, id);
	return true;
}

bool JPC_BodyInterface_GetSoftBodyProperties(const JPC_BodyInterface *bi, JPC_BodyID id, JPC_SoftBodyProperties *out)
{
	BodyLockRead lock(*bi->mLockInterface, ToJph(id));
	const SoftBodyMotionProperties *mp = lock.Succeeded() ? SoftBody(lock.GetBody()) : nullptr;
	if (mp == nullptr)
		return false;
	*out = { mp->GetNumIterations(), mp->GetPressure(), mp->GetVertexRadius(), mp->GetUpdatePosition(), mp->GetFacesDoubleSided() };
	return true;
}

bool JPC_BodyInterface_SetSoftBodyProperties(JPC_BodyInterface *bi, JPC_BodyID id, const JPC_SoftBodyProperties *properties)
{
	if (!(properties->vertex_radius >= 0.0f))
	{
		JPC_SetError("vertex_radius must not be negative");
		return false;
	}
	BodyLockWrite lock(*bi->mLockInterface, ToJph(id));
	SoftBodyMotionProperties *mp = lock.Succeeded() ? SoftBody(lock.GetBody()) : nullptr;
	if (mp == nullptr)
	{
		JPC_SetError("Body is not a soft body");
		return false;
	}
	mp->SetNumIterations(properties->num_iterations);
	mp->SetPressure(properties->pressure);
	mp->SetVertexRadius(properties->vertex_radius);
	mp->SetUpdatePosition(properties->update_position);
	mp->SetFacesDoubleSided(properties->faces_double_sided);
	return true;
}

float JPC_BodyInterface_GetSoftBodyVolume(const JPC_BodyInterface *bi, JPC_BodyID id)
{
	BodyLockRead lock(*bi->mLockInterface, ToJph(id));
	const SoftBodyMotionProperties *mp = lock.Succeeded() ? SoftBody(lock.GetBody()) : nullptr;
	return mp != nullptr ? mp->GetVolume() : 0.0f;
}

uint32_t JPC_BodyInterface_GetSoftBodyRods(const JPC_BodyInterface *bi, JPC_BodyID id, uint32_t first, JPC_SoftBodyRodState *out_rods, uint32_t capacity)
{
	BodyLockRead lock(*bi->mLockInterface, ToJph(id));
	const SoftBodyMotionProperties *mp = lock.Succeeded() ? SoftBody(lock.GetBody()) : nullptr;
	if (mp == nullptr)
		return 0;
	uint32 num_rods = uint32(mp->GetSettings()->mRodStretchShearConstraints.size());
	RMat44 com = lock.GetBody().GetCenterOfMassTransform();
	Quat rotation = com.GetQuaternion();
	for (uint64 i = first, o = 0; i < num_rods && o < capacity; ++i, ++o)
		out_rods[o] = { ToC(rotation * mp->GetRodRotation(uint(i))), ToC(com.Multiply3x3(mp->GetRodAngularVelocity(uint(i)))) };
	return num_rods;
}

bool JPC_BodyInterface_SkinSoftBody(JPC_BodyInterface *bi, JPC_BodyID id, const JPC_RMat44 *joint_matrices, uint32_t num_joints, bool hard_skin_all,
									JPC_TempAllocator *temp_allocator)
{
	BodyLockWrite lock(*bi->mLockInterface, ToJph(id));
	SoftBodyMotionProperties *mp = lock.Succeeded() ? SoftBody(lock.GetBody()) : nullptr;
	if (mp == nullptr || mp->GetSettings()->mSkinnedConstraints.empty())
	{
		JPC_SetError("Body is not a soft body with skinned vertices");
		return false;
	}
	if (num_joints < SkinJointCount(*mp->GetSettings()))
	{
		JPC_SetError("Fewer joint matrices than the skinning refers to");
		return false;
	}
	// Jolt wants the joints relative to the body's center of mass
	RMat44 com = lock.GetBody().GetCenterOfMassTransform();
	RMat44 to_local = com.InversedRotationTranslation();
	Array<Mat44> joints;
	joints.reserve(num_joints);
	for (uint32_t i = 0; i < num_joints; ++i)
		joints.push_back((to_local * ToJph(joint_matrices[i])).ToMat44());
	mp->SkinVertices(com, joints.data(), num_joints, hard_skin_all, ToJphRef(temp_allocator));
	return true;
}

bool JPC_BodyInterface_SetSoftBodySkinConstraintsEnabled(JPC_BodyInterface *bi, JPC_BodyID id, bool enabled, float max_distance_multiplier)
{
	if (!(max_distance_multiplier >= 0.0f))
	{
		JPC_SetError("max_distance_multiplier must not be negative");
		return false;
	}
	BodyLockWrite lock(*bi->mLockInterface, ToJph(id));
	SoftBodyMotionProperties *mp = lock.Succeeded() ? SoftBody(lock.GetBody()) : nullptr;
	if (mp == nullptr)
	{
		JPC_SetError("Body is not a soft body");
		return false;
	}
	mp->SetEnableSkinConstraints(enabled);
	mp->SetSkinnedMaxDistanceMultiplier(max_distance_multiplier);
	return true;
}
