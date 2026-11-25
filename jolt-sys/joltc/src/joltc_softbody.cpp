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

	// All of these derive from the rest pose, so recomputing the generated constraints changes nothing
	s->CalculateEdgeLengths();
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

JPC_BodyID JPC_BodyInterface_CreateSoftBody(JPC_BodyInterface *bi, const JPC_SoftBodyCreationSettings *settings)
{
	SoftBodyCreationSettings s;
	if (!ToJph(bi, *settings, s))
		return JPC_BODY_ID_INVALID;
	Body *body = ToJph(bi)->CreateSoftBody(s);
	if (body == nullptr)
	{
		JPC_SetError("Out of bodies (max_bodies reached)");
		return JPC_BODY_ID_INVALID;
	}
	return ToC(body->GetID());
}

JPC_BodyID JPC_BodyInterface_CreateAndAddSoftBody(JPC_BodyInterface *bi, const JPC_SoftBodyCreationSettings *settings, JPC_Activation activation)
{
	SoftBodyCreationSettings s;
	if (!ToJph(bi, *settings, s))
		return JPC_BODY_ID_INVALID;
	BodyID id;
	{
		std::unique_lock lock(bi->mOwner->mBodyStateMutex);
		id = ToJph(bi)->CreateAndAddSoftBody(s, ToJph(activation));
		bi->mOwner->BumpStructure();
	}
	if (id.IsInvalid())
		JPC_SetError("Out of bodies (max_bodies reached)");
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
	Mat44 transform = world_space ? lock.GetBody().GetCenterOfMassTransform() : Mat44::sIdentity();
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

		Mat44 transform = world_space ? lock.GetBody().GetInverseCenterOfMassTransform() : Mat44::sIdentity();
		bool mass_changed = false;
		for (uint32_t i = 0; i < count; ++i)
		{
			const JPC_SoftBodyVertexState &in = vertices[i];
			SoftBodyVertex &v = out[first + i];
			v.mPosition = transform * ToJph(in.position);
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
