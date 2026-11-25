// Compound shape editing and inspection, and shape serialization.
#include "joltc_internal.h"

#include <Jolt/Core/StreamIn.h>
#include <Jolt/Core/StreamOut.h>
#include <Jolt/Core/UnorderedMap.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>

#include <cstring>

// ---------------------------------------------------------------------------
// Compounds
// ---------------------------------------------------------------------------

static const CompoundShape *AsCompound(const JPC_Shape *inShape)
{
	const Shape *s = ToJph(inShape);
	return s->GetType() == EShapeType::Compound ? static_cast<const CompoundShape *>(s) : nullptr;
}

static MutableCompoundShape *AsMutableCompound(JPC_Shape *inShape)
{
	Shape *s = ToJph(inShape);
	if (s->GetSubType() == EShapeSubType::MutableCompound)
		return static_cast<MutableCompoundShape *>(s);
	JPC_SetError("Shape is not a mutable compound");
	return nullptr;
}

JPC_Shape *JPC_MutableCompoundShape_Create(const JPC_CompoundSubShape *sub_shapes, uint32_t num_sub_shapes)
{
	MutableCompoundShapeSettings settings;
	settings.SetEmbedded();
	for (uint32_t i = 0; i < num_sub_shapes; ++i)
	{
		const JPC_CompoundSubShape &sub = sub_shapes[i];
		if (sub.shape == nullptr)
		{
			JPC_SetError("Inner shape is null");
			return nullptr;
		}
		settings.AddShape(ToJph(sub.position), ToJph(sub.rotation), ToJph(sub.shape), sub.user_data);
	}
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

uint32_t JPC_CompoundShape_GetNumSubShapes(const JPC_Shape *shape)
{
	const CompoundShape *c = AsCompound(shape);
	return c != nullptr ? c->GetNumSubShapes() : 0;
}

bool JPC_CompoundShape_GetSubShape(const JPC_Shape *shape, uint32_t index, JPC_CompoundSubShape *out)
{
	const CompoundShape *c = AsCompound(shape);
	if (c == nullptr || index >= c->GetNumSubShapes())
		return false;
	const CompoundShape::SubShape &sub = c->GetSubShape(index);
	Quat rotation = sub.GetRotation();
	// Undo SubShape::SetTransform to get back the position of the child's origin
	out->position = ToC(sub.GetPositionCOM() + c->GetCenterOfMass() - rotation * sub.mShape->GetCenterOfMass());
	out->rotation = ToC(rotation);
	sub.mShape->AddRef();
	out->shape = ToC(const_cast<Shape *>(sub.mShape.GetPtr()));
	out->user_data = sub.mUserData;
	return true;
}

bool JPC_CompoundShape_GetSubShapeIndexFromID(const JPC_Shape *shape, JPC_SubShapeID id, uint32_t *out_index, JPC_SubShapeID *out_remainder)
{
	const CompoundShape *c = AsCompound(shape);
	if (c == nullptr)
		return false;
	SubShapeID sub_shape_id, remainder;
	sub_shape_id.SetValue(id);
	// Same as GetSubShapeIndexFromID, which only asserts that the index exists
	if (!c->IsSubShapeIDValid(sub_shape_id))
		return false;
	*out_index = c->GetSubShapeIndexFromID(sub_shape_id, remainder);
	*out_remainder = ToC(remainder);
	return true;
}

// Sub shape ids are a fixed number of bits; Jolt only asserts that a hierarchy fits
static bool FitsSubShapeIDBits(const MutableCompoundShape *inShape)
{
	if (inShape->GetSubShapeIDBitsRecursive() <= SubShapeID::MaxBits)
		return true;
	JPC_SetError("Compound hierarchy is too deep and exceeds the amount of available sub shape ID bits");
	return false;
}

uint32_t JPC_MutableCompoundShape_AddShape(JPC_Shape *shape, JPC_Vec3 position, JPC_Quat rotation, const JPC_Shape *child, uint32_t user_data, uint32_t index)
{
	MutableCompoundShape *c = AsMutableCompound(shape);
	if (c == nullptr)
		return UINT32_MAX;
	if (child == nullptr || child == shape)
	{
		JPC_SetError(child == nullptr ? "Inner shape is null" : "A compound cannot contain itself");
		return UINT32_MAX;
	}
	uint added = c->AddShape(ToJph(position), ToJph(rotation), ToJph(child), user_data, index);
	if (!FitsSubShapeIDBits(c))
	{
		c->RemoveShape(added);
		return UINT32_MAX;
	}
	return added;
}

static bool CheckIndex(const MutableCompoundShape *inShape, uint64 inEnd)
{
	if (inEnd <= inShape->GetNumSubShapes())
		return true;
	JPC_SetError("Sub shape index out of range");
	return false;
}

bool JPC_MutableCompoundShape_RemoveShape(JPC_Shape *shape, uint32_t index)
{
	MutableCompoundShape *c = AsMutableCompound(shape);
	if (c == nullptr || !CheckIndex(c, uint64(index) + 1))
		return false;
	c->RemoveShape(index);
	return true;
}

bool JPC_MutableCompoundShape_ModifyShape(JPC_Shape *shape, uint32_t index, JPC_Vec3 position, JPC_Quat rotation, const JPC_Shape *child)
{
	MutableCompoundShape *c = AsMutableCompound(shape);
	if (c == nullptr || !CheckIndex(c, uint64(index) + 1))
		return false;
	if (child == nullptr)
	{
		c->ModifyShape(index, ToJph(position), ToJph(rotation));
		return true;
	}
	if (child == shape)
	{
		JPC_SetError("A compound cannot contain itself");
		return false;
	}

	// Keep the old child so the change can be undone if the new one does not fit
	const CompoundShape::SubShape &old = c->GetSubShape(index);
	RefConst<Shape> old_shape = old.mShape;
	Quat old_rotation = old.GetRotation();
	Vec3 old_position = old.GetPositionCOM() + c->GetCenterOfMass() - old_rotation * old_shape->GetCenterOfMass();

	c->ModifyShape(index, ToJph(position), ToJph(rotation), ToJph(child));
	if (!FitsSubShapeIDBits(c))
	{
		c->ModifyShape(index, old_position, old_rotation, old_shape);
		return false;
	}
	return true;
}

bool JPC_MutableCompoundShape_ModifyShapes(JPC_Shape *shape, uint32_t start_index, uint32_t count, const JPC_Vec3 *positions, const JPC_Quat *rotations)
{
	MutableCompoundShape *c = AsMutableCompound(shape);
	if (c == nullptr || !CheckIndex(c, uint64(start_index) + count))
		return false;
	if (count == 0)
		return true;
	// The C structs are not padded like Jolt's SIMD types, so convert first
	Array<Vec3> p;
	Array<Quat> r;
	p.reserve(count);
	r.reserve(count);
	for (uint32_t i = 0; i < count; ++i)
	{
		p.push_back(ToJph(positions[i]));
		r.push_back(ToJph(rotations[i]));
	}
	c->ModifyShapes(start_index, count, p.data(), r.data());
	return true;
}

bool JPC_MutableCompoundShape_AdjustCenterOfMass(JPC_Shape *shape)
{
	MutableCompoundShape *c = AsMutableCompound(shape);
	if (c == nullptr)
		return false;
	c->AdjustCenterOfMass();
	return true;
}

bool JPC_BodyInterface_ModifyMutableCompound(JPC_BodyInterface *bi, JPC_BodyID id, JPC_MutableCompoundEditFn fn, void *user_data,
											 bool update_mass_properties, JPC_Activation activation)
{
	// The write lock keeps queries and other body interface calls away from the body (and so its shape) meanwhile
	BodyLockWrite lock(*bi->mLockInterface, ToJph(id));
	if (!lock.Succeeded())
	{
		JPC_SetError("Body does not exist");
		return false;
	}
	Body &body = lock.GetBody();
	const Shape *shape = body.GetShape();
	if (shape->GetSubType() != EShapeSubType::MutableCompound)
	{
		JPC_SetError("Body shape is not a mutable compound");
		return false;
	}

	Vec3 previous_com = shape->GetCenterOfMass();
	MutableCompoundShape *compound = const_cast<MutableCompoundShape *>(static_cast<const MutableCompoundShape *>(shape));
	// Anyone else holding the shape (other bodies, handles, characters) may be reading it right now: edit a copy
	Ref<MutableCompoundShape> copy;
	if (compound->GetRefCount() > 1)
	{
		copy = compound->Clone();
		compound = copy.GetPtr();
	}

	fn(user_data, ToC(compound));

	// A shape without mass (e.g. emptied) would give the body an infinite inverse mass
	bool update_mass = update_mass_properties && compound->GetMassProperties().mMass > 0.0f;

	// The body is already locked, so finish through the interface that doesn't lock again
	const BodyInterface &no_lock = bi->mOwner->mSystem.GetBodyInterfaceNoLock();
	if (copy != nullptr)
		no_lock.SetShape(ToJph(id), copy, update_mass, ToJph(activation));
	else
		no_lock.NotifyShapeChanged(ToJph(id), previous_com, update_mass, ToJph(activation));
	bi->mOwner->BumpStructure();
	return true;
}

// ---------------------------------------------------------------------------
// Serialization
// ---------------------------------------------------------------------------

namespace {

class CallbackStreamOut final : public StreamOut
{
public:
							CallbackStreamOut(JPC_WriteFn inWrite, void *inUserData) : mWrite(inWrite), mUserData(inUserData) { }

	virtual void			WriteBytes(const void *inData, size_t inNumBytes) override	{ if (inNumBytes > 0) mWrite(mUserData, inData, inNumBytes); }
	virtual bool			IsFailed() const override									{ return false; }

private:
	JPC_WriteFn				mWrite;
	void *					mUserData;
};

// Reading past the end fails (and zero fills) instead of reading out of bounds. Like std::istream, EOF is only
// reported after trying to read past the end, which is what Jolt's restore code expects.
class MemoryStreamIn final : public StreamIn
{
public:
							MemoryStreamIn(const void *inData, size_t inSize) : mData(static_cast<const uint8 *>(inData)), mSize(inSize) { }

	virtual void			ReadBytes(void *outData, size_t inNumBytes) override
	{
		if (mFailed || inNumBytes > mSize - mPos)
		{
			mFailed = true;
			memset(outData, 0, inNumBytes);
			return;
		}
		memcpy(outData, mData + mPos, inNumBytes);
		mPos += inNumBytes;
	}

	virtual bool			IsEOF() const override										{ return mFailed; }
	virtual bool			IsFailed() const override									{ return mFailed; }

	bool					AtEnd() const												{ return mPos == mSize; }
	// Peeks at the next byte without consuming it
	bool					PeekByte(uint8 &outByte) const								{ if (mFailed || mPos >= mSize) return false; outByte = mData[mPos]; return true; }

private:
	const uint8 *			mData;
	size_t					mSize;
	size_t					mPos = 0;
	bool					mFailed = false;
};

constexpr uint32 cMagic = 0x5343504a; // "JPCS"
constexpr uint32 cFormatVersion = 1;
constexpr uint32 cNullID = ~uint32(0);
constexpr uint32 cDefaultMaterialID = ~uint32(0) - 1;

using ShapeIDs = UnorderedMap<const Shape *, uint32>;
using MaterialIDs = UnorderedMap<const PhysicsMaterial *, uint32>;

// Same layout as Shape::SaveWithChildren, except that materials are written as their user data: joltc's materials
// are not registered with Jolt's factory, so Jolt's own material serialization would lose them.
void SaveShape(const Shape *inShape, StreamOut &ioStream, ShapeIDs &ioShapes, MaterialIDs &ioMaterials)
{
	if (inShape == nullptr)
	{
		ioStream.Write(cNullID);
		return;
	}
	ShapeIDs::const_iterator known = ioShapes.find(inShape);
	if (known != ioShapes.end())
	{
		ioStream.Write(known->second);
		return;
	}
	uint32 shape_id = uint32(ioShapes.size());
	ioShapes[inShape] = shape_id;
	ioStream.Write(shape_id);
	inShape->SaveBinaryState(ioStream);

	ShapeList children;
	inShape->SaveSubShapeState(children);
	ioStream.Write(uint32(children.size()));
	for (const Shape *child : children)
		SaveShape(child, ioStream, ioShapes, ioMaterials);

	PhysicsMaterialList materials;
	inShape->SaveMaterialState(materials);
	ioStream.Write(uint32(materials.size()));
	for (const PhysicsMaterial *material : materials)
	{
		if (material == nullptr)
			ioStream.Write(cNullID);
		else if (material == PhysicsMaterial::sDefault.GetPtr())
			ioStream.Write(cDefaultMaterialID);
		else
		{
			MaterialIDs::const_iterator known_material = ioMaterials.find(material);
			if (known_material != ioMaterials.end())
				ioStream.Write(known_material->second);
			else
			{
				uint32 material_id = uint32(ioMaterials.size());
				ioMaterials[material] = material_id;
				ioStream.Write(material_id);
				// Every material other than the default one comes from JPC_PhysicsMaterial_Create
				ioStream.Write(static_cast<const UserMaterial *>(material)->mUserData);
			}
		}
	}
}

struct RestoreContext
{
	Array<RefConst<Shape>>	mShapes;
	// Shapes whose children are still being read; referring to one of those would make a cycle
	Array<bool>				mComplete;
	Array<RefConst<PhysicsMaterial>> mMaterials;
};

bool RestoreShape(MemoryStreamIn &ioStream, RestoreContext &ioContext, uint inDepth, RefConst<Shape> &outShape)
{
	// Guards the native stack against corrupt data
	if (inDepth > 256)
	{
		JPC_SetError("Shape hierarchy is too deep");
		return false;
	}

	uint32 shape_id;
	ioStream.Read(shape_id);
	if (ioStream.IsFailed())
	{
		JPC_SetError("Shape data is truncated");
		return false;
	}
	if (shape_id == cNullID)
	{
		outShape = nullptr;
		return true;
	}
	if (shape_id < ioContext.mShapes.size())
	{
		if (!ioContext.mComplete[shape_id])
		{
			JPC_SetError("Shape data contains a cycle");
			return false;
		}
		outShape = ioContext.mShapes[shape_id];
		return true;
	}
	if (shape_id != ioContext.mShapes.size())
	{
		JPC_SetError("Shape data refers to a shape that does not exist");
		return false;
	}

	// Jolt indexes its constructor table with the type byte without checking it
	uint8 sub_type;
	if (!ioStream.PeekByte(sub_type) || sub_type >= NumSubShapeTypes || ShapeFunctions::sGet(EShapeSubType(sub_type)).mConstruct == nullptr
		|| EShapeSubType(sub_type) == EShapeSubType::SoftBody)
	{
		JPC_SetError("Shape data contains an unknown shape type");
		return false;
	}
	Shape::ShapeResult result = Shape::sRestoreFromBinaryState(ioStream);
	if (result.HasError())
	{
		JPC_SetError(result.GetError().c_str());
		return false;
	}
	Ref<Shape> shape = result.Get();
	ioContext.mShapes.push_back(shape);
	ioContext.mComplete.push_back(false);

	uint32 num_children;
	ioStream.Read(num_children);
	if (ioStream.IsFailed())
	{
		JPC_SetError("Shape data is truncated");
		return false;
	}
	ShapeList children;
	for (uint32 i = 0; i < num_children; ++i)
	{
		RefConst<Shape> child;
		if (!RestoreShape(ioStream, ioContext, inDepth + 1, child))
			return false;
		children.push_back(child);
	}
	ShapeList expected_children;
	shape->SaveSubShapeState(expected_children);
	if (children.size() != expected_children.size())
	{
		JPC_SetError("Shape data has the wrong number of child shapes");
		return false;
	}
	shape->RestoreSubShapeState(children.data(), uint(children.size()));

	uint32 num_materials;
	ioStream.Read(num_materials);
	if (ioStream.IsFailed())
	{
		JPC_SetError("Shape data is truncated");
		return false;
	}
	PhysicsMaterialList materials;
	for (uint32 i = 0; i < num_materials; ++i)
	{
		uint32 material_id;
		ioStream.Read(material_id);
		if (ioStream.IsFailed())
		{
			JPC_SetError("Shape data is truncated");
			return false;
		}
		if (material_id == cNullID)
			materials.push_back(nullptr);
		else if (material_id == cDefaultMaterialID)
			materials.push_back(PhysicsMaterial::sDefault);
		else if (material_id < ioContext.mMaterials.size())
			materials.push_back(ioContext.mMaterials[material_id]);
		else if (material_id == ioContext.mMaterials.size())
		{
			uint64 user_data;
			ioStream.Read(user_data);
			if (ioStream.IsFailed())
			{
				JPC_SetError("Shape data is truncated");
				return false;
			}
			ioContext.mMaterials.push_back(new UserMaterial(user_data));
			materials.push_back(ioContext.mMaterials.back());
		}
		else
		{
			JPC_SetError("Shape data refers to a material that does not exist");
			return false;
		}
	}
	// Meshes and height fields take any number of materials; every other shape has a fixed number of slots
	// (one for convex shapes and planes, none for the rest), which Jolt only asserts
	EShapeSubType restored_type = shape->GetSubType();
	if (restored_type != EShapeSubType::Mesh && restored_type != EShapeSubType::HeightField)
	{
		PhysicsMaterialList expected_materials;
		shape->SaveMaterialState(expected_materials);
		if (materials.size() != expected_materials.size())
		{
			JPC_SetError("Shape data has the wrong number of materials");
			return false;
		}
	}
	shape->RestoreMaterialState(materials.data(), uint(materials.size()));

	ioContext.mComplete[shape_id] = true;
	outShape = shape;
	return true;
}

} // namespace

void JPC_Shape_SaveBinary(const JPC_Shape *shape, JPC_WriteFn write, void *user_data)
{
	CallbackStreamOut stream(write, user_data);
	stream.Write(cMagic);
	stream.Write(cFormatVersion);
	stream.Write(uint32(JPH_VERSION_ID));
	ShapeIDs shapes;
	MaterialIDs materials;
	SaveShape(ToJph(shape), stream, shapes, materials);
}

JPC_Shape *JPC_Shape_RestoreBinary(const void *data, size_t size)
{
	MemoryStreamIn stream(data, size);
	uint32 magic = 0, version = 0, jolt_version = 0;
	stream.Read(magic);
	stream.Read(version);
	stream.Read(jolt_version);
	if (stream.IsFailed() || magic != cMagic)
	{
		JPC_SetError("Not shape data saved by joltc");
		return nullptr;
	}
	if (version != cFormatVersion || jolt_version != uint32(JPH_VERSION_ID))
	{
		JPC_SetError("Shape data was saved by a different version of joltc or Jolt");
		return nullptr;
	}

	RestoreContext context;
	RefConst<Shape> shape;
	if (!RestoreShape(stream, context, 0, shape))
		return nullptr;
	if (shape == nullptr)
	{
		JPC_SetError("Shape data contains no shape");
		return nullptr;
	}
	if (!stream.AtEnd())
	{
		JPC_SetError("Shape data has trailing bytes");
		return nullptr;
	}
	shape->AddRef();
	return ToC(const_cast<Shape *>(shape.GetPtr()));
}
