/* joltc: a C API for Jolt Physics.
 *
 * Conventions:
 *  - All vectors are plain floats (no SIMD padding). Matrices are column-major.
 *  - Shapes and constraints are reference counted. Every function that returns a
 *    JPC_Shape* or JPC_Constraint* hands the caller one reference, which must be
 *    released with the matching *_Release function.
 *  - Functions that can fail return NULL / JPC_BODY_ID_INVALID and set a thread
 *    local error message that can be read with JPC_GetLastError().
 */
#ifndef JOLTC_H
#define JOLTC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32) && defined(JOLTC_SHARED)
#  ifdef JOLTC_BUILD
#    define JPC_API __declspec(dllexport)
#  else
#    define JPC_API __declspec(dllimport)
#  endif
#elif defined(JOLTC_SHARED)
#  define JPC_API __attribute__((visibility("default")))
#else
#  define JPC_API
#endif

/* ------------------------------------------------------------------------- */
/* Math                                                                       */
/* ------------------------------------------------------------------------- */

typedef struct JPC_Vec3 { float x, y, z; } JPC_Vec3;
typedef struct JPC_Vec4 { float x, y, z, w; } JPC_Vec4;
typedef struct JPC_Quat { float x, y, z, w; } JPC_Quat;
/* Column-major: columns[3] holds the translation. */
typedef struct JPC_Mat44 { JPC_Vec4 columns[4]; } JPC_Mat44;
typedef struct JPC_AABox { JPC_Vec3 min, max; } JPC_AABox;

/* Double precision vectors and transforms, for world space in double precision builds. */
typedef struct JPC_DVec3 { double x, y, z; } JPC_DVec3;
/* The rotation (and scale) columns in single precision, the translation in double precision. */
typedef struct JPC_DMat44 { JPC_Vec4 columns[3]; JPC_DVec3 translation; } JPC_DMat44;

/* World space positions and transforms. Double precision when joltc (and Jolt) are built with JPC_DOUBLE_PRECISION
 * defined (CMake option DOUBLE_PRECISION), which then must be defined wherever this header is included. */
#ifdef JPC_DOUBLE_PRECISION
typedef JPC_DVec3 JPC_RVec3;
typedef JPC_DMat44 JPC_RMat44;
#else
typedef JPC_Vec3 JPC_RVec3;
typedef JPC_Mat44 JPC_RMat44;
#endif

typedef struct JPC_MassProperties {
    float mass;
    JPC_Mat44 inertia;
} JPC_MassProperties;

/* ------------------------------------------------------------------------- */
/* Identifiers and enums                                                      */
/* ------------------------------------------------------------------------- */

typedef uint32_t JPC_BodyID;
typedef uint32_t JPC_SubShapeID;
typedef uint16_t JPC_ObjectLayer;
typedef uint8_t JPC_BroadPhaseLayer;

#define JPC_BODY_ID_INVALID 0xffffffffu
#define JPC_SUB_SHAPE_ID_EMPTY 0xffffffffu
#define JPC_DEFAULT_CONVEX_RADIUS 0.05f

typedef enum JPC_MotionType {
    JPC_MOTION_TYPE_STATIC = 0,
    JPC_MOTION_TYPE_KINEMATIC = 1,
    JPC_MOTION_TYPE_DYNAMIC = 2,
} JPC_MotionType;

typedef enum JPC_MotionQuality {
    JPC_MOTION_QUALITY_DISCRETE = 0,
    JPC_MOTION_QUALITY_LINEAR_CAST = 1,
} JPC_MotionQuality;

typedef enum JPC_Activation {
    JPC_ACTIVATION_ACTIVATE = 0,
    JPC_ACTIVATION_DONT_ACTIVATE = 1,
} JPC_Activation;

typedef enum JPC_AllowedDOFs {
    JPC_ALLOWED_DOFS_NONE = 0x00,
    JPC_ALLOWED_DOFS_TRANSLATION_X = 0x01,
    JPC_ALLOWED_DOFS_TRANSLATION_Y = 0x02,
    JPC_ALLOWED_DOFS_TRANSLATION_Z = 0x04,
    JPC_ALLOWED_DOFS_ROTATION_X = 0x08,
    JPC_ALLOWED_DOFS_ROTATION_Y = 0x10,
    JPC_ALLOWED_DOFS_ROTATION_Z = 0x20,
    JPC_ALLOWED_DOFS_PLANE_2D = 0x01 | 0x02 | 0x20,
    JPC_ALLOWED_DOFS_ALL = 0x3f,
} JPC_AllowedDOFs;

typedef enum JPC_OverrideMassProperties {
    JPC_OVERRIDE_MASS_PROPERTIES_CALCULATE_MASS_AND_INERTIA = 0,
    JPC_OVERRIDE_MASS_PROPERTIES_CALCULATE_INERTIA = 1,
    JPC_OVERRIDE_MASS_PROPERTIES_MASS_AND_INERTIA_PROVIDED = 2,
} JPC_OverrideMassProperties;

typedef enum JPC_PhysicsUpdateError {
    JPC_PHYSICS_UPDATE_ERROR_NONE = 0,
    JPC_PHYSICS_UPDATE_ERROR_MANIFOLD_CACHE_FULL = 1 << 0,
    JPC_PHYSICS_UPDATE_ERROR_BODY_PAIR_CACHE_FULL = 1 << 1,
    JPC_PHYSICS_UPDATE_ERROR_CONTACT_CONSTRAINTS_FULL = 1 << 2,
} JPC_PhysicsUpdateError;

typedef enum JPC_ShapeType {
    JPC_SHAPE_TYPE_CONVEX = 0,
    JPC_SHAPE_TYPE_COMPOUND = 1,
    JPC_SHAPE_TYPE_DECORATED = 2,
    JPC_SHAPE_TYPE_MESH = 3,
    JPC_SHAPE_TYPE_HEIGHT_FIELD = 4,
    JPC_SHAPE_TYPE_SOFT_BODY = 5,
    JPC_SHAPE_TYPE_USER1 = 6,
    JPC_SHAPE_TYPE_USER2 = 7,
    JPC_SHAPE_TYPE_USER3 = 8,
    JPC_SHAPE_TYPE_USER4 = 9,
} JPC_ShapeType;

typedef enum JPC_ShapeSubType {
    JPC_SHAPE_SUB_TYPE_SPHERE = 0,
    JPC_SHAPE_SUB_TYPE_BOX = 1,
    JPC_SHAPE_SUB_TYPE_TRIANGLE = 2,
    JPC_SHAPE_SUB_TYPE_CAPSULE = 3,
    JPC_SHAPE_SUB_TYPE_TAPERED_CAPSULE = 4,
    JPC_SHAPE_SUB_TYPE_CYLINDER = 5,
    JPC_SHAPE_SUB_TYPE_CONVEX_HULL = 6,
    JPC_SHAPE_SUB_TYPE_STATIC_COMPOUND = 7,
    JPC_SHAPE_SUB_TYPE_MUTABLE_COMPOUND = 8,
    JPC_SHAPE_SUB_TYPE_ROTATED_TRANSLATED = 9,
    JPC_SHAPE_SUB_TYPE_SCALED = 10,
    JPC_SHAPE_SUB_TYPE_OFFSET_CENTER_OF_MASS = 11,
    JPC_SHAPE_SUB_TYPE_MESH = 12,
    JPC_SHAPE_SUB_TYPE_HEIGHT_FIELD = 13,
    JPC_SHAPE_SUB_TYPE_SOFT_BODY = 14,
    JPC_SHAPE_SUB_TYPE_PLANE = 31,
    JPC_SHAPE_SUB_TYPE_TAPERED_CYLINDER = 32,
    JPC_SHAPE_SUB_TYPE_EMPTY = 33,
} JPC_ShapeSubType;

typedef enum JPC_ValidateResult {
    JPC_VALIDATE_RESULT_ACCEPT_ALL_CONTACTS = 0,
    JPC_VALIDATE_RESULT_ACCEPT_CONTACT = 1,
    JPC_VALIDATE_RESULT_REJECT_CONTACT = 2,
    JPC_VALIDATE_RESULT_REJECT_ALL_CONTACTS = 3,
} JPC_ValidateResult;

typedef enum JPC_BackFaceMode {
    JPC_BACK_FACE_MODE_IGNORE = 0,
    JPC_BACK_FACE_MODE_COLLIDE = 1,
} JPC_BackFaceMode;

typedef enum JPC_ConstraintSpace {
    JPC_CONSTRAINT_SPACE_LOCAL_TO_BODY_COM = 0,
    JPC_CONSTRAINT_SPACE_WORLD_SPACE = 1,
} JPC_ConstraintSpace;

typedef enum JPC_SpringMode {
    JPC_SPRING_MODE_FREQUENCY_AND_DAMPING = 0,
    JPC_SPRING_MODE_STIFFNESS_AND_DAMPING = 1,
} JPC_SpringMode;

typedef enum JPC_SwingType {
    JPC_SWING_TYPE_CONE = 0,
    JPC_SWING_TYPE_PYRAMID = 1,
} JPC_SwingType;

typedef enum JPC_SixDOFAxis {
    JPC_SIX_DOF_AXIS_TRANSLATION_X = 0,
    JPC_SIX_DOF_AXIS_TRANSLATION_Y = 1,
    JPC_SIX_DOF_AXIS_TRANSLATION_Z = 2,
    JPC_SIX_DOF_AXIS_ROTATION_X = 3,
    JPC_SIX_DOF_AXIS_ROTATION_Y = 4,
    JPC_SIX_DOF_AXIS_ROTATION_Z = 5,
    JPC_SIX_DOF_AXIS_NUM = 6,
} JPC_SixDOFAxis;

typedef enum JPC_ActiveEdgeMode {
    JPC_ACTIVE_EDGE_MODE_COLLIDE_ONLY_WITH_ACTIVE = 0,
    JPC_ACTIVE_EDGE_MODE_COLLIDE_WITH_ALL = 1,
} JPC_ActiveEdgeMode;

typedef enum JPC_StateParts {
    JPC_STATE_PARTS_GLOBAL = 1,
    JPC_STATE_PARTS_BODIES = 2,
    JPC_STATE_PARTS_CONTACTS = 4,
    JPC_STATE_PARTS_CONSTRAINTS = 8,
    JPC_STATE_PARTS_ALL = 15,
} JPC_StateParts;

typedef enum JPC_PathRotationConstraintType {
    JPC_PATH_ROTATION_FREE = 0,
    JPC_PATH_ROTATION_CONSTRAIN_AROUND_TANGENT = 1,
    JPC_PATH_ROTATION_CONSTRAIN_AROUND_NORMAL = 2,
    JPC_PATH_ROTATION_CONSTRAIN_AROUND_BINORMAL = 3,
    JPC_PATH_ROTATION_CONSTRAIN_TO_PATH = 4,
    JPC_PATH_ROTATION_FULLY_CONSTRAINED = 5,
} JPC_PathRotationConstraintType;

typedef enum JPC_ConstraintType {
    JPC_CONSTRAINT_TYPE_NONE = 0,
    JPC_CONSTRAINT_TYPE_FIXED = 1,
    JPC_CONSTRAINT_TYPE_POINT = 2,
    JPC_CONSTRAINT_TYPE_DISTANCE = 3,
    JPC_CONSTRAINT_TYPE_HINGE = 4,
    JPC_CONSTRAINT_TYPE_SLIDER = 5,
    JPC_CONSTRAINT_TYPE_CONE = 6,
    JPC_CONSTRAINT_TYPE_SWING_TWIST = 7,
    JPC_CONSTRAINT_TYPE_SIX_DOF = 8,
    JPC_CONSTRAINT_TYPE_GEAR = 9,
    JPC_CONSTRAINT_TYPE_RACK_AND_PINION = 10,
    JPC_CONSTRAINT_TYPE_PULLEY = 11,
    JPC_CONSTRAINT_TYPE_PATH = 12,
    JPC_CONSTRAINT_TYPE_VEHICLE = 13,
    JPC_CONSTRAINT_TYPE_OTHER = 14,
} JPC_ConstraintType;

typedef enum JPC_TransmissionMode {
    JPC_TRANSMISSION_MODE_AUTO = 0,
    JPC_TRANSMISSION_MODE_MANUAL = 1,
} JPC_TransmissionMode;

typedef enum JPC_GroundState {
    JPC_GROUND_STATE_ON_GROUND = 0,
    JPC_GROUND_STATE_ON_STEEP_GROUND = 1,
    JPC_GROUND_STATE_NOT_SUPPORTED = 2,
    JPC_GROUND_STATE_IN_AIR = 3,
} JPC_GroundState;

typedef enum JPC_VehicleKind {
    JPC_VEHICLE_KIND_WHEELED = 0,
    JPC_VEHICLE_KIND_MOTORCYCLE = 1,
    JPC_VEHICLE_KIND_TRACKED = 2,
} JPC_VehicleKind;

typedef enum JPC_MotorState {
    JPC_MOTOR_STATE_OFF = 0,
    JPC_MOTOR_STATE_VELOCITY = 1,
    JPC_MOTOR_STATE_POSITION = 2,
    JPC_MOTOR_STATE_POSITION_AND_VELOCITY = 3,
} JPC_MotorState;

/* ------------------------------------------------------------------------- */
/* Opaque handles                                                             */
/* ------------------------------------------------------------------------- */

typedef struct JPC_TempAllocator JPC_TempAllocator;
typedef struct JPC_JobSystem JPC_JobSystem;
typedef struct JPC_PhysicsSystem JPC_PhysicsSystem;
typedef struct JPC_BodyInterface JPC_BodyInterface;
typedef struct JPC_Body JPC_Body;
typedef struct JPC_Shape JPC_Shape;
typedef struct JPC_Constraint JPC_Constraint;
typedef struct JPC_PhysicsMaterial JPC_PhysicsMaterial;
typedef struct JPC_GroupFilter JPC_GroupFilter;
typedef struct JPC_StepListener JPC_StepListener;
typedef struct JPC_CharacterVirtual JPC_CharacterVirtual;
typedef struct JPC_CharacterVsCharacterCollision JPC_CharacterVsCharacterCollision;
typedef struct JPC_Character JPC_Character;
typedef struct JPC_PhysicsSnapshot JPC_PhysicsSnapshot;
typedef struct JPC_PathConstraintPath JPC_PathConstraintPath;
typedef struct JPC_VehicleCollisionTester JPC_VehicleCollisionTester;
typedef struct JPC_Skeleton JPC_Skeleton;
typedef struct JPC_RagdollSettings JPC_RagdollSettings;
typedef struct JPC_Ragdoll JPC_Ragdoll;
typedef struct JPC_SkeletalAnimation JPC_SkeletalAnimation;
typedef struct JPC_SkeletonMapper JPC_SkeletonMapper;
typedef struct JPC_SoftBodySharedSettings JPC_SoftBodySharedSettings;
typedef struct JPC_DebugRenderer JPC_DebugRenderer;

/* ------------------------------------------------------------------------- */
/* Global setup                                                               */
/* ------------------------------------------------------------------------- */

typedef void (*JPC_TraceFn)(const char *message);
/* Return true to trigger a breakpoint. Only called in builds with JPH_ENABLE_ASSERTS. */
typedef bool (*JPC_AssertFailedFn)(const char *expression, const char *message, const char *file, uint32_t line);

/* Registers the default allocator, creates the factory and registers all types.
 * Thread-safe and idempotent; only the first call does work. */
JPC_API void JPC_Init(void);
/* Unregisters types and destroys the factory. Every Jolt object must be destroyed first. */
JPC_API void JPC_Shutdown(void);

JPC_API void JPC_SetTraceHandler(JPC_TraceFn handler);
JPC_API void JPC_SetAssertFailedHandler(JPC_AssertFailedFn handler);

/* Whether this build uses double precision world space types; must match JPC_DOUBLE_PRECISION of the includer. */
JPC_API bool JPC_IsDoublePrecision(void);

/* Message from the last failing call on this thread, or "" if none. */
JPC_API const char *JPC_GetLastError(void);

/* ------------------------------------------------------------------------- */
/* Temp allocator and job systems                                             */
/* ------------------------------------------------------------------------- */

JPC_API JPC_TempAllocator *JPC_TempAllocator_Create(uint32_t size_in_bytes);
JPC_API void JPC_TempAllocator_Destroy(JPC_TempAllocator *allocator);

/* num_threads < 0 uses hardware concurrency - 1. max_jobs is raised to at least JPC_MAX_PHYSICS_JOBS (fewer deadlocks
 * the update) and rounded up to a power of two; max_barriers is raised to at least 1.
 * At most max_barriers physics systems can update at once on one job system; further updates wait for a free slot. */
JPC_API JPC_JobSystem *JPC_JobSystemThreadPool_Create(uint32_t max_jobs, uint32_t max_barriers, int32_t num_threads);
JPC_API JPC_JobSystem *JPC_JobSystemSingleThreaded_Create(uint32_t max_jobs);
JPC_API void JPC_JobSystem_Destroy(JPC_JobSystem *job_system);

#define JPC_MAX_PHYSICS_JOBS 2048
#define JPC_MAX_PHYSICS_BARRIERS 8

/* ------------------------------------------------------------------------- */
/* Shapes                                                                     */
/* ------------------------------------------------------------------------- */

typedef struct JPC_CompoundSubShape {
    JPC_Vec3 position;
    JPC_Quat rotation;
    const JPC_Shape *shape;
    uint32_t user_data;
} JPC_CompoundSubShape;

JPC_API JPC_Shape *JPC_SphereShape_Create(float radius);
JPC_API JPC_Shape *JPC_BoxShape_Create(JPC_Vec3 half_extent, float convex_radius);
JPC_API JPC_Shape *JPC_CapsuleShape_Create(float half_height_of_cylinder, float radius);
JPC_API JPC_Shape *JPC_TaperedCapsuleShape_Create(float half_height, float top_radius, float bottom_radius);
JPC_API JPC_Shape *JPC_CylinderShape_Create(float half_height, float radius, float convex_radius);
/* A single triangle; ray casts hit its front (counter-clockwise) side and convex_radius rounds it. It has no volume or
 * mass, so use it for static or kinematic bodies. */
JPC_API JPC_Shape *JPC_TriangleShape_Create(JPC_Vec3 v1, JPC_Vec3 v2, JPC_Vec3 v3, float convex_radius);
JPC_API JPC_Shape *JPC_TaperedCylinderShape_Create(float half_height, float top_radius, float bottom_radius, float convex_radius);
JPC_API JPC_Shape *JPC_ConvexHullShape_Create(const JPC_Vec3 *points, uint32_t num_points, float max_convex_radius);
/* indices holds 3 * num_triangles vertex indices. */
JPC_API JPC_Shape *JPC_MeshShape_Create(const JPC_Vec3 *vertices, uint32_t num_vertices, const uint32_t *indices, uint32_t num_triangles);
/* samples holds sample_count * sample_count heights. */
JPC_API JPC_Shape *JPC_HeightFieldShape_Create(const float *samples, JPC_Vec3 offset, JPC_Vec3 scale, uint32_t sample_count);
/* material_indices holds (sample_count - 1)^2 entries indexing materials (at most 256). */
JPC_API JPC_Shape *JPC_HeightFieldShape_CreateWithMaterials(const float *samples, JPC_Vec3 offset, JPC_Vec3 scale, uint32_t sample_count,
                                                            const uint8_t *material_indices, const JPC_PhysicsMaterial *const *materials, uint32_t num_materials);
/* Infinite-ish plane: normal must be normalized, distance is the plane constant. */
JPC_API JPC_Shape *JPC_PlaneShape_Create(JPC_Vec3 normal, float distance, float half_extent);
JPC_API JPC_Shape *JPC_EmptyShape_Create(JPC_Vec3 center_of_mass);
JPC_API JPC_Shape *JPC_StaticCompoundShape_Create(const JPC_CompoundSubShape *sub_shapes, uint32_t num_sub_shapes);
JPC_API JPC_Shape *JPC_RotatedTranslatedShape_Create(JPC_Vec3 position, JPC_Quat rotation, const JPC_Shape *inner);
JPC_API JPC_Shape *JPC_ScaledShape_Create(const JPC_Shape *inner, JPC_Vec3 scale);
JPC_API JPC_Shape *JPC_OffsetCenterOfMassShape_Create(JPC_Vec3 offset, const JPC_Shape *inner);

JPC_API void JPC_Shape_AddRef(const JPC_Shape *shape);
JPC_API void JPC_Shape_Release(const JPC_Shape *shape);
JPC_API uint32_t JPC_Shape_GetRefCount(const JPC_Shape *shape);
JPC_API JPC_ShapeType JPC_Shape_GetType(const JPC_Shape *shape);
JPC_API JPC_ShapeSubType JPC_Shape_GetSubType(const JPC_Shape *shape);
JPC_API uint64_t JPC_Shape_GetUserData(const JPC_Shape *shape);
JPC_API void JPC_Shape_SetUserData(JPC_Shape *shape, uint64_t user_data);
JPC_API JPC_AABox JPC_Shape_GetLocalBounds(const JPC_Shape *shape);
JPC_API JPC_Vec3 JPC_Shape_GetCenterOfMass(const JPC_Shape *shape);
JPC_API float JPC_Shape_GetInnerRadius(const JPC_Shape *shape);
JPC_API float JPC_Shape_GetVolume(const JPC_Shape *shape);
JPC_API JPC_MassProperties JPC_Shape_GetMassProperties(const JPC_Shape *shape);

/* ------------------------------------------------------------------------- */
/* Compound shapes                                                            */
/* ------------------------------------------------------------------------- */

/* A compound whose children can be added, removed and moved after creation. */
JPC_API JPC_Shape *JPC_MutableCompoundShape_Create(const JPC_CompoundSubShape *sub_shapes, uint32_t num_sub_shapes);

/* Number of children of a static or mutable compound, 0 for other shapes. */
JPC_API uint32_t JPC_CompoundShape_GetNumSubShapes(const JPC_Shape *shape);
/* out->position and out->rotation place the child's origin in the compound, as passed when it was added or last
 * moved. out->shape is a new reference. Returns false for a non compound or an out of range index. */
JPC_API bool JPC_CompoundShape_GetSubShape(const JPC_Shape *shape, uint32_t index, JPC_CompoundSubShape *out_sub_shape);
/* Splits a sub shape id from a hit on this compound into the child index and the id within that child.
 * Returns false for a non compound or an id that does not name a child. */
JPC_API bool JPC_CompoundShape_GetSubShapeIndexFromID(const JPC_Shape *shape, JPC_SubShapeID id, uint32_t *out_index, JPC_SubShapeID *out_remainder);

/* Editing a mutable compound. Only edit a shape nothing else uses at the same time: one held by a single owner,
 * or the one handed to a JPC_BodyInterface_ModifyMutableCompound callback. Editing changes the meaning of sub
 * shape ids from earlier hits. Each function fails (false / UINT32_MAX, with an error) for a shape that is not a
 * mutable compound or an index out of range. */
/* Inserts the child before index (index >= count appends) and returns its index. */
JPC_API uint32_t JPC_MutableCompoundShape_AddShape(JPC_Shape *shape, JPC_Vec3 position, JPC_Quat rotation, const JPC_Shape *child, uint32_t user_data,
                                                   uint32_t index);
JPC_API bool JPC_MutableCompoundShape_RemoveShape(JPC_Shape *shape, uint32_t index);
/* child may be NULL to keep the current child. */
JPC_API bool JPC_MutableCompoundShape_ModifyShape(JPC_Shape *shape, uint32_t index, JPC_Vec3 position, JPC_Quat rotation, const JPC_Shape *child);
/* Moves count children starting at start_index, faster than one at a time. shapes may be NULL to keep the children,
 * or hold count shapes to swap them in too. */
JPC_API bool JPC_MutableCompoundShape_ModifyShapes(JPC_Shape *shape, uint32_t start_index, uint32_t count, const JPC_Vec3 *positions,
                                                   const JPC_Quat *rotations, const JPC_Shape *const *shapes);
/* Moves the compound's center of mass to the children's combined center of mass. Child positions are unchanged; a
 * body edited through JPC_BodyInterface_ModifyMutableCompound keeps its world position. */
JPC_API bool JPC_MutableCompoundShape_AdjustCenterOfMass(JPC_Shape *shape);

typedef void (*JPC_MutableCompoundEditFn)(void *user_data, JPC_Shape *compound);

/* Locks the body and calls fn with its mutable compound shape to edit with the functions above, then updates the
 * body's bounds, center of mass, contact cache and (if update_mass_properties and the shape still has mass) mass
 * and inertia. If anything else
 * holds a reference to the shape the body first gets its own copy, so other users never see the change.
 * fn runs with the body locked: it must not use a body interface. Returns false (without calling fn) if the body
 * does not exist or its shape is not a mutable compound. */
JPC_API bool JPC_BodyInterface_ModifyMutableCompound(JPC_BodyInterface *bi, JPC_BodyID id, JPC_MutableCompoundEditFn fn, void *user_data,
                                                     bool update_mass_properties, JPC_Activation activation);

/* ------------------------------------------------------------------------- */
/* Shape serialization                                                        */
/*                                                                            */
/* Saves a shape with all its children in Jolt's binary format, so baked      */
/* meshes and compounds load without being rebuilt. Shared children are      */
/* stored once. Materials are stored by user data and come back as new        */
/* materials with the same user data. The data only loads with the same Jolt  */
/* version and platform (endianness, float format).                           */
/* ------------------------------------------------------------------------- */

typedef void (*JPC_WriteFn)(void *user_data, const void *data, size_t size);

/* Calls write any number of times with consecutive pieces of the data. */
JPC_API void JPC_Shape_SaveBinary(const JPC_Shape *shape, JPC_WriteFn write, void *user_data);
/* Returns NULL with an error for data with a bad header, a different Jolt version, unknown shape types or
 * truncation. Jolt does not validate the contents beyond that, so only load data this library saved. */
JPC_API JPC_Shape *JPC_Shape_RestoreBinary(const void *data, size_t size);

/* ------------------------------------------------------------------------- */
/* Materials                                                                  */
/* ------------------------------------------------------------------------- */

/* A material carries user data so contacts and hits can report what surface was touched.
 * Reference counted. Shapes that use a material keep it alive. */
JPC_API JPC_PhysicsMaterial *JPC_PhysicsMaterial_Create(uint64_t user_data);
JPC_API void JPC_PhysicsMaterial_AddRef(const JPC_PhysicsMaterial *material);
JPC_API void JPC_PhysicsMaterial_Release(const JPC_PhysicsMaterial *material);
JPC_API uint64_t JPC_PhysicsMaterial_GetUserData(const JPC_PhysicsMaterial *material);

/* Only valid for convex shapes; NULL restores the default material. Must be called before the shape is shared. */
JPC_API void JPC_ConvexShape_SetMaterial(JPC_Shape *shape, const JPC_PhysicsMaterial *material);
/* Like JPC_MeshShape_Create, with material_indices[num_triangles] indexing into materials[num_materials]. */
JPC_API JPC_Shape *JPC_MeshShape_CreateWithMaterials(const JPC_Vec3 *vertices, uint32_t num_vertices, const uint32_t *indices, uint32_t num_triangles,
                                                     const uint32_t *material_indices, const JPC_PhysicsMaterial *const *materials, uint32_t num_materials);
/* Material of a sub shape, or NULL for the default material. sub_shape_id must come from a hit or contact on this exact shape;
 * anything else reads out of bounds. Borrowed. */
JPC_API const JPC_PhysicsMaterial *JPC_Shape_GetMaterialUnchecked(const JPC_Shape *shape, JPC_SubShapeID sub_shape_id);

/* ------------------------------------------------------------------------- */
/* Collision groups                                                           */
/* ------------------------------------------------------------------------- */

/* A table of which sub groups inside one group collide (e.g. the parts of a ragdoll).
 * Bodies in different groups always collide; bodies in the same sub group never do.
 * Reference counted; configure it before handing it to bodies. Out of range sub groups always collide. */
JPC_API JPC_GroupFilter *JPC_GroupFilterTable_Create(uint32_t num_sub_groups);
JPC_API void JPC_GroupFilterTable_DisableCollision(JPC_GroupFilter *filter, uint32_t sub_group1, uint32_t sub_group2);
JPC_API void JPC_GroupFilterTable_EnableCollision(JPC_GroupFilter *filter, uint32_t sub_group1, uint32_t sub_group2);
JPC_API bool JPC_GroupFilterTable_IsCollisionEnabled(const JPC_GroupFilter *filter, uint32_t sub_group1, uint32_t sub_group2);
JPC_API void JPC_GroupFilter_AddRef(const JPC_GroupFilter *filter);
JPC_API void JPC_GroupFilter_Release(const JPC_GroupFilter *filter);

#define JPC_COLLISION_GROUP_INVALID 0xffffffffu

typedef struct JPC_CollisionGroup {
    /* Borrowed when passed in; the body takes its own reference. NULL = no filtering. */
    const JPC_GroupFilter *group_filter;
    uint32_t group_id;
    uint32_t sub_group_id;
} JPC_CollisionGroup;

/* Only valid for convex shapes (JPC_SHAPE_TYPE_CONVEX). Density is in kg/m^3. */
JPC_API void JPC_ConvexShape_SetDensity(JPC_Shape *shape, float density);
JPC_API float JPC_ConvexShape_GetDensity(const JPC_Shape *shape);

/* ------------------------------------------------------------------------- */
/* Physics system                                                             */
/* ------------------------------------------------------------------------- */

typedef struct JPC_ObjectLayerPair {
    JPC_ObjectLayer layer1;
    JPC_ObjectLayer layer2;
} JPC_ObjectLayerPair;

typedef struct JPC_PhysicsSystemSettings {
    /* At most 8388608 (the BodyID index range). */
    uint32_t max_bodies;
    uint32_t num_body_mutexes; /* 0 = automatic */
    uint32_t max_body_pairs;
    uint32_t max_contact_constraints;

    uint32_t num_object_layers;
    uint32_t num_broad_phase_layers;
    /* num_object_layers entries mapping each object layer to a broad phase layer. */
    const JPC_BroadPhaseLayer *object_to_broad_phase_layer;
    /* Pairs of object layers that collide (symmetric). Every other pair is ignored. */
    const JPC_ObjectLayerPair *colliding_pairs;
    uint32_t num_colliding_pairs;
} JPC_PhysicsSystemSettings;

typedef struct JPC_PhysicsSettings {
    int32_t max_in_flight_body_pairs;
    int32_t step_listeners_batch_size;
    int32_t step_listener_batches_per_job;
    float baumgarte;
    float speculative_contact_distance;
    float penetration_slop;
    float linear_cast_threshold;
    float linear_cast_max_penetration;
    float manifold_tolerance;
    float max_penetration_distance;
    float body_pair_cache_max_delta_position_sq;
    float body_pair_cache_cos_max_delta_rotation_div2;
    float contact_normal_cos_max_delta_rotation;
    float contact_point_preserve_lambda_max_dist_sq;
    uint32_t num_velocity_steps;
    uint32_t num_position_steps;
    float min_velocity_for_restitution;
    float time_before_sleep;
    float point_velocity_sleep_threshold;
    bool constraint_warm_start;
    bool use_body_pair_contact_cache;
    bool use_manifold_reduction;
    bool use_large_island_splitter;
    bool allow_sleeping;
    bool check_active_edges;
} JPC_PhysicsSettings;

JPC_API JPC_PhysicsSystem *JPC_PhysicsSystem_Create(const JPC_PhysicsSystemSettings *settings);
JPC_API void JPC_PhysicsSystem_Destroy(JPC_PhysicsSystem *system);

JPC_API JPC_PhysicsUpdateError JPC_PhysicsSystem_Update(JPC_PhysicsSystem *system, float delta_time, int32_t collision_steps,
                                                       JPC_TempAllocator *temp_allocator, JPC_JobSystem *job_system);
JPC_API void JPC_PhysicsSystem_OptimizeBroadPhase(JPC_PhysicsSystem *system);

JPC_API void JPC_PhysicsSystem_SetGravity(JPC_PhysicsSystem *system, JPC_Vec3 gravity);
JPC_API JPC_Vec3 JPC_PhysicsSystem_GetGravity(const JPC_PhysicsSystem *system);
JPC_API void JPC_PhysicsSystem_GetPhysicsSettings(const JPC_PhysicsSystem *system, JPC_PhysicsSettings *out_settings);
/* Returns false (and changes nothing) for values Jolt cannot run with: batch sizes below 1,
 * max_in_flight_body_pairs below 256 or num_velocity_steps below 1. */
JPC_API bool JPC_PhysicsSystem_SetPhysicsSettings(JPC_PhysicsSystem *system, const JPC_PhysicsSettings *settings);

JPC_API uint32_t JPC_PhysicsSystem_GetNumBodies(const JPC_PhysicsSystem *system);
JPC_API uint32_t JPC_PhysicsSystem_GetNumActiveBodies(const JPC_PhysicsSystem *system);
JPC_API uint32_t JPC_PhysicsSystem_GetMaxBodies(const JPC_PhysicsSystem *system);
/* Writes up to capacity ids and returns the total number of bodies (which may exceed capacity). */
JPC_API uint32_t JPC_PhysicsSystem_GetBodies(const JPC_PhysicsSystem *system, JPC_BodyID *out_ids, uint32_t capacity);
JPC_API uint32_t JPC_PhysicsSystem_GetActiveBodies(const JPC_PhysicsSystem *system, JPC_BodyID *out_ids, uint32_t capacity);

/* The locking interface is safe to use from multiple threads (but not during Update).
 * The no-lock interface skips body locks and must only be used when nothing else touches the bodies. */
JPC_API JPC_BodyInterface *JPC_PhysicsSystem_GetBodyInterface(JPC_PhysicsSystem *system);
JPC_API JPC_BodyInterface *JPC_PhysicsSystem_GetBodyInterfaceNoLock(JPC_PhysicsSystem *system);

JPC_API void JPC_PhysicsSystem_AddConstraint(JPC_PhysicsSystem *system, JPC_Constraint *constraint);
JPC_API void JPC_PhysicsSystem_RemoveConstraint(JPC_PhysicsSystem *system, JPC_Constraint *constraint);

/* ------------------------------------------------------------------------- */
/* Listeners                                                                  */
/*                                                                            */
/* Callbacks are invoked from Jolt worker threads during Update, possibly     */
/* concurrently. Bodies passed to callbacks must not be locked or modified    */
/* through a body interface from inside the callback.                         */
/* ------------------------------------------------------------------------- */

typedef struct JPC_CollideShapeResult {
    JPC_Vec3 contact_point_on1;
    JPC_Vec3 contact_point_on2;
    JPC_Vec3 penetration_axis;
    float penetration_depth;
    JPC_SubShapeID sub_shape_id1;
    JPC_SubShapeID sub_shape_id2;
    JPC_BodyID body_id2;
} JPC_CollideShapeResult;

typedef struct JPC_ContactManifold {
    JPC_RVec3 base_offset;
    JPC_Vec3 world_space_normal;
    float penetration_depth;
    JPC_SubShapeID sub_shape_id1;
    JPC_SubShapeID sub_shape_id2;
    uint32_t num_points;
    /* num_points entries each, relative to base_offset. Only valid during the callback. */
    const JPC_Vec3 *relative_points_on1;
    const JPC_Vec3 *relative_points_on2;
    /* Materials at the two sub shapes, NULL for the default material. Borrowed. */
    const JPC_PhysicsMaterial *material1;
    const JPC_PhysicsMaterial *material2;
} JPC_ContactManifold;

typedef struct JPC_ContactSettings {
    float combined_friction;
    float combined_restitution;
    float inv_mass_scale1;
    float inv_inertia_scale1;
    float inv_mass_scale2;
    float inv_inertia_scale2;
    bool is_sensor;
    JPC_Vec3 relative_linear_surface_velocity;
    JPC_Vec3 relative_angular_surface_velocity;
} JPC_ContactSettings;

typedef struct JPC_SubShapeIDPair {
    JPC_BodyID body1_id;
    JPC_SubShapeID sub_shape_id1;
    JPC_BodyID body2_id;
    JPC_SubShapeID sub_shape_id2;
} JPC_SubShapeIDPair;

/* Any callback may be NULL. */
typedef struct JPC_ContactListenerFns {
    JPC_ValidateResult (*on_contact_validate)(void *user_data, const JPC_Body *body1, const JPC_Body *body2,
                                              const JPC_RVec3 *base_offset, const JPC_CollideShapeResult *result);
    void (*on_contact_added)(void *user_data, const JPC_Body *body1, const JPC_Body *body2,
                             const JPC_ContactManifold *manifold, JPC_ContactSettings *settings);
    void (*on_contact_persisted)(void *user_data, const JPC_Body *body1, const JPC_Body *body2,
                                 const JPC_ContactManifold *manifold, JPC_ContactSettings *settings);
    void (*on_contact_removed)(void *user_data, const JPC_SubShapeIDPair *pair);
} JPC_ContactListenerFns;

typedef struct JPC_BodyActivationListenerFns {
    void (*on_body_activated)(void *user_data, JPC_BodyID body_id, uint64_t body_user_data);
    void (*on_body_deactivated)(void *user_data, JPC_BodyID body_id, uint64_t body_user_data);
} JPC_BodyActivationListenerFns;

/* Estimated impulses and velocities of a new contact, e.g. to play an impact sound of the right strength. */
typedef struct JPC_CollisionEstimation {
    JPC_Vec3 linear_velocity1;
    JPC_Vec3 angular_velocity1;
    JPC_Vec3 linear_velocity2;
    JPC_Vec3 angular_velocity2;
    /* Relative to the manifold's base offset. */
    JPC_Vec3 friction_point;
    JPC_Vec3 friction_impulse;
    float angular_friction_impulse;
    /* One per contact point of the manifold. */
    float contact_impulses[64];
    uint32_t num_contact_impulses;
} JPC_CollisionEstimation;

/* Estimates the result of a collision between two bodies, assuming no other bodies are involved. Only call it from
 * on_contact_added or on_contact_persisted, with that callback's bodies, manifold and settings. num_iterations of 1
 * skips friction. Returns false for a manifold with more than 64 points. */
JPC_API bool JPC_EstimateCollisionResponse(const JPC_Body *body1, const JPC_Body *body2, const JPC_ContactManifold *manifold,
                                           const JPC_ContactSettings *settings, float min_velocity_for_restitution, uint32_t num_iterations,
                                           JPC_CollisionEstimation *out_result);

/* fns is copied. Pass NULL to remove the listener.
 * Activation callbacks may also run on whichever thread wakes a body; they must not use a body interface. */
JPC_API void JPC_PhysicsSystem_SetContactListener(JPC_PhysicsSystem *system, const JPC_ContactListenerFns *fns, void *user_data);
JPC_API void JPC_PhysicsSystem_SetBodyActivationListener(JPC_PhysicsSystem *system, const JPC_BodyActivationListenerFns *fns, void *user_data);

typedef struct JPC_PhysicsStepContext {
    float delta_time;
    bool is_first_step;
    bool is_last_step;
} JPC_PhysicsStepContext;

/* Called before every collision step of an update, on a worker thread, while every body is locked.
 * Use the no-lock body interface to read or change existing bodies; do not add or remove bodies.
 * Several step listeners may run in parallel. */
typedef void (*JPC_StepListenerFn)(void *user_data, JPC_PhysicsSystem *system, const JPC_PhysicsStepContext *context);

JPC_API JPC_StepListener *JPC_PhysicsSystem_AddStepListener(JPC_PhysicsSystem *system, JPC_StepListenerFn fn, void *user_data);
/* Unregisters and frees the listener. */
JPC_API void JPC_PhysicsSystem_RemoveStepListener(JPC_PhysicsSystem *system, JPC_StepListener *listener);

/* ------------------------------------------------------------------------- */
/* Bodies                                                                     */
/* ------------------------------------------------------------------------- */

typedef struct JPC_BodyCreationSettings {
    JPC_RVec3 position;
    JPC_Quat rotation;
    JPC_Vec3 linear_velocity;
    JPC_Vec3 angular_velocity;
    uint64_t user_data;
    JPC_ObjectLayer object_layer;
    JPC_MotionType motion_type;
    JPC_AllowedDOFs allowed_dofs;
    bool allow_dynamic_or_kinematic;
    bool is_sensor;
    bool collide_kinematic_vs_non_dynamic;
    bool use_manifold_reduction;
    bool apply_gyroscopic_force;
    JPC_MotionQuality motion_quality;
    bool enhanced_internal_edge_removal;
    bool allow_sleeping;
    float friction;
    float restitution;
    float linear_damping;
    float angular_damping;
    float max_linear_velocity;
    float max_angular_velocity;
    float gravity_factor;
    uint32_t num_velocity_steps_override;
    uint32_t num_position_steps_override;
    JPC_OverrideMassProperties override_mass_properties;
    float inertia_multiplier;
    JPC_MassProperties mass_properties_override;
    JPC_CollisionGroup collision_group;
    /* Borrowed: the body takes its own reference. */
    const JPC_Shape *shape;
} JPC_BodyCreationSettings;

/* Fills settings with Jolt's defaults (shape = NULL). */
JPC_API void JPC_BodyCreationSettings_SetDefault(JPC_BodyCreationSettings *settings);

JPC_API JPC_BodyID JPC_BodyInterface_CreateBody(JPC_BodyInterface *bi, const JPC_BodyCreationSettings *settings);
JPC_API JPC_BodyID JPC_BodyInterface_CreateAndAddBody(JPC_BodyInterface *bi, const JPC_BodyCreationSettings *settings, JPC_Activation activation);
/* Removes the body from the world first if needed. Constraints using the body must be removed before.
 * Does nothing for stale ids and for bodies owned by a character. */
JPC_API void JPC_BodyInterface_DestroyBody(JPC_BodyInterface *bi, JPC_BodyID id);
/* Does nothing if the body is already added. */
JPC_API void JPC_BodyInterface_AddBody(JPC_BodyInterface *bi, JPC_BodyID id, JPC_Activation activation);
JPC_API void JPC_BodyInterface_RemoveBody(JPC_BodyInterface *bi, JPC_BodyID id);
JPC_API bool JPC_BodyInterface_IsAdded(const JPC_BodyInterface *bi, JPC_BodyID id);

JPC_API void JPC_BodyInterface_ActivateBody(JPC_BodyInterface *bi, JPC_BodyID id);
JPC_API void JPC_BodyInterface_DeactivateBody(JPC_BodyInterface *bi, JPC_BodyID id);
JPC_API bool JPC_BodyInterface_IsActive(const JPC_BodyInterface *bi, JPC_BodyID id);
JPC_API void JPC_BodyInterface_ResetSleepTimer(JPC_BodyInterface *bi, JPC_BodyID id);

/* Returns a new reference, or NULL if the body does not exist or is a soft body. Setting does nothing on soft bodies. */
JPC_API JPC_Shape *JPC_BodyInterface_GetShape(const JPC_BodyInterface *bi, JPC_BodyID id);
JPC_API void JPC_BodyInterface_SetShape(JPC_BodyInterface *bi, JPC_BodyID id, const JPC_Shape *shape, bool update_mass_properties, JPC_Activation activation);

JPC_API JPC_ObjectLayer JPC_BodyInterface_GetObjectLayer(const JPC_BodyInterface *bi, JPC_BodyID id);
/* Returns false if the layer does not exist in the system. */
JPC_API bool JPC_BodyInterface_SetObjectLayer(JPC_BodyInterface *bi, JPC_BodyID id, JPC_ObjectLayer layer);

JPC_API JPC_RVec3 JPC_BodyInterface_GetPosition(const JPC_BodyInterface *bi, JPC_BodyID id);
JPC_API void JPC_BodyInterface_SetPosition(JPC_BodyInterface *bi, JPC_BodyID id, JPC_RVec3 position, JPC_Activation activation);
JPC_API JPC_RVec3 JPC_BodyInterface_GetCenterOfMassPosition(const JPC_BodyInterface *bi, JPC_BodyID id);
JPC_API JPC_Quat JPC_BodyInterface_GetRotation(const JPC_BodyInterface *bi, JPC_BodyID id);
JPC_API void JPC_BodyInterface_SetRotation(JPC_BodyInterface *bi, JPC_BodyID id, JPC_Quat rotation, JPC_Activation activation);
JPC_API void JPC_BodyInterface_SetPositionAndRotation(JPC_BodyInterface *bi, JPC_BodyID id, JPC_RVec3 position, JPC_Quat rotation, JPC_Activation activation);
JPC_API void JPC_BodyInterface_SetPositionAndRotationWhenChanged(JPC_BodyInterface *bi, JPC_BodyID id, JPC_RVec3 position, JPC_Quat rotation, JPC_Activation activation);
JPC_API void JPC_BodyInterface_GetPositionAndRotation(const JPC_BodyInterface *bi, JPC_BodyID id, JPC_RVec3 *out_position, JPC_Quat *out_rotation);
JPC_API JPC_RMat44 JPC_BodyInterface_GetWorldTransform(const JPC_BodyInterface *bi, JPC_BodyID id);
JPC_API JPC_RMat44 JPC_BodyInterface_GetCenterOfMassTransform(const JPC_BodyInterface *bi, JPC_BodyID id);
JPC_API void JPC_BodyInterface_MoveKinematic(JPC_BodyInterface *bi, JPC_BodyID id, JPC_RVec3 target_position, JPC_Quat target_rotation, float delta_time);

JPC_API JPC_Vec3 JPC_BodyInterface_GetLinearVelocity(const JPC_BodyInterface *bi, JPC_BodyID id);
JPC_API void JPC_BodyInterface_SetLinearVelocity(JPC_BodyInterface *bi, JPC_BodyID id, JPC_Vec3 velocity);
JPC_API void JPC_BodyInterface_AddLinearVelocity(JPC_BodyInterface *bi, JPC_BodyID id, JPC_Vec3 velocity);
JPC_API JPC_Vec3 JPC_BodyInterface_GetAngularVelocity(const JPC_BodyInterface *bi, JPC_BodyID id);
JPC_API void JPC_BodyInterface_SetAngularVelocity(JPC_BodyInterface *bi, JPC_BodyID id, JPC_Vec3 velocity);
JPC_API void JPC_BodyInterface_SetLinearAndAngularVelocity(JPC_BodyInterface *bi, JPC_BodyID id, JPC_Vec3 linear, JPC_Vec3 angular);
JPC_API JPC_Vec3 JPC_BodyInterface_GetPointVelocity(const JPC_BodyInterface *bi, JPC_BodyID id, JPC_RVec3 point);

JPC_API void JPC_BodyInterface_AddForce(JPC_BodyInterface *bi, JPC_BodyID id, JPC_Vec3 force, JPC_Activation activation);
JPC_API void JPC_BodyInterface_AddForceAtPoint(JPC_BodyInterface *bi, JPC_BodyID id, JPC_Vec3 force, JPC_RVec3 point, JPC_Activation activation);
JPC_API void JPC_BodyInterface_AddTorque(JPC_BodyInterface *bi, JPC_BodyID id, JPC_Vec3 torque, JPC_Activation activation);
JPC_API void JPC_BodyInterface_AddImpulse(JPC_BodyInterface *bi, JPC_BodyID id, JPC_Vec3 impulse);
JPC_API void JPC_BodyInterface_AddImpulseAtPoint(JPC_BodyInterface *bi, JPC_BodyID id, JPC_Vec3 impulse, JPC_RVec3 point);
JPC_API void JPC_BodyInterface_AddAngularImpulse(JPC_BodyInterface *bi, JPC_BodyID id, JPC_Vec3 angular_impulse);

JPC_API JPC_MotionType JPC_BodyInterface_GetMotionType(const JPC_BodyInterface *bi, JPC_BodyID id);
/* Fails (returns false) when making a body movable that was created static without allow_dynamic_or_kinematic. */
JPC_API bool JPC_BodyInterface_SetMotionType(JPC_BodyInterface *bi, JPC_BodyID id, JPC_MotionType motion_type, JPC_Activation activation);
JPC_API JPC_MotionQuality JPC_BodyInterface_GetMotionQuality(const JPC_BodyInterface *bi, JPC_BodyID id);
JPC_API void JPC_BodyInterface_SetMotionQuality(JPC_BodyInterface *bi, JPC_BodyID id, JPC_MotionQuality quality);

JPC_API float JPC_BodyInterface_GetFriction(const JPC_BodyInterface *bi, JPC_BodyID id);
JPC_API void JPC_BodyInterface_SetFriction(JPC_BodyInterface *bi, JPC_BodyID id, float friction);
JPC_API float JPC_BodyInterface_GetRestitution(const JPC_BodyInterface *bi, JPC_BodyID id);
JPC_API void JPC_BodyInterface_SetRestitution(JPC_BodyInterface *bi, JPC_BodyID id, float restitution);
JPC_API float JPC_BodyInterface_GetGravityFactor(const JPC_BodyInterface *bi, JPC_BodyID id);
JPC_API void JPC_BodyInterface_SetGravityFactor(JPC_BodyInterface *bi, JPC_BodyID id, float factor);
JPC_API bool JPC_BodyInterface_IsSensor(const JPC_BodyInterface *bi, JPC_BodyID id);
JPC_API void JPC_BodyInterface_SetIsSensor(JPC_BodyInterface *bi, JPC_BodyID id, bool is_sensor);
JPC_API uint64_t JPC_BodyInterface_GetUserData(const JPC_BodyInterface *bi, JPC_BodyID id);
JPC_API void JPC_BodyInterface_SetUserData(JPC_BodyInterface *bi, JPC_BodyID id, uint64_t user_data);
JPC_API void JPC_BodyInterface_InvalidateContactCache(JPC_BodyInterface *bi, JPC_BodyID id);

/* Adds/removes many bodies at once, which is much faster than one at a time. Ids that are invalid,
 * already in the requested state or duplicated are skipped. */
JPC_API void JPC_BodyInterface_AddBodies(JPC_BodyInterface *bi, const JPC_BodyID *ids, uint32_t count, JPC_Activation activation);
JPC_API void JPC_BodyInterface_RemoveBodies(JPC_BodyInterface *bi, const JPC_BodyID *ids, uint32_t count);

/* out_group->group_filter is a new reference (release it with JPC_GroupFilter_Release). */
JPC_API void JPC_BodyInterface_GetCollisionGroup(const JPC_BodyInterface *bi, JPC_BodyID id, JPC_CollisionGroup *out_group);
JPC_API void JPC_BodyInterface_SetCollisionGroup(JPC_BodyInterface *bi, JPC_BodyID id, const JPC_CollisionGroup *group);

JPC_API JPC_AABox JPC_BodyInterface_GetWorldSpaceBounds(const JPC_BodyInterface *bi, JPC_BodyID id);
/* 0 for static bodies. */
JPC_API float JPC_BodyInterface_GetInverseMass(const JPC_BodyInterface *bi, JPC_BodyID id);
/* Zero for bodies that are not dynamic. */
JPC_API JPC_Mat44 JPC_BodyInterface_GetInverseInertia(const JPC_BodyInterface *bi, JPC_BodyID id);
JPC_API float JPC_BodyInterface_GetMaxLinearVelocity(const JPC_BodyInterface *bi, JPC_BodyID id);
JPC_API void JPC_BodyInterface_SetMaxLinearVelocity(JPC_BodyInterface *bi, JPC_BodyID id, float velocity);
JPC_API float JPC_BodyInterface_GetMaxAngularVelocity(const JPC_BodyInterface *bi, JPC_BodyID id);
JPC_API void JPC_BodyInterface_SetMaxAngularVelocity(JPC_BodyInterface *bi, JPC_BodyID id, float velocity);
JPC_API bool JPC_BodyInterface_GetUseManifoldReduction(const JPC_BodyInterface *bi, JPC_BodyID id);
JPC_API void JPC_BodyInterface_SetUseManifoldReduction(JPC_BodyInterface *bi, JPC_BodyID id, bool use);
/* Applies buoyancy and drag for a fluid surface. Returns false if the body is not submerged or not dynamic. */
JPC_API bool JPC_BodyInterface_ApplyBuoyancyImpulse(JPC_BodyInterface *bi, JPC_BodyID id, JPC_RVec3 surface_position, JPC_Vec3 surface_normal,
                                                    float buoyancy, float linear_drag, float angular_drag, JPC_Vec3 fluid_velocity,
                                                    JPC_Vec3 gravity, float delta_time);

/* Read-only accessors for bodies handed to listener callbacks. */
JPC_API JPC_BodyID JPC_Body_GetID(const JPC_Body *body);
JPC_API uint64_t JPC_Body_GetUserData(const JPC_Body *body);
JPC_API JPC_ObjectLayer JPC_Body_GetObjectLayer(const JPC_Body *body);
JPC_API JPC_MotionType JPC_Body_GetMotionType(const JPC_Body *body);
JPC_API bool JPC_Body_IsSensor(const JPC_Body *body);
JPC_API bool JPC_Body_IsActive(const JPC_Body *body);
JPC_API JPC_RVec3 JPC_Body_GetPosition(const JPC_Body *body);
JPC_API JPC_Quat JPC_Body_GetRotation(const JPC_Body *body);
JPC_API JPC_RVec3 JPC_Body_GetCenterOfMassPosition(const JPC_Body *body);
JPC_API JPC_Vec3 JPC_Body_GetLinearVelocity(const JPC_Body *body);
JPC_API JPC_Vec3 JPC_Body_GetAngularVelocity(const JPC_Body *body);
JPC_API float JPC_Body_GetFriction(const JPC_Body *body);
JPC_API float JPC_Body_GetRestitution(const JPC_Body *body);
/* Borrowed: valid as long as the body keeps this shape. NULL for soft bodies, whose shape lives inside the body. */
JPC_API const JPC_Shape *JPC_Body_GetShape(const JPC_Body *body);
JPC_API bool JPC_Body_IsSoftBody(const JPC_Body *body);
JPC_API JPC_AABox JPC_Body_GetWorldSpaceBounds(const JPC_Body *body);

/* ------------------------------------------------------------------------- */
/* Soft bodies                                                                */
/*                                                                            */
/* A soft body is a body made of particles (vertices) held together by        */
/* constraints. It lives in the same world as rigid bodies and uses the same  */
/* ids, but calls that only make sense for rigid bodies (shapes, kinematic    */
/* motion, rotation, constraints, vehicles, characters) refuse soft bodies.   */
/* Velocity is kept per vertex: setting the body's velocity or adding         */
/* impulses or torque has no effect (forces and gravity do). Vehicles don't   */
/* drive on soft bodies.                                                      */
/* ------------------------------------------------------------------------- */

typedef enum JPC_SoftBodyBendType {
    JPC_SOFT_BODY_BEND_TYPE_NONE = 0,
    JPC_SOFT_BODY_BEND_TYPE_DISTANCE = 1,
    JPC_SOFT_BODY_BEND_TYPE_DIHEDRAL = 2,
} JPC_SoftBodyBendType;

typedef enum JPC_SoftBodyLRAType {
    JPC_SOFT_BODY_LRA_TYPE_NONE = 0,
    JPC_SOFT_BODY_LRA_TYPE_EUCLIDEAN_DISTANCE = 1,
    JPC_SOFT_BODY_LRA_TYPE_GEODESIC_DISTANCE = 2,
} JPC_SoftBodyLRAType;

typedef struct JPC_SoftBodyVertex {
    JPC_Vec3 position;
    JPC_Vec3 velocity;
    /* 0 pins the vertex (it moves only when you move it). */
    float inv_mass;
} JPC_SoftBodyVertex;

typedef struct JPC_SoftBodyFace {
    uint32_t vertices[3];
    /* Index into the materials of the settings (0 when there are none). */
    uint32_t material_index;
} JPC_SoftBodyFace;

/* Compliance is the inverse of stiffness: 0 is rigid. */
typedef struct JPC_SoftBodyEdge {
    uint32_t vertices[2];
    float compliance;
} JPC_SoftBodyEdge;

/* Keeps the angle between the triangles (v0, v1, v2) and (v1, v0, v3) that share the edge v0-v1. */
typedef struct JPC_SoftBodyDihedralBend {
    uint32_t vertices[4];
    float compliance;
} JPC_SoftBodyDihedralBend;

/* Keeps the volume of a tetrahedron. */
typedef struct JPC_SoftBodyVolume {
    uint32_t vertices[4];
    float compliance;
} JPC_SoftBodyVolume;

/* Long range attachment: keeps a dynamic vertex (1) within its rest distance (times the multiplier) of a pinned one (0). */
typedef struct JPC_SoftBodyLRA {
    uint32_t vertices[2];
    float max_distance_multiplier;
} JPC_SoftBodyLRA;

/* Per vertex settings for creating constraints from the faces. */
typedef struct JPC_SoftBodyVertexAttributes {
    float compliance;
    float shear_compliance;
    /* FLT_MAX for no bend constraints. */
    float bend_compliance;
    JPC_SoftBodyLRAType lra_type;
    float lra_max_distance_multiplier;
} JPC_SoftBodyVertexAttributes;

/* Takes a vertex in its bind pose (JPC_SoftBodyVertex::position) into the space of a joint. */
typedef struct JPC_SoftBodyInvBind {
    uint32_t joint_index;
    JPC_Mat44 inv_bind;
} JPC_SoftBodyInvBind;

typedef struct JPC_SoftBodySkinWeight {
    /* Index into the inverse bind matrices. */
    uint32_t inv_bind_index;
    float weight;
} JPC_SoftBodySkinWeight;

/* Skins a vertex to up to 4 joints and limits how far the simulated vertex can move from where skinning puts it. */
typedef struct JPC_SoftBodySkinned {
    uint32_t vertex;
    /* A weight of 0 ends the list. Weights are normalized to add up to 1. */
    JPC_SoftBodySkinWeight weights[4];
    /* 0 hard skins the vertex; FLT_MAX leaves it free. */
    float max_distance;
    /* Keeps the vertex from going further than this behind the skinned surface; disabled when >= max_distance. */
    float back_stop_distance;
    float back_stop_radius;
} JPC_SoftBodySkinned;

/* A rigid segment between two vertices that also has an orientation. Every rod should be in at least one bend twist
 * constraint, or it spins freely around its axis. */
typedef struct JPC_SoftBodyRod {
    uint32_t vertices[2];
    float compliance;
} JPC_SoftBodyRod;

/* Limits bending and twisting between two rods (indices into the rods). */
typedef struct JPC_SoftBodyRodBendTwist {
    uint32_t rods[2];
    float compliance;
} JPC_SoftBodyRodBendTwist;

typedef struct JPC_SoftBodySharedSettingsDesc {
    const JPC_SoftBodyVertex *vertices;
    uint32_t num_vertices;
    const JPC_SoftBodyFace *faces;
    uint32_t num_faces;
    /* Borrowed. None means every face uses the default material. */
    const JPC_PhysicsMaterial *const *materials;
    uint32_t num_materials;
    /* When given, edge, shear, bend and LRA constraints are created from the faces. One per vertex; when there are
     * fewer, the last one is repeated. */
    const JPC_SoftBodyVertexAttributes *vertex_attributes;
    uint32_t num_vertex_attributes;
    JPC_SoftBodyBendType bend_type;
    /* Radians; how far from a square two triangles may be to get a shear edge. */
    float angle_tolerance;
    /* Extra constraints, added to the generated ones. Their rest lengths, angles and volumes come from the vertex positions. */
    const JPC_SoftBodyEdge *edges;
    uint32_t num_edges;
    const JPC_SoftBodyDihedralBend *dihedral_bends;
    uint32_t num_dihedral_bends;
    const JPC_SoftBodyVolume *volumes;
    uint32_t num_volumes;
    const JPC_SoftBodyLRA *lras;
    uint32_t num_lras;
    /* Skinning (e.g. clothing on an animated character). */
    const JPC_SoftBodyInvBind *inv_binds;
    uint32_t num_inv_binds;
    const JPC_SoftBodySkinned *skinned;
    uint32_t num_skinned;
    /* Cosserat rods (e.g. hair, plants): rigid segments with an orientation. */
    const JPC_SoftBodyRod *rods;
    uint32_t num_rods;
    const JPC_SoftBodyRodBendTwist *rod_bend_twists;
    uint32_t num_rod_bend_twists;
} JPC_SoftBodySharedSettingsDesc;

typedef struct JPC_SoftBodySharedSettingsCounts {
    uint32_t vertices;
    uint32_t faces;
    uint32_t edges;
    uint32_t dihedral_bends;
    uint32_t volumes;
    uint32_t lras;
    uint32_t skinned;
    uint32_t rods;
    uint32_t rod_bend_twists;
    /* Joints the skinning refers to (highest joint index + 1). */
    uint32_t skin_joints;
} JPC_SoftBodySharedSettingsCounts;

JPC_API void JPC_SoftBodyVertexAttributes_SetDefault(JPC_SoftBodyVertexAttributes *attributes);
JPC_API void JPC_SoftBodySharedSettingsDesc_SetDefault(JPC_SoftBodySharedSettingsDesc *desc);
/* The particles and constraints of a soft body; one settings object can create many bodies. Immutable and reference
 * counted. NULL with an error for out of range or repeated vertex indices, degenerate faces, bad material indices,
 * constraints or rods between vertices at the same position, negative masses or compliances, or a vertex skinned
 * twice or touching 256 or more faces. */
JPC_API JPC_SoftBodySharedSettings *JPC_SoftBodySharedSettings_Create(const JPC_SoftBodySharedSettingsDesc *desc);
/* A cube of grid_size^3 vertices (2 to 64 per side) with edge and volume constraints, for testing. */
JPC_API JPC_SoftBodySharedSettings *JPC_SoftBodySharedSettings_CreateCube(uint32_t grid_size, float spacing);
JPC_API void JPC_SoftBodySharedSettings_AddRef(const JPC_SoftBodySharedSettings *settings);
JPC_API void JPC_SoftBodySharedSettings_Release(const JPC_SoftBodySharedSettings *settings);
JPC_API void JPC_SoftBodySharedSettings_GetCounts(const JPC_SoftBodySharedSettings *settings, JPC_SoftBodySharedSettingsCounts *out_counts);
/* Writes up to capacity faces and returns the total number (e.g. to render the body). */
JPC_API uint32_t JPC_SoftBodySharedSettings_GetFaces(const JPC_SoftBodySharedSettings *settings, JPC_SoftBodyFace *out_faces, uint32_t capacity);
/* The rods in simulation order, which is not the order they were given in (and the two vertices of a rod may have been
 * swapped). Rod states are reported in this order. Writes up to capacity and returns the total number. */
JPC_API uint32_t JPC_SoftBodySharedSettings_GetRods(const JPC_SoftBodySharedSettings *settings, JPC_SoftBodyRod *out_rods, uint32_t capacity);

typedef struct JPC_SoftBodyCreationSettings {
    /* Borrowed; the body keeps its own reference. Required. */
    const JPC_SoftBodySharedSettings *settings;
    JPC_RVec3 position;
    JPC_Quat rotation;
    uint64_t user_data;
    JPC_ObjectLayer object_layer;
    JPC_CollisionGroup collision_group;
    uint32_t num_iterations;
    float linear_damping;
    float max_linear_velocity;
    float restitution;
    float friction;
    /* Inflates closed bodies (n * R * T of the ideal gas law); 0 for none. */
    float pressure;
    float gravity_factor;
    /* Keeps vertices this far away from other bodies. */
    float vertex_radius;
    /* Keep the body's position at its vertices' center (false for things attached to the static world). */
    bool update_position;
    /* Bake the rotation into the vertices and keep the body's rotation identity (more accurate). */
    bool make_rotation_identity;
    bool allow_sleeping;
    /* Queries hit faces from both sides. */
    bool faces_double_sided;
} JPC_SoftBodyCreationSettings;

JPC_API void JPC_SoftBodyCreationSettings_SetDefault(JPC_SoftBodyCreationSettings *settings);
JPC_API JPC_BodyID JPC_BodyInterface_CreateSoftBody(JPC_BodyInterface *bi, const JPC_SoftBodyCreationSettings *settings);
JPC_API JPC_BodyID JPC_BodyInterface_CreateAndAddSoftBody(JPC_BodyInterface *bi, const JPC_SoftBodyCreationSettings *settings, JPC_Activation activation);
JPC_API bool JPC_BodyInterface_IsSoftBody(const JPC_BodyInterface *bi, JPC_BodyID id);
/* New reference, or NULL if the body is not a soft body. */
JPC_API JPC_SoftBodySharedSettings *JPC_BodyInterface_GetSoftBodySharedSettings(const JPC_BodyInterface *bi, JPC_BodyID id);

typedef struct JPC_SoftBodyVertexState {
    /* World space or relative to the body's center of mass transform, see below. */
    JPC_RVec3 position;
    JPC_Vec3 velocity;
    float inv_mass;
} JPC_SoftBodyVertexState;

/* Copies up to capacity vertices starting at first and returns the body's total vertex count (0 for anything that is
 * not a soft body). world_space gives world positions and velocities, otherwise they are relative to the body's
 * center of mass transform. */
JPC_API uint32_t JPC_BodyInterface_GetSoftBodyVertices(const JPC_BodyInterface *bi, JPC_BodyID id, uint32_t first, JPC_SoftBodyVertexState *out_vertices,
                                                       uint32_t capacity, bool world_space);
/* Overwrites count vertices starting at first; changing inverse masses updates the body's
 * mass. Returns false for anything that is not a soft body or a range past the end. */
JPC_API bool JPC_BodyInterface_SetSoftBodyVertices(JPC_BodyInterface *bi, JPC_BodyID id, uint32_t first, const JPC_SoftBodyVertexState *vertices, uint32_t count,
                                                   bool world_space, JPC_Activation activation);

typedef struct JPC_SoftBodyProperties {
    uint32_t num_iterations;
    float pressure;
    float vertex_radius;
    bool update_position;
    bool faces_double_sided;
} JPC_SoftBodyProperties;

/* Return false for anything that is not a soft body. Setting fails for a negative vertex_radius. */
JPC_API bool JPC_BodyInterface_GetSoftBodyProperties(const JPC_BodyInterface *bi, JPC_BodyID id, JPC_SoftBodyProperties *out_properties);
JPC_API bool JPC_BodyInterface_SetSoftBodyProperties(JPC_BodyInterface *bi, JPC_BodyID id, const JPC_SoftBodyProperties *properties);
/* Current enclosed volume (meaningful for closed bodies), 0 for anything that is not a soft body. */
JPC_API float JPC_BodyInterface_GetSoftBodyVolume(const JPC_BodyInterface *bi, JPC_BodyID id);

typedef struct JPC_SoftBodyRodState {
    JPC_Quat rotation;
    JPC_Vec3 angular_velocity;
} JPC_SoftBodyRodState;

/* Copies up to capacity rod states (world space, in the order of JPC_SoftBodySharedSettings_GetRods) starting at first
 * and returns the total number of rods (0 for anything that is not a soft body). */
JPC_API uint32_t JPC_BodyInterface_GetSoftBodyRods(const JPC_BodyInterface *bi, JPC_BodyID id, uint32_t first, JPC_SoftBodyRodState *out_rods, uint32_t capacity);

/* Skins the body's skinned vertices with num_joints world space joint transforms, e.g. from an animated skeleton. Call it before every update that should follow the animation.
 * hard_skin_all teleports every skinned vertex to its skinned position (e.g. after a teleport). Returns false for
 * anything that is not a soft body, a body without skinned vertices, or fewer joints than the skinning refers to. */
JPC_API bool JPC_BodyInterface_SkinSoftBody(JPC_BodyInterface *bi, JPC_BodyID id, const JPC_RMat44 *joint_matrices, uint32_t num_joints, bool hard_skin_all,
                                            JPC_TempAllocator *temp_allocator);
/* Turns the max distance / back stop limits of skinned vertices on or off (on by default). With them off, only hard
 * skinned vertices (max_distance 0) follow the skin. */
JPC_API bool JPC_BodyInterface_SetSoftBodySkinConstraintsEnabled(JPC_BodyInterface *bi, JPC_BodyID id, bool enabled, float max_distance_multiplier);

/* Soft body contacts. Callbacks run on worker threads during the update, while all bodies are locked: don't use a
 * body interface. Only the soft body is safe to read; the other body may be moved by another thread. */
typedef struct JPC_SoftBodyContactSettings {
    /* Inverse mass scales of the soft body (1) and the other body (2), and inverse inertia scale of the other body:
     * 0 makes that side immovable. */
    float inv_mass_scale1;
    float inv_mass_scale2;
    float inv_inertia_scale2;
    /* Report contacts but don't push. */
    bool is_sensor;
} JPC_SoftBodyContactSettings;

typedef struct JPC_SoftBodyVertexContact {
    uint32_t vertex;
    JPC_BodyID body_id;
    /* World space. */
    JPC_RVec3 contact_point;
    /* Points away from the other body. */
    JPC_Vec3 contact_normal;
} JPC_SoftBodyVertexContact;

typedef struct JPC_SoftBodyManifold {
    /* One entry per vertex that touched something this update. Only valid during the callback. */
    const JPC_SoftBodyVertexContact *contacts;
    uint32_t num_contacts;
    /* Sensors the soft body touches. */
    const JPC_BodyID *sensors;
    uint32_t num_sensors;
} JPC_SoftBodyManifold;

/* Any callback may be NULL. */
typedef struct JPC_SoftBodyContactListenerFns {
    /* Called when the soft body's bounds start overlapping another body. Return false to ignore that body. */
    bool (*on_soft_body_contact_validate)(void *user_data, const JPC_Body *soft_body, const JPC_Body *other_body, JPC_SoftBodyContactSettings *settings);
    /* Called once per update for a soft body that touches anything. */
    void (*on_soft_body_contact_added)(void *user_data, const JPC_Body *soft_body, const JPC_SoftBodyManifold *manifold);
} JPC_SoftBodyContactListenerFns;

/* fns is copied; NULL removes the listener. */
JPC_API void JPC_PhysicsSystem_SetSoftBodyContactListener(JPC_PhysicsSystem *system, const JPC_SoftBodyContactListenerFns *fns, void *user_data);

/* ------------------------------------------------------------------------- */
/* Debug rendering                                                            */
/*                                                                            */
/* Only available when Jolt was built with its debug renderer (the Rust       */
/* crate's debug-renderer feature); otherwise creating a renderer fails.      */
/* ------------------------------------------------------------------------- */

typedef struct JPC_Color {
    uint8_t r, g, b, a;
} JPC_Color;

/* Called on the drawing thread while drawing. draw_triangle may be NULL to get each triangle as 3 lines; text is not
 * NUL terminated. */
typedef struct JPC_DebugRendererFns {
    void (*draw_line)(void *user_data, JPC_RVec3 from, JPC_RVec3 to, JPC_Color color);
    void (*draw_triangle)(void *user_data, JPC_RVec3 v1, JPC_RVec3 v2, JPC_RVec3 v3, JPC_Color color, bool cast_shadow);
    void (*draw_text)(void *user_data, JPC_RVec3 position, const char *text, size_t length, JPC_Color color, float height);
} JPC_DebugRendererFns;

typedef enum JPC_ShapeColor {
    JPC_SHAPE_COLOR_INSTANCE = 0,
    JPC_SHAPE_COLOR_SHAPE_TYPE = 1,
    JPC_SHAPE_COLOR_MOTION_TYPE = 2,
    JPC_SHAPE_COLOR_SLEEP = 3,
    JPC_SHAPE_COLOR_ISLAND = 4,
    JPC_SHAPE_COLOR_MATERIAL = 5,
} JPC_ShapeColor;

typedef enum JPC_SoftBodyConstraintColor {
    JPC_SOFT_BODY_CONSTRAINT_COLOR_TYPE = 0,
    JPC_SOFT_BODY_CONSTRAINT_COLOR_GROUP = 1,
    JPC_SOFT_BODY_CONSTRAINT_COLOR_ORDER = 2,
} JPC_SoftBodyConstraintColor;

typedef struct JPC_BodyDrawSettings {
    bool draw_get_support_function;
    bool draw_support_direction;
    bool draw_get_supporting_face;
    bool draw_shape;
    bool draw_shape_wireframe;
    JPC_ShapeColor shape_color;
    bool draw_bounding_box;
    bool draw_center_of_mass_transform;
    bool draw_world_transform;
    bool draw_velocity;
    bool draw_mass_and_inertia;
    bool draw_sleep_stats;
    bool draw_soft_body_vertices;
    bool draw_soft_body_vertex_velocities;
    bool draw_soft_body_edge_constraints;
    bool draw_soft_body_bend_constraints;
    bool draw_soft_body_volume_constraints;
    bool draw_soft_body_skin_constraints;
    bool draw_soft_body_lra_constraints;
    bool draw_soft_body_rods;
    bool draw_soft_body_rod_states;
    bool draw_soft_body_rod_bend_twist_constraints;
    bool draw_soft_body_predicted_bounds;
    JPC_SoftBodyConstraintColor soft_body_constraint_color;
} JPC_BodyDrawSettings;

JPC_API bool JPC_DebugRenderer_IsAvailable(void);
JPC_API void JPC_BodyDrawSettings_SetDefault(JPC_BodyDrawSettings *settings);
/* Jolt allows one renderer at a time: NULL with an error if one exists or Jolt was built without the debug renderer.
 * fns is copied and can be replaced with JPC_DebugRenderer_SetFns. */
JPC_API JPC_DebugRenderer *JPC_DebugRenderer_Create(const JPC_DebugRendererFns *fns, void *user_data);
JPC_API void JPC_DebugRenderer_Destroy(JPC_DebugRenderer *renderer);
JPC_API void JPC_DebugRenderer_SetFns(JPC_DebugRenderer *renderer, const JPC_DebugRendererFns *fns, void *user_data);
/* Detailed shapes are simplified with distance from the camera. */
JPC_API void JPC_DebugRenderer_SetCameraPosition(JPC_DebugRenderer *renderer, JPC_RVec3 position);

typedef bool (*JPC_BodyDrawFilterFn)(void *user_data, JPC_BodyID body_id);

/* Draws the bodies in the world. Every body is locked meanwhile: filter (which may be NULL) must not use a body
 * interface, and neither may the renderer's callbacks. Must not run during an update. */
JPC_API void JPC_PhysicsSystem_DrawBodies(JPC_PhysicsSystem *system, const JPC_BodyDrawSettings *settings, JPC_DebugRenderer *renderer,
                                          JPC_BodyDrawFilterFn filter, void *filter_user_data);
/* Draw the constraints (vehicles included), their limits or their reference frames. These read bodies without locking
 * them: nothing else may use the system meanwhile. */
JPC_API void JPC_PhysicsSystem_DrawConstraints(JPC_PhysicsSystem *system, JPC_DebugRenderer *renderer);
JPC_API void JPC_PhysicsSystem_DrawConstraintLimits(JPC_PhysicsSystem *system, JPC_DebugRenderer *renderer);
JPC_API void JPC_PhysicsSystem_DrawConstraintReferenceFrames(JPC_PhysicsSystem *system, JPC_DebugRenderer *renderer);
/* Draws a skeleton posed with one model space matrix per joint, offset by root_offset. */
JPC_API void JPC_Skeleton_DrawPose(const JPC_Skeleton *skeleton, const JPC_Mat44 *joint_matrices, JPC_RVec3 root_offset, bool draw_joint_names,
                                   JPC_DebugRenderer *renderer);

/* ------------------------------------------------------------------------- */
/* Queries                                                                    */
/* ------------------------------------------------------------------------- */

typedef bool (*JPC_BodyFilterFn)(void *user_data, JPC_BodyID body_id);

/* Zero-initialise and set the fields you need. Use JPC_QueryFilter_SetDefault to start from "accept all". */
typedef struct JPC_QueryFilter {
    /* When true only bodies whose layer collides with object_layer are considered. */
    bool use_object_layer;
    JPC_ObjectLayer object_layer;
    /* JPC_BODY_ID_INVALID to ignore nothing. */
    JPC_BodyID ignore_body;
    /* Optional: return false to skip a body. */
    JPC_BodyFilterFn body_filter;
    void *body_filter_user_data;
} JPC_QueryFilter;

JPC_API void JPC_QueryFilter_SetDefault(JPC_QueryFilter *filter);

typedef struct JPC_RayCastHit {
    JPC_BodyID body_id;
    JPC_SubShapeID sub_shape_id;
    float fraction;
    JPC_RVec3 point;
    JPC_Vec3 normal;
    /* NULL for the default material. Borrowed. */
    const JPC_PhysicsMaterial *material;
} JPC_RayCastHit;

typedef struct JPC_RayCastSettings {
    JPC_BackFaceMode back_face_mode_triangles;
    JPC_BackFaceMode back_face_mode_convex;
    bool treat_convex_as_solid;
} JPC_RayCastSettings;

typedef struct JPC_CollidePointHit {
    JPC_BodyID body_id;
    JPC_SubShapeID sub_shape_id;
} JPC_CollidePointHit;

typedef struct JPC_ShapeHit {
    JPC_BodyID body_id;
    JPC_SubShapeID sub_shape_id1;
    JPC_SubShapeID sub_shape_id2;
    /* World space. */
    JPC_RVec3 contact_point_on1;
    JPC_RVec3 contact_point_on2;
    JPC_Vec3 penetration_axis;
    float penetration_depth;
    /* Only meaningful for shape casts. */
    float fraction;
    bool is_back_face_hit;
    /* Material of the hit body at sub_shape_id2, NULL for the default material. Borrowed. */
    const JPC_PhysicsMaterial *material;
    /* With collect_faces set in the query settings: the touching faces of the query shape (1) and the hit shape (2),
     * in world space, up to 32 points each. Only valid during the collector callback; NULL and 0 otherwise. */
    const JPC_RVec3 *face1;
    uint32_t num_face1;
    const JPC_RVec3 *face2;
    uint32_t num_face2;
} JPC_ShapeHit;

/* Return false from a collector to stop the query early. Collectors and body filters run on the calling thread while
 * Jolt holds the broad phase query lock and (for collectors) a read lock on the hit body: they must not modify bodies,
 * add or remove bodies, or run other queries, or they may deadlock. Copy what you need and act after the query. */
typedef bool (*JPC_RayCastCollectorFn)(void *user_data, const JPC_RayCastHit *hit);
typedef bool (*JPC_CollidePointCollectorFn)(void *user_data, const JPC_CollidePointHit *hit);
typedef bool (*JPC_ShapeCollectorFn)(void *user_data, const JPC_ShapeHit *hit);

JPC_API void JPC_RayCastSettings_SetDefault(JPC_RayCastSettings *settings);

/* direction includes the ray length. filter may be NULL. Returns true on a hit.
 * out_hit->material is a new reference here (release it with JPC_PhysicsMaterial_Release). */
JPC_API bool JPC_PhysicsSystem_CastRay(const JPC_PhysicsSystem *system, JPC_RVec3 origin, JPC_Vec3 direction,
                                       const JPC_QueryFilter *filter, JPC_RayCastHit *out_hit);
/* Reports every hit in no particular order. settings may be NULL. */
JPC_API void JPC_PhysicsSystem_CastRayAll(const JPC_PhysicsSystem *system, JPC_RVec3 origin, JPC_Vec3 direction,
                                          const JPC_RayCastSettings *settings, const JPC_QueryFilter *filter,
                                          JPC_RayCastCollectorFn collector, void *user_data);
JPC_API void JPC_PhysicsSystem_CollidePoint(const JPC_PhysicsSystem *system, JPC_RVec3 point, const JPC_QueryFilter *filter,
                                            JPC_CollidePointCollectorFn collector, void *user_data);
/* transform places the shape's center of mass in the world. max_separation_distance > 0 also reports near misses. */
JPC_API void JPC_PhysicsSystem_CollideShape(const JPC_PhysicsSystem *system, const JPC_Shape *shape, JPC_Vec3 scale,
                                            const JPC_RMat44 *transform, float max_separation_distance,
                                            const JPC_QueryFilter *filter, JPC_ShapeCollectorFn collector, void *user_data);
typedef struct JPC_CollideShapeSettings {
    float max_separation_distance;
    JPC_BackFaceMode back_face_mode;
    JPC_ActiveEdgeMode active_edge_mode;
    JPC_Vec3 active_edge_movement_direction;
    float collision_tolerance;
    float penetration_tolerance;
    /* Report the touching faces of both shapes with each hit (JPC_ShapeHit::face1 / face2). */
    bool collect_faces;
} JPC_CollideShapeSettings;

typedef struct JPC_ShapeCastSettings {
    JPC_BackFaceMode back_face_mode_triangles;
    JPC_BackFaceMode back_face_mode_convex;
    bool use_shrunken_shape_and_convex_radius;
    bool return_deepest_point;
    JPC_ActiveEdgeMode active_edge_mode;
    JPC_Vec3 active_edge_movement_direction;
    float collision_tolerance;
    float penetration_tolerance;
    bool collect_faces;
} JPC_ShapeCastSettings;

JPC_API void JPC_CollideShapeSettings_SetDefault(JPC_CollideShapeSettings *settings);
JPC_API void JPC_ShapeCastSettings_SetDefault(JPC_ShapeCastSettings *settings);

/* Closest hit with explicit settings; otherwise like JPC_PhysicsSystem_CastRay (out_hit->material is owned). */
JPC_API bool JPC_PhysicsSystem_CastRayWithSettings(const JPC_PhysicsSystem *system, JPC_RVec3 origin, JPC_Vec3 direction,
                                                   const JPC_RayCastSettings *settings, const JPC_QueryFilter *filter, JPC_RayCastHit *out_hit);
JPC_API void JPC_PhysicsSystem_CollideShapeWithSettings(const JPC_PhysicsSystem *system, const JPC_Shape *shape, JPC_Vec3 scale,
                                                        const JPC_RMat44 *transform, const JPC_CollideShapeSettings *settings,
                                                        const JPC_QueryFilter *filter, JPC_ShapeCollectorFn collector, void *user_data);
JPC_API void JPC_PhysicsSystem_CastShapeWithSettings(const JPC_PhysicsSystem *system, const JPC_Shape *shape, JPC_Vec3 scale,
                                                     const JPC_RMat44 *transform, JPC_Vec3 direction, const JPC_ShapeCastSettings *settings,
                                                     const JPC_QueryFilter *filter, JPC_ShapeCollectorFn collector, void *user_data);

/* Broad phase queries test against body bounding boxes only: cheap, but hits may not actually touch the shapes.
 * Same threading rules as the other collectors (called under the broad phase query lock). */
typedef bool (*JPC_BodyCollectorFn)(void *user_data, JPC_BodyID body_id);
typedef struct JPC_BroadPhaseCastHit {
    JPC_BodyID body_id;
    float fraction;
} JPC_BroadPhaseCastHit;
typedef bool (*JPC_BroadPhaseCastCollectorFn)(void *user_data, const JPC_BroadPhaseCastHit *hit);

JPC_API void JPC_PhysicsSystem_BroadPhaseCollideAABox(const JPC_PhysicsSystem *system, const JPC_AABox *box, const JPC_QueryFilter *filter,
                                                      JPC_BodyCollectorFn collector, void *user_data);
JPC_API void JPC_PhysicsSystem_BroadPhaseCollideSphere(const JPC_PhysicsSystem *system, JPC_Vec3 center, float radius, const JPC_QueryFilter *filter,
                                                       JPC_BodyCollectorFn collector, void *user_data);
JPC_API void JPC_PhysicsSystem_BroadPhaseCollidePoint(const JPC_PhysicsSystem *system, JPC_Vec3 point, const JPC_QueryFilter *filter,
                                                      JPC_BodyCollectorFn collector, void *user_data);
/* orientation's rotation and translation place a box of half_extent. */
JPC_API void JPC_PhysicsSystem_BroadPhaseCollideOrientedBox(const JPC_PhysicsSystem *system, const JPC_Mat44 *orientation, JPC_Vec3 half_extent,
                                                            const JPC_QueryFilter *filter, JPC_BodyCollectorFn collector, void *user_data);
JPC_API void JPC_PhysicsSystem_BroadPhaseCastRay(const JPC_PhysicsSystem *system, JPC_Vec3 origin, JPC_Vec3 direction, const JPC_QueryFilter *filter,
                                                 JPC_BroadPhaseCastCollectorFn collector, void *user_data);
JPC_API void JPC_PhysicsSystem_BroadPhaseCastAABox(const JPC_PhysicsSystem *system, const JPC_AABox *box, JPC_Vec3 direction, const JPC_QueryFilter *filter,
                                                   JPC_BroadPhaseCastCollectorFn collector, void *user_data);

/* Sweeps the shape from transform along direction. */
JPC_API void JPC_PhysicsSystem_CastShape(const JPC_PhysicsSystem *system, const JPC_Shape *shape, JPC_Vec3 scale,
                                         const JPC_RMat44 *transform, JPC_Vec3 direction,
                                         const JPC_QueryFilter *filter, JPC_ShapeCollectorFn collector, void *user_data);

/* ------------------------------------------------------------------------- */
/* Height fields                                                              */
/* ------------------------------------------------------------------------- */

typedef struct JPC_HeightFieldSettings {
    /* sample_count * sample_count heights, row by row; FLT_MAX makes a hole. */
    const float *samples;
    uint32_t sample_count;
    JPC_Vec3 offset;
    JPC_Vec3 scale;
    /* Range heights can be edited within later. min > max (the default) takes the range of the samples. */
    float min_height_value;
    float max_height_value;
    /* 2 to 8; samples are compressed and culled per block, and edits work on whole blocks. */
    uint32_t block_size;
    /* 1 to 16. */
    uint32_t bits_per_sample;
    /* Optional: (sample_count - 1)^2 indices into materials (at most 256). */
    const uint8_t *material_indices;
    const JPC_PhysicsMaterial *const *materials;
    uint32_t num_materials;
} JPC_HeightFieldSettings;

JPC_API void JPC_HeightFieldSettings_SetDefault(JPC_HeightFieldSettings *settings);
JPC_API JPC_Shape *JPC_HeightFieldShape_CreateWithSettings(const JPC_HeightFieldSettings *settings);

typedef struct JPC_HeightFieldInfo {
    /* Rounded up to a multiple of the block size. */
    uint32_t sample_count;
    uint32_t block_size;
    /* The range edits are clamped to. */
    float min_height_value;
    float max_height_value;
    uint32_t num_materials;
    /* Whether the heights can be edited (a height field created entirely of holes stores no samples). */
    bool editable;
} JPC_HeightFieldInfo;

/* Return false (with an error) for a shape that is not a height field or a region out of range. */
JPC_API bool JPC_HeightFieldShape_GetInfo(const JPC_Shape *shape, JPC_HeightFieldInfo *out_info);
/* Local space position of sample (x, y); out_no_collision tells whether it is a hole. */
JPC_API bool JPC_HeightFieldShape_GetPosition(const JPC_Shape *shape, uint32_t x, uint32_t y, JPC_Vec3 *out_position, bool *out_no_collision);
/* Reads size_x * size_y heights (holes are FLT_MAX) starting at sample (x, y), row by row. x, y and the sizes must be
 * multiples of the block size. */
JPC_API bool JPC_HeightFieldShape_GetHeights(const JPC_Shape *shape, uint32_t x, uint32_t y, uint32_t size_x, uint32_t size_y, float *out_heights);
/* Reads size_x * size_y material indices of cells starting at cell (x, y); cells go up to sample_count - 2. */
JPC_API bool JPC_HeightFieldShape_GetMaterialIndices(const JPC_Shape *shape, uint32_t x, uint32_t y, uint32_t size_x, uint32_t size_y, uint8_t *out_indices);
/* Edits size_x * size_y heights (FLT_MAX for holes, others clamped to the height range), with the same alignment rules
 * as GetHeights. Like the mutable compound functions, only call it on a shape nothing else uses at the same time:
 * one held by a single owner, or inside JPC_BodyInterface_ModifyHeightField. */
JPC_API bool JPC_HeightFieldShape_SetHeights(JPC_Shape *shape, uint32_t x, uint32_t y, uint32_t size_x, uint32_t size_y, const float *heights,
                                             JPC_TempAllocator *temp_allocator);
/* Sets material indices of cells; indices must refer to the height field's materials. */
JPC_API bool JPC_HeightFieldShape_SetMaterialIndices(JPC_Shape *shape, uint32_t x, uint32_t y, uint32_t size_x, uint32_t size_y, const uint8_t *indices,
                                                     JPC_TempAllocator *temp_allocator);

typedef void (*JPC_HeightFieldEditFn)(void *user_data, JPC_Shape *height_field);

/* Locks the body and calls fn with its height field to edit, then updates the body's bounds and contact cache. If
 * anything else holds the shape the body first gets its own copy. Fails (without calling fn) for bodies that don't
 * exist or have another shape. Bodies resting on the terrain are not woken: use JPC_BodyInterface_ActivateBodiesInAABox.
 * fn must not use a body interface. */
JPC_API bool JPC_BodyInterface_ModifyHeightField(JPC_BodyInterface *bi, JPC_BodyID id, JPC_HeightFieldEditFn fn, void *user_data);

/* Wakes every body in the world whose bounds overlap the box (filter may be NULL; only its object layer is used). */
JPC_API void JPC_BodyInterface_ActivateBodiesInAABox(JPC_BodyInterface *bi, const JPC_AABox *box, const JPC_QueryFilter *filter);

/* ------------------------------------------------------------------------- */
/* Shapes without a world                                                     */
/* ------------------------------------------------------------------------- */

/* Return false to stop. vertices holds 3 * count positions; materials count entries (NULL for the default material). */
typedef bool (*JPC_TriangleCollectorFn)(void *user_data, const JPC_RVec3 *vertices, const JPC_PhysicsMaterial *const *materials, uint32_t count);

/* Triangulates the shape (convex shapes approximately) placed with its center of mass at position_com, optionally only
 * the part inside box. For rendering, navigation meshes and the like. */
JPC_API void JPC_Shape_GetTriangles(const JPC_Shape *shape, JPC_RVec3 position_com, JPC_Quat rotation, JPC_Vec3 scale, const JPC_AABox *box,
                                    JPC_TriangleCollectorFn collector, void *user_data);

/* Queries against a single shape placed with its center of mass at position_com, without a physics system. Hits report
 * JPC_BODY_ID_INVALID as body. Like the system's queries otherwise; settings may be NULL. Returns true on a hit, and
 * out_hit->material is a new reference. */
JPC_API bool JPC_Shape_CastRay(const JPC_Shape *shape, JPC_RVec3 position_com, JPC_Quat rotation, JPC_Vec3 scale, JPC_RVec3 origin, JPC_Vec3 direction,
                               const JPC_RayCastSettings *settings, JPC_RayCastHit *out_hit);
JPC_API bool JPC_Shape_ContainsPoint(const JPC_Shape *shape, JPC_RVec3 position_com, JPC_Quat rotation, JPC_Vec3 scale, JPC_RVec3 point);
JPC_API void JPC_Shape_CollideShape(const JPC_Shape *shape, JPC_RVec3 position_com, JPC_Quat rotation, JPC_Vec3 scale, const JPC_Shape *query_shape,
                                    JPC_Vec3 query_scale, const JPC_RMat44 *query_transform, const JPC_CollideShapeSettings *settings,
                                    JPC_ShapeCollectorFn collector, void *user_data);
JPC_API void JPC_Shape_CastShape(const JPC_Shape *shape, JPC_RVec3 position_com, JPC_Quat rotation, JPC_Vec3 scale, const JPC_Shape *query_shape,
                                 JPC_Vec3 query_scale, const JPC_RMat44 *query_transform, JPC_Vec3 direction, const JPC_ShapeCastSettings *settings,
                                 JPC_ShapeCollectorFn collector, void *user_data);

/* ------------------------------------------------------------------------- */
/* Snapshots                                                                  */
/*                                                                            */
/* Jolt restores state without validating it against the world. A snapshot   */
/* taken with JPC_PhysicsSystem_SaveState can only be restored onto the same  */
/* system while no body, shape, motion type or constraint was added, removed */
/* or changed since; otherwise restoring fails. Snapshots made from raw bytes */
/* skip that check, so only feed them data saved from an identical world.    */
/* Characters are saved separately (JPC_CharacterVirtual_SaveState) and     */
/* their snapshots restore only onto the character they came from.           */
/* ------------------------------------------------------------------------- */

JPC_API JPC_PhysicsSnapshot *JPC_PhysicsSystem_SaveState(const JPC_PhysicsSystem *system, JPC_StateParts parts);
/* Must not run concurrently with anything else using the system. Returns false (with an error) on mismatch;
 * a failure part way through may leave the world partly restored. */
JPC_API bool JPC_PhysicsSystem_RestoreState(JPC_PhysicsSystem *system, const JPC_PhysicsSnapshot *snapshot);
JPC_API JPC_PhysicsSnapshot *JPC_PhysicsSnapshot_CreateFromData(const void *data, size_t size);
JPC_API const void *JPC_PhysicsSnapshot_GetData(const JPC_PhysicsSnapshot *snapshot);
JPC_API size_t JPC_PhysicsSnapshot_GetSize(const JPC_PhysicsSnapshot *snapshot);
JPC_API void JPC_PhysicsSnapshot_Destroy(JPC_PhysicsSnapshot *snapshot);

/* ------------------------------------------------------------------------- */
/* Constraints                                                                */
/* ------------------------------------------------------------------------- */

typedef struct JPC_SpringSettings {
    JPC_SpringMode mode;
    /* Frequency (Hz) or stiffness (N/m), depending on mode. */
    float frequency_or_stiffness;
    float damping;
} JPC_SpringSettings;

typedef struct JPC_MotorSettings {
    JPC_SpringSettings spring;
    float min_force_limit;
    float max_force_limit;
    float min_torque_limit;
    float max_torque_limit;
} JPC_MotorSettings;

/* Fields shared by every constraint. */
typedef struct JPC_ConstraintSettings {
    bool enabled;
    uint32_t constraint_priority;
    uint32_t num_velocity_steps_override;
    uint32_t num_position_steps_override;
    uint64_t user_data;
} JPC_ConstraintSettings;

typedef struct JPC_FixedConstraintSettings {
    JPC_ConstraintSettings base;
    JPC_ConstraintSpace space;
    bool auto_detect_point;
    JPC_RVec3 point1;
    JPC_Vec3 axis_x1;
    JPC_Vec3 axis_y1;
    JPC_RVec3 point2;
    JPC_Vec3 axis_x2;
    JPC_Vec3 axis_y2;
} JPC_FixedConstraintSettings;

typedef struct JPC_PointConstraintSettings {
    JPC_ConstraintSettings base;
    JPC_ConstraintSpace space;
    JPC_RVec3 point1;
    JPC_RVec3 point2;
} JPC_PointConstraintSettings;

typedef struct JPC_DistanceConstraintSettings {
    JPC_ConstraintSettings base;
    JPC_ConstraintSpace space;
    JPC_RVec3 point1;
    JPC_RVec3 point2;
    /* Negative values mean "use the initial distance". */
    float min_distance;
    float max_distance;
    JPC_SpringSettings limits_spring_settings;
} JPC_DistanceConstraintSettings;

typedef struct JPC_HingeConstraintSettings {
    JPC_ConstraintSettings base;
    JPC_ConstraintSpace space;
    JPC_RVec3 point1;
    JPC_Vec3 hinge_axis1;
    JPC_Vec3 normal_axis1;
    JPC_RVec3 point2;
    JPC_Vec3 hinge_axis2;
    JPC_Vec3 normal_axis2;
    float limits_min;
    float limits_max;
    JPC_SpringSettings limits_spring_settings;
    float max_friction_torque;
    JPC_MotorSettings motor_settings;
} JPC_HingeConstraintSettings;

typedef struct JPC_SliderConstraintSettings {
    JPC_ConstraintSettings base;
    JPC_ConstraintSpace space;
    bool auto_detect_point;
    JPC_RVec3 point1;
    JPC_Vec3 slider_axis1;
    JPC_Vec3 normal_axis1;
    JPC_RVec3 point2;
    JPC_Vec3 slider_axis2;
    JPC_Vec3 normal_axis2;
    float limits_min;
    float limits_max;
    JPC_SpringSettings limits_spring_settings;
    float max_friction_force;
    JPC_MotorSettings motor_settings;
} JPC_SliderConstraintSettings;

typedef struct JPC_ConeConstraintSettings {
    JPC_ConstraintSettings base;
    JPC_ConstraintSpace space;
    JPC_RVec3 point1;
    JPC_Vec3 twist_axis1;
    JPC_RVec3 point2;
    JPC_Vec3 twist_axis2;
    float half_cone_angle;
} JPC_ConeConstraintSettings;

typedef struct JPC_SwingTwistConstraintSettings {
    JPC_ConstraintSettings base;
    JPC_ConstraintSpace space;
    JPC_RVec3 position1;
    JPC_Vec3 twist_axis1;
    JPC_Vec3 plane_axis1;
    JPC_RVec3 position2;
    JPC_Vec3 twist_axis2;
    JPC_Vec3 plane_axis2;
    JPC_SwingType swing_type;
    float normal_half_cone_angle;
    float plane_half_cone_angle;
    float twist_min_angle;
    float twist_max_angle;
    float max_friction_torque;
    JPC_MotorSettings swing_motor_settings;
    JPC_MotorSettings twist_motor_settings;
} JPC_SwingTwistConstraintSettings;

/* Arrays are indexed by JPC_SixDOFAxis. limit_min > limit_max fixes an axis; -FLT_MAX..FLT_MAX frees it. */
typedef struct JPC_SixDOFConstraintSettings {
    JPC_ConstraintSettings base;
    JPC_ConstraintSpace space;
    JPC_RVec3 position1;
    JPC_Vec3 axis_x1;
    JPC_Vec3 axis_y1;
    JPC_RVec3 position2;
    JPC_Vec3 axis_x2;
    JPC_Vec3 axis_y2;
    float max_friction[6];
    JPC_SwingType swing_type;
    float limit_min[6];
    float limit_max[6];
    /* Translation axes only. */
    JPC_SpringSettings limits_spring_settings[3];
    JPC_MotorSettings motor_settings[6];
} JPC_SixDOFConstraintSettings;

typedef struct JPC_GearConstraintSettings {
    JPC_ConstraintSettings base;
    JPC_ConstraintSpace space;
    JPC_Vec3 hinge_axis1;
    JPC_Vec3 hinge_axis2;
    /* Teeth of gear 2 / teeth of gear 1: body 2 turns 1 / ratio as fast as body 1, in the opposite direction. */
    float ratio;
} JPC_GearConstraintSettings;

typedef struct JPC_RackAndPinionConstraintSettings {
    JPC_ConstraintSettings base;
    JPC_ConstraintSpace space;
    /* Body 1 is the pinion (rotating), body 2 the rack (sliding). */
    JPC_Vec3 hinge_axis;
    JPC_Vec3 slider_axis;
    /* Pinion rotation (radians) per meter of rack travel. */
    float ratio;
} JPC_RackAndPinionConstraintSettings;

typedef struct JPC_PulleyConstraintSettings {
    JPC_ConstraintSettings base;
    JPC_ConstraintSpace space;
    JPC_RVec3 body_point1;
    JPC_RVec3 fixed_point1;
    JPC_RVec3 body_point2;
    JPC_RVec3 fixed_point2;
    float ratio;
    /* Negative max_length means "the initial length". */
    float min_length;
    float max_length;
} JPC_PulleyConstraintSettings;

typedef struct JPC_PathConstraintSettings {
    JPC_ConstraintSettings base;
    /* Borrowed; the constraint keeps its own reference. Required. */
    const JPC_PathConstraintPath *path;
    /* Placement of the path relative to body 1. */
    JPC_Vec3 path_position;
    JPC_Quat path_rotation;
    float path_fraction;
    float max_friction_force;
    JPC_MotorSettings position_motor_settings;
    JPC_PathRotationConstraintType rotation_constraint_type;
} JPC_PathConstraintSettings;

typedef struct JPC_PathPoint {
    JPC_Vec3 position;
    JPC_Vec3 tangent;
    JPC_Vec3 normal;
} JPC_PathPoint;

/* A Hermite spline through at least 2 points. Reference counted and immutable. */
JPC_API JPC_PathConstraintPath *JPC_PathConstraintPathHermite_Create(const JPC_PathPoint *points, uint32_t num_points, bool looping);
JPC_API void JPC_PathConstraintPath_AddRef(const JPC_PathConstraintPath *path);
JPC_API void JPC_PathConstraintPath_Release(const JPC_PathConstraintPath *path);
JPC_API float JPC_PathConstraintPath_GetMaxFraction(const JPC_PathConstraintPath *path);

JPC_API void JPC_FixedConstraintSettings_SetDefault(JPC_FixedConstraintSettings *settings);
JPC_API void JPC_GearConstraintSettings_SetDefault(JPC_GearConstraintSettings *settings);
JPC_API void JPC_RackAndPinionConstraintSettings_SetDefault(JPC_RackAndPinionConstraintSettings *settings);
JPC_API void JPC_PulleyConstraintSettings_SetDefault(JPC_PulleyConstraintSettings *settings);
JPC_API void JPC_PathConstraintSettings_SetDefault(JPC_PathConstraintSettings *settings);
JPC_API void JPC_SwingTwistConstraintSettings_SetDefault(JPC_SwingTwistConstraintSettings *settings);
JPC_API void JPC_SixDOFConstraintSettings_SetDefault(JPC_SixDOFConstraintSettings *settings);
JPC_API void JPC_PointConstraintSettings_SetDefault(JPC_PointConstraintSettings *settings);
JPC_API void JPC_DistanceConstraintSettings_SetDefault(JPC_DistanceConstraintSettings *settings);
JPC_API void JPC_HingeConstraintSettings_SetDefault(JPC_HingeConstraintSettings *settings);
JPC_API void JPC_SliderConstraintSettings_SetDefault(JPC_SliderConstraintSettings *settings);
JPC_API void JPC_ConeConstraintSettings_SetDefault(JPC_ConeConstraintSettings *settings);

/* Pass JPC_BODY_ID_INVALID for one of the bodies to attach to the world. Returns NULL if a given id does not
 * refer to a live body, both ids refer to the same body, or a body is owned by a character.
 * The constraint is not simulated until it is added with JPC_PhysicsSystem_AddConstraint. */
JPC_API JPC_Constraint *JPC_PhysicsSystem_CreateFixedConstraint(JPC_PhysicsSystem *system, const JPC_FixedConstraintSettings *settings, JPC_BodyID body1, JPC_BodyID body2);
JPC_API JPC_Constraint *JPC_PhysicsSystem_CreatePointConstraint(JPC_PhysicsSystem *system, const JPC_PointConstraintSettings *settings, JPC_BodyID body1, JPC_BodyID body2);
JPC_API JPC_Constraint *JPC_PhysicsSystem_CreateDistanceConstraint(JPC_PhysicsSystem *system, const JPC_DistanceConstraintSettings *settings, JPC_BodyID body1, JPC_BodyID body2);
JPC_API JPC_Constraint *JPC_PhysicsSystem_CreateHingeConstraint(JPC_PhysicsSystem *system, const JPC_HingeConstraintSettings *settings, JPC_BodyID body1, JPC_BodyID body2);
JPC_API JPC_Constraint *JPC_PhysicsSystem_CreateSliderConstraint(JPC_PhysicsSystem *system, const JPC_SliderConstraintSettings *settings, JPC_BodyID body1, JPC_BodyID body2);
JPC_API JPC_Constraint *JPC_PhysicsSystem_CreateSwingTwistConstraint(JPC_PhysicsSystem *system, const JPC_SwingTwistConstraintSettings *settings, JPC_BodyID body1, JPC_BodyID body2);
JPC_API JPC_Constraint *JPC_PhysicsSystem_CreateSixDOFConstraint(JPC_PhysicsSystem *system, const JPC_SixDOFConstraintSettings *settings, JPC_BodyID body1, JPC_BodyID body2);
JPC_API JPC_Constraint *JPC_PhysicsSystem_CreateGearConstraint(JPC_PhysicsSystem *system, const JPC_GearConstraintSettings *settings, JPC_BodyID body1, JPC_BodyID body2);
JPC_API JPC_Constraint *JPC_PhysicsSystem_CreateRackAndPinionConstraint(JPC_PhysicsSystem *system, const JPC_RackAndPinionConstraintSettings *settings, JPC_BodyID body1, JPC_BodyID body2);
JPC_API JPC_Constraint *JPC_PhysicsSystem_CreatePulleyConstraint(JPC_PhysicsSystem *system, const JPC_PulleyConstraintSettings *settings, JPC_BodyID body1, JPC_BodyID body2);
JPC_API JPC_Constraint *JPC_PhysicsSystem_CreatePathConstraint(JPC_PhysicsSystem *system, const JPC_PathConstraintSettings *settings, JPC_BodyID body1, JPC_BodyID body2);
JPC_API JPC_Constraint *JPC_PhysicsSystem_CreateConeConstraint(JPC_PhysicsSystem *system, const JPC_ConeConstraintSettings *settings, JPC_BodyID body1, JPC_BodyID body2);
/* Wakes both bodies so a newly added or modified constraint takes effect. */
JPC_API void JPC_BodyInterface_ActivateConstraint(JPC_BodyInterface *bi, const JPC_Constraint *constraint);

JPC_API JPC_ConstraintType JPC_Constraint_GetType(const JPC_Constraint *constraint);
JPC_API void JPC_Constraint_AddRef(const JPC_Constraint *constraint);
JPC_API void JPC_Constraint_Release(const JPC_Constraint *constraint);
JPC_API bool JPC_Constraint_GetEnabled(const JPC_Constraint *constraint);
JPC_API void JPC_Constraint_SetEnabled(JPC_Constraint *constraint, bool enabled);
JPC_API uint64_t JPC_Constraint_GetUserData(const JPC_Constraint *constraint);
JPC_API void JPC_Constraint_SetUserData(JPC_Constraint *constraint, uint64_t user_data);
/* JPC_BODY_ID_INVALID when attached to the world. */
JPC_API JPC_BodyID JPC_Constraint_GetBody1(const JPC_Constraint *constraint);
JPC_API JPC_BodyID JPC_Constraint_GetBody2(const JPC_Constraint *constraint);

/* The following must only be called on constraints of the matching kind. */
JPC_API float JPC_HingeConstraint_GetCurrentAngle(const JPC_Constraint *constraint);
JPC_API void JPC_HingeConstraint_SetLimits(JPC_Constraint *constraint, float min, float max);
JPC_API void JPC_HingeConstraint_SetMaxFrictionTorque(JPC_Constraint *constraint, float torque);
JPC_API void JPC_HingeConstraint_SetMotorState(JPC_Constraint *constraint, JPC_MotorState state);
JPC_API void JPC_HingeConstraint_SetTargetAngularVelocity(JPC_Constraint *constraint, float angular_velocity);
JPC_API void JPC_HingeConstraint_SetTargetAngle(JPC_Constraint *constraint, float angle);

JPC_API float JPC_SliderConstraint_GetCurrentPosition(const JPC_Constraint *constraint);
JPC_API void JPC_SliderConstraint_SetLimits(JPC_Constraint *constraint, float min, float max);
JPC_API void JPC_SliderConstraint_SetMaxFrictionForce(JPC_Constraint *constraint, float force);
JPC_API void JPC_SliderConstraint_SetMotorState(JPC_Constraint *constraint, JPC_MotorState state);
JPC_API void JPC_SliderConstraint_SetTargetVelocity(JPC_Constraint *constraint, float velocity);
JPC_API void JPC_SliderConstraint_SetTargetPosition(JPC_Constraint *constraint, float position);

JPC_API void JPC_DistanceConstraint_SetDistance(JPC_Constraint *constraint, float min_distance, float max_distance);
JPC_API void JPC_ConeConstraint_SetHalfConeAngle(JPC_Constraint *constraint, float half_cone_angle);

JPC_API void JPC_PulleyConstraint_SetLength(JPC_Constraint *constraint, float min_length, float max_length);
JPC_API float JPC_PulleyConstraint_GetCurrentLength(const JPC_Constraint *constraint);
JPC_API float JPC_PathConstraint_GetPathFraction(const JPC_Constraint *constraint);
JPC_API void JPC_PathConstraint_SetMaxFrictionForce(JPC_Constraint *constraint, float force);
JPC_API void JPC_PathConstraint_SetPositionMotorState(JPC_Constraint *constraint, JPC_MotorState state);
JPC_API void JPC_PathConstraint_SetTargetVelocity(JPC_Constraint *constraint, float velocity);
/* Clamped to the path's range unless the path loops. */
JPC_API void JPC_PathConstraint_SetTargetPathFraction(JPC_Constraint *constraint, float fraction);

JPC_API void JPC_SwingTwistConstraint_SetNormalHalfConeAngle(JPC_Constraint *constraint, float angle);
JPC_API void JPC_SwingTwistConstraint_SetPlaneHalfConeAngle(JPC_Constraint *constraint, float angle);
JPC_API void JPC_SwingTwistConstraint_SetTwistMinAngle(JPC_Constraint *constraint, float angle);
JPC_API void JPC_SwingTwistConstraint_SetTwistMaxAngle(JPC_Constraint *constraint, float angle);
JPC_API void JPC_SwingTwistConstraint_SetMaxFrictionTorque(JPC_Constraint *constraint, float torque);
JPC_API void JPC_SwingTwistConstraint_SetSwingMotorState(JPC_Constraint *constraint, JPC_MotorState state);
JPC_API void JPC_SwingTwistConstraint_SetTwistMotorState(JPC_Constraint *constraint, JPC_MotorState state);
/* CS = constraint space: relative to body 1's constraint frame. */
JPC_API void JPC_SwingTwistConstraint_SetTargetAngularVelocityCS(JPC_Constraint *constraint, JPC_Vec3 angular_velocity);
JPC_API void JPC_SwingTwistConstraint_SetTargetOrientationCS(JPC_Constraint *constraint, JPC_Quat orientation);
JPC_API JPC_Quat JPC_SwingTwistConstraint_GetRotationInConstraintSpace(const JPC_Constraint *constraint);

JPC_API void JPC_SixDOFConstraint_SetTranslationLimits(JPC_Constraint *constraint, JPC_Vec3 min, JPC_Vec3 max);
JPC_API void JPC_SixDOFConstraint_SetRotationLimits(JPC_Constraint *constraint, JPC_Vec3 min, JPC_Vec3 max);
JPC_API void JPC_SixDOFConstraint_SetMaxFriction(JPC_Constraint *constraint, JPC_SixDOFAxis axis, float friction);
JPC_API void JPC_SixDOFConstraint_SetMotorState(JPC_Constraint *constraint, JPC_SixDOFAxis axis, JPC_MotorState state);
JPC_API void JPC_SixDOFConstraint_SetTargetVelocityCS(JPC_Constraint *constraint, JPC_Vec3 velocity);
JPC_API void JPC_SixDOFConstraint_SetTargetAngularVelocityCS(JPC_Constraint *constraint, JPC_Vec3 angular_velocity);
JPC_API void JPC_SixDOFConstraint_SetTargetPositionCS(JPC_Constraint *constraint, JPC_Vec3 position);
JPC_API void JPC_SixDOFConstraint_SetTargetOrientationCS(JPC_Constraint *constraint, JPC_Quat orientation);
JPC_API JPC_Quat JPC_SixDOFConstraint_GetRotationInConstraintSpace(const JPC_Constraint *constraint);

/* ------------------------------------------------------------------------- */
/* Vehicles                                                                   */
/*                                                                            */
/* A vehicle is a constraint on its body; add it with                         */
/* JPC_PhysicsSystem_AddConstraint. Vehicles and step listeners of a system   */
/* run one after another, so they never race each other.                     */
/* ------------------------------------------------------------------------- */

typedef struct JPC_LinearCurvePoint {
    float x;
    float y;
} JPC_LinearCurvePoint;

typedef struct JPC_WheelSettingsWV {
    /* In the body's local space. */
    JPC_Vec3 position;
    JPC_Vec3 suspension_force_point;
    JPC_Vec3 suspension_direction;
    JPC_Vec3 steering_axis;
    JPC_Vec3 wheel_up;
    JPC_Vec3 wheel_forward;
    float suspension_min_length;
    float suspension_max_length;
    float suspension_preload_length;
    JPC_SpringSettings suspension_spring;
    float radius;
    float width;
    bool enable_suspension_force_point;
    float inertia;
    float angular_damping;
    float max_steer_angle;
    /* Friction as a function of slip ratio / slip angle (degrees). NULL keeps Jolt's default curve. */
    const JPC_LinearCurvePoint *longitudinal_friction;
    uint32_t num_longitudinal_friction;
    const JPC_LinearCurvePoint *lateral_friction;
    uint32_t num_lateral_friction;
    float max_brake_torque;
    float max_hand_brake_torque;
} JPC_WheelSettingsWV;

typedef struct JPC_VehicleEngineSettings {
    float max_torque;
    float min_rpm;
    float max_rpm;
    /* Torque fraction as a function of RPM fraction; NULL keeps the default (flat) curve. */
    const JPC_LinearCurvePoint *normalized_torque;
    uint32_t num_normalized_torque;
    float inertia;
    float angular_damping;
} JPC_VehicleEngineSettings;

typedef struct JPC_VehicleTransmissionSettings {
    JPC_TransmissionMode mode;
    /* At least one of each. */
    const float *gear_ratios;
    uint32_t num_gear_ratios;
    const float *reverse_gear_ratios;
    uint32_t num_reverse_gear_ratios;
    float switch_time;
    float clutch_release_time;
    float switch_latency;
    float shift_up_rpm;
    float shift_down_rpm;
    float clutch_strength;
} JPC_VehicleTransmissionSettings;

typedef struct JPC_VehicleDifferentialSettings {
    /* Wheel indices, or -1 for none. */
    int32_t left_wheel;
    int32_t right_wheel;
    float differential_ratio;
    float left_right_split;
    float limited_slip_ratio;
    float engine_torque_ratio;
} JPC_VehicleDifferentialSettings;

typedef struct JPC_VehicleAntiRollBar {
    int32_t left_wheel;
    int32_t right_wheel;
    float stiffness;
} JPC_VehicleAntiRollBar;

typedef struct JPC_WheeledVehicleSettings {
    JPC_ConstraintSettings base;
    /* In the body's local space. */
    JPC_Vec3 up;
    JPC_Vec3 forward;
    float max_pitch_roll_angle;
    const JPC_WheelSettingsWV *wheels;
    uint32_t num_wheels;
    const JPC_VehicleAntiRollBar *anti_roll_bars;
    uint32_t num_anti_roll_bars;
    JPC_VehicleEngineSettings engine;
    JPC_VehicleTransmissionSettings transmission;
    const JPC_VehicleDifferentialSettings *differentials;
    uint32_t num_differentials;
    float differential_limited_slip_ratio;
} JPC_WheeledVehicleSettings;

typedef struct JPC_WheelState {
    bool has_contact;
    JPC_BodyID contact_body_id;
    JPC_SubShapeID contact_sub_shape_id;
    /* Contact fields are zero without contact. */
    JPC_RVec3 contact_position;
    JPC_Vec3 contact_normal;
    JPC_Vec3 contact_point_velocity;
    float suspension_length;
    float angular_velocity;
    float rotation_angle;
    float steer_angle;
    float longitudinal_slip;
    float lateral_slip;
} JPC_WheelState;

/* Fill with Jolt's defaults. Curve and gear pointers point at static data owned by joltc. */
JPC_API void JPC_WheelSettingsWV_SetDefault(JPC_WheelSettingsWV *settings);
JPC_API void JPC_WheeledVehicleSettings_SetDefault(JPC_WheeledVehicleSettings *settings);
JPC_API void JPC_VehicleDifferentialSettings_SetDefault(JPC_VehicleDifferentialSettings *settings);

/* How wheels find the ground. Reference counted. layer is the layer the wheels collide as. Wheels ignore their own
 * vehicle's body and soft bodies. */
JPC_API JPC_VehicleCollisionTester *JPC_VehicleCollisionTesterRay_Create(const JPC_PhysicsSystem *system, JPC_ObjectLayer layer, JPC_Vec3 up, float max_slope_angle);
JPC_API JPC_VehicleCollisionTester *JPC_VehicleCollisionTesterCastSphere_Create(const JPC_PhysicsSystem *system, JPC_ObjectLayer layer, float radius,
                                                                                JPC_Vec3 up, float max_slope_angle);
JPC_API JPC_VehicleCollisionTester *JPC_VehicleCollisionTesterCastCylinder_Create(const JPC_PhysicsSystem *system, JPC_ObjectLayer layer, float convex_radius_fraction);
JPC_API void JPC_VehicleCollisionTester_AddRef(const JPC_VehicleCollisionTester *tester);
JPC_API void JPC_VehicleCollisionTester_Release(const JPC_VehicleCollisionTester *tester);

/* Returns NULL (with an error) for invalid settings or a body that does not exist or is owned by a character. */
JPC_API JPC_Constraint *JPC_PhysicsSystem_CreateWheeledVehicle(JPC_PhysicsSystem *system, const JPC_WheeledVehicleSettings *settings, JPC_BodyID body,
                                                               const JPC_VehicleCollisionTester *tester);

/* A two wheeled vehicle that leans into turns. Usually wheel 0 is the front wheel and wheel 1 the rear wheel. */
typedef struct JPC_MotorcycleSettings {
    JPC_WheeledVehicleSettings vehicle;
    /* Radians. */
    float max_lean_angle;
    float lean_spring_constant;
    float lean_spring_damping;
    float lean_spring_integration_coefficient;
    float lean_spring_integration_coefficient_decay;
    /* 0 follows the target lean immediately, towards 1 smooths it more. */
    float lean_smoothing_factor;
} JPC_MotorcycleSettings;

JPC_API void JPC_MotorcycleSettings_SetDefault(JPC_MotorcycleSettings *settings);
JPC_API JPC_Constraint *JPC_PhysicsSystem_CreateMotorcycle(JPC_PhysicsSystem *system, const JPC_MotorcycleSettings *settings, JPC_BodyID body,
                                                           const JPC_VehicleCollisionTester *tester);

/* A wheel of a tracked vehicle. It does not steer or brake by itself; the track does. */
typedef struct JPC_WheelSettingsTV {
    JPC_Vec3 position;
    JPC_Vec3 suspension_force_point;
    JPC_Vec3 suspension_direction;
    JPC_Vec3 steering_axis;
    JPC_Vec3 wheel_up;
    JPC_Vec3 wheel_forward;
    float suspension_min_length;
    float suspension_max_length;
    float suspension_preload_length;
    JPC_SpringSettings suspension_spring;
    float radius;
    float width;
    bool enable_suspension_force_point;
    float longitudinal_friction;
    float lateral_friction;
} JPC_WheelSettingsTV;

typedef struct JPC_VehicleTrackSettings {
    /* The wheel the engine drives; must be one of wheels. */
    uint32_t driven_wheel;
    /* Indices of the wheels in this track. Every wheel of the vehicle must be in exactly one track. */
    const uint32_t *wheels;
    uint32_t num_wheels;
    float inertia;
    float angular_damping;
    float max_brake_torque;
    float differential_ratio;
} JPC_VehicleTrackSettings;

typedef struct JPC_TrackedVehicleSettings {
    JPC_ConstraintSettings base;
    JPC_Vec3 up;
    JPC_Vec3 forward;
    float max_pitch_roll_angle;
    const JPC_WheelSettingsTV *wheels;
    uint32_t num_wheels;
    const JPC_VehicleAntiRollBar *anti_roll_bars;
    uint32_t num_anti_roll_bars;
    JPC_VehicleEngineSettings engine;
    JPC_VehicleTransmissionSettings transmission;
    /* Left and right. */
    JPC_VehicleTrackSettings tracks[2];
} JPC_TrackedVehicleSettings;

/* Fill with Jolt's defaults (a tank-like engine and gear box, no wheels). Pointers point at static data owned by joltc. */
JPC_API void JPC_WheelSettingsTV_SetDefault(JPC_WheelSettingsTV *settings);
JPC_API void JPC_TrackedVehicleSettings_SetDefault(JPC_TrackedVehicleSettings *settings);
JPC_API JPC_Constraint *JPC_PhysicsSystem_CreateTrackedVehicle(JPC_PhysicsSystem *system, const JPC_TrackedVehicleSettings *settings, JPC_BodyID body,
                                                               const JPC_VehicleCollisionTester *tester);

/* The following must only be called on vehicle constraints. Functions for one kind of vehicle do nothing (or
 * return false / 0) for the others. */
JPC_API JPC_VehicleKind JPC_Vehicle_GetKind(const JPC_Constraint *vehicle);
/* Returns false (with an error) for a tester whose layer the vehicle's system doesn't have. */
JPC_API bool JPC_Vehicle_SetCollisionTester(JPC_Constraint *vehicle, const JPC_VehicleCollisionTester *tester);
/* Wheeled vehicles and motorcycles. forward and right in [-1, 1], brake and hand_brake in [0, 1]. */
JPC_API void JPC_Vehicle_SetDriverInput(JPC_Constraint *vehicle, float forward, float right, float brake, float hand_brake);
/* Tracked vehicles. forward in [-1, 1], brake in [0, 1]. The ratios in [-1, 1] scale each track's speed to steer
 * (e.g. 1 and -1 turn on the spot); ratios closer to 0 than 0.001 are pushed away from it. Returns false for other vehicles. */
JPC_API bool JPC_Vehicle_SetTrackInput(JPC_Constraint *vehicle, float forward, float left_ratio, float right_ratio, float brake);
/* Angular velocity of a track's driven wheel (rad/s); side 0 is left, 1 right. Returns false for other vehicles. */
JPC_API bool JPC_Vehicle_GetTrackAngularVelocity(const JPC_Constraint *vehicle, uint32_t side, float *out_angular_velocity);

/* Motorcycles. Setters return false for other vehicles. */
JPC_API bool JPC_Motorcycle_SetLeanControllerEnabled(JPC_Constraint *vehicle, bool enabled);
JPC_API bool JPC_Motorcycle_IsLeanControllerEnabled(const JPC_Constraint *vehicle);
/* Limits steering so the bike can't turn tighter than its lean angle allows. */
JPC_API bool JPC_Motorcycle_SetLeanSteeringLimitEnabled(JPC_Constraint *vehicle, bool enabled);
JPC_API bool JPC_Motorcycle_IsLeanSteeringLimitEnabled(const JPC_Constraint *vehicle);
JPC_API bool JPC_Motorcycle_SetLeanSpring(JPC_Constraint *vehicle, float spring_constant, float damping, float integration_coefficient,
                                          float integration_coefficient_decay, float smoothing_factor);
/* Distance between the front and rear wheel. */
JPC_API float JPC_Motorcycle_GetWheelBase(const JPC_Constraint *vehicle);

JPC_API uint32_t JPC_Vehicle_GetNumWheels(const JPC_Constraint *vehicle);
/* Returns false for an out of range wheel. */
JPC_API bool JPC_Vehicle_GetWheelState(const JPC_Constraint *vehicle, uint32_t wheel, JPC_WheelState *out_state);
/* Transform of a wheel model whose axle is wheel_right and top is wheel_up in its own space. */
JPC_API bool JPC_Vehicle_GetWheelWorldTransform(const JPC_Constraint *vehicle, uint32_t wheel, JPC_Vec3 wheel_right, JPC_Vec3 wheel_up, JPC_RMat44 *out_transform);
JPC_API float JPC_Vehicle_GetEngineRPM(const JPC_Constraint *vehicle);
/* Positive for forward gears, negative for reverse, 0 for neutral. */
JPC_API int32_t JPC_Vehicle_GetCurrentGear(const JPC_Constraint *vehicle);
JPC_API float JPC_Vehicle_GetClutchFriction(const JPC_Constraint *vehicle);
JPC_API bool JPC_Vehicle_IsSwitchingGear(const JPC_Constraint *vehicle);
/* Manual transmissions: select a gear in [-num_reverse, num_forward] and a clutch friction in [0, 1]. */
JPC_API bool JPC_Vehicle_SetTransmission(JPC_Constraint *vehicle, int32_t gear, float clutch_friction);
JPC_API void JPC_Vehicle_SetMaxPitchRollAngle(JPC_Constraint *vehicle, float angle);
JPC_API void JPC_Vehicle_SetNumStepsBetweenCollisionTests(JPC_Constraint *vehicle, uint32_t active, uint32_t inactive);
JPC_API void JPC_Vehicle_OverrideGravity(JPC_Constraint *vehicle, JPC_Vec3 gravity);
JPC_API void JPC_Vehicle_ResetGravityOverride(JPC_Constraint *vehicle);
JPC_API JPC_BodyID JPC_Vehicle_GetBody(const JPC_Constraint *vehicle);

/* ------------------------------------------------------------------------- */
/* Skeletons and ragdolls                                                     */
/* ------------------------------------------------------------------------- */

/* A joint hierarchy. Joints must be added parent first. Reference counted; build it completely before
 * handing it to ragdoll settings. */
JPC_API JPC_Skeleton *JPC_Skeleton_Create(void);
/* parent_index is -1 for a root. Returns the new joint's index, or -1 if parent_index is not an existing joint. */
JPC_API int32_t JPC_Skeleton_AddJoint(JPC_Skeleton *skeleton, const char *name, int32_t parent_index);
JPC_API uint32_t JPC_Skeleton_GetJointCount(const JPC_Skeleton *skeleton);
/* -1 if no joint has this name. */
JPC_API int32_t JPC_Skeleton_GetJointIndex(const JPC_Skeleton *skeleton, const char *name);
/* NUL terminated; valid as long as the skeleton. NULL for an out of range joint. */
JPC_API const char *JPC_Skeleton_GetJointName(const JPC_Skeleton *skeleton, uint32_t joint);
JPC_API int32_t JPC_Skeleton_GetParentIndex(const JPC_Skeleton *skeleton, uint32_t joint);
JPC_API void JPC_Skeleton_AddRef(const JPC_Skeleton *skeleton);
JPC_API void JPC_Skeleton_Release(const JPC_Skeleton *skeleton);

/* Local joint transforms (relative to the parent joint, or the model for roots). */
typedef struct JPC_JointState {
    JPC_Vec3 translation;
    JPC_Quat rotation;
} JPC_JointState;

/* Converts joint_count local states to model space matrices, and back. */
JPC_API void JPC_Skeleton_CalculateJointMatrices(const JPC_Skeleton *skeleton, const JPC_JointState *states, JPC_Mat44 *out_matrices);
JPC_API void JPC_Skeleton_CalculateJointStates(const JPC_Skeleton *skeleton, const JPC_Mat44 *matrices, JPC_JointState *out_states);

/* Settings for one of the supported joint types, selected by type. */
typedef struct JPC_TwoBodyConstraintSettings {
    JPC_ConstraintType type;
    union {
        JPC_FixedConstraintSettings fixed;
        JPC_PointConstraintSettings point;
        JPC_DistanceConstraintSettings distance;
        JPC_HingeConstraintSettings hinge;
        JPC_SliderConstraintSettings slider;
        JPC_ConeConstraintSettings cone;
        JPC_SwingTwistConstraintSettings swing_twist;
        JPC_SixDOFConstraintSettings six_dof;
    } settings;
} JPC_TwoBodyConstraintSettings;

typedef struct JPC_RagdollPart {
    JPC_BodyCreationSettings body;
    /* Joint to the parent part; type NONE for root parts (and only for them, or the part floats free). */
    JPC_TwoBodyConstraintSettings to_parent;
} JPC_RagdollPart;

typedef struct JPC_RagdollConstraint {
    uint32_t body1;
    uint32_t body2;
    JPC_TwoBodyConstraintSettings constraint;
} JPC_RagdollConstraint;

typedef struct JPC_RagdollSettingsDesc {
    const JPC_Skeleton *skeleton;
    /* One per joint, in joint order. */
    const JPC_RagdollPart *parts;
    uint32_t num_parts;
    /* Extra constraints between any two parts (e.g. to close a loop). */
    const JPC_RagdollConstraint *additional_constraints;
    uint32_t num_additional_constraints;
    /* Limit mass ratios between parents and children so the joints don't jitter. */
    bool stabilize;
    /* Stop parents and children colliding (and parts closer than min_separation_distance in the pose below). */
    bool disable_parent_child_collisions;
    /* Optional num_parts model space matrices for the separation test. */
    const JPC_Mat44 *pose_matrices;
    float min_separation_distance;
    /* Give joints near the root a higher priority. */
    bool calculate_constraint_priorities;
    uint32_t base_priority;
} JPC_RagdollSettingsDesc;

/* Immutable and reference counted; one settings object can create many ragdolls. NULL with an error for bad input. */
JPC_API JPC_RagdollSettings *JPC_RagdollSettings_Create(const JPC_RagdollSettingsDesc *desc);
JPC_API void JPC_RagdollSettings_AddRef(const JPC_RagdollSettings *settings);
JPC_API void JPC_RagdollSettings_Release(const JPC_RagdollSettings *settings);
/* Borrowed. */
JPC_API const JPC_Skeleton *JPC_RagdollSettings_GetSkeleton(const JPC_RagdollSettings *settings);

/* Creates the bodies and constraints, not yet in the world. Bodies get collision group id group_id.
 * The bodies belong to the ragdoll: they can't be destroyed through the body interface or used in other constraints. */
JPC_API JPC_Ragdoll *JPC_RagdollSettings_CreateRagdoll(const JPC_RagdollSettings *settings, JPC_PhysicsSystem *system, uint32_t group_id, uint64_t user_data);
/* Removes the ragdoll from the world if needed and destroys its bodies and constraints. */
JPC_API void JPC_Ragdoll_Destroy(JPC_Ragdoll *ragdoll);
JPC_API void JPC_Ragdoll_AddToPhysicsSystem(JPC_Ragdoll *ragdoll, JPC_Activation activation);
JPC_API void JPC_Ragdoll_RemoveFromPhysicsSystem(JPC_Ragdoll *ragdoll);
JPC_API bool JPC_Ragdoll_IsAdded(const JPC_Ragdoll *ragdoll);
JPC_API void JPC_Ragdoll_Activate(JPC_Ragdoll *ragdoll);
JPC_API bool JPC_Ragdoll_IsActive(const JPC_Ragdoll *ragdoll);
JPC_API void JPC_Ragdoll_SetGroupID(JPC_Ragdoll *ragdoll, uint32_t group_id);
/* matrices holds one model space matrix per joint. */
JPC_API void JPC_Ragdoll_SetPose(JPC_Ragdoll *ragdoll, JPC_RVec3 root_offset, const JPC_Mat44 *matrices);
JPC_API void JPC_Ragdoll_GetPose(JPC_Ragdoll *ragdoll, JPC_RVec3 *out_root_offset, JPC_Mat44 *out_matrices);
/* Moves the bodies towards the pose by setting velocities (for animated, not simulated, ragdolls). */
JPC_API void JPC_Ragdoll_DriveToPoseUsingKinematics(JPC_Ragdoll *ragdoll, JPC_RVec3 root_offset, const JPC_Mat44 *matrices, float delta_time);
/* Sets the joint motors (swing-twist and hinge joints) to reach the pose. */
JPC_API void JPC_Ragdoll_DriveToPoseUsingMotors(JPC_Ragdoll *ragdoll, JPC_RVec3 root_offset, const JPC_Mat44 *matrices);
JPC_API void JPC_Ragdoll_ResetWarmStart(JPC_Ragdoll *ragdoll);
JPC_API void JPC_Ragdoll_SetLinearAndAngularVelocity(JPC_Ragdoll *ragdoll, JPC_Vec3 linear, JPC_Vec3 angular);
JPC_API void JPC_Ragdoll_AddLinearVelocity(JPC_Ragdoll *ragdoll, JPC_Vec3 velocity);
JPC_API void JPC_Ragdoll_AddImpulse(JPC_Ragdoll *ragdoll, JPC_Vec3 impulse);
JPC_API void JPC_Ragdoll_GetRootTransform(const JPC_Ragdoll *ragdoll, JPC_RVec3 *out_position, JPC_Quat *out_rotation);
JPC_API JPC_AABox JPC_Ragdoll_GetWorldSpaceBounds(const JPC_Ragdoll *ragdoll);
JPC_API uint32_t JPC_Ragdoll_GetBodyCount(const JPC_Ragdoll *ragdoll);
JPC_API JPC_BodyID JPC_Ragdoll_GetBodyID(const JPC_Ragdoll *ragdoll, uint32_t index);
JPC_API uint32_t JPC_Ragdoll_GetConstraintCount(const JPC_Ragdoll *ragdoll);
/* Borrowed; valid as long as the ragdoll. Change it only while no update runs. */
JPC_API JPC_Constraint *JPC_Ragdoll_GetConstraint(JPC_Ragdoll *ragdoll, uint32_t index);
/* Borrowed. */
JPC_API const JPC_RagdollSettings *JPC_Ragdoll_GetSettings(const JPC_Ragdoll *ragdoll);

/* ------------------------------------------------------------------------- */
/* Skeletal animation                                                         */
/* ------------------------------------------------------------------------- */

typedef struct JPC_AnimationKeyframe {
    /* Seconds. */
    float time;
    /* Local to the parent joint, like JPC_JointState. */
    JPC_Vec3 translation;
    JPC_Quat rotation;
} JPC_AnimationKeyframe;

typedef struct JPC_AnimatedJoint {
    /* Matched by name against the skeleton when sampling. */
    const char *name;
    /* At least one, in time order. */
    const JPC_AnimationKeyframe *keyframes;
    uint32_t num_keyframes;
} JPC_AnimatedJoint;

/* Keyframed joint animation. Immutable and reference counted. Rotations are normalized. NULL with an error for a
 * joint without keyframes, keyframes out of time order, or a rotation of length 0. */
JPC_API JPC_SkeletalAnimation *JPC_SkeletalAnimation_Create(const JPC_AnimatedJoint *joints, uint32_t num_joints, bool looping);
JPC_API void JPC_SkeletalAnimation_AddRef(const JPC_SkeletalAnimation *animation);
JPC_API void JPC_SkeletalAnimation_Release(const JPC_SkeletalAnimation *animation);
/* Time of the last keyframe of the first animated joint, as Jolt defines it. */
JPC_API float JPC_SkeletalAnimation_GetDuration(const JPC_SkeletalAnimation *animation);
JPC_API bool JPC_SkeletalAnimation_IsLooping(const JPC_SkeletalAnimation *animation);
/* Overwrites the local states (one per skeleton joint) of the joints the animation animates, interpolating between
 * keyframes; looping animations wrap time around the duration, negative times count as 0. Returns false (and
 * changes nothing) if the animation animates a joint the skeleton doesn't have. */
JPC_API bool JPC_SkeletalAnimation_Sample(const JPC_SkeletalAnimation *animation, const JPC_Skeleton *skeleton, float time, JPC_JointState *io_states);

/* Maps poses between a low detail skeleton 1 (e.g. a ragdoll's) and a high detail skeleton 2 (e.g. an animated
 * character's), matching joints by name. Joints of 2 between mapped joints follow as chains; other joints of 2 keep
 * their animated local transform. Neutral poses are model space matrices, one per joint, in the same pose.
 * Optionally the translations of joints of 2 can be locked to their neutral offset from their parent, either per joint
 * (locked_translations, one per joint of 2; root joints can't be locked and are ignored) or for every joint below the
 * first mapped one (lock_all_translations). Immutable and reference counted. NULL with an error if skeleton 1 has
 * more joints than skeleton 2. */
JPC_API JPC_SkeletonMapper *JPC_SkeletonMapper_Create(const JPC_Skeleton *skeleton1, const JPC_Mat44 *neutral_pose1, const JPC_Skeleton *skeleton2,
                                                      const JPC_Mat44 *neutral_pose2, const bool *locked_translations, bool lock_all_translations);
JPC_API void JPC_SkeletonMapper_AddRef(const JPC_SkeletonMapper *mapper);
JPC_API void JPC_SkeletonMapper_Release(const JPC_SkeletonMapper *mapper);
JPC_API void JPC_SkeletonMapper_GetJointCounts(const JPC_SkeletonMapper *mapper, uint32_t *out_count1, uint32_t *out_count2);
/* pose1_model_space has a matrix per joint of 1, pose2_local_space and out_pose2_model_space one per joint of 2. */
JPC_API void JPC_SkeletonMapper_Map(const JPC_SkeletonMapper *mapper, const JPC_Mat44 *pose1_model_space, const JPC_Mat44 *pose2_local_space,
                                    JPC_Mat44 *out_pose2_model_space);
/* Only writes the joints of 1 that are mapped directly. */
JPC_API void JPC_SkeletonMapper_MapReverse(const JPC_SkeletonMapper *mapper, const JPC_Mat44 *pose2_model_space, JPC_Mat44 *out_pose1_model_space);
/* Joint of 2 that a joint of 1 maps to, or -1. */
JPC_API int32_t JPC_SkeletonMapper_GetMappedJointIndex(const JPC_SkeletonMapper *mapper, uint32_t joint1);
JPC_API bool JPC_SkeletonMapper_IsJointTranslationLocked(const JPC_SkeletonMapper *mapper, uint32_t joint2);

/* ------------------------------------------------------------------------- */
/* Characters                                                                 */
/*                                                                            */
/* Both character kinds keep a pointer to their physics system and must be    */
/* destroyed before it. Their bodies are protected: JPC_BodyInterface_       */
/* DestroyBody refuses to destroy them.                                       */
/* ------------------------------------------------------------------------- */

#define JPC_CHARACTER_ID_INVALID 0xffffffffu

typedef struct JPC_CharacterGround {
    JPC_GroundState state;
    JPC_RVec3 position;
    JPC_Vec3 normal;
    JPC_Vec3 velocity;
    JPC_BodyID body_id;
    JPC_SubShapeID sub_shape_id;
    uint64_t user_data;
    /* NULL for the default material. Borrowed. */
    const JPC_PhysicsMaterial *material;
} JPC_CharacterGround;

typedef struct JPC_CharacterVirtualSettings {
    JPC_Vec3 up;
    /* Contacts below this plane (in character space) support the character. */
    JPC_Vec3 supporting_volume_normal;
    float supporting_volume_constant;
    float max_slope_angle;
    bool enhanced_internal_edge_removal;
    /* Borrowed. Required. */
    const JPC_Shape *shape;
    float mass;
    float max_strength;
    JPC_Vec3 shape_offset;
    JPC_BackFaceMode back_face_mode;
    float predictive_contact_distance;
    uint32_t max_collision_iterations;
    uint32_t max_constraint_iterations;
    float min_time_remaining;
    float collision_tolerance;
    float character_padding;
    uint32_t max_num_hits;
    float hit_reduction_cos_max_angle;
    float penetration_recovery_speed;
    /* Optional rigid body that follows the character so raycasts and other bodies can see it. Borrowed. */
    const JPC_Shape *inner_body_shape;
    JPC_ObjectLayer inner_body_layer;
} JPC_CharacterVirtualSettings;

typedef struct JPC_ExtendedUpdateSettings {
    JPC_Vec3 stick_to_floor_step_down;
    JPC_Vec3 walk_stairs_step_up;
    float walk_stairs_min_step_forward;
    float walk_stairs_step_forward_test;
    float walk_stairs_cos_angle_forward_contact;
    JPC_Vec3 walk_stairs_step_down_extra;
} JPC_ExtendedUpdateSettings;

typedef struct JPC_CharacterContact {
    /* JPC_BODY_ID_INVALID when the contact is with another character. */
    JPC_BodyID body_id;
    /* JPC_CHARACTER_ID_INVALID when the contact is with a body. */
    uint32_t character_id;
    JPC_SubShapeID sub_shape_id;
    JPC_RVec3 position;
    JPC_Vec3 linear_velocity;
    JPC_Vec3 contact_normal;
    JPC_Vec3 surface_normal;
    float distance;
    float fraction;
    JPC_MotionType motion_type;
    bool is_sensor;
    uint64_t user_data;
    const JPC_PhysicsMaterial *material;
    bool had_collision;
    bool was_discarded;
    bool can_push_character;
    bool is_back_facing;
} JPC_CharacterContact;

typedef struct JPC_CharacterContactSettings {
    bool can_push_character;
    bool can_receive_impulses;
} JPC_CharacterContactSettings;

/* Called synchronously from the character's update functions. Any callback may be NULL.
 * The character_* variants fire for contacts with other characters (see JPC_CharacterVsCharacterCollision). */
typedef struct JPC_CharacterContactListenerFns {
    void (*on_adjust_body_velocity)(void *user_data, const JPC_CharacterVirtual *character, const JPC_Body *body,
                                    JPC_Vec3 *linear_velocity, JPC_Vec3 *angular_velocity);
    bool (*on_contact_validate)(void *user_data, const JPC_CharacterVirtual *character, const JPC_CharacterContact *contact);
    void (*on_contact_added)(void *user_data, const JPC_CharacterVirtual *character, const JPC_CharacterContact *contact,
                             JPC_CharacterContactSettings *settings);
    void (*on_contact_persisted)(void *user_data, const JPC_CharacterVirtual *character, const JPC_CharacterContact *contact,
                                 JPC_CharacterContactSettings *settings);
    void (*on_contact_removed)(void *user_data, const JPC_CharacterVirtual *character, JPC_BodyID body_id, JPC_SubShapeID sub_shape_id);
    bool (*on_character_contact_validate)(void *user_data, const JPC_CharacterVirtual *character, const JPC_CharacterContact *contact);
    void (*on_character_contact_added)(void *user_data, const JPC_CharacterVirtual *character, const JPC_CharacterContact *contact,
                                       JPC_CharacterContactSettings *settings);
    void (*on_character_contact_persisted)(void *user_data, const JPC_CharacterVirtual *character, const JPC_CharacterContact *contact,
                                           JPC_CharacterContactSettings *settings);
    void (*on_character_contact_removed)(void *user_data, const JPC_CharacterVirtual *character, uint32_t other_character_id,
                                         JPC_SubShapeID sub_shape_id);
    /* contact->body_id / character_id say what was hit. new_character_velocity may be changed. */
    void (*on_contact_solve)(void *user_data, const JPC_CharacterVirtual *character, const JPC_CharacterContact *contact,
                             const JPC_Vec3 *character_velocity, JPC_Vec3 *new_character_velocity);
} JPC_CharacterContactListenerFns;

JPC_API void JPC_CharacterVirtualSettings_SetDefault(JPC_CharacterVirtualSettings *settings);
JPC_API void JPC_ExtendedUpdateSettings_SetDefault(JPC_ExtendedUpdateSettings *settings);

JPC_API JPC_CharacterVirtual *JPC_CharacterVirtual_Create(const JPC_CharacterVirtualSettings *settings, JPC_RVec3 position, JPC_Quat rotation,
                                                          uint64_t user_data, JPC_PhysicsSystem *system);
/* Also removes the character from its character-vs-character collision, if any. */
JPC_API void JPC_CharacterVirtual_Destroy(JPC_CharacterVirtual *character);
JPC_API uint32_t JPC_CharacterVirtual_GetID(const JPC_CharacterVirtual *character);
/* fns is copied; NULL removes the listener. */
JPC_API void JPC_CharacterVirtual_SetListener(JPC_CharacterVirtual *character, const JPC_CharacterContactListenerFns *fns, void *user_data);

/* filter may be NULL. Moves the character by its linear velocity, sliding along geometry. */
JPC_API void JPC_CharacterVirtual_Update(JPC_CharacterVirtual *character, float delta_time, JPC_Vec3 gravity, const JPC_QueryFilter *filter,
                                         JPC_TempAllocator *temp_allocator);
/* Update + stair walking + sticking to the floor. settings may be NULL for the defaults. */
JPC_API void JPC_CharacterVirtual_ExtendedUpdate(JPC_CharacterVirtual *character, float delta_time, JPC_Vec3 gravity,
                                                 const JPC_ExtendedUpdateSettings *settings, const JPC_QueryFilter *filter,
                                                 JPC_TempAllocator *temp_allocator);
JPC_API bool JPC_CharacterVirtual_CanWalkStairs(const JPC_CharacterVirtual *character, JPC_Vec3 linear_velocity);
JPC_API bool JPC_CharacterVirtual_WalkStairs(JPC_CharacterVirtual *character, float delta_time, JPC_Vec3 step_up, JPC_Vec3 step_forward,
                                             JPC_Vec3 step_forward_test, JPC_Vec3 step_down_extra, const JPC_QueryFilter *filter,
                                             JPC_TempAllocator *temp_allocator);
JPC_API bool JPC_CharacterVirtual_StickToFloor(JPC_CharacterVirtual *character, JPC_Vec3 step_down, const JPC_QueryFilter *filter,
                                               JPC_TempAllocator *temp_allocator);
JPC_API void JPC_CharacterVirtual_RefreshContacts(JPC_CharacterVirtual *character, const JPC_QueryFilter *filter, JPC_TempAllocator *temp_allocator);
JPC_API void JPC_CharacterVirtual_UpdateGroundVelocity(JPC_CharacterVirtual *character);
/* Switches shape if it fits (penetration below max_penetration_depth; FLT_MAX always switches). */
JPC_API bool JPC_CharacterVirtual_SetShape(JPC_CharacterVirtual *character, const JPC_Shape *shape, float max_penetration_depth,
                                           const JPC_QueryFilter *filter, JPC_TempAllocator *temp_allocator);
JPC_API void JPC_CharacterVirtual_SetInnerBodyShape(JPC_CharacterVirtual *character, const JPC_Shape *shape);
JPC_API JPC_Vec3 JPC_CharacterVirtual_CancelVelocityTowardsSteepSlopes(const JPC_CharacterVirtual *character, JPC_Vec3 desired_velocity);

JPC_API JPC_Vec3 JPC_CharacterVirtual_GetLinearVelocity(const JPC_CharacterVirtual *character);
JPC_API void JPC_CharacterVirtual_SetLinearVelocity(JPC_CharacterVirtual *character, JPC_Vec3 velocity);
JPC_API JPC_RVec3 JPC_CharacterVirtual_GetPosition(const JPC_CharacterVirtual *character);
JPC_API void JPC_CharacterVirtual_SetPosition(JPC_CharacterVirtual *character, JPC_RVec3 position);
JPC_API JPC_Quat JPC_CharacterVirtual_GetRotation(const JPC_CharacterVirtual *character);
JPC_API void JPC_CharacterVirtual_SetRotation(JPC_CharacterVirtual *character, JPC_Quat rotation);
JPC_API JPC_RVec3 JPC_CharacterVirtual_GetCenterOfMassPosition(const JPC_CharacterVirtual *character);
JPC_API JPC_RMat44 JPC_CharacterVirtual_GetWorldTransform(const JPC_CharacterVirtual *character);
JPC_API JPC_Vec3 JPC_CharacterVirtual_GetUp(const JPC_CharacterVirtual *character);
JPC_API void JPC_CharacterVirtual_SetUp(JPC_CharacterVirtual *character, JPC_Vec3 up);
JPC_API void JPC_CharacterVirtual_SetMaxSlopeAngle(JPC_CharacterVirtual *character, float angle);
JPC_API float JPC_CharacterVirtual_GetCosMaxSlopeAngle(const JPC_CharacterVirtual *character);
JPC_API bool JPC_CharacterVirtual_IsSlopeTooSteep(const JPC_CharacterVirtual *character, JPC_Vec3 normal);
JPC_API float JPC_CharacterVirtual_GetMass(const JPC_CharacterVirtual *character);
JPC_API void JPC_CharacterVirtual_SetMass(JPC_CharacterVirtual *character, float mass);
JPC_API float JPC_CharacterVirtual_GetMaxStrength(const JPC_CharacterVirtual *character);
JPC_API void JPC_CharacterVirtual_SetMaxStrength(JPC_CharacterVirtual *character, float strength);
JPC_API float JPC_CharacterVirtual_GetPenetrationRecoverySpeed(const JPC_CharacterVirtual *character);
JPC_API void JPC_CharacterVirtual_SetPenetrationRecoverySpeed(JPC_CharacterVirtual *character, float speed);
JPC_API JPC_Vec3 JPC_CharacterVirtual_GetShapeOffset(const JPC_CharacterVirtual *character);
JPC_API void JPC_CharacterVirtual_SetShapeOffset(JPC_CharacterVirtual *character, JPC_Vec3 offset);
JPC_API uint64_t JPC_CharacterVirtual_GetUserData(const JPC_CharacterVirtual *character);
JPC_API void JPC_CharacterVirtual_SetUserData(JPC_CharacterVirtual *character, uint64_t user_data);
JPC_API bool JPC_CharacterVirtual_GetEnhancedInternalEdgeRemoval(const JPC_CharacterVirtual *character);
JPC_API void JPC_CharacterVirtual_SetEnhancedInternalEdgeRemoval(JPC_CharacterVirtual *character, bool enabled);
/* Borrowed. */
JPC_API const JPC_Shape *JPC_CharacterVirtual_GetShape(const JPC_CharacterVirtual *character);
/* JPC_BODY_ID_INVALID if the character has no inner body. */
JPC_API JPC_BodyID JPC_CharacterVirtual_GetInnerBodyID(const JPC_CharacterVirtual *character);
JPC_API void JPC_CharacterVirtual_GetGround(const JPC_CharacterVirtual *character, JPC_CharacterGround *out_ground);
JPC_API bool JPC_CharacterVirtual_IsSupported(const JPC_CharacterVirtual *character);
JPC_API bool JPC_CharacterVirtual_HasCollidedWith(const JPC_CharacterVirtual *character, JPC_BodyID body_id);
JPC_API bool JPC_CharacterVirtual_GetMaxHitsExceeded(const JPC_CharacterVirtual *character);
/* Writes up to capacity contacts and returns the total number of active contacts. */
JPC_API uint32_t JPC_CharacterVirtual_GetActiveContacts(const JPC_CharacterVirtual *character, JPC_CharacterContact *out_contacts, uint32_t capacity);

/* Lets a set of characters collide with each other. Characters must be removed (or destroyed) before the collision object. */
JPC_API JPC_CharacterVsCharacterCollision *JPC_CharacterVsCharacterCollision_Create(void);
JPC_API void JPC_CharacterVsCharacterCollision_Destroy(JPC_CharacterVsCharacterCollision *collision);
/* A character can be in at most one collision set; adding it moves it. */
JPC_API void JPC_CharacterVsCharacterCollision_Add(JPC_CharacterVsCharacterCollision *collision, JPC_CharacterVirtual *character);
JPC_API void JPC_CharacterVsCharacterCollision_Remove(JPC_CharacterVsCharacterCollision *collision, JPC_CharacterVirtual *character);

/* A character backed by a regular dynamic rigid body. Call PostSimulation after every physics update. */
typedef struct JPC_CharacterSettings {
    JPC_Vec3 up;
    JPC_Vec3 supporting_volume_normal;
    float supporting_volume_constant;
    float max_slope_angle;
    bool enhanced_internal_edge_removal;
    /* Borrowed. Required. */
    const JPC_Shape *shape;
    JPC_ObjectLayer layer;
    float mass;
    float friction;
    float gravity_factor;
    JPC_AllowedDOFs allowed_dofs;
} JPC_CharacterSettings;

JPC_API void JPC_CharacterSettings_SetDefault(JPC_CharacterSettings *settings);
/* The body is created but not added; call JPC_Character_AddToPhysicsSystem. */
JPC_API JPC_Character *JPC_Character_Create(const JPC_CharacterSettings *settings, JPC_RVec3 position, JPC_Quat rotation, uint64_t user_data,
                                            JPC_PhysicsSystem *system);
/* Removes and destroys the body too. */
JPC_API void JPC_Character_Destroy(JPC_Character *character);
JPC_API JPC_BodyID JPC_Character_GetBodyID(const JPC_Character *character);
JPC_API void JPC_Character_AddToPhysicsSystem(JPC_Character *character, JPC_Activation activation);
JPC_API void JPC_Character_RemoveFromPhysicsSystem(JPC_Character *character);
JPC_API void JPC_Character_Activate(JPC_Character *character);
/* Updates the ground state from the body's contacts. */
JPC_API void JPC_Character_PostSimulation(JPC_Character *character, float max_separation_distance);
JPC_API JPC_Vec3 JPC_Character_GetLinearVelocity(const JPC_Character *character);
JPC_API void JPC_Character_SetLinearVelocity(JPC_Character *character, JPC_Vec3 velocity);
JPC_API void JPC_Character_AddLinearVelocity(JPC_Character *character, JPC_Vec3 velocity);
JPC_API void JPC_Character_SetLinearAndAngularVelocity(JPC_Character *character, JPC_Vec3 linear, JPC_Vec3 angular);
JPC_API void JPC_Character_AddImpulse(JPC_Character *character, JPC_Vec3 impulse);
JPC_API JPC_RVec3 JPC_Character_GetPosition(const JPC_Character *character);
JPC_API void JPC_Character_SetPosition(JPC_Character *character, JPC_RVec3 position, JPC_Activation activation);
JPC_API JPC_Quat JPC_Character_GetRotation(const JPC_Character *character);
JPC_API void JPC_Character_SetRotation(JPC_Character *character, JPC_Quat rotation, JPC_Activation activation);
JPC_API JPC_RVec3 JPC_Character_GetCenterOfMassPosition(const JPC_Character *character);
JPC_API JPC_RMat44 JPC_Character_GetWorldTransform(const JPC_Character *character);
JPC_API JPC_ObjectLayer JPC_Character_GetLayer(const JPC_Character *character);
JPC_API bool JPC_Character_SetLayer(JPC_Character *character, JPC_ObjectLayer layer);
JPC_API bool JPC_Character_SetShape(JPC_Character *character, const JPC_Shape *shape, float max_penetration_depth);
/* Borrowed. */
JPC_API const JPC_Shape *JPC_Character_GetShape(const JPC_Character *character);
JPC_API JPC_Vec3 JPC_Character_GetUp(const JPC_Character *character);
JPC_API void JPC_Character_SetUp(JPC_Character *character, JPC_Vec3 up);
JPC_API void JPC_Character_SetMaxSlopeAngle(JPC_Character *character, float angle);
JPC_API void JPC_Character_GetGround(const JPC_Character *character, JPC_CharacterGround *out_ground);
JPC_API bool JPC_Character_IsSupported(const JPC_Character *character);

/* Character snapshots, for rollback alongside JPC_PhysicsSystem_SaveState. A virtual character saves its position,
 * rotation, velocity, ground state and the contacts it collided with; a rigid character saves its ground state (its
 * body is in the physics snapshot). User data and materials of the ground and contacts are not saved and read as
 * 0 / the default material until the next update. Restoring fails (with an error) for a snapshot of another
 * character or of something else; snapshots made from raw bytes are restored unchecked. Restoring does not move a
 * virtual character's inner body: restore the physics snapshot too. */
JPC_API JPC_PhysicsSnapshot *JPC_CharacterVirtual_SaveState(const JPC_CharacterVirtual *character);
JPC_API bool JPC_CharacterVirtual_RestoreState(JPC_CharacterVirtual *character, const JPC_PhysicsSnapshot *snapshot);
JPC_API JPC_PhysicsSnapshot *JPC_Character_SaveState(const JPC_Character *character);
JPC_API bool JPC_Character_RestoreState(JPC_Character *character, const JPC_PhysicsSnapshot *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* JOLTC_H */
