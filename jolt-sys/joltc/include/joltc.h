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

/* World space positions. Single precision builds only for now. */
typedef JPC_Vec3 JPC_RVec3;
typedef JPC_Mat44 JPC_RMat44;

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

typedef enum JPC_GroundState {
    JPC_GROUND_STATE_ON_GROUND = 0,
    JPC_GROUND_STATE_ON_STEEP_GROUND = 1,
    JPC_GROUND_STATE_NOT_SUPPORTED = 2,
    JPC_GROUND_STATE_IN_AIR = 3,
} JPC_GroundState;

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
JPC_API JPC_Shape *JPC_TaperedCylinderShape_Create(float half_height, float top_radius, float bottom_radius, float convex_radius);
JPC_API JPC_Shape *JPC_ConvexHullShape_Create(const JPC_Vec3 *points, uint32_t num_points, float max_convex_radius);
/* indices holds 3 * num_triangles vertex indices. */
JPC_API JPC_Shape *JPC_MeshShape_Create(const JPC_Vec3 *vertices, uint32_t num_vertices, const uint32_t *indices, uint32_t num_triangles);
/* samples holds sample_count * sample_count heights. */
JPC_API JPC_Shape *JPC_HeightFieldShape_Create(const float *samples, JPC_Vec3 offset, JPC_Vec3 scale, uint32_t sample_count);
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
    bool deterministic_simulation;
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

/* Returns a new reference, or NULL if the body does not exist. */
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
/* Borrowed: valid as long as the body keeps this shape. */
JPC_API const JPC_Shape *JPC_Body_GetShape(const JPC_Body *body);
JPC_API JPC_AABox JPC_Body_GetWorldSpaceBounds(const JPC_Body *body);

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
/* Sweeps the shape from transform along direction. */
JPC_API void JPC_PhysicsSystem_CastShape(const JPC_PhysicsSystem *system, const JPC_Shape *shape, JPC_Vec3 scale,
                                         const JPC_RMat44 *transform, JPC_Vec3 direction,
                                         const JPC_QueryFilter *filter, JPC_ShapeCollectorFn collector, void *user_data);

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

JPC_API void JPC_FixedConstraintSettings_SetDefault(JPC_FixedConstraintSettings *settings);
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
JPC_API JPC_Constraint *JPC_PhysicsSystem_CreateConeConstraint(JPC_PhysicsSystem *system, const JPC_ConeConstraintSettings *settings, JPC_BodyID body1, JPC_BodyID body2);
/* Wakes both bodies so a newly added or modified constraint takes effect. */
JPC_API void JPC_BodyInterface_ActivateConstraint(JPC_BodyInterface *bi, const JPC_Constraint *constraint);

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

#ifdef __cplusplus
}
#endif

#endif /* JOLTC_H */
