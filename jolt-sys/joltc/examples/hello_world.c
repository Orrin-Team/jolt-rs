/* Drops a sphere onto a static floor and prints its position, mirroring Jolt's HelloWorld sample. */
#include <joltc.h>

#include <stdio.h>

enum { LAYER_NON_MOVING = 0, LAYER_MOVING = 1, NUM_LAYERS = 2 };
enum { BP_LAYER_NON_MOVING = 0, BP_LAYER_MOVING = 1, NUM_BP_LAYERS = 2 };

static void on_contact_added(void *user_data, const JPC_Body *body1, const JPC_Body *body2,
                             const JPC_ContactManifold *manifold, JPC_ContactSettings *settings)
{
    (void)settings;
    int *count = (int *)user_data;
    ++*count;
    printf("contact added: %u <-> %u, %u points, depth %.4f\n", JPC_Body_GetID(body1), JPC_Body_GetID(body2),
           manifold->num_points, manifold->penetration_depth);
}

int main(void)
{
    JPC_Init();

    JPC_TempAllocator *temp_allocator = JPC_TempAllocator_Create(10 * 1024 * 1024);
    JPC_JobSystem *job_system = JPC_JobSystemThreadPool_Create(JPC_MAX_PHYSICS_JOBS, JPC_MAX_PHYSICS_BARRIERS, -1);

    const JPC_BroadPhaseLayer object_to_bp[NUM_LAYERS] = { BP_LAYER_NON_MOVING, BP_LAYER_MOVING };
    const JPC_ObjectLayerPair pairs[] = { { LAYER_NON_MOVING, LAYER_MOVING }, { LAYER_MOVING, LAYER_MOVING } };

    JPC_PhysicsSystemSettings system_settings = {
        .max_bodies = 1024,
        .num_body_mutexes = 0,
        .max_body_pairs = 1024,
        .max_contact_constraints = 1024,
        .num_object_layers = NUM_LAYERS,
        .num_broad_phase_layers = NUM_BP_LAYERS,
        .object_to_broad_phase_layer = object_to_bp,
        .colliding_pairs = pairs,
        .num_colliding_pairs = 2,
    };
    JPC_PhysicsSystem *system = JPC_PhysicsSystem_Create(&system_settings);
    if (system == NULL) {
        fprintf(stderr, "failed to create physics system: %s\n", JPC_GetLastError());
        return 1;
    }

    int contacts = 0;
    JPC_ContactListenerFns listener = { .on_contact_added = on_contact_added };
    JPC_PhysicsSystem_SetContactListener(system, &listener, &contacts);

    JPC_BodyInterface *bi = JPC_PhysicsSystem_GetBodyInterface(system);

    JPC_Shape *floor_shape = JPC_BoxShape_Create((JPC_Vec3){ 100.0f, 1.0f, 100.0f }, JPC_DEFAULT_CONVEX_RADIUS);
    JPC_BodyCreationSettings floor_settings;
    JPC_BodyCreationSettings_SetDefault(&floor_settings);
    floor_settings.position = (JPC_RVec3){ 0.0f, -1.0f, 0.0f };
    floor_settings.motion_type = JPC_MOTION_TYPE_STATIC;
    floor_settings.object_layer = LAYER_NON_MOVING;
    floor_settings.shape = floor_shape;
    JPC_BodyID floor_id = JPC_BodyInterface_CreateAndAddBody(bi, &floor_settings, JPC_ACTIVATION_DONT_ACTIVATE);
    JPC_Shape_Release(floor_shape);

    JPC_Shape *sphere_shape = JPC_SphereShape_Create(0.5f);
    JPC_BodyCreationSettings sphere_settings;
    JPC_BodyCreationSettings_SetDefault(&sphere_settings);
    sphere_settings.position = (JPC_RVec3){ 0.0f, 2.0f, 0.0f };
    sphere_settings.object_layer = LAYER_MOVING;
    sphere_settings.shape = sphere_shape;
    JPC_BodyID sphere_id = JPC_BodyInterface_CreateAndAddBody(bi, &sphere_settings, JPC_ACTIVATION_ACTIVATE);
    JPC_Shape_Release(sphere_shape);
    JPC_BodyInterface_SetLinearVelocity(bi, sphere_id, (JPC_Vec3){ 0.0f, -5.0f, 0.0f });

    JPC_PhysicsSystem_OptimizeBroadPhase(system);

    const float delta_time = 1.0f / 60.0f;
    for (int step = 1; JPC_BodyInterface_IsActive(bi, sphere_id) && step < 600; ++step) {
        JPC_RVec3 p = JPC_BodyInterface_GetCenterOfMassPosition(bi, sphere_id);
        JPC_Vec3 v = JPC_BodyInterface_GetLinearVelocity(bi, sphere_id);
        if (step % 10 == 0)
            printf("step %3d: position (%.3f, %.3f, %.3f) velocity (%.3f, %.3f, %.3f)\n", step, p.x, p.y, p.z, v.x, v.y, v.z);
        JPC_PhysicsUpdateError error = JPC_PhysicsSystem_Update(system, delta_time, 1, temp_allocator, job_system);
        if (error != JPC_PHYSICS_UPDATE_ERROR_NONE)
            fprintf(stderr, "update error %u\n", error);
    }

    JPC_RayCastHit hit;
    if (JPC_PhysicsSystem_CastRay(system, (JPC_RVec3){ 0.0f, 10.0f, 0.0f }, (JPC_Vec3){ 0.0f, -20.0f, 0.0f }, NULL, &hit))
        printf("ray hit body %u at (%.3f, %.3f, %.3f) normal (%.2f, %.2f, %.2f)\n", hit.body_id, hit.point.x, hit.point.y,
               hit.point.z, hit.normal.x, hit.normal.y, hit.normal.z);

    printf("sphere came to rest, %d contact(s) added\n", contacts);

    JPC_BodyInterface_RemoveBody(bi, sphere_id);
    JPC_BodyInterface_DestroyBody(bi, sphere_id);
    JPC_BodyInterface_RemoveBody(bi, floor_id);
    JPC_BodyInterface_DestroyBody(bi, floor_id);

    JPC_PhysicsSystem_Destroy(system);
    JPC_JobSystem_Destroy(job_system);
    JPC_TempAllocator_Destroy(temp_allocator);
    JPC_Shutdown();
    return 0;
}
