//! One test per finding of the code review, so the fixes stay fixed.

use std::ops::ControlFlow;

use jolt::*;

const DT: f32 = 1.0 / 60.0;

fn layers() -> (LayerConfig, ObjectLayer, ObjectLayer) {
    let mut layers = LayerConfig::new();
    let static_layer = layers.add_object_layer(BroadPhaseLayer(0));
    let moving_layer = layers.add_object_layer(BroadPhaseLayer(1));
    layers.enable_collision(static_layer, moving_layer).enable_collision(moving_layer, moving_layer);
    (layers, static_layer, moving_layer)
}

fn system(max_bodies: u32) -> (PhysicsSystem, ObjectLayer, ObjectLayer) {
    let (layers, s, m) = layers();
    (PhysicsSystem::new(&PhysicsSystemSettings { max_bodies, ..Default::default() }, &layers).unwrap(), s, m)
}

fn body(system: &PhysicsSystem, shape: Shape, position: Vec3, motion: MotionType, layer: ObjectLayer) -> BodyId {
    let s = BodyCreationSettings::new(shape, position, Quat::IDENTITY, motion, layer);
    system.body_interface().create_and_add_body(&s, Activation::Activate).unwrap()
}

fn capsule() -> Shape {
    let capsule = Shape::capsule(0.5, 0.3).unwrap();
    Shape::rotated_translated(Vec3::new(0.0, 0.8, 0.0), Quat::IDENTITY, &capsule).unwrap()
}

#[test]
fn c1_unknown_object_layers_are_rejected() {
    let (system, static_layer, moving_layer) = system(64);
    let bogus = ObjectLayer(60000);
    let bi = system.body_interface();

    let settings = BodyCreationSettings::new(Shape::sphere(1.0).unwrap(), Vec3::ZERO, Quat::IDENTITY, MotionType::Dynamic, bogus);
    assert!(bi.create_body(&settings).unwrap_err().message().contains("layer"));
    assert!(bi.create_and_add_body(&settings, Activation::Activate).is_err());
    assert_eq!(system.num_bodies(), 0);

    let ball = body(&system, Shape::sphere(1.0).unwrap(), Vec3::ZERO, MotionType::Dynamic, moving_layer);
    assert!(bi.set_object_layer(ball, bogus).is_err());
    assert_eq!(bi.object_layer(ball), moving_layer);
    bi.set_object_layer(ball, static_layer).unwrap();

    // A query from an unknown layer sees nothing.
    let filter = QueryFilter::new().with_object_layer(bogus);
    assert!(system.cast_ray(Vec3::new(0.0, 5.0, 0.0), Vec3::new(0.0, -10.0, 0.0), &filter).is_none());
    assert!(system.cast_ray(Vec3::new(0.0, 5.0, 0.0), Vec3::new(0.0, -10.0, 0.0), &QueryFilter::new()).is_some());

    let settings = CharacterSettings { layer: bogus, ..Default::default() };
    assert!(Character::new(&system, &settings, &capsule(), Vec3::ZERO, Quat::IDENTITY, 0).is_err());
    let settings = CharacterVirtualSettings { inner_body_layer: bogus, ..Default::default() };
    assert!(CharacterVirtual::new(&system, &settings, &capsule(), Some(&capsule()), Vec3::ZERO, Quat::IDENTITY, 0).is_err());

    let settings = CharacterSettings { layer: moving_layer, ..Default::default() };
    let mut ch = Character::new(&system, &settings, &capsule(), Vec3::ZERO, Quat::IDENTITY, 0).unwrap();
    assert!(ch.set_layer(&system, bogus).is_err());
    assert_eq!(ch.layer(), moving_layer);
}

#[test]
fn h1_constraints_cannot_use_character_bodies() {
    let (mut system, _, moving_layer) = system(64);
    let other = body(&system, Shape::sphere(0.5).unwrap(), Vec3::new(3.0, 0.0, 0.0), MotionType::Dynamic, moving_layer);

    let settings = CharacterVirtualSettings { inner_body_layer: moving_layer, ..Default::default() };
    let virtual_ch = CharacterVirtual::new(&system, &settings, &capsule(), Some(&capsule()), Vec3::ZERO, Quat::IDENTITY, 0).unwrap();
    let inner = virtual_ch.inner_body_id().unwrap();
    let err = system.add_constraint(&PointConstraintSettings::at(Vec3::ZERO), inner, other).unwrap_err();
    assert!(err.message().contains("character"), "{err}");

    let settings = CharacterSettings { layer: moving_layer, ..Default::default() };
    let rigid = Character::new(&system, &settings, &capsule(), Vec3::new(-3.0, 0.0, 0.0), Quat::IDENTITY, 0).unwrap();
    assert!(system.add_constraint_to_world(&FixedConstraintSettings::auto_detect(), rigid.body_id()).is_err());
    assert!(system.constraint_ids().is_empty());

    drop(virtual_ch);
    rigid.destroy(&mut system);
    let mut temp = TempAllocator::default();
    system.update(DT, 1, &mut temp, &JobSystem::single_threaded()).unwrap();
}

#[test]
fn h2_job_system_limits_are_clamped_and_updates_queue() {
    let jobs = JobSystem::thread_pool_with_limits(3, 0, Some(2));
    let mut systems: Vec<_> = (0..4)
        .map(|_| {
            let (system, _, moving_layer) = system(64);
            body(&system, Shape::sphere(0.5).unwrap(), Vec3::new(0.0, 5.0, 0.0), MotionType::Dynamic, moving_layer);
            system
        })
        .collect();

    // Four systems update concurrently through one job system that only has a single barrier.
    std::thread::scope(|scope| {
        for system in &mut systems {
            let jobs = &jobs;
            scope.spawn(move || {
                let mut temp = TempAllocator::default();
                for _ in 0..30 {
                    system.update(DT, 1, &mut temp, jobs).unwrap();
                }
            });
        }
    });
    for system in &systems {
        let id = system.bodies()[0];
        assert!(system.body_interface().position(id).y < 5.0);
    }
}

#[test]
fn h3_invalid_physics_settings_are_rejected() {
    let (mut system, _, moving_layer) = system(64);
    struct Nop;
    impl StepListener for Nop {
        fn on_step(&self, _: &StepContext<'_>) {}
    }
    system.set_step_listener(Nop);
    body(&system, Shape::sphere(0.5).unwrap(), Vec3::new(0.0, 5.0, 0.0), MotionType::Dynamic, moving_layer);

    let good = system.physics_settings();
    for bad in [
        PhysicsSettings { step_listeners_batch_size: 0, ..good },
        PhysicsSettings { step_listener_batches_per_job: 0, ..good },
        PhysicsSettings { max_in_flight_body_pairs: 0, ..good },
        PhysicsSettings { num_velocity_steps: 0, ..good },
    ] {
        assert!(system.set_physics_settings(&bad).is_err(), "{bad:?}");
    }
    assert_eq!(system.physics_settings(), good);

    // With the bad values rejected the update still runs.
    let mut temp = TempAllocator::default();
    system.update(DT, 1, &mut temp, &JobSystem::thread_pool(Some(1))).unwrap();
}

#[test]
fn m1_destroying_needs_exclusive_access() {
    // destroy_body lives on PhysicsSystem and takes &mut self, so it can't overlap with reads on other threads.
    let (mut system, _, moving_layer) = system(64);
    let a = body(&system, Shape::sphere(0.5).unwrap(), Vec3::ZERO, MotionType::Dynamic, moving_layer);
    let b = body(&system, Shape::sphere(0.5).unwrap(), Vec3::X, MotionType::Dynamic, moving_layer);
    system.destroy_bodies(&[a, b, a]);
    assert_eq!(system.num_bodies(), 0);
}

#[test]
fn m2_rigid_character_out_of_bodies_is_an_error() {
    let (system, _, moving_layer) = system(2);
    let settings = CharacterSettings { layer: moving_layer, ..Default::default() };
    let first = Character::new(&system, &settings, &capsule(), Vec3::ZERO, Quat::IDENTITY, 0).unwrap();
    let second = Character::new(&system, &settings, &capsule(), Vec3::X, Quat::IDENTITY, 0).unwrap();
    let err = Character::new(&system, &settings, &capsule(), Vec3::Y, Quat::IDENTITY, 0).unwrap_err();
    assert!(err.message().contains("Out of bodies"), "{err}");
    assert_eq!(system.num_bodies(), 2);
    first.add_to_physics_system(&system, Activation::Activate);
    second.add_to_physics_system(&system, Activation::Activate);
    assert!(system.body_interface().is_added(first.body_id()) && system.body_interface().is_added(second.body_id()));
}

#[test]
fn m3_collectors_may_use_the_body_interface() {
    let (system, static_layer, moving_layer) = system(64);
    let floor = body(&system, Shape::cuboid(Vec3::new(10.0, 0.5, 10.0)).unwrap(), Vec3::ZERO, MotionType::Static, static_layer);
    let ball = body(&system, Shape::sphere(0.5).unwrap(), Vec3::new(0.0, 3.0, 0.0), MotionType::Dynamic, moving_layer);

    // Pushing every body the ray hits used to deadlock on the hit body's lock.
    let bi = system.body_interface();
    let mut hits = Vec::new();
    system.cast_ray_all(Vec3::new(0.0, 10.0, 0.0), Vec3::new(0.0, -20.0, 0.0), &RayCastSettings::default(), &QueryFilter::new(), |hit| {
        bi.add_impulse(hit.body_id, Vec3::new(0.0, 100.0, 0.0));
        bi.set_user_data(hit.body_id, 42);
        hits.push(hit.body_id);
        ControlFlow::Continue(())
    });
    hits.sort();
    assert_eq!(hits, { let mut v = vec![floor, ball]; v.sort(); v });
    assert_eq!(bi.user_data(ball), 42);
    assert!(bi.linear_velocity(ball).y > 0.0);

    let probe = Shape::sphere(5.0).unwrap();
    let mut seen = 0;
    system.collide_shape(&probe, Vec3::ONE, &Mat44::IDENTITY, 0.0, &QueryFilter::new(), |hit| {
        bi.set_friction(hit.body_id, 0.9);
        seen += 1;
        ControlFlow::Break(())
    });
    assert_eq!(seen, 1, "Break still stops delivery");
}

#[test]
fn m4_stale_constraint_body_is_an_error() {
    let (mut system, _, moving_layer) = system(64);
    let a = body(&system, Shape::sphere(0.5).unwrap(), Vec3::ZERO, MotionType::Dynamic, moving_layer);
    let b = body(&system, Shape::sphere(0.5).unwrap(), Vec3::X, MotionType::Dynamic, moving_layer);
    system.destroy_body(b);
    let err = system.add_constraint(&PointConstraintSettings::at(Vec3::ZERO), a, b).unwrap_err();
    assert!(err.message().contains("does not exist"), "{err}");
    assert!(system.add_constraint_to_world(&PointConstraintSettings::at(Vec3::ZERO), b).is_err());
    assert!(system.constraint_ids().is_empty());
}

#[test]
fn m5_returned_materials_and_filters_outlive_their_body() {
    let (mut system, static_layer, moving_layer) = system(64);
    let floor = {
        let material = PhysicsMaterial::new(7);
        let shape = Shape::cuboid(Vec3::new(10.0, 0.5, 10.0)).unwrap().with_material(&material).unwrap();
        body(&system, shape, Vec3::ZERO, MotionType::Static, static_layer)
    };
    let hit = system.cast_ray(Vec3::new(0.0, 5.0, 0.0), Vec3::new(0.0, -10.0, 0.0), &QueryFilter::new()).unwrap();
    let mut all = Vec::new();
    system.cast_ray_all(Vec3::new(0.0, 5.0, 0.0), Vec3::new(0.0, -10.0, 0.0), &RayCastSettings::default(), &QueryFilter::new(), |h| {
        all.push(h.clone());
        ControlFlow::Continue(())
    });

    let group = {
        let filter = GroupFilterTable::new(2).build();
        let shape = Shape::sphere(0.5).unwrap();
        let mut s = BodyCreationSettings::new(shape, Vec3::new(0.0, 3.0, 0.0), Quat::IDENTITY, MotionType::Dynamic, moving_layer);
        s.collision_group = CollisionGroup::new(&filter, 1, 0);
        let id = system.body_interface().create_and_add_body(&s, Activation::Activate).unwrap();
        let group = system.body_interface().collision_group(id);
        system.destroy_body(id);
        group
    };

    // The only other references died with the floor; ours keep the objects alive.
    system.destroy_body(floor);
    assert_eq!(hit.material.unwrap().user_data(), 7);
    assert_eq!(all[0].material.as_ref().unwrap().user_data(), 7);
    assert!(group.filter.unwrap().is_collision_enabled(0, 1));
}

#[test]
fn m5_character_ground_material_outlives_the_ground() {
    let (mut system, static_layer, _) = system(64);
    let floor = {
        let material = PhysicsMaterial::new(9);
        let shape = Shape::cuboid(Vec3::new(10.0, 0.5, 10.0)).unwrap().with_material(&material).unwrap();
        body(&system, shape, Vec3::new(0.0, -0.5, 0.0), MotionType::Static, static_layer)
    };
    let mut ch = CharacterVirtual::new(&system, &CharacterVirtualSettings::default(), &capsule(), None, Vec3::ZERO, Quat::IDENTITY, 0).unwrap();
    let mut temp = TempAllocator::default();
    ch.update(&system, DT, Vec3::new(0.0, -9.81, 0.0), &QueryFilter::new(), &mut temp);
    assert_eq!(ch.ground_state(), GroundState::OnGround);

    system.destroy_body(floor);
    assert_eq!(ch.ground().material.unwrap().user_data(), 9);
    assert!(ch.active_contacts().iter().all(|c| c.material.as_ref().is_none_or(|m| m.user_data() == 9)));
}

#[test]
fn m6_max_bodies_is_bounded() {
    let (layers, _, _) = layers();
    let err = PhysicsSystem::new(&PhysicsSystemSettings { max_bodies: 0x80_0001, ..Default::default() }, &layers).unwrap_err();
    assert!(err.message().contains("max_bodies"), "{err}");
}

#[test]
fn batch_error_reports_created_bodies() {
    let (mut system, _, moving_layer) = system(3);
    let shape = Shape::sphere(0.5).unwrap();
    let settings: Vec<_> =
        (0..5).map(|i| BodyCreationSettings::new(shape.clone(), Vec3::new(i as f32, 0.0, 0.0), Quat::IDENTITY, MotionType::Dynamic, moving_layer)).collect();
    let err = system.body_interface().create_and_add_bodies(&settings, Activation::Activate).unwrap_err();
    assert_eq!(err.created.len(), 3);
    assert!(err.created.iter().all(|&id| !system.body_interface().is_added(id)));
    system.destroy_bodies(&err.created);
    assert_eq!(system.num_bodies(), 0);
}
