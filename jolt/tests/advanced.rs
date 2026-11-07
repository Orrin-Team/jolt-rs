use std::ops::ControlFlow;

use jolt::*;

const DT: f32 = 1.0 / 60.0;

struct World {
    system: PhysicsSystem,
    temp: TempAllocator,
    jobs: JobSystem,
    static_layer: ObjectLayer,
    moving_layer: ObjectLayer,
}

impl World {
    fn new() -> Self {
        let mut layers = LayerConfig::new();
        let static_layer = layers.add_object_layer(BroadPhaseLayer(0));
        let moving_layer = layers.add_object_layer(BroadPhaseLayer(1));
        layers.enable_collision(static_layer, moving_layer).enable_collision(moving_layer, moving_layer);
        let system = PhysicsSystem::new(&PhysicsSystemSettings { max_bodies: 256, ..Default::default() }, &layers).unwrap();
        Self { system, temp: TempAllocator::default(), jobs: JobSystem::thread_pool(Some(2)), static_layer, moving_layer }
    }

    fn add(&self, shape: Shape, position: Vec3, motion: MotionType) -> BodyId {
        let layer = if motion == MotionType::Static { self.static_layer } else { self.moving_layer };
        let s = BodyCreationSettings::new(shape, position, Quat::IDENTITY, motion, layer);
        self.system.body_interface().create_and_add_body(&s, Activation::Activate).unwrap()
    }

    fn floor(&self) -> BodyId {
        self.add(Shape::cuboid(Vec3::new(200.0, 1.0, 200.0)).unwrap(), Vec3::new(0.0, -1.0, 0.0), MotionType::Static)
    }

    fn step(&mut self, n: usize) {
        for _ in 0..n {
            self.system.update(DT, 1, &mut self.temp, &self.jobs).unwrap();
        }
    }

    fn car(&mut self) -> (BodyId, ConstraintId) {
        let mut s = BodyCreationSettings::new(
            Shape::cuboid(Vec3::new(0.9, 0.2, 2.0)).unwrap(),
            Vec3::new(0.0, 1.0, 0.0),
            Quat::IDENTITY,
            MotionType::Dynamic,
            self.moving_layer,
        );
        s.override_mass_properties = OverrideMassProperties::CalculateInertia { mass: 1500.0 };
        let body = self.system.body_interface().create_and_add_body(&s, Activation::Activate).unwrap();

        let mut settings = WheeledVehicleSettings::default();
        for (x, z) in [(-0.9, 1.4), (0.9, 1.4), (-0.9, -1.4), (0.9, -1.4)] {
            settings.wheels.push(WheelSettings::at(Vec3::new(x, -0.2, z)));
        }
        settings.wheels[2].max_steer_angle = 0.0;
        settings.wheels[3].max_steer_angle = 0.0;
        settings.differentials.push(VehicleDifferentialSettings::new(Some(0), Some(1)));
        settings.anti_roll_bars.push(AntiRollBar { left_wheel: 0, right_wheel: 1, stiffness: 1000.0 });
        let tester = VehicleCollisionTester::ray(&self.system, self.moving_layer, Vec3::Y, 80f32.to_radians()).unwrap();
        let vehicle = self.system.add_vehicle(&settings, body, &tester).unwrap();
        (body, vehicle)
    }
}

#[test]
fn vehicle_drives_and_steers() {
    let mut world = World::new();
    world.floor();
    let (body, car) = world.car();

    // Let the suspension settle, then drive.
    world.step(60);
    {
        let v = world.system.vehicle(car).unwrap();
        assert_eq!(v.wheel_count(), 4);
        assert_eq!(v.body(), body);
        for i in 0..4 {
            let wheel = v.wheel(i).unwrap();
            assert!(wheel.contact_body.is_some(), "wheel {i} should touch the floor: {wheel:?}");
            assert!((wheel.contact_normal.y - 1.0).abs() < 1e-3);
        }
        assert!(v.wheel(4).is_none());
    }
    let start = world.system.body_interface().position(body);

    for _ in 0..180 {
        world.system.vehicle(car).unwrap().set_driver_input(1.0, 0.0, 0.0, 0.0);
        world.step(1);
    }
    let p = world.system.body_interface().position(body);
    assert!(p.z - start.z > 5.0, "car should drive forward: {start:?} -> {p:?}");
    assert!(p.x.abs() < 0.5, "car should drive straight: {p:?}");
    let v = world.system.vehicle(car).unwrap();
    assert!(v.engine_rpm() > 1000.0 && v.current_gear() >= 1, "rpm {} gear {}", v.engine_rpm(), v.current_gear());
    assert!(v.wheel(0).unwrap().angular_velocity.abs() > 1.0);
    assert!(v.wheel_world_transform(0, Vec3::X, Vec3::Y).is_some());

    for _ in 0..120 {
        world.system.vehicle(car).unwrap().set_driver_input(1.0, 1.0, 0.0, 0.0);
        world.step(1);
    }
    let turned = world.system.body_interface().position(body);
    assert!((turned.x - p.x).abs() > 1.0, "steering should change direction: {p:?} -> {turned:?}");

    // The vehicle dies with its body.
    world.system.destroy_body(body);
    assert!(world.system.vehicle(car).is_none());
    world.step(5);
}

#[test]
fn vehicle_settings_are_validated() {
    let mut world = World::new();
    let body = world.add(Shape::cuboid(Vec3::splat(1.0)).unwrap(), Vec3::ZERO, MotionType::Dynamic);
    let floor = world.floor();
    let tester = VehicleCollisionTester::cast_cylinder(&world.system, world.moving_layer, 0.1).unwrap();
    assert!(VehicleCollisionTester::ray(&world.system, ObjectLayer(99), Vec3::Y, 1.0).is_err());
    assert!(VehicleCollisionTester::cast_sphere(&world.system, world.moving_layer, 0.0, Vec3::Y, 1.0).is_err());

    let mut settings = WheeledVehicleSettings::default();
    settings.wheels.push(WheelSettings::default());
    settings.differentials.push(VehicleDifferentialSettings::new(Some(0), Some(7)));
    assert!(world.system.add_vehicle(&settings, body, &tester).unwrap_err().message().contains("wheel"));

    let mut settings = WheeledVehicleSettings::default();
    settings.wheels.push(WheelSettings::default());
    settings.transmission.gear_ratios.clear();
    assert!(world.system.add_vehicle(&settings, body, &tester).is_err());

    let mut settings = WheeledVehicleSettings::default();
    settings.wheels.push(WheelSettings::default());
    assert!(world.system.add_vehicle(&settings, floor, &tester).unwrap_err().message().contains("dynamic"));

    assert!(world.system.add_vehicle(&WheeledVehicleSettings::default(), body, &tester).unwrap_err().message().contains("wheel"));
    let mut settings = WheeledVehicleSettings::default();
    settings.wheels.push(WheelSettings { max_steer_angle: 2.0, ..Default::default() });
    assert!(world.system.add_vehicle(&settings, body, &tester).is_err());
    let mut settings = WheeledVehicleSettings::default();
    settings.wheels.push(WheelSettings::default());
    settings.transmission.reverse_gear_ratios = vec![1.0];
    assert!(world.system.add_vehicle(&settings, body, &tester).unwrap_err().message().contains("negative"));

    // Manual gears are range checked.
    let mut settings = WheeledVehicleSettings::default();
    settings.wheels.push(WheelSettings::default());
    settings.transmission.mode = TransmissionMode::Manual;
    let car = world.system.add_vehicle(&settings, body, &tester).unwrap();
    let mut v = world.system.vehicle(car).unwrap();
    v.set_transmission(2, 1.0).unwrap();
    assert!(v.set_transmission(6, 1.0).is_err());
    assert!(v.set_transmission(-2, 1.0).is_err());
    assert_eq!(v.current_gear(), 2);
    assert!(world.system.constraint(car).unwrap().kind() == ConstraintKind::Vehicle);
    assert_eq!(world.system.constraint(car).unwrap().bodies(), (None, Some(body)));
    assert!(world.system.remove_constraint(car));
    world.step(2);
}

#[test]
fn snapshots_rewind_the_simulation() {
    let mut world = World::new();
    world.floor();
    let ball = world.add(Shape::sphere(0.5).unwrap(), Vec3::new(0.0, 5.0, 0.0), MotionType::Dynamic);
    let cube = world.add(Shape::cuboid(Vec3::splat(0.5)).unwrap(), Vec3::new(0.2, 8.0, 0.1), MotionType::Dynamic);
    world.step(20);

    let snapshot = world.system.save_state(StateParts::ALL);
    assert!(!snapshot.as_bytes().is_empty());
    world.step(60);
    let bi = world.system.body_interface();
    let expected = (bi.position(ball), bi.position(cube), bi.rotation(cube));

    world.system.restore_state(&snapshot).unwrap();
    world.step(60);
    let bi = world.system.body_interface();
    assert_eq!((bi.position(ball), bi.position(cube), bi.rotation(cube)), expected, "replay must be bit exact");

    // Bytes round trip onto the same world.
    let copy = unsafe { PhysicsSnapshot::from_bytes(snapshot.as_bytes()) };
    world.system.restore_state(&copy).unwrap();
    world.step(60);
    assert_eq!(world.system.body_interface().position(ball), expected.0);

    // Changing the world invalidates in-memory snapshots.
    world.add(Shape::sphere(0.5).unwrap(), Vec3::new(5.0, 5.0, 0.0), MotionType::Dynamic);
    assert!(world.system.restore_state(&snapshot).unwrap_err().message().contains("changed"));

    // Snapshots belong to their system.
    let mut other = World::new();
    assert!(other.system.restore_state(&world.system.save_state(StateParts::BODIES)).is_err());
}

#[test]
fn broad_phase_queries() {
    let world = World::new();
    let a = world.add(Shape::sphere(1.0).unwrap(), Vec3::new(0.0, 0.0, 0.0), MotionType::Static);
    let b = world.add(Shape::sphere(1.0).unwrap(), Vec3::new(10.0, 0.0, 0.0), MotionType::Static);
    let s = &world.system;
    type Query<'a> = &'a dyn Fn(&mut dyn FnMut(BodyId) -> ControlFlow<()>);
    let collect = |f: Query<'_>| {
        let mut ids = Vec::new();
        f(&mut |id| {
            ids.push(id);
            ControlFlow::Continue(())
        });
        ids.sort();
        ids
    };

    let all = AABox { min: Vec3::splat(-20.0), max: Vec3::splat(20.0) };
    assert_eq!(collect(&|c| s.broad_phase_collide_aabox(&all, &QueryFilter::new(), c)), { let mut v = vec![a, b]; v.sort(); v });
    assert_eq!(collect(&|c| s.broad_phase_collide_sphere(Vec3::new(9.0, 0.0, 0.0), 0.5, &QueryFilter::new(), c)), vec![b]);
    // The sphere's bounding box contains the corner even though the sphere does not.
    assert_eq!(collect(&|c| s.broad_phase_collide_point(Vec3::splat(0.9), &QueryFilter::new(), c)), vec![a]);
    let box_at_b = Mat44::from_translation(Vec3::new(10.0, 0.0, 0.0));
    assert_eq!(collect(&|c| s.broad_phase_collide_oriented_box(&box_at_b, Vec3::splat(0.1), &QueryFilter::new(), c)), vec![b]);
    assert_eq!(collect(&|c| s.broad_phase_collide_aabox(&all, &QueryFilter::new().ignoring(a), c)), vec![b]);

    let mut hits = Vec::new();
    s.broad_phase_cast_ray(Vec3::new(-5.0, 0.0, 0.0), Vec3::new(20.0, 0.0, 0.0), &QueryFilter::new(), |h| {
        hits.push(*h);
        ControlFlow::Continue(())
    });
    hits.sort_by(|x, y| x.fraction.total_cmp(&y.fraction));
    assert_eq!(hits.iter().map(|h| h.body_id).collect::<Vec<_>>(), vec![a, b]);
    assert!((hits[0].fraction - 0.2).abs() < 0.01, "{hits:?}");

    let mut cast = Vec::new();
    let probe = AABox { min: Vec3::new(4.0, -0.1, -0.1), max: Vec3::new(4.2, 0.1, 0.1) };
    s.broad_phase_cast_aabox(&probe, Vec3::new(10.0, 0.0, 0.0), &QueryFilter::new(), |h| {
        cast.push(h.body_id);
        ControlFlow::Continue(())
    });
    assert_eq!(cast, vec![b]);
}

#[test]
fn query_settings() {
    let world = World::new();
    let ball = world.add(Shape::sphere(1.0).unwrap(), Vec3::ZERO, MotionType::Static);
    let s = &world.system;

    // From inside a solid sphere the hit is at fraction 0; treated as hollow it hits the far side.
    let solid = s.cast_ray_with_settings(Vec3::ZERO, Vec3::new(5.0, 0.0, 0.0), &RayCastSettings::default(), &QueryFilter::new()).unwrap();
    assert_eq!(solid.fraction, 0.0);
    let hollow = RayCastSettings { treat_convex_as_solid: false, back_face_mode_convex: BackFaceMode::Collide, ..Default::default() };
    let hit = s.cast_ray_with_settings(Vec3::ZERO, Vec3::new(5.0, 0.0, 0.0), &hollow, &QueryFilter::new()).unwrap();
    assert!((hit.point.x - 1.0).abs() < 1e-3, "{hit:?}");

    // A probe 0.5 away is only reported with a separation distance.
    let probe = Shape::sphere(0.5).unwrap();
    let at = Mat44::from_translation(Vec3::new(2.0, 0.0, 0.0));
    let mut near = Vec::new();
    let settings = CollideShapeSettings { max_separation_distance: 1.0, ..Default::default() };
    s.collide_shape_with_settings(&probe, Vec3::ONE, &at, &settings, &QueryFilter::new(), |h| {
        near.push(h.clone());
        ControlFlow::Continue(())
    });
    assert_eq!(near.len(), 1);
    assert_eq!(near[0].body_id, ball);
    assert!((near[0].penetration_depth + 0.5).abs() < 1e-3, "{:?}", near[0]);

    let mut swept = Vec::new();
    let from = Mat44::from_translation(Vec3::new(5.0, 0.0, 0.0));
    let settings = ShapeCastSettings { return_deepest_point: true, ..Default::default() };
    s.cast_shape_with_settings(&probe, Vec3::ONE, &from, Vec3::new(-10.0, 0.0, 0.0), &settings, &QueryFilter::new(), |h| {
        swept.push(h.clone());
        ControlFlow::Continue(())
    });
    assert_eq!(swept.len(), 1);
    assert!((swept[0].fraction - 0.35).abs() < 0.01, "{:?}", swept[0]);
}

#[test]
fn height_field_materials() {
    let world = World::new();
    let rock = PhysicsMaterial::new(1);
    let sand = PhysicsMaterial::new(2);
    let n = 8u32;
    let samples = vec![0.0; (n * n) as usize];
    // Left half rock, right half sand.
    let cells: Vec<u8> = (0..(n - 1) * (n - 1)).map(|i| if i % (n - 1) < (n - 1) / 2 { 0 } else { 1 }).collect();
    let shape = Shape::height_field_with_materials(&samples, Vec3::ZERO, Vec3::ONE, n, &cells, &[rock, sand]).unwrap();
    assert!(Shape::height_field_with_materials(&samples, Vec3::ZERO, Vec3::ONE, n, &cells[1..], &[PhysicsMaterial::new(0)]).is_err());
    assert!(Shape::height_field_with_materials(&samples, Vec3::ZERO, Vec3::ONE, n, &cells, &[PhysicsMaterial::new(0)]).is_err());
    world.add(shape, Vec3::ZERO, MotionType::Static);

    let down = Vec3::new(0.0, -2.0, 0.0);
    let left = world.system.cast_ray(Vec3::new(1.0, 1.0, 3.0), down, &QueryFilter::new()).unwrap();
    let right = world.system.cast_ray(Vec3::new(6.0, 1.0, 3.0), down, &QueryFilter::new()).unwrap();
    assert_eq!(left.material.unwrap().user_data(), 1);
    assert_eq!(right.material.unwrap().user_data(), 2);
}

#[test]
fn gear_rack_and_pinion() {
    let mut world = World::new();
    world.system.set_gravity(Vec3::ZERO);
    let gear1 = world.add(Shape::cylinder(0.1, 1.0).unwrap(), Vec3::new(0.0, 5.0, 0.0), MotionType::Dynamic);
    let gear2 = world.add(Shape::cylinder(0.1, 0.5).unwrap(), Vec3::new(1.5, 5.0, 0.0), MotionType::Dynamic);
    for (body, x) in [(gear1, 0.0), (gear2, 1.5)] {
        world.system.add_constraint_to_world(&HingeConstraintSettings::at(Vec3::new(x, 5.0, 0.0), Vec3::Y, Vec3::X), body).unwrap();
    }
    // 20 teeth driving 10 teeth: the small gear turns twice as fast.
    world.system.add_constraint(&GearConstraintSettings::with_teeth(Vec3::Y, Vec3::Y, 20, 10), gear1, gear2).unwrap();
    world.system.body_interface().set_angular_velocity(gear1, Vec3::new(0.0, 1.0, 0.0));
    world.step(30);
    let bi = world.system.body_interface();
    let (w1, w2) = (bi.angular_velocity(gear1).y, bi.angular_velocity(gear2).y);
    assert!(w1.abs() > 0.1 && (w2 / w1 + 2.0).abs() < 0.05, "gear2 should turn twice as fast the other way: {w1} {w2}");

    let pinion = world.add(Shape::cylinder(0.1, 0.5).unwrap(), Vec3::new(0.0, 10.0, 0.0), MotionType::Dynamic);
    let rack = world.add(Shape::cuboid(Vec3::new(2.0, 0.1, 0.1)).unwrap(), Vec3::new(0.0, 9.4, 0.0), MotionType::Dynamic);
    let axle = world.system.add_constraint_to_world(&HingeConstraintSettings::at(Vec3::new(0.0, 10.0, 0.0), Vec3::Z, Vec3::X), pinion).unwrap();
    world.system.add_constraint_to_world(&SliderConstraintSettings::at(Vec3::new(0.0, 9.4, 0.0), Vec3::X, Vec3::Y), rack).unwrap();
    world.system.add_constraint(&RackAndPinionConstraintSettings::new(Vec3::Z, Vec3::X, 0.5), pinion, rack).unwrap();
    // Drive the pinion at 2 rad/s; with 0.5 rad per meter the rack must move at 4 m/s.
    let mut motor = world.system.constraint(axle).unwrap();
    assert!(motor.set_motor_state(MotorState::Velocity) && motor.set_motor_target_velocity(2.0));
    world.step(60);
    let v = world.system.body_interface().linear_velocity(rack).x.abs();
    assert!((v - 4.0).abs() < 0.4, "rack speed {v}");
}

#[test]
fn pulley_and_path() {
    let mut world = World::new();
    let a = world.add(Shape::cuboid(Vec3::splat(0.25)).unwrap(), Vec3::new(-2.0, 5.0, 0.0), MotionType::Dynamic);
    let b = world.add(Shape::cuboid(Vec3::splat(0.25)).unwrap(), Vec3::new(2.0, 5.0, 0.0), MotionType::Dynamic);
    let settings = PulleyConstraintSettings::new(Vec3::new(-2.0, 5.0, 0.0), Vec3::new(-2.0, 10.0, 0.0), Vec3::new(2.0, 5.0, 0.0), Vec3::new(2.0, 10.0, 0.0));
    let pulley = world.system.add_constraint(&settings, a, b).unwrap();
    assert!(world.system.add_constraint(&PulleyConstraintSettings { ratio: 0.0, ..settings }, a, b).is_err());
    // Pulling one side up must lower the other.
    world.system.body_interface().set_linear_velocity(a, Vec3::new(0.0, 3.0, 0.0));
    world.step(20);
    let bi = world.system.body_interface();
    assert!(bi.position(a).y > 5.1 && bi.position(b).y < 4.9, "{:?} {:?}", bi.position(a), bi.position(b));
    let mut c = world.system.constraint(pulley).unwrap();
    assert!((c.pulley_length().unwrap() - 10.0).abs() < 0.1);
    assert!(c.set_pulley_length(0.0, 12.0) && !c.set_pulley_length(5.0, 1.0));

    // A straight rail along X with a velocity motor.
    let path = PathConstraintPath::hermite(
        &[
            PathPoint { position: Vec3::new(0.0, 20.0, 0.0), tangent: Vec3::X, normal: Vec3::Y },
            PathPoint { position: Vec3::new(10.0, 20.0, 0.0), tangent: Vec3::X, normal: Vec3::Y },
        ],
        false,
    )
    .unwrap();
    assert_eq!(path.max_fraction(), 1.0);
    assert!(PathConstraintPath::hermite(&[PathPoint { position: Vec3::ZERO, tangent: Vec3::X, normal: Vec3::Y }], false).is_err());
    let cart = world.add(Shape::cuboid(Vec3::splat(0.25)).unwrap(), Vec3::new(0.0, 20.0, 0.0), MotionType::Dynamic);
    let mut settings = PathConstraintSettings::new(path);
    settings.position_motor = MotorSettings { spring: SpringSettings::FrequencyAndDamping { frequency: 2.0, damping: 1.0 }, ..settings.position_motor };
    let rail = world.system.add_constraint_to_world(&settings, cart).unwrap();
    let mut c = world.system.constraint(rail).unwrap();
    assert!(c.set_path_motor_state(MotorState::Velocity));
    assert!(c.set_path_target_velocity(2.0));
    assert!(c.set_path_target_fraction(5.0), "clamped, not rejected");
    world.step(90);
    let p = world.system.body_interface().position(cart);
    assert!(p.x > 1.0 && (p.y - 20.0).abs() < 0.05, "cart should ride the rail despite gravity: {p:?}");
    assert!(world.system.constraint(rail).unwrap().path_fraction().unwrap() > 0.1);
}
