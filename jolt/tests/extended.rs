//! Mutable compounds, shape serialization, character snapshots, motorcycles and tracked vehicles.

use jolt::*;

const DT: f32 = 1.0 / 60.0;
const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);

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

    fn vehicle_body(&self, half_extent: Vec3, position: Vec3, mass: f32) -> BodyId {
        let mut s = BodyCreationSettings::new(Shape::cuboid(half_extent).unwrap(), position, Quat::IDENTITY, MotionType::Dynamic, self.moving_layer);
        s.override_mass_properties = OverrideMassProperties::CalculateInertia { mass };
        self.system.body_interface().create_and_add_body(&s, Activation::Activate).unwrap()
    }
}

fn close(a: Vec3, b: Vec3, tolerance: f32) -> bool {
    (a - b).length() < tolerance
}

fn unit_box() -> Shape {
    Shape::cuboid(Vec3::splat(0.5)).unwrap()
}

// ---------------------------------------------------------------------------
// Mutable compounds
// ---------------------------------------------------------------------------

#[test]
fn mutable_compound_edits_in_place() {
    let parts = [
        CompoundSubShape { user_data: 7, ..CompoundSubShape::new(Vec3::new(-1.0, 0.0, 0.0), Quat::IDENTITY, unit_box()) },
        CompoundSubShape::new(Vec3::new(1.0, 0.0, 0.0), Quat::from_axis_angle(Vec3::Y, 0.5), Shape::sphere(0.5).unwrap()),
    ];
    let mut compound = Shape::mutable_compound(&parts).unwrap();
    assert_eq!(compound.sub_type(), ShapeSubType::MutableCompound);
    assert_eq!(compound.sub_shape_count(), 2);

    // Children read back where they were put, even though Jolt stores them relative to the center of mass.
    let read = compound.sub_shapes();
    assert!(close(read[0].position, parts[0].position, 1e-5) && read[0].user_data == 7);
    assert!(close(read[1].position, parts[1].position, 1e-5));
    assert!((read[1].rotation.w - parts[1].rotation.w).abs() < 1e-5);
    assert_eq!(read[1].shape.sub_type(), ShapeSubType::Sphere);

    {
        let mut edit = compound.edit_mutable_compound().unwrap();
        assert_eq!(edit.push(&CompoundSubShape::new(Vec3::new(0.0, 3.0, 0.0), Quat::IDENTITY, unit_box())).unwrap(), 2);
        assert_eq!(edit.insert(0, &CompoundSubShape::new(Vec3::new(0.0, -3.0, 0.0), Quat::IDENTITY, unit_box())).unwrap(), 0);
        assert!(edit.insert(9, &CompoundSubShape::new(Vec3::ZERO, Quat::IDENTITY, unit_box())).is_err());
        assert_eq!(edit.len(), 4);
        edit.remove(1).unwrap(); // the first original box
        assert!(edit.remove(3).is_err());
        edit.set_transform(0, Vec3::new(0.0, -5.0, 0.0), Quat::IDENTITY).unwrap();
        edit.replace(2, Vec3::new(0.0, 4.0, 0.0), Quat::IDENTITY, &Shape::sphere(1.0).unwrap()).unwrap();
        assert!(edit.set_transform(3, Vec3::ZERO, Quat::IDENTITY).is_err());
        edit.set_transforms(1, &[(Vec3::new(2.0, 0.0, 0.0), Quat::IDENTITY)]).unwrap();
        assert!(edit.set_transforms(1, &[(Vec3::ZERO, Quat::IDENTITY); 3]).is_err());

        let before = edit.sub_shape(0).unwrap().position;
        edit.adjust_center_of_mass();
        // Adjusting the center of mass leaves the children where they were.
        assert!(close(edit.sub_shape(0).unwrap().position, before, 1e-4));
    }
    let positions: Vec<Vec3> = compound.sub_shapes().iter().map(|s| s.position).collect();
    assert_eq!(positions.len(), 3);
    assert!(close(positions[0], Vec3::new(0.0, -5.0, 0.0), 1e-4), "{positions:?}");
    assert!(close(positions[1], Vec3::new(2.0, 0.0, 0.0), 1e-4), "{positions:?}");
    assert!(close(positions[2], Vec3::new(0.0, 4.0, 0.0), 1e-4), "{positions:?}");
    assert_eq!(compound.sub_shape(2).unwrap().shape.sub_type(), ShapeSubType::Sphere);
    // Local bounds are relative to the center of mass: from the box's bottom at -5.5 to the sphere's top at 5.
    let bounds = compound.local_bounds();
    assert!((bounds.max.y - bounds.min.y - 10.5).abs() < 1e-3, "{bounds:?}");

    // Only the sole owner may edit, and only mutable compounds.
    let shared = compound.clone();
    assert!(compound.edit_mutable_compound().is_err());
    drop(shared);
    assert!(compound.edit_mutable_compound().is_ok());
    let mut static_compound = Shape::static_compound(&parts).unwrap();
    assert!(static_compound.edit_mutable_compound().is_err());
    assert_eq!(static_compound.sub_shape_count(), 2);
    assert!(unit_box().sub_shapes().is_empty());
    assert!(unit_box().compound_child(SubShapeId::EMPTY).is_none());
}

#[test]
fn mutable_compound_on_a_body() {
    let mut world = World::new();
    let shape = Shape::mutable_compound(&[CompoundSubShape::new(Vec3::ZERO, Quat::IDENTITY, unit_box())]).unwrap();
    let body = world.add(shape.clone(), Vec3::new(0.0, 10.0, 0.0), MotionType::Dynamic);
    let bodies = world.system.body_interface();
    let mass = bodies.mass(body).unwrap();
    let snapshot = world.system.save_state(StateParts::ALL);

    // A ray off to the side misses until a part is added there.
    let probe = |system: &PhysicsSystem| system.cast_ray(Vec3::new(3.0, 20.0, 0.0), Vec3::new(0.0, -20.0, 0.0), &QueryFilter::new());
    assert!(probe(&world.system).is_none());

    let index = bodies
        .modify_mutable_compound(body, true, Activation::Activate, |compound| {
            let index = compound.push(&CompoundSubShape { user_data: 42, ..CompoundSubShape::new(Vec3::new(3.0, 0.0, 0.0), Quat::IDENTITY, unit_box()) });
            compound.adjust_center_of_mass();
            index
        })
        .unwrap()
        .unwrap();
    assert_eq!(index, 1);

    // The body stays put, gains mass, and its center of mass moves to the middle of the two boxes.
    assert!(close(bodies.position(body), Vec3::new(0.0, 10.0, 0.0), 1e-4));
    assert!((bodies.mass(body).unwrap() - 2.0 * mass).abs() < 1e-2);
    assert!(close(bodies.center_of_mass_position(body), Vec3::new(1.5, 10.0, 0.0), 1e-4));

    // The new part is hit, and the hit maps back to the child that was added.
    let hit = probe(&world.system).expect("added part should be hit");
    assert_eq!(hit.body_id, body);
    let body_shape = bodies.shape(body).unwrap();
    let (child, _) = body_shape.compound_child(hit.sub_shape_id).unwrap();
    assert_eq!(child, 1);
    assert_eq!(body_shape.sub_shape(child).unwrap().user_data, 42);
    drop(body_shape);

    // The handle kept above still sees the original shape: the body got its own copy.
    assert_eq!(shape.sub_shape_count(), 1);
    assert_eq!(bodies.shape(body).unwrap().sub_shape_count(), 2);
    drop(shape);

    // Now the body is the only owner, so the edit happens in place.
    let before = bodies.shape(body).unwrap().as_raw();
    bodies.modify_mutable_compound(body, false, Activation::DontActivate, |c| c.remove(1).unwrap()).unwrap();
    assert_eq!(bodies.shape(body).unwrap().as_raw(), before);
    assert!(probe(&world.system).is_none());

    // Emptying the compound keeps the last mass instead of making it infinite.
    bodies.modify_mutable_compound(body, true, Activation::DontActivate, |c| c.remove(0).unwrap()).unwrap();
    assert!(bodies.mass(body).unwrap().is_finite());
    bodies.modify_mutable_compound(body, true, Activation::Activate, |c| c.push(&CompoundSubShape::new(Vec3::ZERO, Quat::IDENTITY, unit_box())).unwrap()).unwrap();

    // Editing changes the world's structure, so the old snapshot no longer applies.
    assert!(world.system.restore_state(&snapshot).is_err());

    // A panic inside the edit comes out after the body is unlocked.
    let bodies = world.system.body_interface();
    let panic = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let _ = bodies.modify_mutable_compound(body, false, Activation::DontActivate, |_| panic!("boom"));
    }));
    assert!(panic.is_err());
    assert_eq!(bodies.shape(body).unwrap().sub_shape_count(), 1);

    // Bodies with other shapes and stale ids are refused without calling the closure.
    let plain = world.add(unit_box(), Vec3::new(5.0, 0.0, 0.0), MotionType::Dynamic);
    let bodies = world.system.body_interface();
    assert!(bodies.modify_mutable_compound(plain, true, Activation::Activate, |_| unreachable!()).is_err());
    world.system.destroy_body(plain);
    let bodies = world.system.body_interface();
    assert!(bodies.modify_mutable_compound(plain, true, Activation::Activate, |_| unreachable!()).is_err());

    world.step(30);
    assert!(world.system.body_interface().position(body).y < 10.0, "the body still falls");
}

#[test]
fn mutable_compound_rejects_hierarchies_too_deep_for_sub_shape_ids() {
    // Every level of 2^16 children needs 16 bits; two such levels use up all 32 bits of a sub shape id.
    let children: Vec<CompoundSubShape> =
        (0..40_000).map(|i| CompoundSubShape::new(Vec3::new(i as f32, 0.0, 0.0), Quat::IDENTITY, Shape::sphere(0.1).unwrap())).collect();
    let wide = Shape::static_compound(&children).unwrap();
    let mut outer = Shape::mutable_compound(&[CompoundSubShape::new(Vec3::ZERO, Quat::IDENTITY, Shape::sphere(1.0).unwrap())]).unwrap();
    let mut edit = outer.edit_mutable_compound().unwrap();
    for _ in 0..(1 << 16) {
        edit.push(&CompoundSubShape::new(Vec3::ZERO, Quat::IDENTITY, Shape::sphere(0.1).unwrap())).unwrap();
    }
    let err = edit.push(&CompoundSubShape::new(Vec3::ZERO, Quat::IDENTITY, wide.clone())).unwrap_err();
    assert!(err.message().contains("sub shape ID bits"), "{err}");
    assert!(edit.replace(0, Vec3::ZERO, Quat::IDENTITY, &wide).is_err());
    assert_eq!(edit.len(), (1 << 16) + 1);
    assert_eq!(edit.sub_shape(0).unwrap().shape.sub_type(), ShapeSubType::Sphere);
}

// ---------------------------------------------------------------------------
// Serialization
// ---------------------------------------------------------------------------

#[test]
fn shapes_round_trip_through_bytes() {
    let ice = PhysicsMaterial::new(11);
    let mud = PhysicsMaterial::new(22);
    let vertices = [Vec3::new(-5.0, 0.0, -5.0), Vec3::new(5.0, 0.0, -5.0), Vec3::new(0.0, 0.0, 5.0)];
    let mesh = Shape::mesh_with_materials(&vertices, &[[0, 2, 1]], &[1], &[ice.clone(), mud.clone()]).unwrap();
    let ball = Shape::sphere(0.5).unwrap().with_material(&ice).unwrap();
    let shared = Shape::scaled(&unit_box(), Vec3::new(1.0, 2.0, 1.0)).unwrap();
    let compound = Shape::static_compound(&[
        CompoundSubShape::new(Vec3::new(-2.0, 0.0, 0.0), Quat::IDENTITY, shared.clone()),
        CompoundSubShape::new(Vec3::new(2.0, 0.0, 0.0), Quat::IDENTITY, shared.clone()),
        CompoundSubShape { user_data: 5, ..CompoundSubShape::new(Vec3::new(0.0, 3.0, 0.0), Quat::IDENTITY, ball) },
    ])
    .unwrap();

    for original in [compound.clone(), mesh.clone(), Shape::mutable_compound(&compound.sub_shapes()).unwrap()] {
        let bytes = original.to_bytes();
        let restored = unsafe { Shape::from_bytes(&bytes) }.unwrap();
        assert_eq!(restored.sub_type(), original.sub_type());
        assert_eq!(restored.local_bounds(), original.local_bounds());
        assert_eq!(restored.mass_properties(), original.mass_properties());
        assert_eq!(restored.to_bytes(), bytes, "saving again gives the same bytes");
    }

    let restored = unsafe { Shape::from_bytes(&compound.to_bytes()) }.unwrap();
    let children = restored.sub_shapes();
    // The shared child is stored once and comes back shared.
    assert_eq!(children[0].shape.as_raw(), children[1].shape.as_raw());
    assert_eq!(children[2].user_data, 5);
    // Materials come back by user data.
    let ball_material = children[2].shape.sub_shape(0).map(|s| s.shape).unwrap_or_else(|| children[2].shape.clone());
    let world = World::new();
    let restored_ball = world.add(ball_material, Vec3::new(0.0, 0.0, 10.0), MotionType::Static);
    let hit = world.system.cast_ray(Vec3::new(0.0, 5.0, 10.0), Vec3::new(0.0, -10.0, 0.0), &QueryFilter::new()).unwrap();
    assert_eq!(hit.body_id, restored_ball);
    assert_eq!(hit.material.map(|m| m.user_data()), Some(11));

    let restored_mesh = unsafe { Shape::from_bytes(&mesh.to_bytes()) }.unwrap();
    world.add(restored_mesh, Vec3::new(30.0, 0.0, 0.0), MotionType::Static);
    let hit = world.system.cast_ray(Vec3::new(30.0, 5.0, 0.0), Vec3::new(0.0, -10.0, 0.0), &QueryFilter::new()).unwrap();
    assert_eq!(hit.material.map(|m| m.user_data()), Some(22));
}

#[test]
fn shape_bytes_are_checked() {
    let shape = Shape::static_compound(&[
        CompoundSubShape::new(Vec3::ZERO, Quat::IDENTITY, unit_box()),
        CompoundSubShape::new(Vec3::Y, Quat::IDENTITY, Shape::sphere(0.5).unwrap().with_material(&PhysicsMaterial::new(3)).unwrap()),
    ])
    .unwrap();
    let bytes = shape.to_bytes();

    // Every truncation is rejected.
    for len in 0..bytes.len() {
        assert!(unsafe { Shape::from_bytes(&bytes[..len]) }.is_err(), "prefix of {len} bytes was accepted");
    }
    let mut longer = bytes.clone();
    longer.push(0);
    assert!(unsafe { Shape::from_bytes(&longer) }.unwrap_err().message().contains("trailing"));

    let mut wrong_magic = bytes.clone();
    wrong_magic[0] ^= 0xff;
    assert!(unsafe { Shape::from_bytes(&wrong_magic) }.is_err());
    let mut wrong_version = bytes.clone();
    wrong_version[8] ^= 0xff;
    assert!(unsafe { Shape::from_bytes(&wrong_version) }.unwrap_err().message().contains("version"));
    // The first shape's type byte follows the 12 byte header and its 4 byte id.
    let mut wrong_type = bytes.clone();
    wrong_type[16] = 200;
    assert!(unsafe { Shape::from_bytes(&wrong_type) }.unwrap_err().message().contains("unknown shape type"));
}

// ---------------------------------------------------------------------------
// Character snapshots
// ---------------------------------------------------------------------------

fn capsule() -> Shape {
    Shape::rotated_translated(Vec3::new(0.0, 0.9, 0.0), Quat::IDENTITY, &Shape::capsule(0.5, 0.4).unwrap()).unwrap()
}

#[test]
fn virtual_character_state_rolls_back() {
    let mut world = World::new();
    world.floor();
    let mut ch = CharacterVirtual::new(&world.system, &CharacterVirtualSettings::default(), &capsule(), None, Vec3::ZERO, Quat::IDENTITY, 0).unwrap();
    let filter = QueryFilter::new();
    for _ in 0..10 {
        ch.update(&world.system, DT, GRAVITY, &filter, &mut world.temp);
    }
    assert_eq!(ch.ground_state(), GroundState::OnGround);
    ch.set_linear_velocity(Vec3::new(2.0, 0.0, 0.0));
    let saved = ch.save_state();
    let (position, velocity) = (ch.position(), ch.linear_velocity());

    for _ in 0..30 {
        ch.update(&world.system, DT, GRAVITY, &filter, &mut world.temp);
    }
    ch.set_position(&world.system, Vec3::new(0.0, 50.0, 0.0));
    ch.set_linear_velocity(Vec3::ZERO);
    ch.update(&world.system, DT, GRAVITY, &filter, &mut world.temp);
    assert_eq!(ch.ground_state(), GroundState::InAir);

    ch.restore_state(&saved).unwrap();
    assert_eq!(ch.position(), position);
    assert_eq!(ch.linear_velocity(), velocity);
    assert_eq!(ch.ground_state(), GroundState::OnGround);
    assert_eq!(ch.ground().material, None);

    // Replaying from the snapshot gives the same result as the first time.
    let replay = |ch: &mut CharacterVirtual, world: &mut World| {
        ch.restore_state(&saved).unwrap();
        for _ in 0..20 {
            ch.update(&world.system, DT, GRAVITY, &filter, &mut world.temp);
        }
        ch.position()
    };
    let first = replay(&mut ch, &mut world);
    assert_eq!(replay(&mut ch, &mut world), first);

    // Snapshots only restore onto what they were taken of.
    let mut other = CharacterVirtual::new(&world.system, &CharacterVirtualSettings::default(), &capsule(), None, Vec3::X, Quat::IDENTITY, 0).unwrap();
    assert!(other.restore_state(&saved).unwrap_err().message().contains("character"));
    assert!(ch.restore_state(&world.system.save_state(StateParts::ALL)).is_err());
    assert!(world.system.restore_state(&saved).is_err());
    // Raw bytes are trusted.
    let raw = unsafe { PhysicsSnapshot::from_bytes(saved.as_bytes()) };
    other.restore_state(&raw).unwrap();
    assert_eq!(other.position(), position);
}

#[test]
fn rigid_character_ground_state_rolls_back() {
    let mut world = World::new();
    world.floor();
    let settings = CharacterSettings { layer: world.moving_layer, ..Default::default() };
    let mut ch = Character::new(&world.system, &settings, &capsule(), Vec3::new(0.0, 0.05, 0.0), Quat::IDENTITY, 0).unwrap();
    ch.add_to_physics_system(&world.system, Activation::Activate);
    world.step(10);
    ch.post_simulation(&world.system, 0.05);
    assert_eq!(ch.ground().state, GroundState::OnGround);
    let saved = ch.save_state();
    let world_saved = world.system.save_state(StateParts::ALL);

    ch.set_position(&world.system, Vec3::new(0.0, 20.0, 0.0), Activation::Activate);
    world.step(1);
    ch.post_simulation(&world.system, 0.05);
    assert_eq!(ch.ground().state, GroundState::InAir);

    world.system.restore_state(&world_saved).unwrap();
    ch.restore_state(&saved).unwrap();
    assert_eq!(ch.ground().state, GroundState::OnGround);
    assert!(ch.position(&world.system).y < 1.0);

    let other = Character::new(&world.system, &settings, &capsule(), Vec3::new(5.0, 0.0, 0.0), Quat::IDENTITY, 0).unwrap();
    assert!(ch.restore_state(&other.save_state()).is_err());
}

// ---------------------------------------------------------------------------
// Motorcycles and tracked vehicles
// ---------------------------------------------------------------------------

/// Jolt's motorcycle sample: a 240 kg bike with its center of mass lowered and a raked front fork.
fn motorcycle(world: &mut World) -> (BodyId, ConstraintId) {
    let hull = Shape::offset_center_of_mass(Vec3::new(0.0, -0.3, 0.0), &Shape::cuboid(Vec3::new(0.2, 0.3, 0.4)).unwrap()).unwrap();
    let mut body = BodyCreationSettings::new(hull, Vec3::new(0.0, 1.0, 0.0), Quat::IDENTITY, MotionType::Dynamic, world.moving_layer);
    body.override_mass_properties = OverrideMassProperties::CalculateInertia { mass: 240.0 };
    let body = world.system.body_interface().create_and_add_body(&body, Activation::Activate).unwrap();

    let caster = 30f32.to_radians();
    let fork = Vec3::new(0.0, -1.0, caster.tan()) * (1.0 / (1.0 + caster.tan().powi(2)).sqrt());
    let wheel = |z: f32| WheelSettings {
        position: Vec3::new(0.0, -0.27, z),
        radius: 0.31,
        width: 0.05,
        suspension_min_length: 0.3,
        suspension_max_length: 0.5,
        ..Default::default()
    };
    let front = WheelSettings {
        max_steer_angle: 30f32.to_radians(),
        suspension_direction: fork,
        steering_axis: fork * -1.0,
        suspension_spring: SpringSettings::FrequencyAndDamping { frequency: 1.5, damping: 0.5 },
        max_brake_torque: 500.0,
        ..wheel(0.75)
    };
    let back = WheelSettings {
        max_steer_angle: 0.0,
        suspension_spring: SpringSettings::FrequencyAndDamping { frequency: 2.0, damping: 0.5 },
        max_brake_torque: 250.0,
        ..wheel(-0.75)
    };

    let mut settings = MotorcycleSettings::default();
    let v = &mut settings.vehicle;
    v.max_pitch_roll_angle = 60f32.to_radians();
    v.wheels = vec![front, back];
    v.engine.max_torque = 150.0;
    v.engine.min_rpm = 1000.0;
    v.engine.max_rpm = 10000.0;
    v.transmission.shift_down_rpm = 2000.0;
    v.transmission.shift_up_rpm = 8000.0;
    v.transmission.gear_ratios = vec![2.27, 1.63, 1.3, 1.09, 0.96, 0.88];
    v.transmission.reverse_gear_ratios = vec![-4.0];
    v.transmission.clutch_strength = 2.0;
    // Rear wheel drive
    v.differentials.push(VehicleDifferentialSettings { differential_ratio: 1.93 * 40.0 / 16.0, ..VehicleDifferentialSettings::new(None, Some(1)) });
    let tester = VehicleCollisionTester::cast_cylinder(&world.system, world.moving_layer, 1.0).unwrap();
    let bike = world.system.add_motorcycle(&settings, body, &tester).unwrap();
    (body, bike)
}

#[test]
fn motorcycle_drives_upright() {
    let mut world = World::new();
    world.floor();
    let (body, bike) = motorcycle(&mut world);
    {
        let mut v = world.system.vehicle(bike).unwrap();
        assert_eq!(v.kind(), VehicleKind::Motorcycle);
        // Measured at full suspension extension, where the raked fork reaches 0.5 * sin(30 deg) further forward.
        assert!((v.wheel_base().unwrap() - 1.75).abs() < 1e-3, "{:?}", v.wheel_base());
        assert!(v.lean_controller_enabled() && v.lean_steering_limit_enabled());
        v.set_lean_steering_limit_enabled(false).unwrap();
        assert!(!v.lean_steering_limit_enabled());
        v.set_lean_steering_limit_enabled(true).unwrap();
        assert!(v.set_lean_spring(-1.0, 1000.0, 0.0, 4.0, 0.8).is_err());
        v.set_lean_spring(5000.0, 1000.0, 0.0, 4.0, 0.8).unwrap();
        assert!(v.set_track_input(1.0, 1.0, 1.0, 0.0).is_err());
        assert_eq!(v.track_angular_velocity(TrackSide::Left), None);
    }
    world.step(30);
    let start = world.system.body_interface().position(body);
    for _ in 0..240 {
        world.system.vehicle(bike).unwrap().set_driver_input(1.0, 0.0, 0.0, 0.0);
        world.step(1);
    }
    let bodies = world.system.body_interface();
    let p = bodies.position(body);
    assert!(p.z - start.z > 5.0, "bike should drive forward: {start:?} -> {p:?}");
    let up = bodies.rotation(body).rotate(Vec3::Y);
    assert!(up.y > 0.8, "bike should stay upright: up {up:?}");
}

fn tank(world: &mut World) -> (BodyId, ConstraintId) {
    let body = world.vehicle_body(Vec3::new(1.5, 0.4, 3.0), Vec3::new(0.0, 1.2, 0.0), 4000.0);
    let mut settings = TrackedVehicleSettings::default();
    let zs = [2.5, 1.25, 0.0, -1.25, -2.5];
    for x in [-1.4f32, 1.4] {
        for z in zs {
            settings.wheels.push(TrackedWheelSettings {
                radius: 0.3,
                width: 0.4,
                suspension_min_length: 0.3,
                suspension_max_length: 0.5,
                ..TrackedWheelSettings::at(Vec3::new(x, -0.4, z))
            });
        }
    }
    let n = zs.len() as u32;
    settings.tracks[0].wheels = (0..n).collect();
    settings.tracks[0].driven_wheel = n - 1;
    settings.tracks[1].wheels = (n..2 * n).collect();
    settings.tracks[1].driven_wheel = 2 * n - 1;
    let tester = VehicleCollisionTester::cast_cylinder(&world.system, world.moving_layer, 1.0).unwrap();
    let tank = world.system.add_tracked_vehicle(&settings, body, &tester).unwrap();
    (body, tank)
}

#[test]
fn tracked_vehicle_drives_and_turns_on_the_spot() {
    let mut world = World::new();
    world.floor();
    let (body, tank) = tank(&mut world);
    world.step(30);
    {
        let mut v = world.system.vehicle(tank).unwrap();
        assert_eq!(v.kind(), VehicleKind::Tracked);
        assert_eq!(v.wheel_count(), 10);
        assert!(v.wheel(0).unwrap().contact_body.is_some());
        assert_eq!(v.wheel_base(), None);
        assert!(v.set_lean_controller_enabled(false).is_err());
        // Wheeled input is ignored rather than misread.
        v.set_driver_input(1.0, 1.0, 0.0, 0.0);
    }

    let start = world.system.body_interface().position(body);
    for _ in 0..180 {
        world.system.vehicle(tank).unwrap().set_track_input(1.0, 1.0, 1.0, 0.0).unwrap();
        world.step(1);
    }
    let bodies = world.system.body_interface();
    let p = bodies.position(body);
    assert!(p.z - start.z > 3.0, "tank should drive forward: {start:?} -> {p:?}");
    assert!(p.x.abs() < 0.5, "tank should drive straight: {p:?}");
    {
        let v = world.system.vehicle(tank).unwrap();
        let (left, right) = (v.track_angular_velocity(TrackSide::Left).unwrap(), v.track_angular_velocity(TrackSide::Right).unwrap());
        assert!(left > 1.0 && (left - right).abs() < 0.1 * left, "tracks {left} {right}");
        assert!(v.engine_rpm() > 0.0);
    }

    // Stop, then turn on the spot.
    for _ in 0..120 {
        world.system.vehicle(tank).unwrap().set_track_input(0.0, 1.0, 1.0, 1.0).unwrap();
        world.step(1);
    }
    let before = world.system.body_interface().position(body);
    let heading = |w: &World| {
        let f = w.system.body_interface().rotation(body).rotate(Vec3::Z);
        f.x.atan2(f.z)
    };
    let heading_before = heading(&world);
    for _ in 0..180 {
        world.system.vehicle(tank).unwrap().set_track_input(1.0, 1.0, -1.0, 0.0).unwrap();
        world.step(1);
    }
    let turned = heading(&world) - heading_before;
    let after = world.system.body_interface().position(body);
    assert!(turned.abs() > 0.3, "tank should turn: {turned}");
    assert!(close(Vec3::new(after.x, 0.0, after.z), Vec3::new(before.x, 0.0, before.z), 1.5), "turning on the spot: {before:?} -> {after:?}");
}

#[test]
fn tracked_vehicle_settings_are_validated() {
    let mut world = World::new();
    let body = world.vehicle_body(Vec3::splat(1.0), Vec3::ZERO, 1000.0);
    let tester = VehicleCollisionTester::ray(&world.system, world.moving_layer, Vec3::Y, 1.0).unwrap();
    let base = || {
        let mut s = TrackedVehicleSettings {
            wheels: vec![TrackedWheelSettings::at(Vec3::new(-1.0, 0.0, 0.0)), TrackedWheelSettings::at(Vec3::new(1.0, 0.0, 0.0))],
            ..Default::default()
        };
        s.tracks[0].wheels = vec![0];
        s.tracks[0].driven_wheel = 0;
        s.tracks[1].wheels = vec![1];
        s.tracks[1].driven_wheel = 1;
        s
    };
    let err = |s: &TrackedVehicleSettings, world: &mut World| world.system.add_tracked_vehicle(s, body, &tester).unwrap_err().message().to_owned();

    let mut s = base();
    s.tracks[1].wheels.clear();
    assert!(err(&s, &mut world).contains("track"));
    let mut s = base();
    s.tracks[0].driven_wheel = 1;
    assert!(err(&s, &mut world).contains("driven_wheel"));
    let mut s = base();
    s.tracks[1].wheels = vec![1, 7];
    assert!(err(&s, &mut world).contains("does not exist"));
    let mut s = base();
    s.tracks[1].wheels = vec![0, 1];
    assert!(err(&s, &mut world).contains("exactly one track"));
    let mut s = base();
    s.wheels.push(TrackedWheelSettings::default());
    assert!(err(&s, &mut world).contains("exactly one track"));
    let mut s = base();
    s.tracks[0].inertia = 0.0;
    assert!(err(&s, &mut world).contains("inertia"));
    let mut s = base();
    s.transmission.gear_ratios.clear();
    assert!(err(&s, &mut world).contains("gear"));

    let tank = world.system.add_tracked_vehicle(&base(), body, &tester).unwrap();
    // Zero ratios would divide by zero inside Jolt; they are nudged instead.
    world.system.vehicle(tank).unwrap().set_track_input(1.0, 0.0, 0.0, 0.0).unwrap();
    world.step(5);
    assert!(world.system.body_interface().position(body).y.is_finite());
}

#[test]
fn motorcycle_settings_are_validated() {
    let mut world = World::new();
    let body = world.vehicle_body(Vec3::splat(0.5), Vec3::ZERO, 200.0);
    let tester = VehicleCollisionTester::ray(&world.system, world.moving_layer, Vec3::Y, 1.0).unwrap();
    let mut s = MotorcycleSettings::default();
    s.vehicle.wheels.push(WheelSettings::default());
    assert!(world.system.add_motorcycle(&s, body, &tester).unwrap_err().message().contains("two wheels"));
    s.vehicle.wheels.push(WheelSettings::at(Vec3::new(0.0, 0.0, -1.0)));
    s.lean_smoothing_factor = 1.0;
    assert!(world.system.add_motorcycle(&s, body, &tester).is_err());
    s.lean_smoothing_factor = 0.8;
    s.max_lean_angle = 3.0;
    assert!(world.system.add_motorcycle(&s, body, &tester).is_err());
    s.max_lean_angle = 0.7;
    world.system.add_motorcycle(&s, body, &tester).unwrap();
}
