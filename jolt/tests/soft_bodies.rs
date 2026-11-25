//! Soft bodies (and the rigid-only calls that must refuse them), collected faces, skeletal animation and mapping.

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

    fn soft_cube(&self, position: Vec3) -> BodyId {
        let settings = SoftBodyCreationSettings::new(SoftBodySharedSettings::cube(5, 0.2).unwrap(), position, Quat::IDENTITY, self.moving_layer);
        self.system.body_interface().create_and_add_soft_body(&settings, Activation::Activate).unwrap()
    }
}

/// An n x n cloth in the XZ plane, centred on the origin, pinned at the two corners with the highest z.
fn cloth(n: u32, spacing: f32) -> SoftBodyDesc {
    let mut desc = SoftBodyDesc::default();
    let half = (n - 1) as f32 * spacing * 0.5;
    for z in 0..n {
        for x in 0..n {
            let position = Vec3::new(x as f32 * spacing - half, 0.0, z as f32 * spacing - half);
            let pinned = z == n - 1 && (x == 0 || x == n - 1);
            desc.vertices.push(if pinned { SoftBodyVertex::pinned(position) } else { SoftBodyVertex::new(position) });
        }
    }
    for z in 0..n - 1 {
        for x in 0..n - 1 {
            let i = x + z * n;
            desc.faces.push(SoftBodyFace::new([i, i + n, i + 1]));
            desc.faces.push(SoftBodyFace::new([i + 1, i + n, i + n + 1]));
        }
    }
    desc.vertex_attributes.push(VertexAttributes { compliance: 1.0e-5, shear_compliance: 1.0e-5, bend_compliance: 1.0e-3, ..Default::default() });
    desc
}

// ---------------------------------------------------------------------------
// Soft bodies
// ---------------------------------------------------------------------------

#[test]
fn cloth_hangs_from_its_pinned_corners() {
    let mut world = World::new();
    let n = 10;
    let settings = SoftBodySharedSettings::new(&cloth(n, 0.1)).unwrap();
    let counts = settings.counts();
    assert_eq!((counts.vertices, counts.faces), (100, 162));
    assert!(counts.edges > 0 && counts.lras == 0, "{counts:?}");
    assert_eq!(settings.faces().len(), 162);

    let body = world
        .system
        .body_interface()
        .create_and_add_soft_body(&SoftBodyCreationSettings::new(settings.clone(), Vec3::new(0.0, 5.0, 0.0), Quat::IDENTITY, world.moving_layer), Activation::Activate)
        .unwrap();
    let bodies = world.system.body_interface();
    assert!(bodies.is_soft_body(body));
    assert_eq!(bodies.soft_body_shared_settings(body), Some(settings.clone()));
    let before = bodies.soft_body_vertices(body, true).unwrap();
    assert_eq!(before.len(), 100);
    assert!((before[0].position.y - 5.0).abs() < 1e-5);

    world.step(120);
    let after = world.system.body_interface().soft_body_vertices(body, true).unwrap();
    let pinned = (n * n - 1) as usize;
    assert!((after[pinned].position - before[pinned].position).length() < 1e-4, "pinned vertices stay put");
    assert!(after[0].position.y < 4.8, "the free edge falls: {:?}", after[0].position);
    // The cloth hangs instead of falling or stretching: its far edge is at most about its length (0.9) below the pins.
    assert!(after[0].position.y > 3.8, "{:?}", after[0].position);

    // Queries see the cloth.
    let hit = world.system.cast_ray(Vec3::new(0.0, 10.0, 0.0), Vec3::new(0.0, -10.0, 0.0), &QueryFilter::new());
    assert_eq!(hit.map(|h| h.body_id), Some(body));

    // Local vertices are relative to the center of mass transform.
    let local = world.system.body_interface().soft_body_vertices(body, false).unwrap();
    let com = world.system.body_interface().center_of_mass_position(body);
    assert!((local[0].position + com - after[0].position).length() < 1e-3);
}

#[test]
fn soft_body_lands_and_can_be_moved() {
    let mut world = World::new();
    world.floor();
    let body = world.soft_cube(Vec3::new(0.0, 2.0, 0.0));
    let bodies = world.system.body_interface();
    let volume = bodies.soft_body_volume(body);
    assert!((volume - 0.8f32.powi(3)).abs() < 0.01, "{volume}");
    assert!(bodies.mass(body).unwrap() > 0.0);

    world.step(120);
    let vertices = world.system.body_interface().soft_body_vertices(body, true).unwrap();
    let lowest = vertices.iter().map(|v| v.position.y).fold(f32::MAX, f32::min);
    assert!(lowest.abs() < 0.1, "the cube rests on the floor: {lowest}");

    // Pin a vertex and drag it.
    let bodies = world.system.body_interface();
    let mut first = vertices[0];
    first.inv_mass = 0.0;
    first.position += Vec3::new(0.0, 1.0, 0.0);
    first.velocity = Vec3::ZERO;
    bodies.set_soft_body_vertices(body, 0, &[first], true, Activation::Activate).unwrap();
    assert_eq!(bodies.mass(body), None, "a pinned vertex gives the body infinite mass");
    let read = bodies.soft_body_vertices(body, true).unwrap();
    assert!((read[0].position - first.position).length() < 1e-4);
    assert!(bodies.set_soft_body_vertices(body, vertices.len(), &[first], true, Activation::Activate).is_err());
    assert!(bodies.set_soft_body_vertices(body, 0, &[SoftBodyVertexState { inv_mass: -1.0, ..first }], true, Activation::Activate).is_err());
    world.step(30);
    let read = world.system.body_interface().soft_body_vertices(body, true).unwrap();
    assert!((read[0].position - first.position).length() < 1e-3, "pinned vertices don't move");

    // Properties round trip.
    let bodies = world.system.body_interface();
    let mut properties = bodies.soft_body_properties(body).unwrap();
    properties.pressure = 100.0;
    properties.num_iterations = 8;
    bodies.set_soft_body_properties(body, &properties).unwrap();
    assert_eq!(bodies.soft_body_properties(body), Some(properties));
    assert!(bodies.set_soft_body_properties(body, &SoftBodyProperties { vertex_radius: -1.0, ..properties }).is_err());
}

#[test]
fn soft_body_settings_are_validated() {
    let triangle = || SoftBodyDesc {
        vertices: vec![SoftBodyVertex::new(Vec3::ZERO), SoftBodyVertex::new(Vec3::X), SoftBodyVertex::new(Vec3::Z)],
        faces: vec![SoftBodyFace::new([0, 1, 2])],
        ..Default::default()
    };
    SoftBodySharedSettings::new(&triangle()).unwrap();
    let err = |d: SoftBodyDesc| SoftBodySharedSettings::new(&d).unwrap_err().message().to_owned();

    assert!(err(SoftBodyDesc::default()).contains("vertex"));
    assert!(err(SoftBodyDesc { faces: vec![SoftBodyFace::new([0, 1, 7])], ..triangle() }).contains("face"));
    assert!(err(SoftBodyDesc { faces: vec![SoftBodyFace::new([0, 1, 1])], ..triangle() }).contains("face"));
    assert!(err(SoftBodyDesc { faces: vec![SoftBodyFace { vertices: [0, 1, 2], material_index: 1 }], ..triangle() }).contains("material"));
    let mut d = triangle();
    d.vertices[0].inv_mass = -1.0;
    assert!(err(d).contains("inverse mass"));
    let mut d = triangle();
    d.vertices.push(SoftBodyVertex::new(Vec3::ZERO));
    d.edges.push(SoftBodyEdge { vertices: [0, 3], compliance: 0.0 });
    assert!(err(d).contains("same position"));
    // Generating constraints from a face whose corners coincide would make an edge of length 0.
    let mut d = triangle();
    d.vertices[2].position = d.vertices[1].position;
    d.vertex_attributes.push(VertexAttributes::default());
    assert!(err(d).contains("same position"));
    let mut d = triangle();
    d.vertices.push(SoftBodyVertex::new(Vec3::Z));
    d.faces.push(SoftBodyFace::new([1, 0, 3])); // folds back onto the first face: its opposite corner coincides
    d.vertex_attributes.push(VertexAttributes::default());
    assert!(err(d).contains("same position"));
    let mut d = triangle();
    d.lras.push(SoftBodyLra { vertices: [0, 1], max_distance_multiplier: 1.0 });
    d.vertices[1].position = Vec3::ZERO;
    assert!(err(d).contains("same position"));
    let mut d = triangle();
    d.edges.push(SoftBodyEdge { vertices: [0, 1], compliance: -1.0 });
    assert!(err(d).contains("compliance"));
    let mut d = triangle();
    d.volumes.push(SoftBodyVolume { vertices: [0, 1, 2, 2], compliance: 0.0 });
    assert!(err(d).contains("volume"));
    let mut d = triangle();
    d.lras.push(SoftBodyLra { vertices: [0, 1], max_distance_multiplier: 0.0 });
    assert!(err(d).contains("LRA"));
    let mut d = triangle();
    d.vertex_attributes.push(VertexAttributes { lra_max_distance_multiplier: 0.0, ..Default::default() });
    assert!(err(d).contains("attributes"));

    // Explicit constraints come on top of the generated ones.
    let mut d = triangle();
    d.vertex_attributes.push(VertexAttributes { lra_type: LraType::EuclideanDistance, ..Default::default() });
    d.vertices[0].inv_mass = 0.0;
    d.vertices.push(SoftBodyVertex::new(Vec3::new(0.0, 1.0, 0.0)));
    d.volumes.push(SoftBodyVolume { vertices: [0, 1, 2, 3], compliance: 0.0 });
    d.dihedral_bends.push(SoftBodyDihedralBend { vertices: [1, 2, 0, 3], compliance: 0.0 });
    let counts = SoftBodySharedSettings::new(&d).unwrap().counts();
    assert_eq!((counts.edges, counts.volumes, counts.dihedral_bends, counts.lras), (3, 1, 1, 2), "{counts:?}");

    assert!(SoftBodySharedSettings::cube(1, 0.1).is_err());
    assert!(SoftBodySharedSettings::cube(65, 0.1).is_err());
    assert!(SoftBodySharedSettings::cube(3, 0.0).is_err());

    let world = World::new();
    let settings = SoftBodySharedSettings::new(&triangle()).unwrap();
    let bodies = world.system.body_interface();
    assert!(bodies.create_soft_body(&SoftBodyCreationSettings::new(settings.clone(), Vec3::ZERO, Quat::IDENTITY, ObjectLayer(9))).is_err());
    let mut s = SoftBodyCreationSettings::new(settings, Vec3::ZERO, Quat::IDENTITY, world.moving_layer);
    s.vertex_radius = -1.0;
    assert!(bodies.create_soft_body(&s).is_err());
}

#[test]
fn rigid_only_calls_refuse_soft_bodies() {
    let mut world = World::new();
    world.floor();
    let soft = world.soft_cube(Vec3::new(0.0, 3.0, 0.0));
    let rigid = world.add(Shape::sphere(0.5).unwrap(), Vec3::new(5.0, 3.0, 0.0), MotionType::Dynamic);
    let bodies = world.system.body_interface();

    // The soft body's shape lives inside the body, so no handle to it may escape.
    assert!(bodies.shape(soft).is_none());
    bodies.set_shape(soft, &Shape::sphere(1.0).unwrap(), true, Activation::Activate);
    assert!(bodies.shape(soft).is_none() && bodies.soft_body_vertices(soft, false).is_some());
    assert!(bodies.set_motion_type(soft, MotionType::Kinematic, Activation::Activate).is_err());
    assert!(bodies.set_motion_type(soft, MotionType::Static, Activation::Activate).is_err());
    bodies.set_motion_type(soft, MotionType::Dynamic, Activation::Activate).unwrap();
    let before = bodies.position(soft);
    bodies.move_kinematic(soft, Vec3::new(0.0, 50.0, 0.0), Quat::IDENTITY, DT);
    bodies.set_is_sensor(soft, true);
    assert!(!bodies.is_sensor(soft));
    assert!(!bodies.apply_buoyancy_impulse(soft, Vec3::new(0.0, 10.0, 0.0), Vec3::Y, 1.0, 0.3, 0.05, Vec3::ZERO, Vec3::new(0.0, -9.81, 0.0), DT));
    assert!(!bodies.is_soft_body(rigid));

    // Constraints and vehicles would corrupt Jolt's islands.
    let err = world.system.add_constraint(&PointConstraintSettings::at(Vec3::new(0.0, 3.0, 0.0)), soft, rigid).unwrap_err();
    assert!(err.message().contains("soft"), "{err}");
    assert!(world.system.add_constraint_to_world(&FixedConstraintSettings::auto_detect(), soft).is_err());
    let tester = VehicleCollisionTester::ray(&world.system, world.moving_layer, Vec3::Y, 1.0).unwrap();
    let mut car = WheeledVehicleSettings::default();
    car.wheels.push(WheelSettings::default());
    assert!(world.system.add_vehicle(&car, soft, &tester).unwrap_err().message().contains("rigid"));
    assert!(world.system.constraint_ids().is_empty());

    world.step(10);
    let after = world.system.body_interface().position(soft);
    assert!(after.y <= before.y + 1e-3, "move_kinematic did nothing: {before:?} -> {after:?}");

    // Static bodies have no inertia to read.
    let floor_inertia = world.system.body_interface().inverse_inertia(world.floor());
    assert_eq!(floor_inertia.columns, [[0.0; 4]; 4]);
}

#[test]
fn vehicle_wheels_ignore_soft_bodies() {
    let mut world = World::new();
    world.floor();
    // A soft cube right under the car's front wheels.
    let soft = world.soft_cube(Vec3::new(0.0, 0.4, 1.4));
    let mut body = BodyCreationSettings::new(Shape::cuboid(Vec3::new(0.9, 0.2, 2.0)).unwrap(), Vec3::new(0.0, 2.0, 0.0), Quat::IDENTITY, MotionType::Dynamic, world.moving_layer);
    body.override_mass_properties = OverrideMassProperties::CalculateInertia { mass: 1500.0 };
    let body = world.system.body_interface().create_and_add_body(&body, Activation::Activate).unwrap();
    let mut settings = WheeledVehicleSettings::default();
    for (x, z) in [(-0.9, 1.4), (0.9, 1.4), (-0.9, -1.4), (0.9, -1.4)] {
        settings.wheels.push(WheelSettings::at(Vec3::new(x, -0.2, z)));
    }
    settings.differentials.push(VehicleDifferentialSettings::new(Some(0), Some(1)));
    // One tester shared by two vehicles: each still ignores its own body.
    let tester = VehicleCollisionTester::cast_cylinder(&world.system, world.moving_layer, 0.5).unwrap();
    let car = world.system.add_vehicle(&settings, body, &tester).unwrap();
    let mut other = BodyCreationSettings::new(Shape::cuboid(Vec3::new(0.9, 0.2, 2.0)).unwrap(), Vec3::new(10.0, 2.0, 0.0), Quat::IDENTITY, MotionType::Dynamic, world.moving_layer);
    other.override_mass_properties = OverrideMassProperties::CalculateInertia { mass: 1500.0 };
    let other = world.system.body_interface().create_and_add_body(&other, Activation::Activate).unwrap();
    let other_car = world.system.add_vehicle(&settings, other, &tester).unwrap();

    for _ in 0..90 {
        world.system.vehicle(car).unwrap().set_driver_input(0.3, 0.0, 0.0, 0.0);
        world.step(1);
        let v = world.system.vehicle(car).unwrap();
        for i in 0..v.wheel_count() {
            let contact = v.wheel(i).unwrap().contact_body;
            assert!(contact != Some(soft) && contact != Some(body), "wheel {i} touched {contact:?}");
        }
    }
    let v = world.system.vehicle(other_car).unwrap();
    assert!((0..4).all(|i| v.wheel(i).unwrap().contact_body.is_some_and(|b| b != other)));
    assert!(world.system.body_interface().position(body).y.is_finite());
}

#[test]
fn soft_bodies_roll_back_with_snapshots() {
    let mut world = World::new();
    world.floor();
    let body = world.soft_cube(Vec3::new(0.0, 2.0, 0.0));
    world.step(10);
    let snapshot = world.system.save_state(StateParts::ALL);
    world.step(30);
    let expected = world.system.body_interface().soft_body_vertices(body, true).unwrap();
    world.system.restore_state(&snapshot).unwrap();
    world.step(30);
    let replayed = world.system.body_interface().soft_body_vertices(body, true).unwrap();
    assert_eq!(expected, replayed);

    world.system.destroy_body(body);
    assert!(world.system.body_interface().soft_body_vertices(body, true).is_none());
    assert!(world.system.restore_state(&snapshot).is_err());
}

// ---------------------------------------------------------------------------
// Collected faces
// ---------------------------------------------------------------------------

#[test]
fn shape_queries_collect_faces() {
    let world = World::new();
    world.floor();
    let shape = Shape::cuboid(Vec3::splat(0.5)).unwrap();
    let transform = Mat44::from_rotation_translation(Quat::IDENTITY, Vec3::new(0.0, 0.45, 0.0));

    let mut hits = Vec::new();
    world.system.collide_shape(&shape, Vec3::splat(1.0), &transform, 0.0, &QueryFilter::new(), |h| {
        hits.push(h.clone());
        ControlFlow::Continue(())
    });
    assert!(!hits.is_empty() && hits.iter().all(|h| h.face1.is_empty() && h.face2.is_empty()));

    let settings = CollideShapeSettings { collect_faces: true, ..Default::default() };
    let mut hits = Vec::new();
    world.system.collide_shape_with_settings(&shape, Vec3::splat(1.0), &transform, &settings, &QueryFilter::new(), |h| {
        hits.push(h.clone());
        ControlFlow::Continue(())
    });
    let hit = hits.first().expect("box touches the floor");
    assert_eq!(hit.face1.len(), 4, "bottom face of the box: {:?}", hit.face1);
    assert!(hit.face1.iter().all(|p| (p.y + 0.05).abs() < 1e-3));
    assert!(hit.face2.len() >= 3 && hit.face2.iter().all(|p| p.y.abs() < 1e-3), "top face of the floor: {:?}", hit.face2);

    let settings = ShapeCastSettings { collect_faces: true, ..Default::default() };
    let start = Mat44::from_rotation_translation(Quat::IDENTITY, Vec3::new(0.0, 3.0, 0.0));
    let mut hits = Vec::new();
    world.system.cast_shape_with_settings(&shape, Vec3::splat(1.0), &start, Vec3::new(0.0, -5.0, 0.0), &settings, &QueryFilter::new(), |h| {
        hits.push(h.clone());
        ControlFlow::Continue(())
    });
    assert!(hits.first().is_some_and(|h| !h.face1.is_empty() && !h.face2.is_empty()));
}

// ---------------------------------------------------------------------------
// Animation
// ---------------------------------------------------------------------------

fn arm() -> Skeleton {
    let mut b = SkeletonBuilder::new();
    let root = b.add_joint("root", None).unwrap();
    let elbow = b.add_joint("elbow", Some(root)).unwrap();
    b.add_joint("hand", Some(elbow)).unwrap();
    b.build()
}

fn key(time: f32, x: f32, angle: f32) -> Keyframe {
    Keyframe { time, translation: Vec3::new(x, 0.0, 0.0), rotation: Quat::from_axis_angle(Vec3::Y, angle) }
}

#[test]
fn animations_sample_between_keyframes() {
    let skeleton = arm();
    let walk = SkeletalAnimation::new(
        &[
            AnimatedJoint { name: "elbow".into(), keyframes: vec![key(0.0, 1.0, 0.0), key(1.0, 3.0, 1.0)] },
            AnimatedJoint { name: "hand".into(), keyframes: vec![key(0.5, 0.5, 0.0)] },
        ],
        true,
    )
    .unwrap();
    assert_eq!(walk.duration(), 1.0);
    assert!(walk.is_looping());

    let mut states = vec![JointState { translation: Vec3::new(9.0, 9.0, 9.0), rotation: Quat::IDENTITY }; 3];
    walk.sample(&skeleton, 0.25, &mut states).unwrap();
    assert_eq!(states[0].translation, Vec3::new(9.0, 9.0, 9.0), "joints that aren't animated keep their state");
    assert!((states[1].translation.x - 1.5).abs() < 1e-5);
    let expected = Quat::from_axis_angle(Vec3::Y, 0.25);
    assert!((states[1].rotation.y - expected.y).abs() < 1e-4 && (states[1].rotation.w - expected.w).abs() < 1e-4);
    assert_eq!(states[2].translation.x, 0.5, "a single keyframe holds");

    // Looping wraps around, negative times clamp to the start.
    walk.sample(&skeleton, 1.25, &mut states).unwrap();
    assert!((states[1].translation.x - 1.5).abs() < 1e-4);
    walk.sample(&skeleton, -3.0, &mut states).unwrap();
    assert!((states[1].translation.x - 1.0).abs() < 1e-5);
    let once = SkeletalAnimation::new(&[AnimatedJoint { name: "elbow".into(), keyframes: vec![key(0.0, 1.0, 0.0), key(1.0, 3.0, 0.0)] }], false).unwrap();
    once.sample(&skeleton, 5.0, &mut states).unwrap();
    assert_eq!(states[1].translation.x, 3.0);

    // A joint the skeleton lacks is refused rather than written out of bounds.
    let other = SkeletalAnimation::new(&[AnimatedJoint { name: "tail".into(), keyframes: vec![key(0.0, 1.0, 0.0)] }], true).unwrap();
    let before = states.clone();
    assert!(other.sample(&skeleton, 0.0, &mut states).is_err());
    assert_eq!(states, before);

    assert!(SkeletalAnimation::new(&[AnimatedJoint { name: "elbow".into(), keyframes: vec![] }], true).is_err());
    assert!(SkeletalAnimation::new(&[AnimatedJoint { name: "elbow".into(), keyframes: vec![key(1.0, 0.0, 0.0), key(0.0, 0.0, 0.0)] }], true).is_err());
    let zero = Keyframe { rotation: Quat::from_xyzw(0.0, 0.0, 0.0, 0.0), ..key(0.0, 0.0, 0.0) };
    assert!(SkeletalAnimation::new(&[AnimatedJoint { name: "elbow".into(), keyframes: vec![zero] }], true).is_err());
}

#[test]
fn skeleton_mapper_maps_between_skeletons() {
    // A two joint ragdoll skeleton and a detailed arm with an extra elbow joint in between.
    let ragdoll = {
        let mut b = SkeletonBuilder::new();
        let root = b.add_joint("root", None).unwrap();
        b.add_joint("hand", Some(root)).unwrap();
        b.build()
    };
    let detailed = arm();
    let detailed_local = [Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)]
        .map(|t| JointState { translation: t, rotation: Quat::IDENTITY })
        .to_vec();
    let neutral2 = detailed.joint_matrices(&detailed_local);
    let neutral1 = vec![neutral2[0], neutral2[2]];

    let mapper = SkeletonMapper::new(&ragdoll, &neutral1, &detailed, &neutral2, &LockedTranslations::None).unwrap();
    assert_eq!(mapper.joint_counts(), (2, 3));
    assert_eq!(mapper.mapped_joint(0), Some(0));
    assert_eq!(mapper.mapped_joint(1), Some(2));
    assert_eq!(mapper.mapped_joint(7), None);

    // Swing the ragdoll's hand up: the elbow follows along the chain.
    let pose1 = vec![Mat44::IDENTITY, Mat44::from_rotation_translation(Quat::IDENTITY, Vec3::new(0.0, 2.0, 0.0))];
    let local2: Vec<Mat44> =
        detailed_local.iter().map(|s| Mat44::from_rotation_translation(s.rotation, s.translation)).collect();
    let mut out2 = vec![Mat44::IDENTITY; 3];
    mapper.map(&pose1, &local2, &mut out2);
    let elbow = out2[1].translation();
    assert!((elbow - Vec3::new(0.0, 1.0, 0.0)).length() < 1e-3, "{elbow:?}");
    assert!((out2[2].translation() - Vec3::new(0.0, 2.0, 0.0)).length() < 1e-3);

    let mut back = vec![Mat44::IDENTITY; 2];
    mapper.map_reverse(&out2, &mut back);
    assert!((back[1].translation() - Vec3::new(0.0, 2.0, 0.0)).length() < 1e-3);

    // Locking: roots are ignored (Jolt would index their parent, -1), the rest keep their neutral offset.
    let locked = SkeletonMapper::new(&ragdoll, &neutral1, &detailed, &neutral2, &LockedTranslations::Joints(vec![true, true, true])).unwrap();
    assert!(!locked.is_translation_locked(0) && locked.is_translation_locked(1));
    locked.map(&pose1, &local2, &mut out2);
    let all = SkeletonMapper::new(&ragdoll, &neutral1, &detailed, &neutral2, &LockedTranslations::All).unwrap();
    assert!(all.is_translation_locked(2) && !all.is_translation_locked(9));
    all.map(&pose1, &local2, &mut out2);

    assert!(SkeletonMapper::new(&detailed, &neutral2, &ragdoll, &neutral1, &LockedTranslations::None).is_err());
    // Without any mapping, locking everything is a no-op instead of reading the first mapping.
    let unrelated = {
        let mut b = SkeletonBuilder::new();
        b.add_joint("other", None).unwrap();
        b.build()
    };
    SkeletonMapper::new(&unrelated, &[Mat44::IDENTITY], &detailed, &neutral2, &LockedTranslations::All).unwrap();
}

#[test]
fn vehicle_testers_are_checked_against_the_vehicle_system() {
    let mut world = World::new();
    // A tester made for a system with more layers than this one has.
    let mut layers = LayerConfig::new();
    for _ in 0..5 {
        layers.add_object_layer(BroadPhaseLayer(0));
    }
    let big = PhysicsSystem::new(&PhysicsSystemSettings::default(), &layers).unwrap();
    let foreign = VehicleCollisionTester::ray(&big, ObjectLayer(4), Vec3::Y, 1.0).unwrap();
    let local = VehicleCollisionTester::ray(&world.system, world.moving_layer, Vec3::Y, 1.0).unwrap();

    let body = world.add(Shape::cuboid(Vec3::splat(1.0)).unwrap(), Vec3::ZERO, MotionType::Dynamic);
    let mut settings = WheeledVehicleSettings::default();
    settings.wheels.push(WheelSettings::default());
    // Jolt's asserts expect the engine to drive something.
    settings.differentials.push(VehicleDifferentialSettings::new(Some(0), None));
    assert!(world.system.add_vehicle(&settings, body, &foreign).unwrap_err().message().contains("layer"));
    let car = world.system.add_vehicle(&settings, body, &local).unwrap();
    let mut v = world.system.vehicle(car).unwrap();
    assert!(v.set_collision_tester(&foreign).is_err());
    v.set_collision_tester(&local).unwrap();
    world.step(5);
}
