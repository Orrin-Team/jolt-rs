use std::ops::ControlFlow;
use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};

use jolt::*;

const DT: f32 = 1.0 / 60.0;

struct World {
    system: PhysicsSystem,
    temp: TempAllocator,
    jobs: JobSystem,
    static_layer: ObjectLayer,
    moving_layer: ObjectLayer,
    ghost_layer: ObjectLayer,
    floor: BodyId,
}

impl World {
    fn new() -> Self {
        let mut layers = LayerConfig::new();
        let static_layer = layers.add_object_layer(BroadPhaseLayer(0));
        let moving_layer = layers.add_object_layer(BroadPhaseLayer(1));
        // Collides with nothing, to check layer filtering.
        let ghost_layer = layers.add_object_layer(BroadPhaseLayer(1));
        layers.enable_collision(static_layer, moving_layer).enable_collision(moving_layer, moving_layer);

        let system = PhysicsSystem::new(&PhysicsSystemSettings { max_bodies: 1024, ..Default::default() }, &layers).unwrap();
        let floor = BodyCreationSettings::new(
            Shape::cuboid(Vec3::new(50.0, 1.0, 50.0)).unwrap(),
            Vec3::new(0.0, -1.0, 0.0),
            Quat::IDENTITY,
            MotionType::Static,
            static_layer,
        );
        let floor = system.body_interface().create_and_add_body(&floor, Activation::DontActivate).unwrap();
        Self { system, temp: TempAllocator::default(), jobs: JobSystem::thread_pool(Some(2)), static_layer, moving_layer, ghost_layer, floor }
    }

    fn add_dynamic(&self, shape: Shape, position: Vec3) -> BodyId {
        let settings = BodyCreationSettings::new(shape, position, Quat::IDENTITY, MotionType::Dynamic, self.moving_layer);
        self.system.body_interface().create_and_add_body(&settings, Activation::Activate).unwrap()
    }

    fn step(&mut self, n: usize) {
        for _ in 0..n {
            self.system.update(DT, 1, &mut self.temp, &self.jobs).unwrap();
        }
    }
}

#[test]
fn sphere_falls_and_comes_to_rest() {
    let mut world = World::new();
    let ball = world.add_dynamic(Shape::sphere(0.5).unwrap(), Vec3::new(0.0, 5.0, 0.0));
    world.step(300);
    let bi = world.system.body_interface();
    assert!((bi.position(ball).y - 0.5).abs() < 0.05, "ball at {:?}", bi.position(ball));
    assert!(!bi.is_active(ball), "ball should have gone to sleep");
    assert_eq!(world.system.num_bodies(), 2);
    assert_eq!(world.system.bodies().len(), 2);
}

#[test]
fn gravity_and_velocity() {
    let mut world = World::new();
    world.system.set_gravity(Vec3::ZERO);
    let ball = world.add_dynamic(Shape::sphere(0.5).unwrap(), Vec3::new(0.0, 10.0, 0.0));
    let bi = world.system.body_interface();
    bi.set_linear_velocity(ball, Vec3::new(1.0, 0.0, 0.0));
    bi.set_gravity_factor(ball, 0.0);
    world.step(60);
    let bi = world.system.body_interface();
    let p = bi.position(ball);
    // Linear damping (0.05 by default) slows the ball slightly.
    assert!((p.x - 1.0).abs() < 0.05 && (p.y - 10.0).abs() < 1e-4, "{p:?}");
}

struct CountingListener {
    added: Arc<AtomicU32>,
    removed: Arc<AtomicU32>,
}

impl ContactListener for CountingListener {
    fn on_contact_added(&self, _: &Body, _: &Body, manifold: &ContactManifold<'_>, settings: &mut ContactSettings) {
        assert!(!manifold.relative_points_on1.is_empty());
        assert_eq!(manifold.relative_points_on1.len(), manifold.relative_points_on2.len());
        settings.combined_restitution = 1.0;
        self.added.fetch_add(1, Ordering::Relaxed);
    }

    fn on_contact_removed(&self, _: &SubShapeIdPair) {
        self.removed.fetch_add(1, Ordering::Relaxed);
    }
}

#[test]
fn contact_listener_sees_and_modifies_contacts() {
    let mut world = World::new();
    let added = Arc::new(AtomicU32::new(0));
    let removed = Arc::new(AtomicU32::new(0));
    world.system.set_contact_listener(CountingListener { added: added.clone(), removed: removed.clone() });
    let ball = world.add_dynamic(Shape::sphere(0.5).unwrap(), Vec3::new(0.0, 3.0, 0.0));

    // Fall until the first contact, then a few frames more: restitution 1 must send the ball back up.
    let mut frames = 0;
    while added.load(Ordering::Relaxed) == 0 && frames < 120 {
        world.step(1);
        frames += 1;
    }
    world.step(5);
    assert_eq!(added.load(Ordering::Relaxed), 1);
    assert!(world.system.body_interface().linear_velocity(ball).y > 1.0, "ball should bounce");
    world.step(10);
    assert!(removed.load(Ordering::Relaxed) >= 1, "contact should be removed once the ball leaves the floor");
}

struct PanickingListener;

impl ContactListener for PanickingListener {
    fn on_contact_added(&self, _: &Body, _: &Body, _: &ContactManifold<'_>, _: &mut ContactSettings) {
        panic!("boom");
    }
}

#[test]
fn listener_panic_is_resumed_from_update() {
    let mut world = World::new();
    world.system.set_contact_listener(PanickingListener);
    world.add_dynamic(Shape::sphere(0.5).unwrap(), Vec3::new(0.0, 0.6, 0.0));
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| world.step(30)));
    let payload = result.expect_err("update should re-raise the listener panic");
    assert_eq!(payload.downcast_ref::<&str>(), Some(&"boom"));
    // The world is still usable afterwards.
    world.system.clear_contact_listener();
    world.step(1);
}

#[derive(Default)]
struct ActivationCounter(AtomicU32, AtomicU32);

struct ActivationListener(Arc<ActivationCounter>);

impl BodyActivationListener for ActivationListener {
    fn on_body_activated(&self, _: BodyId, _: u64) {
        self.0.0.fetch_add(1, Ordering::Relaxed);
    }
    fn on_body_deactivated(&self, _: BodyId, _: u64) {
        self.0.1.fetch_add(1, Ordering::Relaxed);
    }
}

#[test]
fn activation_listener() {
    let mut world = World::new();
    let counter = Arc::new(ActivationCounter::default());
    world.system.set_body_activation_listener(ActivationListener(counter.clone()));
    world.add_dynamic(Shape::sphere(0.5).unwrap(), Vec3::new(0.0, 0.5, 0.0));
    world.step(120);
    assert_eq!(counter.0.load(Ordering::Relaxed), 1);
    assert_eq!(counter.1.load(Ordering::Relaxed), 1);
}

#[test]
fn ray_casts_and_filters() {
    let world = World::new();
    let ball = world.add_dynamic(Shape::sphere(1.0).unwrap(), Vec3::new(0.0, 5.0, 0.0));
    let ghost = BodyCreationSettings::new(Shape::sphere(1.0).unwrap(), Vec3::new(0.0, 8.0, 0.0), Quat::IDENTITY, MotionType::Dynamic, world.ghost_layer);
    let ghost = world.system.body_interface().create_and_add_body(&ghost, Activation::DontActivate).unwrap();

    let down = Vec3::new(0.0, -20.0, 0.0);
    let origin = Vec3::new(0.0, 10.0, 0.0);
    let s = &world.system;

    let hit = s.cast_ray(origin, down, &QueryFilter::new()).unwrap();
    assert_eq!(hit.body_id, ghost);

    let hit = s.cast_ray(origin, down, &QueryFilter::new().with_object_layer(world.moving_layer)).unwrap();
    assert_eq!(hit.body_id, ball);
    assert!((hit.point.y - 6.0).abs() < 1e-3 && (hit.normal.y - 1.0).abs() < 1e-3, "{hit:?}");

    let skip_ball = |id: BodyId| id != ball;
    let filter = QueryFilter::new().with_object_layer(world.moving_layer).with_body_filter(&skip_ball);
    let hit = s.cast_ray(origin, down, &filter).unwrap();
    assert_eq!(hit.body_id, world.floor);
    assert!(hit.point.y.abs() < 1e-3);

    let mut hits = Vec::new();
    s.cast_ray_all(origin, down, &RayCastSettings::default(), &QueryFilter::new(), |h| {
        hits.push(h.body_id);
        ControlFlow::Continue(())
    });
    hits.sort();
    let mut expected = vec![world.floor, ball, ghost];
    expected.sort();
    assert_eq!(hits, expected);

    let mut count = 0;
    s.cast_ray_all(origin, down, &RayCastSettings::default(), &QueryFilter::new(), |_| {
        count += 1;
        ControlFlow::Break(())
    });
    assert_eq!(count, 1);

    assert!(s.cast_ray(origin, Vec3::new(0.0, 5.0, 0.0), &QueryFilter::new()).is_none());
}

#[test]
fn point_and_shape_queries() {
    let world = World::new();
    let ball = world.add_dynamic(Shape::sphere(1.0).unwrap(), Vec3::new(0.0, 5.0, 0.0));
    let s = &world.system;

    let mut inside = Vec::new();
    s.collide_point(Vec3::new(0.0, 5.2, 0.0), &QueryFilter::new(), |h| {
        inside.push(h.body_id);
        ControlFlow::Continue(())
    });
    assert_eq!(inside, vec![ball]);

    let probe = Shape::sphere(0.5).unwrap();
    let mut overlaps = Vec::new();
    s.collide_shape(&probe, Vec3::ONE, &Mat44::from_translation(Vec3::new(0.0, 3.8, 0.0)), 0.0, &QueryFilter::new(), |h| {
        overlaps.push(h.clone());
        ControlFlow::Continue(())
    });
    assert_eq!(overlaps.len(), 1);
    assert_eq!(overlaps[0].body_id, ball);
    assert!((overlaps[0].penetration_depth - 0.3).abs() < 0.01, "{:?}", overlaps[0]);

    // Sweep the probe down from above the ball: it should stop 1.5 above the ball's center.
    let start = Mat44::from_translation(Vec3::new(0.0, 10.0, 0.0));
    let hit = s.cast_shape_closest(&probe, Vec3::ONE, &start, Vec3::new(0.0, -10.0, 0.0), &QueryFilter::new()).unwrap();
    assert_eq!(hit.body_id, ball);
    assert!((10.0 - 10.0 * hit.fraction - 6.5).abs() < 0.01, "{hit:?}");
    assert!((hit.normal().y - 1.0).abs() < 1e-3, "{:?}", hit.normal());
}

#[test]
fn query_filter_panic_is_resumed() {
    let world = World::new();
    let filter = |_: BodyId| -> bool { panic!("filter") };
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        world.system.cast_ray(Vec3::new(0.0, 5.0, 0.0), Vec3::new(0.0, -10.0, 0.0), &QueryFilter::new().with_body_filter(&filter))
    }));
    assert!(result.is_err());
}

#[test]
fn shapes() {
    let sphere = Shape::sphere(2.0).unwrap();
    assert_eq!(sphere.shape_type(), ShapeType::Convex);
    assert_eq!(sphere.sub_type(), ShapeSubType::Sphere);
    let expected = 4.0 / 3.0 * std::f32::consts::PI * 8.0;
    assert!((sphere.volume() - expected).abs() < 1e-3);
    assert!((sphere.mass_properties().mass - expected * 1000.0).abs() < 1.0);

    let light = Shape::cuboid(Vec3::splat(0.5)).unwrap().with_density(10.0).unwrap();
    assert!((light.mass_properties().mass - 10.0).abs() < 1e-3);
    assert_eq!(light.density(), Some(10.0));

    let mut shared = Shape::sphere(1.0).unwrap();
    let other = shared.clone();
    assert!(shared.set_density(1.0).is_err(), "shared shapes must not be mutated");
    drop(other);
    assert!(shared.set_density(1.0).is_ok());

    let points = [
        Vec3::new(-1.0, 0.0, -1.0),
        Vec3::new(1.0, 0.0, -1.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(0.0, 1.0, 0.0),
    ];
    assert_eq!(Shape::convex_hull(&points, 0.0).unwrap().sub_type(), ShapeSubType::ConvexHull);
    assert!(Shape::convex_hull(&[], 0.0).is_err());

    let vertices = [Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0)];
    assert_eq!(Shape::mesh(&vertices, &[[0, 2, 1]]).unwrap().sub_type(), ShapeSubType::Mesh);
    let err = Shape::mesh(&vertices, &[[0, 1, 7]]).unwrap_err();
    assert!(err.message().contains("out of range"), "{err}");

    assert!(Shape::height_field(&[0.0; 15], Vec3::ZERO, Vec3::ONE, 4).is_err());
    assert_eq!(Shape::height_field(&[0.0; 16], Vec3::ZERO, Vec3::ONE, 4).unwrap().sub_type(), ShapeSubType::HeightField);

    let compound = Shape::static_compound(&[
        CompoundSubShape::new(Vec3::new(-1.0, 0.0, 0.0), Quat::IDENTITY, sphere.clone()),
        CompoundSubShape::new(Vec3::new(1.0, 0.0, 0.0), Quat::IDENTITY, light.clone()),
    ])
    .unwrap();
    assert_eq!(compound.shape_type(), ShapeType::Compound);

    let scaled = Shape::scaled(&light, Vec3::splat(2.0)).unwrap();
    assert_eq!(scaled.sub_type(), ShapeSubType::Scaled);
    assert!((scaled.local_bounds().extent().x - 1.0).abs() < 1e-4);
    assert_eq!(Shape::rotated_translated(Vec3::Y, Quat::IDENTITY, &sphere).unwrap().sub_type(), ShapeSubType::RotatedTranslated);
    assert_eq!(Shape::offset_center_of_mass(Vec3::Y, &sphere).unwrap().center_of_mass(), Vec3::Y);
    assert_eq!(Shape::capsule(1.0, 0.5).unwrap().sub_type(), ShapeSubType::Capsule);
    assert_eq!(Shape::cylinder(1.0, 0.5).unwrap().sub_type(), ShapeSubType::Cylinder);
    assert_eq!(Shape::tapered_capsule(1.0, 0.5, 0.25).unwrap().sub_type(), ShapeSubType::TaperedCapsule);
    assert_eq!(Shape::tapered_cylinder(1.0, 0.5, 0.25, 0.05).unwrap().sub_type(), ShapeSubType::TaperedCylinder);
    assert_eq!(Shape::plane(Vec3::Y, 0.0, 100.0).unwrap().sub_type(), ShapeSubType::Plane);
    assert_eq!(Shape::empty(Vec3::ZERO).unwrap().sub_type(), ShapeSubType::Empty);
}

#[test]
fn body_shape_is_shared() {
    let world = World::new();
    let shape = Shape::sphere(0.5).unwrap();
    let ball = world.add_dynamic(shape.clone(), Vec3::new(0.0, 2.0, 0.0));
    let from_body = world.system.body_interface().shape(ball).unwrap();
    assert_eq!(from_body.as_raw(), shape.as_raw());
}

#[test]
fn misuse_does_not_crash() {
    let mut world = World::new();
    let ball = world.add_dynamic(Shape::sphere(0.5).unwrap(), Vec3::new(0.0, 2.0, 0.0));

    // Adding twice is a no-op.
    world.system.body_interface().add_body(ball, Activation::Activate);
    assert!(world.system.body_interface().is_added(ball));

    // Destroying an added body removes it first; afterwards the id is dead.
    world.system.destroy_body(ball);
    let bi = world.system.body_interface();
    assert!(!bi.is_added(ball));
    assert_eq!(bi.position(ball), Vec3::ZERO);
    bi.set_position(ball, Vec3::ONE, Activation::Activate);
    bi.add_body(ball, Activation::Activate);
    world.system.destroy_body(ball);
    world.system.destroy_body(BodyId::INVALID);
    let bi = world.system.body_interface();
    assert!(bi.shape(ball).is_none());

    // Static bodies cannot move.
    bi.move_kinematic(world.floor, Vec3::ONE, Quat::IDENTITY, DT);
    bi.activate_body(world.floor);
    bi.reset_sleep_timer(world.floor);
    assert!(!bi.is_active(world.floor));
    let err = bi.set_motion_type(world.floor, MotionType::Dynamic, Activation::Activate).unwrap_err();
    assert!(err.message().contains("allow_dynamic_or_kinematic"), "{err}");

    // A static body created with allow_dynamic_or_kinematic can become dynamic.
    let mut settings = BodyCreationSettings::new(Shape::sphere(0.5).unwrap(), Vec3::new(5.0, 2.0, 0.0), Quat::IDENTITY, MotionType::Static, world.static_layer);
    settings.allow_dynamic_or_kinematic = true;
    let crate_body = bi.create_and_add_body(&settings, Activation::DontActivate).unwrap();
    bi.set_motion_type(crate_body, MotionType::Dynamic, Activation::Activate).unwrap();
    assert_eq!(bi.motion_type(crate_body), MotionType::Dynamic);
}

#[test]
fn out_of_bodies_is_an_error() {
    let mut layers = LayerConfig::new();
    let layer = layers.add_object_layer(BroadPhaseLayer(0));
    let system = PhysicsSystem::new(&PhysicsSystemSettings { max_bodies: 2, ..Default::default() }, &layers).unwrap();
    let settings = BodyCreationSettings::new(Shape::sphere(1.0).unwrap(), Vec3::ZERO, Quat::IDENTITY, MotionType::Static, layer);
    let bi = system.body_interface();
    bi.create_body(&settings).unwrap();
    bi.create_body(&settings).unwrap();
    assert!(bi.create_body(&settings).is_err());
}

#[test]
fn invalid_layer_config_is_an_error() {
    let mut layers = LayerConfig::new();
    let a = layers.add_object_layer(BroadPhaseLayer(0));
    layers.enable_collision(a, ObjectLayer(5));
    let err = PhysicsSystem::new(&PhysicsSystemSettings::default(), &layers).unwrap_err();
    assert!(err.message().contains("object layer"), "{err}");
    assert!(PhysicsSystem::new(&PhysicsSystemSettings::default(), &LayerConfig::new()).is_err());
}

#[test]
fn hinge_pendulum() {
    let mut world = World::new();
    world.system.set_gravity(Vec3::new(0.0, -9.81, 0.0));
    let bob = world.add_dynamic(Shape::cuboid(Vec3::splat(0.25)).unwrap(), Vec3::new(2.0, 10.0, 0.0));

    // Pivot at (0, 10, 0) around Z: the bob swings down in the XY plane, staying 2 m from the pivot.
    let pivot = Vec3::new(0.0, 10.0, 0.0);
    let hinge = world.system.add_constraint_to_world(&HingeConstraintSettings::at(pivot, Vec3::Z, Vec3::X), bob).unwrap();
    world.step(30);

    let p = world.system.body_interface().position(bob);
    assert!(((p - pivot).length() - 2.0).abs() < 0.05, "bob at {p:?}");
    assert!(p.y < 9.5 && p.z.abs() < 1e-3, "bob should swing down in the XY plane: {p:?}");

    let mut c = world.system.constraint(hinge).unwrap();
    assert_eq!(c.kind(), ConstraintKind::Hinge);
    assert_eq!(c.bodies(), (None, Some(bob)));
    assert!(c.hinge_angle().unwrap() < -0.3);
    assert!(c.slider_position().is_none());
    assert!(!c.set_distance(0.0, 1.0));
    assert!(c.set_limits(-0.1, 0.1));

    // Destroying the bob destroys the constraint with it.
    world.system.destroy_body(bob);
    assert!(world.system.constraint(hinge).is_none());
    assert!(world.system.constraint_ids().is_empty());
    world.step(5);
}

#[test]
fn constraints_between_bodies() {
    let mut world = World::new();
    world.system.set_gravity(Vec3::ZERO);
    let a = world.add_dynamic(Shape::sphere(0.25).unwrap(), Vec3::new(0.0, 5.0, 0.0));
    let b = world.add_dynamic(Shape::sphere(0.25).unwrap(), Vec3::new(2.0, 5.0, 0.0));

    let rope = world.system.add_constraint(&DistanceConstraintSettings::between(Vec3::new(0.0, 5.0, 0.0), Vec3::new(2.0, 5.0, 0.0)), a, b).unwrap();
    world.system.body_interface().set_linear_velocity(b, Vec3::new(5.0, 0.0, 0.0));
    world.step(60);

    let bi = world.system.body_interface();
    let distance = (bi.position(b) - bi.position(a)).length();
    assert!((distance - 2.0).abs() < 0.05, "distance {distance}");

    assert!(world.system.add_constraint(&PointConstraintSettings::at(Vec3::ZERO), a, a).is_err());
    assert!(world.system.add_constraint(&FixedConstraintSettings::auto_detect(), a, BodyId::INVALID).is_err());

    let weld = world.system.add_constraint(&FixedConstraintSettings::auto_detect(), a, b).unwrap();
    assert_eq!(world.system.constraint_ids().len(), 2);
    assert!(world.system.remove_constraint(rope));
    assert!(!world.system.remove_constraint(rope));
    assert!(world.system.constraint(weld).unwrap().is_enabled());

    let slider = SliderConstraintSettings::at(Vec3::new(0.0, 5.0, 0.0), Vec3::X, Vec3::Y);
    let slider = world.system.add_constraint_to_world(&slider, a).unwrap();
    let mut c = world.system.constraint(slider).unwrap();
    assert!(c.slider_position().is_some());
    assert!(c.set_motor_state(MotorState::Velocity));
    assert!(c.set_motor_target_velocity(1.0));

    let cone = world.system.add_constraint(&ConeConstraintSettings::at(Vec3::new(1.0, 5.0, 0.0), Vec3::X, 0.5), a, b).unwrap();
    assert!(world.system.constraint(cone).unwrap().set_half_cone_angle(0.25));
    world.step(10);
}

#[test]
fn physics_settings_roundtrip() {
    let mut world = World::new();
    let mut settings = world.system.physics_settings();
    assert_eq!(settings.num_velocity_steps, 10);
    settings.num_velocity_steps = 4;
    world.system.set_physics_settings(&settings).unwrap();
    assert_eq!(world.system.physics_settings().num_velocity_steps, 4);
}

#[test]
fn bodies_from_many_threads() {
    let mut world = World::new();
    let shape = Shape::sphere(0.25).unwrap();
    let ids: Vec<BodyId> = std::thread::scope(|scope| {
        let handles: Vec<_> = (0..4)
            .map(|t| {
                let system = &world.system;
                let shape = shape.clone();
                let layer = world.moving_layer;
                scope.spawn(move || {
                    let bi = system.body_interface();
                    (0..25)
                        .map(|i| {
                            let p = Vec3::new(t as f32 * 2.0, 1.0 + i as f32, 0.0);
                            let s = BodyCreationSettings::new(shape.clone(), p, Quat::IDENTITY, MotionType::Dynamic, layer);
                            bi.create_and_add_body(&s, Activation::Activate).unwrap()
                        })
                        .collect::<Vec<_>>()
                })
            })
            .collect();
        handles.into_iter().flat_map(|h| h.join().unwrap()).collect()
    });
    assert_eq!(ids.len(), 100);
    assert_eq!(world.system.num_bodies(), 101);
    world.system.optimize_broad_phase();
    world.step(10);
    assert_eq!(world.system.active_bodies().len(), 100);
}
