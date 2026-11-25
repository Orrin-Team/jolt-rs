//! Simulation and queries 10 000 km from the origin, where `f32` positions are a meter apart. Needs the
//! `double-precision` feature.
#![cfg(feature = "double-precision")]

use std::ops::ControlFlow;
use std::sync::Mutex;

use jolt::*;

const DT: f32 = 1.0 / 60.0;
/// Far enough that the nearest `f32` values are 1 m apart.
const FAR: f64 = 10_000_000.0;

fn far(x: f64, y: f64, z: f64) -> DVec3 {
    DVec3::new(FAR + x, y, FAR + z)
}

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
        let system = PhysicsSystem::new(&PhysicsSystemSettings::default(), &layers).unwrap();
        let world = Self { system, temp: TempAllocator::default(), jobs: JobSystem::thread_pool(Some(2)), static_layer, moving_layer };
        world.add(Shape::cuboid(Vec3::new(50.0, 1.0, 50.0)).unwrap(), far(0.0, -1.0, 0.0), MotionType::Static);
        world
    }

    fn add(&self, shape: Shape, position: DVec3, motion: MotionType) -> BodyId {
        let layer = if motion == MotionType::Static { self.static_layer } else { self.moving_layer };
        let s = BodyCreationSettings::new(shape, position, Quat::IDENTITY, motion, layer);
        self.system.body_interface().create_and_add_body(&s, Activation::Activate).unwrap()
    }

    fn step(&mut self, n: usize) {
        for _ in 0..n {
            self.system.update(DT, 1, &mut self.temp, &self.jobs).unwrap();
        }
    }
}

fn near(a: DVec3, b: DVec3, tolerance: f64) -> bool {
    (a - b).length() < tolerance
}

#[derive(Default)]
struct Contacts(Mutex<Vec<DVec3>>);

impl ContactListener for &'static Contacts {
    fn on_contact_added(&self, _: &Body, _: &Body, manifold: &ContactManifold<'_>, _: &mut ContactSettings) {
        let points = (0..manifold.relative_points_on1.len()).map(|i| manifold.world_point_on1(i));
        self.0.lock().unwrap().extend(points);
    }
}

#[test]
fn bodies_far_from_the_origin_simulate_precisely() {
    assert!(is_double_precision());
    let mut world = World::new();
    let contacts: &'static Contacts = Box::leak(Box::default());
    world.system.set_contact_listener(contacts);
    let ball = world.add(Shape::sphere(0.5).unwrap(), far(0.25, 2.0, 0.25), MotionType::Dynamic);
    world.step(120);

    let position = world.system.body_interface().position(ball);
    assert!(near(position, far(0.25, 0.5, 0.25), 0.02), "the ball rests on the floor where it fell: {position:?}");
    let touching = contacts.0.lock().unwrap().clone();
    assert!(!touching.is_empty() && touching.iter().all(|&p| near(p, far(0.25, 0.0, 0.25), 0.05)), "{touching:?}");

    // Queries report world space results at full precision.
    let top = position + Vec3::new(0.0, 0.5, 0.0);
    let hit = world.system.cast_ray(far(0.25, 10.0, 0.25), Vec3::new(0.0, -20.0, 0.0), &QueryFilter::new()).unwrap();
    assert_eq!(hit.body_id, ball);
    assert!(near(hit.point, top, 1e-3), "{hit:?}");

    let mut inside = Vec::new();
    world.system.collide_point(far(0.25, 0.5, 0.25), &QueryFilter::new(), |h| {
        inside.push(h.body_id);
        ControlFlow::Continue(())
    });
    assert_eq!(inside, [ball]);

    let probe = Shape::sphere(0.5).unwrap();
    let mut hits = Vec::new();
    world.system.collide_shape(&probe, Vec3::ONE, &RMat44::from_translation(far(0.25, 1.4, 0.25)), 0.0, &QueryFilter::new(), |h| {
        hits.push(h.clone());
        ControlFlow::Continue(())
    });
    assert_eq!(hits.len(), 1);
    assert!(near(hits[0].contact_point_on2, top, 1e-3), "{:?}", hits[0]);

    let start = RMat44::from_translation(far(0.25, 10.0, 0.25));
    let swept = world.system.cast_shape_closest(&probe, Vec3::ONE, &start, Vec3::new(0.0, -10.0, 0.0), &QueryFilter::new()).unwrap();
    assert_eq!(swept.body_id, ball);
    assert!(near(swept.contact_point_on2, top, 1e-3), "{swept:?}");
}

#[test]
fn constraints_far_from_the_origin_hold() {
    let mut world = World::new();
    // A pendulum 1.5 m long, hinged around Z. Constraints to the world are measured from the origin in f32, so it
    // hangs from a static body instead.
    let anchor = world.add(Shape::sphere(0.05).unwrap(), far(0.0, 5.0, 0.0), MotionType::Static);
    let bob = world.add(Shape::sphere(0.2).unwrap(), far(1.5, 5.0, 0.0), MotionType::Dynamic);
    let hinge = HingeConstraintSettings::at(far(0.0, 5.0, 0.0), Vec3::Z, Vec3::X);
    world.system.add_constraint(&hinge, anchor, bob).unwrap();
    world.step(30);
    let position = world.system.body_interface().position(bob);
    assert!(position.y < 4.5, "the pendulum swings down: {position:?}");
    assert!(((position - far(0.0, 5.0, 0.0)).length() - 1.5).abs() < 0.01, "{position:?}");
    assert!((position.x - FAR - 1.038).abs() < 0.01, "swings like it would at the origin: {position:?}");
    assert!((position.z - FAR).abs() < 1e-3, "{position:?}");
}

#[test]
fn soft_bodies_and_shapes_far_from_the_origin() {
    let mut world = World::new();
    let shared = SoftBodySharedSettings::cube(3, 0.2).unwrap();
    let cube = SoftBodyCreationSettings::new(shared, far(0.3, 1.0, 0.3), Quat::IDENTITY, world.moving_layer);
    let cube = world.system.body_interface().create_and_add_soft_body(&cube, Activation::Activate).unwrap();
    world.step(90);
    let vertices = world.system.body_interface().soft_body_vertices(cube, true).unwrap();
    let lowest = vertices.iter().map(|v| v.position.y).fold(f64::MAX, f64::min);
    let mean_x = vertices.iter().map(|v| v.position.x).sum::<f64>() / vertices.len() as f64;
    assert!(lowest.abs() < 0.1, "the cube rests on the floor: {lowest}");
    assert!((mean_x - (FAR + 0.3)).abs() < 0.05, "{mean_x}");

    let box_shape = Shape::cuboid_with_convex_radius(Vec3::splat(1.0), 0.0).unwrap();
    let placed = ShapePlacement::new(far(0.25, 0.0, 0.0), Quat::IDENTITY);
    let triangles = box_shape.triangles(&placed, None);
    assert_eq!(triangles.len(), 12);
    let xs: Vec<f64> = triangles.iter().flat_map(|t| t.vertices).map(|v| v.x - FAR).collect();
    assert!(xs.iter().all(|x| (x - 1.25).abs() < 1e-4 || (x + 0.75).abs() < 1e-4), "{xs:?}");

    let hit = box_shape.cast_ray(&placed, far(0.25, 5.0, 0.0), Vec3::new(0.0, -10.0, 0.0), &RayCastSettings::default()).unwrap();
    assert!(near(hit.point, far(0.25, 1.0, 0.0), 1e-3), "{hit:?}");
    assert!(box_shape.contains_point(&placed, far(1.0, 0.0, 0.0)));
    assert!(!box_shape.contains_point(&placed, far(1.5, 0.0, 0.0)));
}
