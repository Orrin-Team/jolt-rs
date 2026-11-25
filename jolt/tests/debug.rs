//! Debug rendering (with the `debug-renderer` feature) and collision response estimation.

use std::sync::{Arc, Mutex};

use jolt::*;

const DT: f32 = 1.0 / 60.0;

fn world() -> (PhysicsSystem, ObjectLayer, ObjectLayer) {
    let mut layers = LayerConfig::new();
    let static_layer = layers.add_object_layer(BroadPhaseLayer(0));
    let moving_layer = layers.add_object_layer(BroadPhaseLayer(1));
    layers.enable_collision(static_layer, moving_layer).enable_collision(moving_layer, moving_layer);
    let system = PhysicsSystem::new(&PhysicsSystemSettings { max_bodies: 64, ..Default::default() }, &layers).unwrap();
    (system, static_layer, moving_layer)
}

fn add(system: &PhysicsSystem, shape: Shape, position: Vec3, motion: MotionType, layer: ObjectLayer) -> BodyId {
    let s = BodyCreationSettings::new(shape, position, Quat::IDENTITY, motion, layer);
    system.body_interface().create_and_add_body(&s, Activation::Activate).unwrap()
}

#[derive(Default)]
struct Recorder {
    lines: Vec<(RVec3, RVec3)>,
    triangles: Vec<[RVec3; 3]>,
    texts: Vec<String>,
}

impl Recorder {
    fn points(&self) -> impl Iterator<Item = RVec3> + '_ {
        self.lines.iter().flat_map(|l| [l.0, l.1]).chain(self.triangles.iter().flatten().copied())
    }

    fn is_empty(&self) -> bool {
        self.lines.is_empty() && self.triangles.is_empty()
    }
}

impl DebugDraw for Recorder {
    fn draw_line(&mut self, from: RVec3, to: RVec3, _color: Color) {
        self.lines.push((from, to));
    }

    fn draw_triangle(&mut self, v1: RVec3, v2: RVec3, v3: RVec3, _color: Color, _cast_shadow: bool) {
        self.triangles.push([v1, v2, v3]);
    }

    fn draw_text(&mut self, _position: RVec3, text: &str, _color: Color, _height: f32) {
        self.texts.push(text.to_owned());
    }
}

#[test]
fn debug_renderer_draws_bodies_constraints_and_skeletons() {
    if !cfg!(feature = "debug-renderer") {
        assert!(!DebugRenderer::is_available());
        assert!(DebugRenderer::new().unwrap_err().message().contains("debug renderer"));
        return;
    }
    assert!(DebugRenderer::is_available());

    let (mut system, static_layer, moving_layer) = world();
    let floor = add(&system, Shape::cuboid(Vec3::new(10.0, 1.0, 10.0)).unwrap(), Vec3::new(0.0, -1.0, 0.0), MotionType::Static, static_layer);
    let ball = add(&system, Shape::sphere(0.5).unwrap(), Vec3::new(20.0, 5.0, 0.0), MotionType::Dynamic, moving_layer);
    let vertices = [Vec3::new(-1.0, 0.0, -1.0), Vec3::new(1.0, 0.0, -1.0), Vec3::new(0.0, 0.0, 1.0)];
    add(&system, Shape::mesh(&vertices, &[[0, 2, 1]]).unwrap(), Vec3::new(-20.0, 0.0, 0.0), MotionType::Static, static_layer);
    let cube = SoftBodyCreationSettings::new(SoftBodySharedSettings::cube(3, 0.2).unwrap(), Vec3::new(0.0, 3.0, 20.0), Quat::IDENTITY, moving_layer);
    system.body_interface().create_and_add_soft_body(&cube, Activation::Activate).unwrap();
    system.add_constraint_to_world(&HingeConstraintSettings::at(Vec3::new(20.0, 6.0, 0.0), Vec3::Z, Vec3::X), ball).unwrap();
    let (mut temp, jobs) = (TempAllocator::default(), JobSystem::thread_pool(Some(2)));
    system.update(DT, 1, &mut temp, &jobs).unwrap();

    let mut renderer = DebugRenderer::new().unwrap();
    // Jolt allows only one renderer at a time.
    assert!(DebugRenderer::new().unwrap_err().message().contains("one debug renderer"));
    renderer.set_camera_position(Vec3::new(0.0, 5.0, -10.0));

    let mut all = Recorder::default();
    let settings = BodyDrawSettings { draw_bounding_box: true, draw_soft_body_vertices: true, ..Default::default() };
    system.draw_bodies(&mut renderer, &settings, &mut all, None);
    assert!(!all.is_empty());
    let xs: Vec<Real> = all.points().map(|p| p.x).collect();
    assert!(xs.iter().any(|&x| x > 19.0) && xs.iter().any(|&x| x < -19.0), "every body is drawn");

    // The filter limits drawing to the bodies it accepts.
    let mut only_floor = Recorder::default();
    system.draw_bodies(&mut renderer, &settings, &mut only_floor, Some(&|id| id == floor));
    assert!(!only_floor.is_empty());
    assert!(only_floor.points().all(|p| p.x.abs() <= 10.5 && p.z.abs() <= 10.5), "only the floor is drawn");

    let mut constraints = Recorder::default();
    system.draw_constraints(&mut renderer, &mut constraints);
    system.draw_constraint_limits(&mut renderer, &mut constraints);
    system.draw_constraint_reference_frames(&mut renderer, &mut constraints);
    assert!(!constraints.is_empty());

    let mut skeleton = SkeletonBuilder::new();
    let root = skeleton.add_joint("root", None).unwrap();
    skeleton.add_joint("hand", Some(root)).unwrap();
    let skeleton = skeleton.build();
    let pose = vec![Mat44::IDENTITY, Mat44::from_translation(Vec3::new(0.0, 1.0, 0.0))];
    let mut bones = Recorder::default();
    skeleton.draw_pose(&mut renderer, &pose, Vec3::new(0.0, 0.0, 5.0), true, &mut bones);
    assert!(!bones.lines.is_empty());
    assert!(bones.texts.contains(&"hand".to_owned()), "{:?}", bones.texts);

    // A panic while drawing comes out after Jolt is done.
    struct Panics;
    impl DebugDraw for Panics {
        fn draw_line(&mut self, _: RVec3, _: RVec3, _: Color) {
            panic!("boom");
        }
    }
    let panic = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| system.draw_bodies(&mut renderer, &settings, &mut Panics, None)));
    assert!(panic.is_err());
    system.draw_bodies(&mut renderer, &settings, &mut Recorder::default(), None);

    drop(renderer);
    let _again = DebugRenderer::new().unwrap();
}

#[derive(Default)]
struct Impacts(Mutex<Vec<(f32, CollisionEstimation)>>);

struct ImpactListener(Arc<Impacts>);

impl ContactListener for ImpactListener {
    fn on_contact_added(&self, body1: &Body, body2: &Body, manifold: &ContactManifold<'_>, settings: &mut ContactSettings) {
        // Approach speed along the normal, which a restitution of 0 turns into an impulse of mass * speed.
        let speed = (body2.linear_velocity() - body1.linear_velocity()).dot(manifold.world_space_normal).abs();
        let estimate = estimate_collision_response(body1, body2, manifold, settings, 1.0, 10);
        self.0.0.lock().unwrap().push((speed, estimate));
    }
}

#[test]
fn collision_response_is_estimated_in_contact_callbacks() {
    let (mut system, static_layer, moving_layer) = world();
    add(&system, Shape::cuboid(Vec3::new(10.0, 1.0, 10.0)).unwrap(), Vec3::new(0.0, -1.0, 0.0), MotionType::Static, static_layer);
    let ball = add(&system, Shape::sphere(0.5).unwrap(), Vec3::new(0.0, 3.0, 0.0), MotionType::Dynamic, moving_layer);
    let mass = system.body_interface().mass(ball).unwrap();
    let impacts = Arc::new(Impacts::default());
    system.set_contact_listener(ImpactListener(impacts.clone()));
    let (mut temp, jobs) = (TempAllocator::default(), JobSystem::thread_pool(Some(2)));
    for _ in 0..90 {
        system.update(DT, 1, &mut temp, &jobs).unwrap();
    }

    let impacts = impacts.0.lock().unwrap();
    let (speed, estimate) = impacts.first().expect("the ball hits the floor");
    assert!(*speed > 5.0, "{speed}");
    assert!(!estimate.contact_impulses.is_empty());
    let impulse: f32 = estimate.contact_impulses.iter().sum();
    let expected = mass * speed;
    assert!((impulse - expected).abs() < 0.2 * expected, "impulse {impulse}, expected about {expected}");
    // The ball (whichever body it is) ends up at rest vertically.
    let ball_velocity = if estimate.linear_velocity1.length() > 0.0 { estimate.linear_velocity1 } else { estimate.linear_velocity2 };
    assert!(ball_velocity.y.abs() < 0.2 * speed, "{estimate:?}");
}
