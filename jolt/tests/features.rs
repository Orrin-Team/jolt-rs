use std::ops::ControlFlow;
use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};

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
        let system = PhysicsSystem::new(&PhysicsSystemSettings { max_bodies: 1024, ..Default::default() }, &layers).unwrap();
        Self { system, temp: TempAllocator::default(), jobs: JobSystem::thread_pool(Some(2)), static_layer, moving_layer }
    }

    fn add_static(&self, shape: Shape, position: Vec3) -> BodyId {
        let s = BodyCreationSettings::new(shape, position, Quat::IDENTITY, MotionType::Static, self.static_layer);
        self.system.body_interface().create_and_add_body(&s, Activation::DontActivate).unwrap()
    }

    fn add_floor(&self) -> BodyId {
        self.add_static(Shape::cuboid(Vec3::new(50.0, 1.0, 50.0)).unwrap(), Vec3::new(0.0, -1.0, 0.0))
    }

    fn add_dynamic(&self, shape: Shape, position: Vec3) -> BodyId {
        let s = BodyCreationSettings::new(shape, position, Quat::IDENTITY, MotionType::Dynamic, self.moving_layer);
        self.system.body_interface().create_and_add_body(&s, Activation::Activate).unwrap()
    }

    fn step(&mut self, n: usize) {
        for _ in 0..n {
            self.system.update(DT, 1, &mut self.temp, &self.jobs).unwrap();
        }
    }

    fn character(&self, position: Vec3) -> CharacterVirtual {
        let capsule = Shape::capsule(0.5, 0.3).unwrap();
        let shape = Shape::rotated_translated(Vec3::new(0.0, 0.8, 0.0), Quat::IDENTITY, &capsule).unwrap();
        CharacterVirtual::new(&self.system, &CharacterVirtualSettings::default(), &shape, None, position, Quat::IDENTITY, 0).unwrap()
    }

    /// One frame of a typical character controller followed by a physics step.
    fn drive(&mut self, character: &mut CharacterVirtual, horizontal: Vec3) {
        let mut v = horizontal;
        v.y = if character.ground_state() == GroundState::OnGround { 0.0 } else { character.linear_velocity().y + GRAVITY.y * DT };
        character.set_linear_velocity(v);
        let filter = QueryFilter::new().with_object_layer(self.moving_layer);
        character.extended_update(&self.system, DT, GRAVITY, &ExtendedUpdateSettings::default(), &filter, &mut self.temp);
        self.step(1);
    }
}

#[test]
fn character_virtual_walks_on_the_floor() {
    let mut world = World::new();
    world.add_floor();
    let mut ch = world.character(Vec3::new(0.0, 1.0, 0.0));
    for _ in 0..60 {
        world.drive(&mut ch, Vec3::ZERO);
    }
    assert_eq!(ch.ground_state(), GroundState::OnGround);
    assert!(ch.position().y.abs() < 0.05, "{:?}", ch.position());
    assert!((ch.ground().normal.y - 1.0).abs() < 1e-3);

    for _ in 0..60 {
        world.drive(&mut ch, Vec3::new(2.0, 0.0, 0.0));
    }
    assert!((ch.position().x - 2.0).abs() < 0.1, "{:?}", ch.position());
    assert!(ch.is_supported());
    assert!(!ch.active_contacts().is_empty());
}

#[test]
fn character_virtual_is_stopped_by_walls_and_climbs_stairs() {
    let mut world = World::new();
    world.add_floor();
    // A wall at x = 3 and a 0.25 high step at z = 3.
    world.add_static(Shape::cuboid(Vec3::new(0.5, 2.0, 1.0)).unwrap(), Vec3::new(3.5, 2.0, 0.0));
    world.add_static(Shape::cuboid(Vec3::new(1.0, 0.125, 2.0)).unwrap(), Vec3::new(0.0, 0.125, 5.0));

    let mut ch = world.character(Vec3::ZERO);
    for _ in 0..120 {
        world.drive(&mut ch, Vec3::new(3.0, 0.0, 0.0));
    }
    let x = ch.position().x;
    assert!(x > 2.5 && x < 2.71, "character should stop at the wall: {x}");

    let mut ch = world.character(Vec3::new(0.0, 0.0, 0.0));
    for _ in 0..120 {
        world.drive(&mut ch, Vec3::new(0.0, 0.0, 3.0));
    }
    let p = ch.position();
    assert!(p.z > 4.0 && (p.y - 0.25).abs() < 0.05, "character should be on the step: {p:?}");
}

#[derive(Default)]
struct Counters {
    added: AtomicU32,
    validated: AtomicU32,
}

/// Lets the character fall through `ignored` and counts the other contacts.
struct CharacterListener {
    counters: Arc<Counters>,
    ignored: BodyId,
}

impl CharacterContactListener for CharacterListener {
    fn on_contact_validate(&mut self, _: &CharacterView, contact: &CharacterContact) -> bool {
        self.counters.validated.fetch_add(1, Ordering::Relaxed);
        contact.body_id != Some(self.ignored)
    }

    fn on_contact_added(&mut self, character: &CharacterView, contact: &CharacterContact, _: &mut CharacterContactSettings) {
        assert_ne!(contact.body_id, Some(self.ignored));
        assert!(character.position().y < 0.5);
        self.counters.added.fetch_add(1, Ordering::Relaxed);
    }
}

#[test]
fn character_listener() {
    let mut world = World::new();
    world.add_floor();
    let trapdoor = world.add_static(Shape::cuboid(Vec3::new(2.0, 0.05, 2.0)).unwrap(), Vec3::new(0.0, 2.0, 0.0));
    let counters = Arc::new(Counters::default());
    let mut ch = world.character(Vec3::new(0.0, 2.05, 0.0));
    ch.set_listener(CharacterListener { counters: counters.clone(), ignored: trapdoor });
    for _ in 0..90 {
        world.drive(&mut ch, Vec3::ZERO);
    }
    assert!(ch.position().y.abs() < 0.05, "character should fall through the rejected body: {:?}", ch.position());
    assert!(counters.validated.load(Ordering::Relaxed) >= 2);
    assert!(counters.added.load(Ordering::Relaxed) >= 1, "the floor contact is still reported");
    assert!(!ch.has_collided_with(trapdoor));
}

struct PanickingCharacterListener;

impl CharacterContactListener for PanickingCharacterListener {
    fn on_contact_added(&mut self, _: &CharacterView, _: &CharacterContact, _: &mut CharacterContactSettings) {
        panic!("character boom");
    }
}

#[test]
fn character_listener_panic_is_resumed() {
    let mut world = World::new();
    world.add_floor();
    let mut ch = world.character(Vec3::new(0.0, 0.05, 0.0));
    ch.set_listener(PanickingCharacterListener);
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        for _ in 0..10 {
            world.drive(&mut ch, Vec3::ZERO);
        }
    }));
    assert_eq!(result.unwrap_err().downcast_ref::<&str>(), Some(&"character boom"));
}

#[test]
fn character_inner_body_lifecycle() {
    let mut world = World::new();
    world.add_floor();
    let capsule = Shape::capsule(0.5, 0.3).unwrap();
    let shape = Shape::rotated_translated(Vec3::new(0.0, 0.8, 0.0), Quat::IDENTITY, &capsule).unwrap();
    let settings = CharacterVirtualSettings { inner_body_layer: world.moving_layer, ..Default::default() };
    let mut ch = CharacterVirtual::new(&world.system, &settings, &shape, Some(&shape), Vec3::ZERO, Quat::IDENTITY, 77).unwrap();
    let inner = ch.inner_body_id().expect("inner body");
    assert_eq!(world.system.num_bodies(), 2);

    // Ray casts see the character through its inner body.
    let hit = world.system.cast_ray(Vec3::new(-5.0, 1.0, 0.0), Vec3::new(10.0, 0.0, 0.0), &QueryFilter::new()).unwrap();
    assert_eq!(hit.body_id, inner);
    assert_eq!(world.system.body_interface().user_data(inner), 77);

    // The inner body cannot be destroyed behind the character's back.
    world.system.destroy_body(inner);
    assert!(world.system.body_interface().is_added(inner));

    // The inner body follows the character, and the character does not collide with its own inner body.
    for _ in 0..30 {
        world.drive(&mut ch, Vec3::new(2.0, 0.0, 0.0));
    }
    assert!(ch.position().x > 0.8, "{:?}", ch.position());
    let inner_pos = world.system.body_interface().position(inner);
    assert!((inner_pos.x - ch.position().x).abs() < 0.1);

    // Dropping defers destruction to the next update.
    drop(ch);
    assert_eq!(world.system.num_bodies(), 2);
    world.step(1);
    assert_eq!(world.system.num_bodies(), 1);
}

#[test]
fn character_destroy_is_immediate_and_outliving_the_system_is_fine() {
    let mut world = World::new();
    let ch = world.character(Vec3::ZERO);
    ch.destroy(&mut world.system);

    let ch = world.character(Vec3::ZERO);
    let World { system, .. } = world;
    drop(system);
    // The native system is kept alive until the character is gone.
    assert!(ch.position().y.abs() < 1e-6);
    drop(ch);
}

#[test]
#[should_panic(expected = "different physics system")]
fn character_with_wrong_system_panics() {
    let mut a = World::new();
    let b = World::new();
    let mut ch = a.character(Vec3::ZERO);
    ch.update(&b.system, DT, GRAVITY, &QueryFilter::new(), &mut a.temp);
}

struct Immovable;

impl CharacterContactListener for Immovable {
    fn on_character_contact_added(&mut self, _: &CharacterView, contact: &CharacterContact, settings: &mut CharacterContactSettings) {
        assert!(contact.character_id.is_some() && contact.body_id.is_none());
        settings.can_push_character = false;
    }

    fn on_character_contact_persisted(&mut self, _: &CharacterView, _: &CharacterContact, settings: &mut CharacterContactSettings) {
        settings.can_push_character = false;
    }
}

#[test]
fn characters_in_a_group_collide() {
    let mut world = World::new();
    world.add_floor();
    let group = CharacterGroup::new();
    let mut walker = world.character(Vec3::new(0.0, 0.0, 0.0));
    let mut other = world.character(Vec3::new(2.0, 0.0, 0.0));
    walker.join_group(&group);
    other.join_group(&group);

    // By default a moving character pushes the other one along.
    for _ in 0..90 {
        world.drive(&mut walker, Vec3::new(2.0, 0.0, 0.0));
        world.drive(&mut other, Vec3::ZERO);
    }
    assert!(other.position().x > 2.5, "other character should be pushed: {:?}", other.position());
    assert!(walker.position().x < other.position().x - 0.5);

    // An immovable character blocks the walker instead.
    other.set_listener(Immovable);
    let blocked_at = other.position().x;
    for _ in 0..60 {
        world.drive(&mut walker, Vec3::new(2.0, 0.0, 0.0));
        world.drive(&mut other, Vec3::ZERO);
    }
    assert!((other.position().x - blocked_at).abs() < 0.05, "{:?}", other.position());
    assert!(walker.position().x < blocked_at - 0.5, "{:?}", walker.position());

    // Outside the group they pass through each other.
    walker.leave_group();
    for _ in 0..90 {
        world.drive(&mut walker, Vec3::new(2.0, 0.0, 0.0));
    }
    assert!(walker.position().x > blocked_at + 1.0, "{:?}", walker.position());
}

#[test]
fn rigid_character() {
    let mut world = World::new();
    world.add_floor();
    let settings = CharacterSettings { layer: world.moving_layer, ..Default::default() };
    let capsule = Shape::capsule(0.5, 0.3).unwrap();
    let mut ch = Character::new(&world.system, &settings, &capsule, Vec3::new(0.0, 2.0, 0.0), Quat::IDENTITY, 0).unwrap();
    ch.add_to_physics_system(&world.system, Activation::Activate);
    assert!(world.system.body_interface().is_added(ch.body_id()));

    for _ in 0..90 {
        world.step(1);
        ch.post_simulation(&world.system, 0.05);
    }
    assert_eq!(ch.ground().state, GroundState::OnGround);
    assert!((ch.position(&world.system).y - 0.8).abs() < 0.05, "{:?}", ch.position(&world.system));

    ch.set_linear_velocity(&world.system, Vec3::new(2.0, 0.0, 0.0));
    world.step(30);
    assert!(ch.position(&world.system).x > 0.5);
    // Only translation is allowed, so it stays upright.
    assert!((ch.rotation(&world.system).w.abs() - 1.0).abs() < 1e-4);

    let body = ch.body_id();
    world.system.destroy_body(body);
    assert!(world.system.body_interface().is_added(body), "character bodies are protected");
    ch.destroy(&mut world.system);
    assert_eq!(world.system.num_bodies(), 1);
}

#[test]
fn materials_are_reported() {
    let mut world = World::new();
    let grass = PhysicsMaterial::new(1);
    let ice = PhysicsMaterial::new(2);
    assert_ne!(grass, ice);

    // A floor made of two triangles per material.
    let v = [
        Vec3::new(-10.0, 0.0, -10.0),
        Vec3::new(0.0, 0.0, -10.0),
        Vec3::new(0.0, 0.0, 10.0),
        Vec3::new(-10.0, 0.0, 10.0),
        Vec3::new(10.0, 0.0, -10.0),
        Vec3::new(10.0, 0.0, 10.0),
    ];
    let tris = [[0, 3, 2], [0, 2, 1], [1, 2, 5], [1, 5, 4]];
    let floor = Shape::mesh_with_materials(&v, &tris, &[0, 0, 1, 1], &[grass.clone(), ice.clone()]).unwrap();
    assert!(Shape::mesh_with_materials(&v, &tris, &[0, 0, 1, 2], &[grass.clone(), ice.clone()]).is_err());
    world.add_static(floor, Vec3::ZERO);

    let down = Vec3::new(0.0, -2.0, 0.0);
    let hit = world.system.cast_ray(Vec3::new(-5.0, 1.0, 0.0), down, &QueryFilter::new()).unwrap();
    assert_eq!(hit.material.as_ref().map(PhysicsMaterial::user_data), Some(1));
    let hit = world.system.cast_ray(Vec3::new(5.0, 1.0, 0.0), down, &QueryFilter::new()).unwrap();
    assert_eq!(hit.material, Some(ice.clone()));

    let metal = PhysicsMaterial::new(3);
    let crate_shape = Shape::cuboid(Vec3::splat(0.5)).unwrap().with_material(&metal).unwrap();
    let crate_body = world.add_dynamic(crate_shape, Vec3::new(0.0, 5.0, 0.0));
    let hit = world.system.cast_ray(Vec3::new(0.0, 10.0, 0.0), Vec3::new(0.0, -10.0, 0.0), &QueryFilter::new()).unwrap();
    assert_eq!((hit.body_id, hit.material.map(|m| m.user_data())), (crate_body, Some(3)));

    struct MaterialListener(Arc<AtomicU32>);
    impl ContactListener for MaterialListener {
        fn on_contact_added(&self, _: &Body, _: &Body, manifold: &ContactManifold<'_>, _: &mut ContactSettings) {
            let ids: Vec<u64> = [&manifold.material1, &manifold.material2].iter().filter_map(|m| m.as_ref().map(|m| m.user_data())).collect();
            if ids.contains(&3) && ids.iter().any(|&id| id == 1 || id == 2) {
                self.0.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
    let seen = Arc::new(AtomicU32::new(0));
    world.system.set_contact_listener(MaterialListener(seen.clone()));
    world.step(90);
    assert!(seen.load(Ordering::Relaxed) >= 1);

    // Characters report the material they stand on.
    let mut ch = world.character(Vec3::new(5.0, 0.5, 5.0));
    for _ in 0..30 {
        world.drive(&mut ch, Vec3::ZERO);
    }
    assert_eq!(ch.ground().material, Some(ice));
}

#[test]
fn collision_groups() {
    let mut world = World::new();
    world.add_floor();
    let filter = GroupFilterTable::new(2).disable_collision(0, 1).build();
    assert!(!filter.is_collision_enabled(0, 1));
    assert!(filter.is_collision_enabled(0, 7), "out of range sub groups collide");

    let shape = Shape::cuboid(Vec3::splat(0.5)).unwrap();
    let mk = |y: f32, sub: u32| {
        let mut s = BodyCreationSettings::new(shape.clone(), Vec3::new(0.0, y, 0.0), Quat::IDENTITY, MotionType::Dynamic, world.moving_layer);
        s.collision_group = CollisionGroup::new(&filter, 1, sub);
        world.system.body_interface().create_and_add_body(&s, Activation::Activate).unwrap()
    };
    let bottom = mk(0.5, 0);
    let top = mk(2.0, 1);
    world.step(90);
    let bi = world.system.body_interface();
    // The top box falls through the bottom one and lands on the floor next to it (same height).
    assert!((bi.position(top).y - bi.position(bottom).y).abs() < 0.05, "{:?} {:?}", bi.position(top), bi.position(bottom));
    assert_eq!(bi.collision_group(top).sub_group_id, 1);
    assert_eq!(bi.collision_group(top).filter, Some(filter.clone()));

    // Moving the top box into another group makes them collide again.
    bi.set_collision_group(top, &CollisionGroup::new(&filter, 2, 1));
    bi.set_position(top, Vec3::new(0.0, 2.0, 0.0), Activation::Activate);
    world.step(90);
    let bi = world.system.body_interface();
    assert!(bi.position(top).y > 1.3, "top box should rest on the bottom one: {:?}", bi.position(top));
}

struct AntiGravity {
    body: BodyId,
    steps: Arc<AtomicU32>,
}

impl StepListener for AntiGravity {
    fn on_step(&self, ctx: &StepContext<'_>) {
        assert!(ctx.delta_time > 0.0);
        // Forces last for the whole update, so add them once.
        if ctx.is_first_step {
            let mass = 1000.0; // 1 m^3 box at the default density
            ctx.bodies.add_force(self.body, Vec3::new(0.0, 9.81 * mass, 0.0));
        }
        self.steps.fetch_add(1, Ordering::Relaxed);
    }
}

#[test]
fn step_listener_runs_every_collision_step() {
    let mut world = World::new();
    let body = world.add_dynamic(Shape::cuboid(Vec3::splat(0.5)).unwrap(), Vec3::new(0.0, 10.0, 0.0));
    assert!((world.system.body_interface().mass(body).unwrap() - 1000.0).abs() < 1.0);
    let steps = Arc::new(AtomicU32::new(0));
    world.system.set_step_listener(AntiGravity { body, steps: steps.clone() });
    for _ in 0..30 {
        world.system.update(DT, 2, &mut world.temp, &world.jobs).unwrap();
    }
    assert_eq!(steps.load(Ordering::Relaxed), 60);
    let y = world.system.body_interface().position(body).y;
    assert!((y - 10.0).abs() < 0.01, "forces should cancel gravity: {y}");

    world.system.clear_step_listener();
    world.step(30);
    assert!(world.system.body_interface().position(body).y < 9.0);
}

#[test]
fn six_dof_and_swing_twist() {
    let mut world = World::new();
    let body = world.add_dynamic(Shape::cuboid(Vec3::splat(0.25)).unwrap(), Vec3::new(0.0, 5.0, 0.0));

    // Everything fixed except translation along X: gravity can't pull it down.
    let mut settings = SixDofConstraintSettings::at(Vec3::new(0.0, 5.0, 0.0), Vec3::X, Vec3::Y);
    for axis in SixDofAxis::ALL {
        settings.make_fixed_axis(axis);
    }
    settings.make_free_axis(SixDofAxis::TranslationX);
    let c = world.system.add_constraint_to_world(&settings, body).unwrap();
    world.system.body_interface().set_linear_velocity(body, Vec3::new(1.0, 0.0, 1.0));
    world.step(60);
    let p = world.system.body_interface().position(body);
    assert!(p.x > 0.5 && (p.y - 5.0).abs() < 0.02 && p.z.abs() < 0.02, "{p:?}");

    let mut cm = world.system.constraint(c).unwrap();
    assert_eq!(cm.kind(), ConstraintKind::SixDof);
    assert!(cm.set_six_dof_translation_limits(Vec3::new(-1.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)));
    assert!(cm.set_six_dof_motor_state(SixDofAxis::TranslationX, MotorState::Velocity));
    assert!(cm.set_six_dof_target_velocity_cs(Vec3::new(-1.0, 0.0, 0.0)));
    assert!(cm.rotation_in_constraint_space().is_some());
    assert!(!cm.set_swing_twist_limits(0.1, 0.1, -0.1, 0.1));
    world.step(120);
    let x = world.system.body_interface().position(body).x;
    assert!((x + 1.0).abs() < 0.05, "motor should drive it to the -1 limit: {x}");

    let a = world.add_dynamic(Shape::cuboid(Vec3::splat(0.25)).unwrap(), Vec3::new(5.0, 5.0, 0.0));
    let mut st = SwingTwistConstraintSettings::at(Vec3::new(5.0, 5.5, 0.0), Vec3::new(0.0, -1.0, 0.0), Vec3::X);
    st.normal_half_cone_angle = 0.3;
    st.plane_half_cone_angle = 0.3;
    st.twist_min_angle = -0.1;
    st.twist_max_angle = 0.1;
    let shoulder = world.system.add_constraint_to_world(&st, a).unwrap();
    world.system.body_interface().set_linear_velocity(a, Vec3::new(5.0, 0.0, 0.0));
    world.step(60);
    let p = world.system.body_interface().position(a);
    // The arm hangs from (5, 5.5, 0) and may swing at most ~0.3 rad: x offset <= 0.5 * sin(0.3) plus slack.
    assert!((p.x - 5.0).abs() < 0.2 && p.y < 5.5, "{p:?}");
    let mut cm = world.system.constraint(shoulder).unwrap();
    assert!(cm.set_swing_twist_limits(0.5, 0.5, -0.2, 0.2));
    assert!(cm.set_swing_twist_motor_states(MotorState::Position, MotorState::Off));
    assert!(cm.set_target_orientation_cs(Quat::IDENTITY));
    world.step(10);
}

#[test]
fn batch_add_and_remove() {
    let mut world = World::new();
    world.add_floor();
    let shape = Shape::sphere(0.25).unwrap();
    let settings: Vec<_> = (0..50)
        .map(|i| BodyCreationSettings::new(shape.clone(), Vec3::new(i as f32, 1.0, 0.0), Quat::IDENTITY, MotionType::Dynamic, world.moving_layer))
        .collect();
    let ids = world.system.body_interface().create_and_add_bodies(&settings, Activation::Activate).unwrap();
    assert_eq!(ids.len(), 50);
    world.system.optimize_broad_phase();
    assert_eq!(world.system.num_active_bodies(), 50);

    let bi = world.system.body_interface();
    let mut doubled = ids.clone();
    doubled.extend_from_slice(&ids);
    doubled.push(BodyId::INVALID);
    bi.remove_bodies(&doubled);
    assert!(ids.iter().all(|&id| !bi.is_added(id)));
    bi.add_bodies(&doubled, Activation::DontActivate);
    bi.add_bodies(&ids, Activation::Activate);
    assert!(ids.iter().all(|&id| bi.is_added(id)));
    world.step(5);
}

#[test]
fn forces_on_removed_bodies_are_ignored() {
    let mut world = World::new();
    let body = world.add_dynamic(Shape::sphere(0.5).unwrap(), Vec3::new(0.0, 5.0, 0.0));
    let bi = world.system.body_interface();
    bi.remove_body(body);
    bi.add_force(body, Vec3::new(0.0, 1000.0, 0.0), Activation::Activate);
    bi.add_impulse(body, Vec3::new(0.0, 10.0, 0.0));
    bi.add_angular_impulse(body, Vec3::Y);
    bi.activate_body(body);
    assert!(!bi.is_active(body));
    world.step(5);
    assert_eq!(world.system.num_active_bodies(), 0);
}

#[test]
fn buoyancy_and_body_queries() {
    let mut world = World::new();
    let body = world.add_dynamic(Shape::cuboid(Vec3::splat(0.5)).unwrap(), Vec3::new(0.0, -1.0, 0.0));
    for _ in 0..120 {
        let bi = world.system.body_interface();
        // Water surface at y = 0; the box is half as dense as the water.
        bi.apply_buoyancy_impulse(body, Vec3::ZERO, Vec3::Y, 2.0, 0.5, 0.05, Vec3::ZERO, GRAVITY, DT);
        world.step(1);
    }
    let bi = world.system.body_interface();
    let y = bi.position(body).y;
    assert!(y > -0.4 && y < 0.4, "box should float at the surface: {y}");

    let bounds = bi.world_space_bounds(body);
    assert!((bounds.extent().x - 0.5).abs() < 0.05);
    let floor = world.add_floor();
    assert_eq!(world.system.body_interface().mass(floor), None);
    let bi = world.system.body_interface();
    bi.set_max_linear_velocity(body, 3.0);
    assert_eq!(bi.max_linear_velocity(body), 3.0);
    assert!(!bi.apply_buoyancy_impulse(floor, Vec3::ZERO, Vec3::Y, 2.0, 0.5, 0.05, Vec3::ZERO, GRAVITY, DT));

    let mut overlapping = 0;
    world.system.collide_point(Vec3::new(0.0, y, 0.0), &QueryFilter::new(), |_| {
        overlapping += 1;
        ControlFlow::Continue(())
    });
    assert_eq!(overlapping, 1);
}
