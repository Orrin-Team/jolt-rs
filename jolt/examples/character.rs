//! A CharacterVirtual walking over a floor, up a step and into a crate it pushes along.

use jolt::*;

const DT: f32 = 1.0 / 60.0;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut layers = LayerConfig::new();
    let non_moving = layers.add_object_layer(BroadPhaseLayer(0));
    let moving = layers.add_object_layer(BroadPhaseLayer(1));
    layers.enable_collision(non_moving, moving).enable_collision(moving, moving);

    let mut system = PhysicsSystem::new(&PhysicsSystemSettings::default(), &layers)?;
    let mut temp = TempAllocator::default();
    let jobs = JobSystem::thread_pool(None);
    let gravity = system.gravity();

    let bodies = system.body_interface();
    let add_static = |shape: Shape, position: Vec3| {
        let settings = BodyCreationSettings::new(shape, position, Quat::IDENTITY, MotionType::Static, non_moving);
        bodies.create_and_add_body(&settings, Activation::DontActivate)
    };
    add_static(Shape::cuboid(Vec3::new(50.0, 1.0, 50.0))?, Vec3::new(0.0, -1.0, 0.0))?;
    add_static(Shape::cuboid(Vec3::new(2.0, 0.15, 2.0))?, Vec3::new(4.0, 0.15, 0.0))?;
    // Light enough (about 5 kg) for the character's default 100 N strength to push.
    let crate_shape = Shape::cuboid(Vec3::splat(0.4))?.with_density(10.0)?;
    let crate_settings = BodyCreationSettings::new(crate_shape, Vec3::new(8.0, 0.4, 0.0), Quat::IDENTITY, MotionType::Dynamic, moving);
    let crate_id = bodies.create_and_add_body(&crate_settings, Activation::Activate)?;

    // Capsule standing on its bottom, so the character's position is at its feet.
    let capsule = Shape::capsule(0.6, 0.3)?;
    let shape = Shape::rotated_translated(Vec3::new(0.0, 0.9, 0.0), Quat::IDENTITY, &capsule)?;
    let mut character = CharacterVirtual::new(&system, &CharacterVirtualSettings::default(), &shape, None, Vec3::ZERO, Quat::IDENTITY, 0)?;

    let filter = QueryFilter::new().with_object_layer(moving);
    let walk = Vec3::new(2.5, 0.0, 0.0);
    for frame in 0..300 {
        // Walk at a fixed speed; follow the ground vertically when standing, fall otherwise.
        let ground = character.ground();
        let mut velocity = walk;
        velocity.y = if ground.state == GroundState::OnGround { ground.velocity.y } else { character.linear_velocity().y + gravity.y * DT };
        character.set_linear_velocity(velocity);
        character.extended_update(&system, DT, gravity, &ExtendedUpdateSettings::default(), &filter, &mut temp);
        system.update(DT, 1, &mut temp, &jobs)?;

        if frame % 30 == 0 {
            let p = character.position();
            let c = system.body_interface().position(crate_id);
            println!(
                "frame {frame:3}: character ({:5.2}, {:5.2}) {:?}, crate x = {:5.2}",
                p.x,
                p.y,
                character.ground_state(),
                c.x
            );
        }
    }
    Ok(())
}
