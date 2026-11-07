//! Rust version of Jolt's HelloWorld: a sphere dropped onto a floor.

use jolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut layers = LayerConfig::new();
    let non_moving = layers.add_object_layer(BroadPhaseLayer(0));
    let moving = layers.add_object_layer(BroadPhaseLayer(1));
    layers.enable_collision(non_moving, moving).enable_collision(moving, moving);

    let mut system = PhysicsSystem::new(&PhysicsSystemSettings::default(), &layers)?;
    let mut temp_allocator = TempAllocator::default();
    let job_system = JobSystem::thread_pool(None);

    let bodies = system.body_interface();
    let floor = BodyCreationSettings::new(
        Shape::cuboid(Vec3::new(100.0, 1.0, 100.0))?,
        Vec3::new(0.0, -1.0, 0.0),
        Quat::IDENTITY,
        MotionType::Static,
        non_moving,
    );
    let floor = bodies.create_and_add_body(&floor, Activation::DontActivate)?;

    let sphere = BodyCreationSettings::new(Shape::sphere(0.5)?, Vec3::new(0.0, 2.0, 0.0), Quat::IDENTITY, MotionType::Dynamic, moving);
    let sphere = bodies.create_and_add_body(&sphere, Activation::Activate)?;
    bodies.set_linear_velocity(sphere, Vec3::new(0.0, -5.0, 0.0));

    system.optimize_broad_phase();

    let mut step = 0;
    while system.body_interface().is_active(sphere) {
        step += 1;
        let bodies = system.body_interface();
        let p = bodies.center_of_mass_position(sphere);
        let v = bodies.linear_velocity(sphere);
        println!("Step {step}: Position = ({:.3}, {:.3}, {:.3}), Velocity = ({:.3}, {:.3}, {:.3})", p.x, p.y, p.z, v.x, v.y, v.z);
        system.update(1.0 / 60.0, 1, &mut temp_allocator, &job_system)?;
    }

    system.destroy_bodies(&[sphere, floor]);
    Ok(())
}
