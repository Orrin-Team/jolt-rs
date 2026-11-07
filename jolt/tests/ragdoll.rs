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

    fn floor(&self) -> BodyId {
        let s = BodyCreationSettings::new(Shape::cuboid(Vec3::new(50.0, 1.0, 50.0)).unwrap(), Vec3::new(0.0, -1.0, 0.0), Quat::IDENTITY, MotionType::Static, self.static_layer);
        self.system.body_interface().create_and_add_body(&s, Activation::DontActivate).unwrap()
    }

    fn step(&mut self, n: usize) {
        for _ in 0..n {
            self.system.update(DT, 1, &mut self.temp, &self.jobs).unwrap();
        }
    }

    /// A vertical chain pelvis -> spine -> head, 0.5 apart, starting at `base`.
    fn chain(&self) -> (Skeleton, Vec<RagdollPart>) {
        let mut b = SkeletonBuilder::new();
        let pelvis = b.add_joint("pelvis", None).unwrap();
        let spine = b.add_joint("spine", Some(pelvis)).unwrap();
        b.add_joint("head", Some(spine)).unwrap();
        let skeleton = b.build();

        let part = |y: f32| {
            BodyCreationSettings::new(Shape::capsule(0.15, 0.1).unwrap(), Vec3::new(0.0, y, 0.0), Quat::IDENTITY, MotionType::Dynamic, self.moving_layer)
        };
        let joint = |y: f32| {
            let mut s = SwingTwistConstraintSettings::at(Vec3::new(0.0, y, 0.0), Vec3::Y, Vec3::X);
            s.normal_half_cone_angle = 0.5;
            s.plane_half_cone_angle = 0.5;
            s.twist_min_angle = -0.3;
            s.twist_max_angle = 0.3;
            RagdollJoint::SwingTwist(s)
        };
        let parts = vec![RagdollPart::root(part(1.0)), RagdollPart::new(part(1.5), joint(1.25)), RagdollPart::new(part(2.0), joint(1.75))];
        (skeleton, parts)
    }
}

#[test]
fn skeleton_builder_and_poses() {
    let mut b = SkeletonBuilder::new();
    let root = b.add_joint("root", None).unwrap();
    let arm = b.add_joint("arm", Some(root)).unwrap();
    assert!(b.add_joint("bad", Some(7)).is_err());
    assert!(b.add_joint("nul\0", None).is_err());
    let skeleton = b.build();
    assert_eq!(skeleton.joint_count(), 2);
    assert_eq!(skeleton.joint_index("arm"), Some(arm));
    assert_eq!(skeleton.joint_index("leg"), None);
    assert_eq!(skeleton.joint_name(1), Some("arm"));
    assert_eq!(skeleton.joint_name(2), None);
    assert_eq!((skeleton.parent(root), skeleton.parent(arm)), (None, Some(root)));

    // Local states compose down the hierarchy.
    let states = [
        JointState { translation: Vec3::new(0.0, 1.0, 0.0), rotation: Quat::from_rotation_y(std::f32::consts::FRAC_PI_2) },
        JointState { translation: Vec3::new(1.0, 0.0, 0.0), rotation: Quat::IDENTITY },
    ];
    let matrices = skeleton.joint_matrices(&states);
    let arm_pos = matrices[1].translation();
    assert!((arm_pos - Vec3::new(0.0, 1.0, -1.0)).length() < 1e-4, "{arm_pos:?}");
    let back = skeleton.joint_states(&matrices);
    assert!((back[1].translation - states[1].translation).length() < 1e-4);
    assert_eq!(format!("{skeleton:?}"), r#"Skeleton { joints: [("root", None), ("arm", Some(0))] }"#);
}

#[test]
fn ragdoll_falls_and_stays_connected() {
    let mut world = World::new();
    world.floor();
    let (skeleton, parts) = world.chain();
    let settings = RagdollSettings::new(&skeleton, &parts, &RagdollOptions::default()).unwrap();
    assert_eq!(settings.skeleton().joint_count(), 3);

    let ragdoll = settings.create_ragdoll(&world.system, 1, 42).unwrap();
    assert_eq!((ragdoll.body_count(), ragdoll.constraint_count()), (3, 2));
    assert!(!ragdoll.is_added());
    ragdoll.add_to_physics_system(&world.system, Activation::Activate);
    assert!(ragdoll.is_added());
    let ids = ragdoll.body_ids();
    assert_eq!(world.system.body_interface().user_data(ids[2]), 42);

    // Knock it over and let it settle on the floor.
    ragdoll.add_linear_velocity(&world.system, Vec3::new(2.0, 0.0, 0.0));
    world.step(240);
    let bi = world.system.body_interface();
    for &id in &ids {
        assert!(bi.position(id).y < 0.5, "part should lie on the floor: {:?}", bi.position(id));
    }
    // Joints keep neighbours ~0.5 apart.
    for pair in ids.windows(2) {
        let d = (bi.position(pair[0]) - bi.position(pair[1])).length();
        assert!((d - 0.5).abs() < 0.05, "joint stretched to {d}");
    }
    let (root, _) = ragdoll.root_transform(&world.system);
    assert_eq!(root, bi.position(ids[0]));
    assert!(ragdoll.world_space_bounds(&world.system).max.y < 0.6);
}

#[test]
fn parent_and_child_do_not_collide() {
    let world = World::new();
    let (skeleton, parts) = world.chain();
    let settings = RagdollSettings::new(&skeleton, &parts, &RagdollOptions::default()).unwrap();
    let ragdoll = settings.create_ragdoll(&world.system, 3, 0).unwrap();
    let group = world.system.body_interface().collision_group(ragdoll.body_ids()[1]);
    assert_eq!((group.group_id, group.sub_group_id), (3, 1));
    let filter = group.filter.expect("ragdoll parts share a group filter");
    assert!(!filter.is_collision_enabled(0, 1) && !filter.is_collision_enabled(1, 2));
    assert!(filter.is_collision_enabled(0, 2), "pelvis and head are not neighbours");
    assert!(filter.is_collision_enabled(0, 99), "the filter is the bounds checked one");
}

#[test]
fn ragdoll_settings_are_validated() {
    let world = World::new();
    let (skeleton, parts) = world.chain();
    assert!(RagdollSettings::new(&skeleton, &parts[..2], &RagdollOptions::default()).is_err());

    let mut bad_root = parts.clone();
    bad_root[0].to_parent = parts[1].to_parent;
    assert!(RagdollSettings::new(&skeleton, &bad_root, &RagdollOptions::default()).unwrap_err().message().contains("Root"));

    let joint = parts[1].to_parent.unwrap();
    let options = RagdollOptions { additional_constraints: vec![RagdollConstraint { body1: 0, body2: 9, joint }], ..Default::default() };
    assert!(RagdollSettings::new(&skeleton, &parts, &options).is_err());
    let options = RagdollOptions { additional_constraints: vec![RagdollConstraint { body1: 0, body2: 2, joint }], ..Default::default() };
    let settings = RagdollSettings::new(&skeleton, &parts, &options).unwrap();
    assert_eq!(settings.create_ragdoll(&world.system, 0, 0).unwrap().constraint_count(), 3);

    let options = RagdollOptions { pose: Some(vec![Mat44::IDENTITY]), ..Default::default() };
    assert!(RagdollSettings::new(&skeleton, &parts, &options).is_err());

    // Layers are checked against the system the ragdoll is created in.
    let mut wrong_layer = parts.clone();
    wrong_layer[2].body.object_layer = ObjectLayer(50);
    let settings = RagdollSettings::new(&skeleton, &wrong_layer, &RagdollOptions::default()).unwrap();
    assert!(settings.create_ragdoll(&world.system, 0, 0).is_err());
}

#[test]
fn ragdoll_bodies_are_protected() {
    let mut world = World::new();
    let (skeleton, parts) = world.chain();
    let settings = RagdollSettings::new(&skeleton, &parts, &RagdollOptions::default()).unwrap();
    let ragdoll = settings.create_ragdoll(&world.system, 0, 0).unwrap();
    ragdoll.add_to_physics_system(&world.system, Activation::Activate);
    let head = ragdoll.body_id(2).unwrap();
    world.system.destroy_body(head);
    assert!(world.system.body_interface().is_added(head));
    assert!(world.system.add_constraint_to_world(&PointConstraintSettings::at(Vec3::ZERO), head).is_err());

    // Destroying the ragdoll removes all three bodies.
    assert_eq!(world.system.num_bodies(), 3);
    ragdoll.destroy(&mut world.system);
    assert_eq!(world.system.num_bodies(), 0);

    // Dropping defers it to the next update.
    let ragdoll = settings.create_ragdoll(&world.system, 0, 0).unwrap();
    ragdoll.add_impulse(&world.system, Vec3::Y); // not in the world: ignored
    drop(ragdoll);
    assert_eq!(world.system.num_bodies(), 3);
    world.step(1);
    assert_eq!(world.system.num_bodies(), 0);
}

#[test]
fn poses_and_motors() {
    let mut world = World::new();
    world.system.set_gravity(Vec3::ZERO);
    let (skeleton, parts) = world.chain();
    let settings = RagdollSettings::new(&skeleton, &parts, &RagdollOptions::default()).unwrap();
    let mut ragdoll = settings.create_ragdoll(&world.system, 0, 0).unwrap();
    ragdoll.add_to_physics_system(&world.system, Activation::Activate);

    let (root, rest) = ragdoll.pose(&world.system);
    assert_eq!(rest.len(), 3);
    assert!(ragdoll.set_pose(&world.system, root, &rest[..1]).is_err());

    // Teleport the whole ragdoll 5 m sideways.
    let moved = root + Vec3::new(5.0, 0.0, 0.0);
    ragdoll.set_pose(&world.system, moved, &rest).unwrap();
    let (root_after, _) = ragdoll.pose(&world.system);
    assert!((root_after - moved).length() < 1e-4, "{root_after:?}");
    ragdoll.reset_warm_start(&world.system);

    // Kinematic driving reaches the target pose in one step.
    let target = moved + Vec3::new(0.0, 2.0, 0.0);
    ragdoll.drive_to_pose_using_kinematics(&world.system, target, &rest, DT).unwrap();
    world.step(1);
    let (root_now, _) = ragdoll.pose(&world.system);
    assert!((root_now - target).length() < 0.05, "{root_now:?} vs {target:?}");

    // Motor driving switches the joint motors on.
    ragdoll.set_linear_and_angular_velocity(&world.system, Vec3::ZERO, Vec3::ZERO);
    ragdoll.drive_to_pose_using_motors(&world.system, target, &rest).unwrap();
    world.step(30);
    let mut joint = ragdoll.constraint(&world.system, 0).unwrap();
    assert_eq!(joint.kind(), ConstraintKind::SwingTwist);
    assert!(joint.rotation_in_constraint_space().is_some());
    joint.set_enabled(false);
    assert!(!joint.is_enabled());
    assert!(ragdoll.constraint(&world.system, 5).is_none());
}

#[test]
fn ragdolls_in_different_groups_collide() {
    let mut world = World::new();
    world.floor();
    let (skeleton, parts) = world.chain();
    let settings = RagdollSettings::new(&skeleton, &parts, &RagdollOptions::default()).unwrap();

    // Lay the chain along X by rotating the whole pose 90 degrees around Z.
    let flat: Vec<Mat44> =
        (0..3).map(|i| Mat44::from_rotation_translation(Quat::from_rotation_z(-std::f32::consts::FRAC_PI_2), Vec3::new(i as f32 * 0.5, 0.0, 0.0))).collect();
    let bottom = settings.create_ragdoll(&world.system, 1, 0).unwrap();
    let top = settings.create_ragdoll(&world.system, 2, 0).unwrap();
    bottom.set_pose(&world.system, Vec3::new(0.0, 0.11, 0.0), &flat).unwrap();
    top.set_pose(&world.system, Vec3::new(0.0, 0.6, 0.0), &flat).unwrap();
    bottom.add_to_physics_system(&world.system, Activation::Activate);
    top.add_to_physics_system(&world.system, Activation::Activate);
    world.step(120);

    let bi = world.system.body_interface();
    for (b, t) in bottom.body_ids().into_iter().zip(top.body_ids()) {
        let (by, ty) = (bi.position(b).y, bi.position(t).y);
        assert!(by < 0.15 && ty > 0.25, "the top ragdoll should rest on the bottom one: {by} {ty}");
    }
    bottom.destroy(&mut world.system);
    top.destroy(&mut world.system);
    assert_eq!(world.system.num_bodies(), 1);
}
