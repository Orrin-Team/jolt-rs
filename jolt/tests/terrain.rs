//! Editable height fields, shape triangulation and queries against a single shape.

use jolt::*;

const DT: f32 = 1.0 / 60.0;

/// 16 x 16 flat terrain at height 0 that can be edited between -5 and 5, in blocks of 4.
fn terrain() -> Shape {
    let samples = vec![0.0; 16 * 16];
    let settings = HeightFieldSettings { height_range: Some((-5.0, 5.0)), block_size: 4, ..HeightFieldSettings::new(&samples, 16) };
    Shape::height_field_with_settings(&settings).unwrap()
}

#[test]
fn height_fields_are_read_and_edited_in_blocks() {
    let mut shape = terrain();
    let info = shape.height_field_info().unwrap();
    assert_eq!((info.sample_count, info.block_size), (16, 4));
    assert!(info.editable && (info.min_height + 5.0).abs() < 1e-3 && (info.max_height - 5.0).abs() < 1e-3, "{info:?}");
    assert!(shape.height_field_heights(0, 0, 4, 4).unwrap().iter().all(|h| h.abs() < 0.01));
    assert!(Shape::sphere(1.0).unwrap().height_field_info().is_none());
    let (position, hole) = shape.height_field_position(3, 5).unwrap();
    assert!(!hole && position.x == 3.0 && position.z == 5.0);
    assert!(shape.height_field_position(16, 0).is_none());

    let mut temp = TempAllocator::default();
    {
        let mut edit = shape.edit_height_field().unwrap();
        let mut bump = vec![2.0; 16];
        bump[5] = f32::MAX; // a hole
        bump[6] = 100.0; // clamped to the range
        edit.set_heights(4, 4, 4, 4, &bump, &mut temp).unwrap();
        // Regions must be aligned to the block size and in range, and match the data.
        assert!(edit.set_heights(2, 4, 4, 4, &bump, &mut temp).unwrap_err().message().contains("block"));
        assert!(edit.set_heights(12, 12, 8, 4, &[0.0; 32], &mut temp).is_err());
        assert!(edit.set_heights(4, 4, 4, 4, &bump[..15], &mut temp).is_err());
        let read = edit.heights(4, 4, 4, 4).unwrap();
        assert!((read[0] - 2.0).abs() < 0.05, "{read:?}");
        assert_eq!(read[5], f32::MAX);
        assert!(read[6] <= info.max_height + 0.05 && read[6] > 4.9, "{}", read[6]);
    }
    assert!(shape.height_field_position(5, 5).unwrap().1, "sample (5, 5) is now a hole");

    // A ray finds the bump; editing a shared shape is refused.
    let hit = shape.cast_ray(&ShapePlacement::default(), Vec3::new(4.0, 10.0, 4.0), Vec3::new(0.0, -20.0, 0.0), &RayCastSettings::default()).unwrap();
    assert!((hit.point.y - 2.0).abs() < 0.05, "{hit:?}");
    let shared = shape.clone();
    assert!(shape.edit_height_field().is_err());
    drop(shared);

    // A height field made entirely of holes stores nothing to edit.
    let holes = vec![f32::MAX; 16];
    let mut empty = Shape::height_field_with_settings(&HeightFieldSettings::new(&holes, 4)).unwrap();
    assert!(!empty.height_field_info().unwrap().editable);
    assert!(empty.edit_height_field().unwrap().set_heights(0, 0, 2, 2, &[1.0; 4], &mut temp).is_err());
}

#[test]
fn height_field_materials_are_read_and_edited() {
    let (grass, rock) = (PhysicsMaterial::new(1), PhysicsMaterial::new(2));
    let samples = vec![0.0; 8 * 8];
    let indices = vec![0u8; 7 * 7];
    let settings = HeightFieldSettings { materials: Some((&indices, &[grass, rock])), ..HeightFieldSettings::new(&samples, 8) };
    let mut shape = Shape::height_field_with_settings(&settings).unwrap();
    assert_eq!(shape.height_field_info().unwrap().num_materials, 2);
    assert_eq!(shape.height_field_material_indices(0, 0, 7, 7).unwrap(), indices);
    assert!(shape.height_field_material_indices(0, 0, 8, 7).is_err());

    let mut temp = TempAllocator::default();
    {
        let mut edit = shape.edit_height_field().unwrap();
        edit.set_material_indices(2, 2, 2, 2, &[1; 4], &mut temp).unwrap();
        assert!(edit.set_material_indices(0, 0, 1, 1, &[2], &mut temp).unwrap_err().message().contains("material"));
        assert_eq!(edit.material_indices(2, 2, 2, 2).unwrap(), vec![1; 4]);
    }
    let down = Vec3::new(0.0, -2.0, 0.0);
    let on_rock = shape.cast_ray(&ShapePlacement::default(), Vec3::new(2.5, 1.0, 2.5), down, &RayCastSettings::default()).unwrap();
    let on_grass = shape.cast_ray(&ShapePlacement::default(), Vec3::new(5.5, 1.0, 5.5), down, &RayCastSettings::default()).unwrap();
    assert_eq!(on_rock.material.map(|m| m.user_data()), Some(2));
    assert_eq!(on_grass.material.map(|m| m.user_data()), Some(1));

    assert!(Shape::height_field_with_settings(&HeightFieldSettings { materials: Some((&indices[1..], &[PhysicsMaterial::new(0)])), ..HeightFieldSettings::new(&samples, 8) }).is_err());
}

#[test]
fn terrain_under_bodies_is_edited_and_wakes_them() {
    let mut layers = LayerConfig::new();
    let static_layer = layers.add_object_layer(BroadPhaseLayer(0));
    let moving_layer = layers.add_object_layer(BroadPhaseLayer(1));
    layers.enable_collision(static_layer, moving_layer).enable_collision(moving_layer, moving_layer);
    let mut system = PhysicsSystem::new(&PhysicsSystemSettings::default(), &layers).unwrap();
    let (mut temp, jobs) = (TempAllocator::default(), JobSystem::thread_pool(Some(2)));

    let shape = terrain();
    let ground = BodyCreationSettings::new(shape.clone(), Vec3::ZERO, Quat::IDENTITY, MotionType::Static, static_layer);
    let ground = system.body_interface().create_and_add_body(&ground, Activation::DontActivate).unwrap();
    let ball = BodyCreationSettings::new(Shape::sphere(0.5).unwrap(), Vec3::new(6.0, 0.6, 6.0), Quat::IDENTITY, MotionType::Dynamic, moving_layer);
    let ball = system.body_interface().create_and_add_body(&ball, Activation::Activate).unwrap();
    for _ in 0..180 {
        system.update(DT, 1, &mut temp, &jobs).unwrap();
    }
    assert!(!system.body_interface().is_active(ball), "the ball went to sleep");

    // Raise the terrain under the ball (less than its radius: height fields only collide from above). The test still
    // holds `shape`, so the body gets its own copy.
    let before = system.body_interface().shape(ground).unwrap().as_raw();
    system.body_interface().modify_height_field(ground, |edit| edit.set_heights(4, 4, 4, 4, &[0.3; 16], &mut temp).unwrap()).unwrap();
    assert_ne!(system.body_interface().shape(ground).unwrap().as_raw(), before);
    assert!(shape.height_field_heights(4, 4, 4, 4).unwrap().iter().all(|h| h.abs() < 0.01), "the shared shape is unchanged");
    assert!(!system.body_interface().is_active(ball), "editing doesn't wake bodies");

    system.body_interface().activate_bodies_in_box(AABox { min: Vec3::new(4.0, -1.0, 4.0), max: Vec3::new(8.0, 3.0, 8.0) }, &QueryFilter::new());
    assert!(system.body_interface().is_active(ball));
    for _ in 0..60 {
        system.update(DT, 1, &mut temp, &jobs).unwrap();
    }
    let y = system.body_interface().position(ball).y;
    assert!(y > 0.75, "the ball rests on the raised terrain: {y}");

    let not_terrain = system.body_interface().modify_height_field(ball, |_| unreachable!());
    assert!(not_terrain.is_err());
}

#[test]
fn shapes_are_triangulated() {
    let cube = Shape::cuboid_with_convex_radius(Vec3::splat(1.0), 0.0).unwrap();
    let placed = ShapePlacement::new(Vec3::new(10.0, 0.0, 0.0), Quat::IDENTITY);
    let triangles = cube.triangles(&placed, None);
    assert_eq!(triangles.len(), 12);
    assert!(triangles.iter().flat_map(|t| t.vertices).all(|v| (v.x - 10.0).abs() <= 1.0 + 1e-4 && v.y.abs() <= 1.0 + 1e-4));
    // Only what is inside the bounds, at the granularity of the shape.
    let ground = terrain();
    let all = ground.triangles(&ShapePlacement::default(), None).len();
    let part = ground.triangles(&ShapePlacement::default(), Some(AABox { min: Vec3::new(0.0, -1.0, 0.0), max: Vec3::new(3.0, 1.0, 3.0) })).len();
    assert!(all >= 2 * 15 * 15 && part < all, "{part} of {all}");

    let ice = PhysicsMaterial::new(9);
    let mesh = Shape::mesh_with_materials(&[Vec3::ZERO, Vec3::X, Vec3::Z], &[[0, 2, 1]], &[0], &[ice]).unwrap();
    let triangles = mesh.triangles(&ShapePlacement::default(), None);
    assert_eq!(triangles.len(), 1);
    assert_eq!(triangles[0].material.as_ref().map(PhysicsMaterial::user_data), Some(9));
    assert!(!Shape::sphere(1.0).unwrap().triangles(&ShapePlacement::default(), None).is_empty());
}

#[test]
fn queries_against_a_single_shape() {
    let sphere = Shape::sphere(1.0).unwrap();
    let at = ShapePlacement::new(Vec3::new(0.0, 0.0, 5.0), Quat::IDENTITY);
    let hit = sphere.cast_ray(&at, Vec3::ZERO, Vec3::new(0.0, 0.0, 10.0), &RayCastSettings::default()).unwrap();
    assert_eq!(hit.body_id, BodyId::INVALID);
    assert!((hit.fraction - 0.4).abs() < 1e-4 && (hit.normal.z + 1.0).abs() < 1e-3, "{hit:?}");
    assert!(sphere.cast_ray(&at, Vec3::new(5.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 10.0), &RayCastSettings::default()).is_none());
    assert!(sphere.contains_point(&at, Vec3::new(0.0, 0.5, 5.0)));
    assert!(!sphere.contains_point(&at, Vec3::ZERO));
    // Scale applies to the shape.
    let big = ShapePlacement { scale: Vec3::splat(3.0), ..at };
    assert!(sphere.contains_point(&big, Vec3::new(0.0, 2.5, 5.0)));

    let cube = Shape::cuboid(Vec3::splat(0.5)).unwrap();
    let overlapping = RMat44::from_translation(Vec3::new(0.0, 0.0, 6.2));
    let hits = sphere.collide_shape(&at, &cube, Vec3::splat(1.0), &overlapping, &CollideShapeSettings::default());
    assert_eq!(hits.len(), 1);
    assert!(hits[0].penetration_depth > 0.1, "{:?}", hits[0]);
    let apart = RMat44::from_translation(Vec3::new(0.0, 0.0, 9.0));
    assert!(sphere.collide_shape(&at, &cube, Vec3::splat(1.0), &apart, &CollideShapeSettings::default()).is_empty());
    let swept = sphere.cast_shape(&at, &cube, Vec3::splat(1.0), &apart, Vec3::new(0.0, 0.0, -6.0), &ShapeCastSettings::default());
    assert_eq!(swept.len(), 1);
    assert!((swept[0].fraction - 2.5 / 6.0).abs() < 0.02, "{:?}", swept[0]);
}
