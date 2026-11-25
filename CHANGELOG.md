# Changelog

## Unreleased

### Added

- **Double precision** behind the `double-precision` feature: Jolt keeps world space positions in `f64`. `RVec3`,
  `RMat44` and `Real` follow the feature (`Vec3`, `Mat44`, `f32` by default; `DVec3`, `DMat44`, `f64` with it). Methods
  that take a world space position accept `impl Into<RVec3>`, so a `Vec3` works in either build. Query hits, contact
  manifolds, collected faces, triangulated shapes and soft body vertices come back in full precision. joltc gains
  `JPC_DVec3`, `JPC_DMat44`, `JPC_RVec3`, `JPC_RMat44` and `JPC_IsDoublePrecision`, switched by `JPC_DOUBLE_PRECISION`
  (CMake option `DOUBLE_PRECISION`).
- `is_double_precision()`.

### Changed

- World space positions and transforms in the API are typed `RVec3`/`RMat44`, which are `Vec3`/`Mat44` without the new
  feature, so single precision code is unaffected. `DebugDraw` methods take `RVec3`. Methods taking a position take
  `impl Into<RVec3>`, which can need a type annotation where a bare `.into()` was passed before.
- Collision query results from the system now use the query's own position as base offset instead of the origin, so
  hits keep their precision far from the origin.

## 0.2.0

Built on Jolt Physics 5.7.0 plus upstream fixes up to `78f09c1c`, which include a fix for 5.7.0's job system running out of
jobs (an assert in asserts builds, a hang otherwise) when worker threads don't get scheduled.

### Added

- **Shapes**: mutable compounds, editable in place or on a live body (copy-on-write when shared); compound child lookup
  from hits; triangle shapes; editable height fields (heights, holes and material indices, in blocks) with control over
  block size, compression and the editable height range; triangulation of any shape; ray, point, collide and cast queries
  against a single shape without a world; binary serialization of shapes with their children and materials.
- **Bodies**: `activate_bodies_in_box` to wake bodies after editing terrain under them.
- **Soft bodies**: cloth, ropes, balloons and jelly from vertices and faces, with generated or explicit edge, shear, bend,
  volume and long range constraints; pinned vertices; pressure; vertex read and write; skinning to animated joints;
  Cosserat rods; a soft body contact listener; snapshots.
- **Vehicles**: motorcycles with lean control, and tracked vehicles.
- **Characters**: snapshots of virtual and rigid character state.
- **Animation**: keyframed skeletal animation and skeleton mapping between a ragdoll and an animation skeleton.
- **Queries**: collected faces on shape hits.
- **Contacts**: collision response estimation (impulses and resulting velocities) from contact callbacks.
- **Debug rendering** behind the `debug-renderer` feature: bodies, constraints, limits, reference frames and skeleton
  poses as lines, triangles and text.

### Changed

- `Body::shape` returns `Option<Shape>` (`None` for soft bodies, whose shape lives inside the body).
- `VehicleMut::set_collision_tester` returns a `Result`: testers are checked against the vehicle's system's layers.
- `PhysicsSettings::deterministic_simulation` is gone; Jolt's simulation is always deterministic now.
- `CollideShapeSettings`, `ShapeCastSettings` and `ShapeHit` gained public fields (`collect_faces`, `face1`, `face2`).

### Fixed

- Calls that only make sense for rigid bodies refuse soft bodies instead of reaching code that could corrupt Jolt's
  simulation islands or hand out a dangling shape: shapes, kinematic motion, non-dynamic motion types, buoyancy, sensors,
  constraints and vehicle bodies. Vehicle wheels ignore soft bodies.
- Reading the inverse inertia of a static body no longer dereferences null.
- Vehicle collision testers from another system can no longer index layer tables out of bounds.
- MSVC: Jolt no longer traps floating point exceptions in its worker threads, where Rust callbacks also run.

## 0.1.0

First release.
