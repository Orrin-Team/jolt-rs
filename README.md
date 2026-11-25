# jolt-rs

[![CI](https://github.com/Orrin-Team/jolt-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/Orrin-Team/jolt-rs/actions/workflows/ci.yml)

Rust bindings to [Jolt Physics](https://github.com/jrouwe/JoltPhysics) 5.7.0 (plus upstream fixes up to `78f09c1c`, including a job system hang), built on **joltc**, a C API over Jolt.

| Path | |
|---|---|
| `jolt-sys/joltc` | The C API (`include/joltc.h`). Usable from any language. |
| `jolt-sys` | Builds Jolt + joltc with CMake; raw bindgen bindings. Published as [`orrin-jolt-sys`](https://crates.io/crates/orrin-jolt-sys). |
| `jolt` | Safe Rust wrapper. Published as [`orrin-jolt`](https://crates.io/crates/orrin-jolt). |

```sh
git clone --recursive https://github.com/Orrin-Team/jolt-rs
cargo run --example hello_world   # or: --example character
```

Needs CMake ≥ 3.20 and a C++17 compiler.

## Usage

The names `jolt` and `jolt-sys` were taken on crates.io, so the packages are published as `orrin-jolt` and `orrin-jolt-sys`. The libraries keep their names: depend on the package, write `use jolt`.

```toml
[dependencies]
jolt = { package = "orrin-jolt", version = "0.2" }
```

```rust
use jolt::*;

let mut layers = LayerConfig::new();
let static_layer = layers.add_object_layer(BroadPhaseLayer(0));
let moving_layer = layers.add_object_layer(BroadPhaseLayer(1));
layers.enable_collision(static_layer, moving_layer).enable_collision(moving_layer, moving_layer);

let mut system = PhysicsSystem::new(&PhysicsSystemSettings::default(), &layers)?;
let ball = BodyCreationSettings::new(Shape::sphere(0.5)?, Vec3::new(0.0, 2.0, 0.0), Quat::IDENTITY, MotionType::Dynamic, moving_layer);
let ball = system.body_interface().create_and_add_body(&ball, Activation::Activate)?;

let (mut temp, jobs) = (TempAllocator::default(), JobSystem::thread_pool(None));
system.update(1.0 / 60.0, 1, &mut temp, &jobs)?;
```

## Coverage

| Area | Status |
|---|---|
| Shapes | ✅ sphere, box, capsule, tapered capsule, cylinder, tapered cylinder, triangle, convex hull, mesh, height field (editable heights and materials, also on live bodies), plane, empty, static and mutable compound (editable in place, also on live bodies), rotated/translated, scaled, offset COM, compound child lookup from hits, triangulation, queries against a single shape, binary serialization |
| Bodies | ✅ creation settings, add/remove/destroy (single and batch), transforms, velocities, forces, impulses, buoyancy, kinematics, motion type/quality, material properties, collision groups, bounds, mass, user data, waking bodies in a box |
| Queries | ✅ ray (closest/all), point, collide shape, cast shape, broad phase (box, sphere, point, oriented box, ray, box cast), back face / active edge / tolerance settings, collected faces, layer and body filters |
| Constraints | ✅ fixed, point, distance, hinge, slider, cone, swing-twist, six-DOF, gear, rack-and-pinion, pulley, path (Hermite), motors and limits |
| Vehicles | ✅ wheeled vehicles (engine, transmission, differentials, anti-roll bars, friction curves, ray/sphere/cylinder testers), motorcycles (lean control), tracked vehicles |
| Ragdolls | ✅ skeletons (joint hierarchy, local ↔ model space poses), ragdoll settings (any supported joint type, stabilisation, parent-child collision filtering), ragdolls (pose, kinematic and motor driving, impulses), keyframed skeletal animation, skeleton mapping (chains, locked translations) |
| Snapshots | ✅ save/restore (global, bodies, contacts, constraints), character state, bytes for networking |
| Characters | ✅ `CharacterVirtual` (stairs, floor sticking, shape switching, inner body, listener, character-vs-character), rigid `Character` |
| Listeners | ✅ contact (with collision response estimation), soft body contact, body activation, step |
| Materials | ✅ per shape, mesh triangle and height field cell, reported by hits, contacts and characters |
| Soft bodies | ✅ cloth, cubes and custom meshes (vertices, faces, materials, edge/shear/bend/volume/long range constraints, generated or explicit), pinned vertices, pressure, vertex read/write, skinning to animated joints, Cosserat rods, contact listener, snapshots |
| Debug rendering | ✅ bodies (shapes, bounds, transforms, velocities, soft body internals), constraints, limits, reference frames and skeleton poses as lines, triangles and text, behind the `debug-renderer` feature |
| Double precision | ✅ `f64` world space positions and transforms (`RVec3`, `RMat44`) throughout bodies, queries, contacts, constraints, characters, vehicles, ragdolls, soft bodies and debug rendering, behind the `double-precision` feature |

## Features

| Feature | |
|---|---|
| `double-precision` | World space positions in `f64`, for worlds thousands of kilometers across. `RVec3`/`RMat44` become `DVec3`/`DMat44`; methods taking a position still accept a `Vec3`. Constraints to the world stay `f32` relative to the origin, so far out attach them to a static body. |
| `debug-renderer` | Compiles Jolt's debug renderer in, for `DebugRenderer`. |
| `asserts` | Compiles Jolt's asserts in. |
| `cross-platform-deterministic` | Identical simulation results across platforms and compilers (slower). |
| `glam` | Conversions to and from `glam` types. |

## Safety

- `PhysicsSystem::update` and `destroy_body` take `&mut self`; everything else is `&self`, thread-safe, and goes through Jolt's locking interfaces.
- Panics in callbacks are caught at the FFI boundary and resumed in Rust.
- joltc validates what Jolt only asserts in release builds: stale ids, double adds, unknown object layers, out-of-range indices, constraints on freed or character-owned bodies, invalid settings.
- Soft bodies share ids with rigid bodies, so rigid-only calls refuse them: shapes (a soft body's shape lives inside the body), kinematic motion, non-dynamic motion types, buoyancy, constraints and vehicle bodies. Vehicle wheels ignore soft bodies, which Jolt would otherwise link into the wrong simulation island.
- Constraints are owned by the system and die with their bodies. Characters and ragdolls keep the system alive, own their bodies, and defer native destruction until no update can be running.
- Query collectors run after the native query returns. Body filters and listeners run inside Jolt and must not use the body interface.
- Vehicles and step listeners run one after another from a single Jolt step listener, so they never race each other.
- In-memory snapshots only restore onto the same, structurally unchanged world (or the same character); restoring raw bytes is `unsafe`.
- Shapes stay immutable once shared. A body's mutable compound is edited under the body's write lock, and if anything else holds the shape the body gets its own copy first.
- Loading serialized shapes is `unsafe`: the header, Jolt version, shape types, references and length are checked, the rest is trusted.

## Development

```sh
cargo test                            # integration, feature and regression tests
cargo test -p orrin-jolt --features asserts # with Jolt's asserts compiled in
cargo test -p orrin-jolt --features debug-renderer # with Jolt's debug renderer compiled in
cargo test -p orrin-jolt --features double-precision # with f64 world space positions
./scripts/test-asan.sh                # against an AddressSanitizer build (macOS)
JOLT_SYS_UPDATE_BINDINGS=1 cargo build -p orrin-jolt-sys --features bindgen  # after editing joltc.h
```

Jolt is always built in Release. On x86 its SIMD level follows the Rust target features; use `-C target-cpu=native` for AVX2.
