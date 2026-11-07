# jolt-rs

[![CI](https://github.com/Orrin-Team/jolt-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/Orrin-Team/jolt-rs/actions/workflows/ci.yml)

Rust bindings to [Jolt Physics](https://github.com/jrouwe/JoltPhysics) 5.6.0, built on **joltc**, a C API over Jolt.

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
jolt = { package = "orrin-jolt", version = "0.1" }
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
| Shapes | ✅ sphere, box, capsule, tapered capsule, cylinder, tapered cylinder, convex hull, mesh, height field, plane, empty, static compound, rotated/translated, scaled, offset COM · ❌ mutable compound |
| Bodies | ✅ creation settings, add/remove/destroy (single and batch), transforms, velocities, forces, impulses, buoyancy, kinematics, motion type/quality, material properties, collision groups, bounds, mass, user data |
| Queries | ✅ ray (closest/all), point, collide shape, cast shape, broad phase (box, sphere, point, oriented box, ray, box cast), back face / active edge / tolerance settings, layer and body filters · ❌ collect-faces |
| Constraints | ✅ fixed, point, distance, hinge, slider, cone, swing-twist, six-DOF, gear, rack-and-pinion, pulley, path (Hermite), motors and limits |
| Vehicles | ✅ wheeled vehicles (engine, transmission, differentials, anti-roll bars, friction curves, ray/sphere/cylinder testers) · ❌ tracked vehicles, motorcycles |
| Ragdolls | ✅ skeletons (joint hierarchy, local ↔ model space poses), ragdoll settings (any supported joint type, stabilisation, parent-child collision filtering), ragdolls (pose, kinematic and motor driving, impulses) · ❌ skeletal animation, skeleton mapping |
| Snapshots | ✅ save/restore (global, bodies, contacts, constraints), bytes for networking · ❌ character state |
| Characters | ✅ `CharacterVirtual` (stairs, floor sticking, shape switching, inner body, listener, character-vs-character), rigid `Character` |
| Listeners | ✅ contact, body activation, step |
| Materials | ✅ per shape, mesh triangle and height field cell, reported by hits, contacts and characters |
| Not started | ❌ soft bodies, mutable compounds, debug renderer, shape serialization, double precision |

## Safety

- `PhysicsSystem::update` and `destroy_body` take `&mut self`; everything else is `&self`, thread-safe, and goes through Jolt's locking interfaces.
- Panics in callbacks are caught at the FFI boundary and resumed in Rust.
- joltc validates what Jolt only asserts in release builds: stale ids, double adds, unknown object layers, out-of-range indices, constraints on freed or character-owned bodies, invalid settings.
- Constraints are owned by the system and die with their bodies. Characters and ragdolls keep the system alive, own their bodies, and defer native destruction until no update can be running.
- Query collectors run after the native query returns. Body filters and listeners run inside Jolt and must not use the body interface.
- Vehicles and step listeners run one after another from a single Jolt step listener, so they never race each other.
- In-memory snapshots only restore onto the same, structurally unchanged world; restoring raw bytes is `unsafe`.

## Development

```sh
cargo test                            # integration, feature and regression tests
cargo test -p orrin-jolt --features asserts # with Jolt's asserts compiled in
./scripts/test-asan.sh                # against an AddressSanitizer build (macOS)
JOLT_SYS_UPDATE_BINDINGS=1 cargo build -p orrin-jolt-sys --features bindgen  # after editing joltc.h
```

Jolt is always built in Release. On x86 its SIMD level follows the Rust target features; use `-C target-cpu=native` for AVX2.
