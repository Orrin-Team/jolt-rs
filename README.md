# jolt-rs

[![CI](https://github.com/Orrin-Team/jolt-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/Orrin-Team/jolt-rs/actions/workflows/ci.yml)

Rust bindings to [Jolt Physics](https://github.com/jrouwe/JoltPhysics) 5.6.0, built on **joltc**, a C API over Jolt.

| Path | |
|---|---|
| `jolt-sys/joltc` | The C API (`include/joltc.h`). Usable from any language. |
| `jolt-sys` | Builds Jolt + joltc with CMake; raw bindgen bindings. |
| `jolt` | Safe Rust wrapper. |

```sh
git clone --recursive https://github.com/Orrin-Team/jolt-rs
cargo run --example hello_world   # or: --example character
```

Needs CMake ≥ 3.20 and a C++17 compiler.

## Usage

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
| Queries | ✅ ray (closest/all), point, collide shape, cast shape, with layer/body filters · ❌ broad phase AABB queries, collect-faces settings |
| Constraints | ✅ fixed, point, distance, hinge, slider, cone, swing-twist, six-DOF, motors and limits · ❌ path, gear, pulley, rack-and-pinion |
| Characters | ✅ `CharacterVirtual` (stairs, floor sticking, shape switching, inner body, listener, character-vs-character), rigid `Character` |
| Listeners | ✅ contact, body activation, step |
| Materials | ✅ per shape and per mesh triangle, reported by hits, contacts and characters · ❌ height field materials |
| Not started | ❌ vehicles, ragdolls/skeletons, soft bodies, debug renderer, serialization/state recording, double precision |

## Safety

- `PhysicsSystem::update` and `destroy_body` take `&mut self`; everything else is `&self`, thread-safe, and goes through Jolt's locking interfaces.
- Panics in callbacks are caught at the FFI boundary and resumed in Rust.
- joltc validates what Jolt only asserts in release builds: stale ids, double adds, unknown object layers, out-of-range indices, constraints on freed or character-owned bodies, invalid settings.
- Constraints are owned by the system and die with their bodies. Characters keep the system alive and defer native destruction until no update can be running.
- Query collectors run after the native query returns. Body filters and listeners run inside Jolt and must not use the body interface.

## Development

```sh
cargo test                            # integration, feature and regression tests
cargo test -p jolt --features asserts # with Jolt's asserts compiled in
./scripts/test-asan.sh                # against an AddressSanitizer build (macOS)
JOLT_SYS_UPDATE_BINDINGS=1 cargo build -p jolt-sys --features bindgen  # after editing joltc.h
```

Jolt is always built in Release. On x86 its SIMD level follows the Rust target features; use `-C target-cpu=native` for AVX2.
