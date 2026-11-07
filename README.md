# jolt-rs

Rust bindings to [Jolt Physics](https://github.com/jrouwe/JoltPhysics) (v5.6.0), built in two layers:

| Crate / dir | What it is |
|---|---|
| `jolt-sys/joltc` | **joltc**, a C API over Jolt (`include/joltc.h`, `src/joltc.cpp`). Usable from any language, not just Rust. |
| `jolt-sys` | Builds Jolt + joltc with CMake and exposes bindgen-generated raw bindings. |
| `jolt` | Safe, idiomatic Rust wrapper. |

```
git clone --recursive …        # or: git submodule update --init
cargo run --example hello_world
cargo run --example character
cargo test
```

Requires CMake ≥ 3.20 and a C++17 compiler.

## Using the Rust crate

```rust
use jolt::*;

let mut layers = LayerConfig::new();
let static_layer = layers.add_object_layer(BroadPhaseLayer(0));
let moving_layer = layers.add_object_layer(BroadPhaseLayer(1));
layers.enable_collision(static_layer, moving_layer).enable_collision(moving_layer, moving_layer);

let mut system = PhysicsSystem::new(&PhysicsSystemSettings::default(), &layers)?;
let bodies = system.body_interface();
let ball = BodyCreationSettings::new(Shape::sphere(0.5)?, Vec3::new(0.0, 2.0, 0.0), Quat::IDENTITY, MotionType::Dynamic, moving_layer);
let ball = bodies.create_and_add_body(&ball, Activation::Activate)?;

let mut temp = TempAllocator::default();
let jobs = JobSystem::thread_pool(None);
system.update(1.0 / 60.0, 1, &mut temp, &jobs)?;
```

Covered:

- **Shapes:** sphere, box, capsule, tapered capsule, cylinder, tapered cylinder, convex hull, mesh (optionally with per-triangle materials), height field, plane, empty, static compound, rotated/translated, scaled, offset centre of mass. Shapes are ref-counted and cheap to clone.
- **Bodies:** the full creation settings, and through `BodyInterface`: add/remove/destroy (also in batches), activation, transforms, velocities, forces, impulses, buoyancy, kinematic motion, motion type and quality, friction/restitution/velocity limits, collision groups, bounds, mass, user data.
- **Characters:** `CharacterVirtual` (sweep-based controller with stair walking, floor sticking, shape switching for crouching, an optional inner body, contact listener, character-vs-character groups) and the rigid-body `Character`.
- **Materials:** `PhysicsMaterial` handles with user data, reported by ray and shape hits, contact manifolds, character contacts and the character's ground.
- **Collision groups:** `GroupFilterTable` for ragdoll-style "these parts don't collide" rules on top of object layers.
- **Listeners:** `ContactListener` (validate/added/persisted/removed, with editable `ContactSettings`), `BodyActivationListener` and `StepListener` (runs every collision step).
- **Queries:** closest ray and all rays, collide point, collide shape, cast shape, each filtered by object layer, an ignored body or a closure.
- **Constraints:** fixed, point, distance, hinge, slider, cone, swing-twist and six-DOF, including motors and limits.
- **Settings:** physics settings, gravity, broad phase optimisation.
- **Features:** `glam` (conversions), `asserts` (Jolt asserts), `cross-platform-deterministic`.

### Safety model

- `PhysicsSystem::update` and `PhysicsSystem::destroy_body` take `&mut self`, so nothing can touch bodies during a step or while one is freed. The rest of the API is `&self` and thread-safe, because it goes through Jolt's locking interface.
- Every object layer coming in through the API is checked against the system's `LayerConfig`; Jolt indexes its layer tables without bounds checks. Queries from an unknown layer hit nothing.
- Listener callbacks run on worker threads, so listeners must be `Send + Sync`. A panic inside any callback (listener, query collector or filter) is caught at the FFI boundary and re-raised from `update` or from the query call. It never unwinds through C++.
- Constraints are owned by the `PhysicsSystem`, and you refer to them by `ConstraintId`. Destroying a body also destroys its constraints, and constraints can't use character bodies or stale ids, so a constraint can never point at a freed body.
- Collectors of `cast_ray_all`, `collide_point`, `collide_shape` and `cast_shape` get their hits after the native query returns, so they may use the body interface. Body filters run inside the query and must not touch the system; listeners must not use the body interface either (Jolt holds locks there).
- Materials and group filters handed back to Rust are owned references, taken while the source body was locked, so they stay valid after the body is gone. Characters keep their ground and contact materials alive the same way.
- A job system lets at most `max_barriers` physics systems update at once; further updates wait for a slot. `max_jobs` is raised to Jolt's minimum. Physics settings Jolt can't run with (zero batch sizes and the like) are rejected.
- Characters keep the native system alive. Methods that touch the world take `&PhysicsSystem`, which stops them from running during an update. Dropping a character defers its destruction to the next update; `destroy(&mut system)` removes it immediately. Bodies owned by characters can't be destroyed or used in constraints.
- In release builds Jolt guards several kinds of misuse only with asserts. The C layer checks them up front instead: adding a body twice, destroying a body that is still in the world, moving a static body, making a body dynamic when it has no motion properties, forces and impulses on bodies that aren't in the world, out-of-range mesh and material indices, out-of-range collision sub groups, and bad layer tables.
- Stale `BodyId`s are harmless no-ops. Jolt's own `DestroyBody` doesn't check an ID's sequence number, so the C layer validates IDs first and serialises adds, removes and destroys.
- Shapes can only be mutated (`set_density`, `set_user_data`) while no other handle or body shares them.

## Testing

`cargo test` runs the integration, feature and regression suites; add `--features asserts` to compile Jolt's asserts in (a firing assert aborts the test run). `scripts/test-asan.sh` runs the same tests against an AddressSanitizer build of Jolt and joltc, which catches out-of-bounds and use-after-free bugs that don't crash on their own.

## Using the C API directly

`joltc.h` is self-contained and documents ownership: every returned `JPC_Shape*`/`JPC_Constraint*` is one reference you must release, and failures set `JPC_GetLastError()`. To build the C example:

```
cmake -S jolt-sys/joltc -B build -DCMAKE_BUILD_TYPE=Release -DJOLTC_BUILD_EXAMPLES=ON \
      -DJPH_USE_DX12=OFF -DJPH_USE_VK=OFF -DJPH_USE_MTL=OFF -DJPH_USE_CPU_COMPUTE=OFF
cmake --build build --target joltc_hello_world && ./build/joltc_hello_world
```

joltc compiles into the `Jolt` target itself, so it always gets exactly the same SIMD/RTTI flags and defines as Jolt. Mismatching those is the classic way Jolt wrappers break.

## Build notes

- Jolt is always built in Release, even for `cargo build`, because a debug build of Jolt is too slow to use.
- On x86, the SIMD level (SSE4/AVX/AVX2/…) follows the Rust target features, so the default target gets a portable SSE2 build. Build with `RUSTFLAGS="-C target-cpu=native"` to get AVX2.
- Regenerating the bindings after editing `joltc.h` needs libclang:
  `JOLT_SYS_UPDATE_BINDINGS=1 cargo build -p jolt-sys --features bindgen`

## Not covered yet

Vehicles, ragdolls and skeletons, soft bodies, path / gear / pulley / rack-and-pinion constraints, mutable compounds, height field materials, the debug renderer, serialization and state recording, and double precision (`JPH_DOUBLE_PRECISION`). Each one follows the same pattern: a C function in `joltc.h`/`joltc.cpp`, then a safe wrapper.

Only tested on macOS/arm64 so far. The build script has Linux, Windows and x86 paths, but none of them have been tried.
