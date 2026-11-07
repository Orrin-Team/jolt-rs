//! Safe Rust bindings to [Jolt Physics](https://github.com/jrouwe/JoltPhysics), built on the `joltc` C API in
//! [`jolt_sys`].
//!
//! ```
//! use jolt::*;
//!
//! let mut layers = LayerConfig::new();
//! let static_layer = layers.add_object_layer(BroadPhaseLayer(0));
//! let moving_layer = layers.add_object_layer(BroadPhaseLayer(1));
//! layers.enable_collision(static_layer, moving_layer).enable_collision(moving_layer, moving_layer);
//!
//! let mut system = PhysicsSystem::new(&PhysicsSystemSettings::default(), &layers).unwrap();
//! let bodies = system.body_interface();
//!
//! let floor = BodyCreationSettings::new(
//!     Shape::cuboid(Vec3::new(100.0, 1.0, 100.0)).unwrap(),
//!     Vec3::new(0.0, -1.0, 0.0),
//!     Quat::IDENTITY,
//!     MotionType::Static,
//!     static_layer,
//! );
//! bodies.create_and_add_body(&floor, Activation::DontActivate).unwrap();
//!
//! let ball = BodyCreationSettings::new(Shape::sphere(0.5).unwrap(), Vec3::new(0.0, 2.0, 0.0), Quat::IDENTITY, MotionType::Dynamic, moving_layer);
//! let ball = bodies.create_and_add_body(&ball, Activation::Activate).unwrap();
//!
//! let mut temp = TempAllocator::default();
//! let jobs = JobSystem::thread_pool(None);
//! for _ in 0..120 {
//!     system.update(1.0 / 60.0, 1, &mut temp, &jobs).unwrap();
//! }
//! assert!((system.body_interface().position(ball).y - 0.5).abs() < 0.05);
//! ```

mod body;
pub mod character;
mod constraint;
mod error;
mod group;
mod layers;
mod listener;
mod material;
mod math;
mod query;
pub mod ragdoll;
mod shape;
mod snapshot;
mod system;
pub mod vehicle;

use std::ffi::{CStr, c_char};
use std::sync::{Once, RwLock};

pub use jolt_sys as sys;

pub use body::{
    Activation, AllowedDofs, BatchError, Body, BodyCreationSettings, BodyId, BodyInterface, MotionQuality, MotionType, OverrideMassProperties,
    SubShapeId,
};
pub use character::{
    Character, CharacterContact, CharacterContactListener, CharacterContactSettings, CharacterGroup, CharacterGround, CharacterId,
    CharacterSettings, CharacterView, CharacterVirtual, CharacterVirtualSettings, ExtendedUpdateSettings, GroundState,
};
pub use constraint::{
    ConeConstraintSettings, ConstraintCommon, ConstraintId, ConstraintKind, ConstraintMut, ConstraintSettings, ConstraintSpace,
    DistanceConstraintSettings, FixedConstraintSettings, HingeConstraintSettings, MotorSettings, MotorState, PointConstraintSettings,
    GearConstraintSettings, PathConstraintPath, PathConstraintSettings, PathPoint, PathRotationConstraint, PulleyConstraintSettings,
    RackAndPinionConstraintSettings, SixDofAxis, SixDofConstraintSettings, SliderConstraintSettings, SpringSettings,
    SwingTwistConstraintSettings, SwingType,
};
pub use error::{Error, Result};
pub use group::{CollisionGroup, GroupFilter, GroupFilterTable};
pub use layers::{BroadPhaseLayer, LayerConfig, ObjectLayer};
pub use listener::{
    BodyActivationListener, CollideShapeResult, ContactListener, ContactManifold, ContactSettings, StepBodies, StepContext, StepListener,
    SubShapeIdPair, ValidateResult,
};
pub use material::PhysicsMaterial;
pub use vehicle::{
    AntiRollBar, TransmissionMode, VehicleCollisionTester, VehicleDifferentialSettings, VehicleEngineSettings, VehicleMut,
    VehicleTransmissionSettings, WheelSettings, WheelState, WheeledVehicleSettings,
};
pub use math::{AABox, Mat44, Quat, Vec3};
pub use query::{
    ActiveEdgeMode, BackFaceMode, BroadPhaseCastHit, CollideShapeSettings, PointHit, QueryFilter, RayCastHit, RayCastSettings, ShapeCastSettings,
    ShapeHit,
};
pub use ragdoll::{JointState, Ragdoll, RagdollConstraint, RagdollJoint, RagdollOptions, RagdollPart, RagdollSettings, Skeleton, SkeletonBuilder};
pub use shape::{CompoundSubShape, DEFAULT_CONVEX_RADIUS, MassProperties, Shape, ShapeSubType, ShapeType};
pub use snapshot::{PhysicsSnapshot, StateParts};
pub use system::{JobSystem, PhysicsSettings, PhysicsSystem, PhysicsSystemSettings, TempAllocator, UpdateError};

/// Sets up Jolt's allocator, factory and type registry. Called automatically by every constructor,
/// so there is normally no need to call it yourself.
pub fn init() {
    ensure_init();
}

pub(crate) fn ensure_init() {
    static INIT: Once = Once::new();
    INIT.call_once(|| unsafe {
        sys::JPC_Init();
        sys::JPC_SetTraceHandler(Some(trace_trampoline));
    });
}

static TRACE_HANDLER: RwLock<Option<fn(&str)>> = RwLock::new(None);

/// Routes Jolt's diagnostic messages to `handler` instead of stderr. Pass `None` to restore stderr.
pub fn set_trace_handler(handler: Option<fn(&str)>) {
    ensure_init();
    *TRACE_HANDLER.write().unwrap_or_else(|e| e.into_inner()) = handler;
}

unsafe extern "C" fn trace_trampoline(message: *const c_char) {
    let message = unsafe { CStr::from_ptr(message) }.to_string_lossy();
    match *TRACE_HANDLER.read().unwrap_or_else(|e| e.into_inner()) {
        // A panicking handler must not unwind into C++.
        Some(handler) => {
            if std::panic::catch_unwind(|| handler(&message)).is_err() {
                eprintln!("jolt: trace handler panicked");
            }
        }
        None => eprintln!("{message}"),
    }
}
