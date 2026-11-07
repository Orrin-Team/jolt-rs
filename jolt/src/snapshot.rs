use std::fmt;
use std::ptr::NonNull;

use jolt_sys as sys;

use crate::{Error, PhysicsSystem, Result};

/// Which parts of the simulation a snapshot contains.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct StateParts(u32);

impl StateParts {
    /// Gravity and the previous step's delta time.
    pub const GLOBAL: Self = Self(sys::JPC_STATE_PARTS_GLOBAL);
    /// Positions, velocities and sleep state of every body in the world.
    pub const BODIES: Self = Self(sys::JPC_STATE_PARTS_BODIES);
    /// Cached contacts, for warm starting.
    pub const CONTACTS: Self = Self(sys::JPC_STATE_PARTS_CONTACTS);
    /// Constraint impulses and motor state.
    pub const CONSTRAINTS: Self = Self(sys::JPC_STATE_PARTS_CONSTRAINTS);
    pub const ALL: Self = Self(sys::JPC_STATE_PARTS_ALL);
}

impl std::ops::BitOr for StateParts {
    type Output = Self;
    fn bitor(self, rhs: Self) -> Self {
        Self(self.0 | rhs.0)
    }
}

/// A saved simulation state, for rollback netcode, replays or undo. Restoring all parts onto the same world
/// reproduces the simulation exactly (with `deterministic_simulation` on, which is the default).
///
/// Character state is not included.
pub struct PhysicsSnapshot {
    raw: NonNull<sys::JPC_PhysicsSnapshot>,
}

// SAFETY: a snapshot is an immutable byte buffer plus bookkeeping.
unsafe impl Send for PhysicsSnapshot {}
unsafe impl Sync for PhysicsSnapshot {}

impl PhysicsSnapshot {
    /// The serialized state, e.g. to send over the network.
    pub fn as_bytes(&self) -> &[u8] {
        unsafe {
            let data = sys::JPC_PhysicsSnapshot_GetData(self.raw.as_ptr()).cast::<u8>();
            let size = sys::JPC_PhysicsSnapshot_GetSize(self.raw.as_ptr());
            if size == 0 { &[] } else { std::slice::from_raw_parts(data, size) }
        }
    }

    /// Wraps bytes produced by [`as_bytes`](Self::as_bytes).
    ///
    /// # Safety
    /// Jolt restores state without validating it. The bytes must come from `as_bytes` of a snapshot of a world
    /// with exactly the same bodies (ids, shapes, motion types) and constraints, in the same order, as the one it
    /// is restored onto. Anything else, including corrupted or untrusted network data, is undefined behaviour.
    pub unsafe fn from_bytes(bytes: &[u8]) -> Self {
        crate::ensure_init();
        let raw = unsafe { sys::JPC_PhysicsSnapshot_CreateFromData(bytes.as_ptr().cast(), bytes.len()) };
        Self { raw: NonNull::new(raw).expect("allocation failed") }
    }
}

impl Drop for PhysicsSnapshot {
    fn drop(&mut self) {
        unsafe { sys::JPC_PhysicsSnapshot_Destroy(self.raw.as_ptr()) }
    }
}

impl fmt::Debug for PhysicsSnapshot {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PhysicsSnapshot").field("bytes", &self.as_bytes().len()).finish()
    }
}

impl PhysicsSystem {
    pub fn save_state(&self, parts: StateParts) -> PhysicsSnapshot {
        let raw = unsafe { sys::JPC_PhysicsSystem_SaveState(self.raw(), parts.0 as sys::JPC_StateParts) };
        PhysicsSnapshot { raw: NonNull::new(raw).expect("allocation failed") }
    }

    /// Restores a snapshot of this system. Fails if bodies, shapes, motion types or constraints were added,
    /// removed or changed since it was taken. A failure part way through can leave the world partly restored.
    pub fn restore_state(&mut self, snapshot: &PhysicsSnapshot) -> Result<()> {
        if unsafe { sys::JPC_PhysicsSystem_RestoreState(self.raw(), snapshot.raw.as_ptr()) } { Ok(()) } else { Err(Error::last()) }
    }
}
