use std::fmt;
use std::hash::{Hash, Hasher};
use std::ptr::NonNull;

use jolt_sys as sys;

/// A surface material. Its only property is user data, which contacts, hits and character ground
/// queries report back so you can tell surfaces apart (footstep sounds, friction tables, ...).
/// Reference counted; equality is identity.
pub struct PhysicsMaterial {
    raw: NonNull<sys::JPC_PhysicsMaterial>,
}

// SAFETY: materials are immutable and their reference count is atomic.
unsafe impl Send for PhysicsMaterial {}
unsafe impl Sync for PhysicsMaterial {}

impl PhysicsMaterial {
    pub fn new(user_data: u64) -> Self {
        crate::ensure_init();
        let raw = unsafe { sys::JPC_PhysicsMaterial_Create(user_data) };
        Self { raw: NonNull::new(raw).expect("JPC_PhysicsMaterial_Create returned null") }
    }

    /// Takes a new reference to a material the C layer handed out as borrowed (`None` for the default material).
    pub(crate) fn from_borrowed(raw: *const sys::JPC_PhysicsMaterial) -> Option<Self> {
        let raw = NonNull::new(raw.cast_mut())?;
        unsafe { sys::JPC_PhysicsMaterial_AddRef(raw.as_ptr()) };
        Some(Self { raw })
    }

    /// Takes over a reference the C layer handed out as owned.
    pub(crate) fn from_owned(raw: *const sys::JPC_PhysicsMaterial) -> Option<Self> {
        NonNull::new(raw.cast_mut()).map(|raw| Self { raw })
    }

    pub fn as_raw(&self) -> *const sys::JPC_PhysicsMaterial {
        self.raw.as_ptr()
    }

    pub fn user_data(&self) -> u64 {
        unsafe { sys::JPC_PhysicsMaterial_GetUserData(self.as_raw()) }
    }
}

impl Clone for PhysicsMaterial {
    fn clone(&self) -> Self {
        unsafe { sys::JPC_PhysicsMaterial_AddRef(self.as_raw()) };
        Self { raw: self.raw }
    }
}

impl Drop for PhysicsMaterial {
    fn drop(&mut self) {
        unsafe { sys::JPC_PhysicsMaterial_Release(self.as_raw()) };
    }
}

impl PartialEq for PhysicsMaterial {
    fn eq(&self, other: &Self) -> bool {
        self.raw == other.raw
    }
}

impl Eq for PhysicsMaterial {}

impl Hash for PhysicsMaterial {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.raw.hash(state);
    }
}

impl fmt::Debug for PhysicsMaterial {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PhysicsMaterial").field("user_data", &self.user_data()).finish()
    }
}
