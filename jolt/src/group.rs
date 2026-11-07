use std::fmt;
use std::ptr::NonNull;

use jolt_sys as sys;

/// Decides which sub groups within one [`CollisionGroup`] collide, e.g. which parts of a ragdoll may touch.
/// Built with [`GroupFilterTable`] and immutable afterwards.
pub struct GroupFilter {
    raw: NonNull<sys::JPC_GroupFilter>,
}

// SAFETY: the filter cannot be changed once built and its reference count is atomic.
unsafe impl Send for GroupFilter {}
unsafe impl Sync for GroupFilter {}

impl GroupFilter {
    /// Takes over a reference the C layer handed out as owned.
    pub(crate) fn from_owned(raw: *const sys::JPC_GroupFilter) -> Option<Self> {
        NonNull::new(raw.cast_mut()).map(|raw| Self { raw })
    }

    pub fn as_raw(&self) -> *const sys::JPC_GroupFilter {
        self.raw.as_ptr()
    }

    /// Whether two different sub groups of the same group collide. Out of range sub groups always collide.
    pub fn is_collision_enabled(&self, sub_group1: u32, sub_group2: u32) -> bool {
        unsafe { sys::JPC_GroupFilterTable_IsCollisionEnabled(self.as_raw(), sub_group1, sub_group2) }
    }
}

impl Clone for GroupFilter {
    fn clone(&self) -> Self {
        unsafe { sys::JPC_GroupFilter_AddRef(self.as_raw()) };
        Self { raw: self.raw }
    }
}

impl Drop for GroupFilter {
    fn drop(&mut self) {
        unsafe { sys::JPC_GroupFilter_Release(self.as_raw()) };
    }
}

impl PartialEq for GroupFilter {
    fn eq(&self, other: &Self) -> bool {
        self.raw == other.raw
    }
}

impl Eq for GroupFilter {}

impl fmt::Debug for GroupFilter {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("GroupFilter").field("ptr", &self.raw).finish()
    }
}

/// Builder for a [`GroupFilter`]. Every pair of different sub groups collides until disabled.
///
/// ```
/// use jolt::GroupFilterTable;
///
/// // A ragdoll with 3 parts where neighbouring parts don't collide.
/// let filter = GroupFilterTable::new(3).disable_collision(0, 1).disable_collision(1, 2).build();
/// assert!(filter.is_collision_enabled(0, 2));
/// assert!(!filter.is_collision_enabled(1, 0));
/// ```
#[derive(Clone, Debug)]
pub struct GroupFilterTable {
    num_sub_groups: u32,
    changes: Vec<(u32, u32, bool)>,
}

impl GroupFilterTable {
    pub fn new(num_sub_groups: u32) -> Self {
        Self { num_sub_groups, changes: Vec::new() }
    }

    pub fn disable_collision(mut self, sub_group1: u32, sub_group2: u32) -> Self {
        self.changes.push((sub_group1, sub_group2, false));
        self
    }

    pub fn enable_collision(mut self, sub_group1: u32, sub_group2: u32) -> Self {
        self.changes.push((sub_group1, sub_group2, true));
        self
    }

    pub fn build(&self) -> GroupFilter {
        crate::ensure_init();
        let raw = unsafe { sys::JPC_GroupFilterTable_Create(self.num_sub_groups) };
        for &(a, b, enabled) in &self.changes {
            unsafe {
                if enabled {
                    sys::JPC_GroupFilterTable_EnableCollision(raw, a, b)
                } else {
                    sys::JPC_GroupFilterTable_DisableCollision(raw, a, b)
                }
            }
        }
        GroupFilter { raw: NonNull::new(raw).expect("JPC_GroupFilterTable_Create returned null") }
    }
}

/// Fine grained collision filtering on top of object layers. Bodies in different groups always collide;
/// within a group, the filter decides per sub group pair, and bodies in the same sub group never collide.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CollisionGroup {
    pub filter: Option<GroupFilter>,
    pub group_id: u32,
    pub sub_group_id: u32,
}

impl CollisionGroup {
    pub const INVALID_ID: u32 = sys::JPC_COLLISION_GROUP_INVALID;

    pub fn new(filter: &GroupFilter, group_id: u32, sub_group_id: u32) -> Self {
        Self { filter: Some(filter.clone()), group_id, sub_group_id }
    }

    pub(crate) fn to_sys(&self) -> sys::JPC_CollisionGroup {
        sys::JPC_CollisionGroup {
            group_filter: self.filter.as_ref().map_or(std::ptr::null(), |f| f.as_raw()),
            group_id: self.group_id,
            sub_group_id: self.sub_group_id,
        }
    }

    /// `raw.group_filter` must be an owned reference.
    pub(crate) fn from_owned_sys(raw: &sys::JPC_CollisionGroup) -> Self {
        Self { filter: GroupFilter::from_owned(raw.group_filter), group_id: raw.group_id, sub_group_id: raw.sub_group_id }
    }
}

impl Default for CollisionGroup {
    fn default() -> Self {
        Self { filter: None, group_id: Self::INVALID_ID, sub_group_id: Self::INVALID_ID }
    }
}
