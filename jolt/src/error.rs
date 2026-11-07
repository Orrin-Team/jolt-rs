use std::ffi::CStr;
use std::fmt;

use jolt_sys as sys;

/// An error reported by Jolt or by argument validation in this crate.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Error {
    message: String,
}

impl Error {
    pub(crate) fn new(message: impl Into<String>) -> Self {
        Self { message: message.into() }
    }

    /// Reads the error the C layer recorded for the current thread.
    pub(crate) fn last() -> Self {
        // SAFETY: JPC_GetLastError always returns a valid, NUL-terminated thread-local string.
        let message = unsafe { CStr::from_ptr(sys::JPC_GetLastError()) };
        Self::new(message.to_string_lossy())
    }

    pub fn message(&self) -> &str {
        &self.message
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.message)
    }
}

impl std::error::Error for Error {}

pub type Result<T, E = Error> = std::result::Result<T, E>;
