//! Raw bindings to [Jolt Physics](https://github.com/jrouwe/JoltPhysics) through the `joltc` C API.
//!
//! The C header lives in `joltc/include/joltc.h` and documents ownership and threading rules.
//! For a safe interface use the `jolt` crate.

#![no_std]
#![allow(non_camel_case_types, non_snake_case, non_upper_case_globals)]

#[cfg(feature = "bindgen")]
include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

#[cfg(not(feature = "bindgen"))]
include!("bindings.rs");
