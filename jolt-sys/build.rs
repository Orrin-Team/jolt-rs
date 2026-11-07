use std::env;
use std::path::PathBuf;

fn main() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let joltc_dir = manifest_dir.join("joltc");
    let jolt_dir = manifest_dir.join("JoltPhysics");

    if !jolt_dir.join("Jolt").join("Jolt.h").exists() {
        panic!(
            "JoltPhysics sources not found at {}. Run `git submodule update --init`.",
            jolt_dir.display()
        );
    }

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=joltc");
    // Picks up submodule bumps that leave joltc untouched
    println!("cargo:rerun-if-changed=JoltPhysics/Jolt");
    println!("cargo:rerun-if-changed=JoltPhysics/Build/CMakeLists.txt");
    println!("cargo:rerun-if-env-changed=JOLT_SYS_UPDATE_BINDINGS");

    build_jolt(&joltc_dir, &jolt_dir);

    #[cfg(feature = "bindgen")]
    generate_bindings(&manifest_dir, &joltc_dir);
}

fn on_off(value: bool) -> &'static str {
    if value { "ON" } else { "OFF" }
}

fn build_jolt(joltc_dir: &std::path::Path, jolt_dir: &std::path::Path) {
    let target_arch = env::var("CARGO_CFG_TARGET_ARCH").unwrap();
    let target_os = env::var("CARGO_CFG_TARGET_OS").unwrap();
    let target_env = env::var("CARGO_CFG_TARGET_ENV").unwrap_or_default();
    let features = env::var("CARGO_CFG_TARGET_FEATURE").unwrap_or_default();
    let has = |feature: &str| features.split(',').any(|f| f == feature);

    let mut config = cmake::Config::new(joltc_dir);
    // Jolt is unusably slow without optimisations, so it is always built in Release.
    config
        .profile("Release")
        .build_target("Jolt")
        .define("JOLT_SOURCE_DIR", jolt_dir)
        .define("CMAKE_POLICY_VERSION_MINIMUM", "3.5")
        .define("JPH_BUILD_SHARED_LIBS", "OFF")
        .define("ENABLE_INSTALL", "OFF")
        .define("ENABLE_ALL_WARNINGS", "OFF")
        .define("GENERATE_DEBUG_SYMBOLS", "OFF")
        // Rust's linker cannot read the LLVM bitcode that IPO would put in the archive.
        .define("INTERPROCEDURAL_OPTIMIZATION", "OFF")
        .define("ENABLE_OBJECT_STREAM", "OFF")
        .define("PROFILER_IN_DEBUG_AND_RELEASE", "OFF")
        .define("DEBUG_RENDERER_IN_DEBUG_AND_RELEASE", "OFF")
        .define("JPH_USE_DX12", "OFF")
        .define("JPH_USE_VK", "OFF")
        .define("JPH_USE_MTL", "OFF")
        .define("JPH_USE_CPU_COMPUTE", "OFF")
        .define("DOUBLE_PRECISION", "OFF")
        .define("OBJECT_LAYER_BITS", "16")
        .define("USE_ASSERTS", on_off(cfg!(feature = "asserts")))
        .define("CROSS_PLATFORM_DETERMINISTIC", on_off(cfg!(feature = "cross-platform-deterministic")))
        // Match the C runtime Rust links against on MSVC.
        .define("USE_STATIC_MSVC_RUNTIME_LIBRARY", on_off(has("crt-static")));

    if target_arch == "x86_64" || target_arch == "x86" {
        // Only use the instruction sets the Rust target allows, so the binary runs wherever Rust code does.
        config
            .define("USE_SSE4_1", on_off(has("sse4.1")))
            .define("USE_SSE4_2", on_off(has("sse4.2")))
            .define("USE_AVX", on_off(has("avx")))
            .define("USE_AVX2", on_off(has("avx2")))
            .define("USE_AVX512", on_off(has("avx512f") && has("avx512vl") && has("avx512dq")))
            .define("USE_LZCNT", on_off(has("lzcnt")))
            .define("USE_TZCNT", on_off(has("bmi1")))
            .define("USE_F16C", on_off(has("f16c")))
            .define("USE_FMADD", on_off(has("fma")));
    }

    let dst = config.build();
    let build_dir = dst.join("build").join("Jolt");
    println!("cargo:rustc-link-search=native={}", build_dir.display());
    println!("cargo:rustc-link-search=native={}", build_dir.join("Release").display());
    println!("cargo:rustc-link-lib=static=Jolt");

    if target_os == "macos" || target_os == "ios" || target_os == "freebsd" {
        println!("cargo:rustc-link-lib=c++");
    } else if target_os == "android" {
        println!("cargo:rustc-link-lib=c++_shared");
    } else if target_env != "msvc" {
        println!("cargo:rustc-link-lib=stdc++");
    }

    println!("cargo:include={}", joltc_dir.join("include").display());
}

#[cfg(feature = "bindgen")]
fn generate_bindings(manifest_dir: &std::path::Path, joltc_dir: &std::path::Path) {
    let header = joltc_dir.join("include").join("joltc.h");
    let bindings = bindgen::Builder::default()
        .header(header.to_string_lossy())
        .allowlist_item("JPC_.*")
        .prepend_enum_name(false)
        .default_enum_style(bindgen::EnumVariation::Consts)
        .derive_default(true)
        .derive_debug(true)
        .derive_copy(true)
        .use_core()
        .rust_edition(bindgen::RustEdition::Edition2024)
        .generate()
        .expect("failed to generate bindings");

    let out = PathBuf::from(env::var("OUT_DIR").unwrap()).join("bindings.rs");
    bindings.write_to_file(&out).expect("failed to write bindings");

    if env::var_os("JOLT_SYS_UPDATE_BINDINGS").is_some() {
        std::fs::copy(&out, manifest_dir.join("src").join("bindings.rs")).expect("failed to update src/bindings.rs");
    }
}
