#!/bin/sh
# Runs the integration tests against an AddressSanitizer build of Jolt and joltc (macOS/clang).
# Doctests are skipped because rustdoc does not get the sanitizer runtime linked in.
set -eu
cd "$(dirname "$0")/.."
ASAN=$(clang -print-file-name=libclang_rt.asan_osx_dynamic.dylib)
CXXFLAGS="-fsanitize=address -fno-omit-frame-pointer" \
RUSTFLAGS="-C link-arg=$ASAN -C link-arg=-Wl,-rpath,$(dirname "$ASAN")" \
	cargo test -p orrin-jolt --tests --target-dir target/asan "$@"
