cmake_minimum_required(VERSION 3.20 FATAL_ERROR)

project(joltc VERSION 0.1.0 LANGUAGES C CXX)

set(JOLT_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../JoltPhysics" CACHE PATH "Path to the JoltPhysics repository")
option(JOLTC_BUILD_EXAMPLES "Build the C examples" OFF)

# Jolt sets its compiler flags (SIMD, RTTI, exceptions) at directory scope, so the wrapper is compiled
# as part of the Jolt target to guarantee both see identical flags and definitions.
add_subdirectory("${JOLT_SOURCE_DIR}/Build" "${CMAKE_CURRENT_BINARY_DIR}/Jolt")
target_sources(Jolt PRIVATE
	"${CMAKE_CURRENT_SOURCE_DIR}/src/joltc.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/src/joltc_character.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/src/joltc_internal.h")
target_include_directories(Jolt PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

if (JOLTC_BUILD_EXAMPLES)
	add_executable(joltc_hello_world examples/hello_world.c)
	target_link_libraries(joltc_hello_world PRIVATE Jolt)
endif()
