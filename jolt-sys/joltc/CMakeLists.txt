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
	"${CMAKE_CURRENT_SOURCE_DIR}/src/joltc_vehicle.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/src/joltc_ragdoll.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/src/joltc_shape.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/src/joltc_animation.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/src/joltc_softbody.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/src/joltc_internal.h")
target_include_directories(Jolt PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

# Jolt 5.6 builds with /fp:fast on MSVC, under which MSVC Build Tools 14.52 replace -0.0f with 0.0f and break quaternion
# math (fixed upstream in Jolt 5.7, jrouwe/JoltPhysics#2222). /fp:precise keeps signed zeros; target options come after
# Jolt's global flags on the command line, so they win.
if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
	target_compile_options(Jolt PRIVATE /fp:precise)
endif()

if (JOLTC_BUILD_EXAMPLES)
	add_executable(joltc_hello_world examples/hello_world.c)
	target_link_libraries(joltc_hello_world PRIVATE Jolt)
endif()
