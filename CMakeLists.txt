cmake_minimum_required(VERSION 3.20)
project(motion_tennis_physics VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(MOTION_TENNIS_BUILD_TESTS "Build the S1 fixture tests" ON)

# Shared warnings and floating-point settings. Contraction into fused
# multiply-add is disabled so replays do not depend on compiler choices.
add_library(motion_tennis_physics_options INTERFACE)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(motion_tennis_physics_options INTERFACE
        -Wall -Wextra -Wpedantic -Wconversion -Werror -ffp-contract=off)
elseif(MSVC)
    target_compile_options(motion_tennis_physics_options INTERFACE /W4 /WX /fp:strict)
endif()

add_library(motion_tennis_physics_core
    src/simulation.cpp
    src/input_scheduler.cpp
    src/batch_core.cpp)
target_include_directories(motion_tennis_physics_core PUBLIC include)
target_link_libraries(motion_tennis_physics_core PRIVATE motion_tennis_physics_options)

# Test doubles for the S1 slice; never linked into a runtime target.
add_library(motion_tennis_physics_fakes src/fakes.cpp)
target_link_libraries(motion_tennis_physics_fakes
    PUBLIC motion_tennis_physics_core
    PRIVATE motion_tennis_physics_options)

if(MOTION_TENNIS_BUILD_TESTS)
    enable_testing()
    add_executable(physics_s1_tests tests/s1_batch_core_tests.cpp)
    target_link_libraries(physics_s1_tests
        PRIVATE motion_tennis_physics_fakes motion_tennis_physics_options)
    add_test(NAME physics_s1_tests COMMAND physics_s1_tests)
endif()
