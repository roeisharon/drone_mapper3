# Builds the mapping-algorithm plugin Algorithm_322526864_323020347.so.

cmake_minimum_required(VERSION 3.15)

# Standalone-build guard: when this project is configured on its own (the spec requires each
# project to be buildable separately), replicate the root superbuild's setup. Under the root
# CMakeLists this whole block is skipped.
if(NOT DEFINED PROJECT_NAME)
    set(VCPKG_MANIFEST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)  # vcpkg.json lives at the repo root
    project(algorithm_plugin_322526864_323020347 LANGUAGES CXX)
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    find_package(mp-units CONFIG REQUIRED)
endif()

if(NOT COMMAND drone_warnings)
    function(drone_warnings target)
        target_compile_options(${target} PRIVATE -Wall -Wextra -Werror -pedantic)
    endfunction()
endif()

if(NOT TARGET drone_common)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../common ${CMAKE_CURRENT_BINARY_DIR}/common)
endif()

add_library(Algorithm_322526864_323020347 SHARED
    src/MappingAlgorithmImpl.cpp
)
# The loadable file must be named exactly Algorithm_<ids>.so — strip the default "lib" prefix.
set_target_properties(Algorithm_322526864_323020347 PROPERTIES PREFIX "")
target_include_directories(Algorithm_322526864_323020347 PRIVATE
    include
    # UserCommon is header-only for this project (DroneFootprint templates) — include dir only.
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/include
)
target_link_libraries(Algorithm_322526864_323020347 PRIVATE common::common)
drone_warnings(Algorithm_322526864_323020347)

if(DRONE_BUILD_TESTS)
    find_package(GTest CONFIG REQUIRED)
    add_executable(algorithm_tests
        tests/mapping_algorithm_test.cpp
        tests/termination_test.cpp
        # The implementation under test is compiled directly into the test binary; its REGISTER_*
        # global needs the registration ctor, provided as a no-op by the support file.
        tests/support/noop_registration.cpp
        src/MappingAlgorithmImpl.cpp
    )
    target_include_directories(algorithm_tests PRIVATE
        include
        tests
        ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/include
    )
    target_link_libraries(algorithm_tests PRIVATE
        common::common GTest::gtest GTest::gtest_main GTest::gmock)
    drone_warnings(algorithm_tests)
endif()
