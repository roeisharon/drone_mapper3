# Builds the simulator executable simulator_322526864_323020347.

cmake_minimum_required(VERSION 3.15)

# Standalone-build guard: when this project is configured on its own (the spec requires each
# project to be buildable separately), replicate the root superbuild's setup. Under the root
# CMakeLists this whole block is skipped.
if(NOT DEFINED PROJECT_NAME)
    set(VCPKG_MANIFEST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)  # vcpkg.json lives at the repo root
    project(simulator_322526864_323020347 LANGUAGES CXX)
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

# Simulator-only third-party deps (the root superbuild only finds mp-units).
if(NOT TARGET yaml-cpp::yaml-cpp)
    find_package(yaml-cpp CONFIG REQUIRED)
endif()
if(NOT TARGET TinyNPY::TinyNPY)
    # Canonical package-name case matters: the config file is TinyNPYConfig.cmake, and a lowercase
    # find_package(tinynpy) only ever worked here because the dev bind-mount is case-insensitive —
    # on a real (case-sensitive) Linux filesystem it fails.
    find_package(TinyNPY CONFIG REQUIRED)
endif()
if(NOT TARGET Threads::Threads)
    find_package(Threads REQUIRED)
endif()

# All Simulator sources except main.cpp, compiled once and shared by the executable and the test
# binary (OBJECT library: no extra artifact in the required build, no double compilation).
add_library(simulator_core OBJECT
    src/args/ArgsParser.cpp
    src/config/YamlConfigParser.cpp
    src/exec/TaskExecutor.cpp
    src/map/HiddenMapCache.cpp
    src/map/Map3DImpl.cpp
    src/mocks/MockGPS.cpp
    src/mocks/MockLidar.cpp
    src/mocks/MockMovement.cpp
    src/modes/ModeRunner.cpp
    src/plugins/PluginManager.cpp
    src/registration/Registration.cpp
    src/report/ComparativeReportWriter.cpp
    src/report/CompetitiveReportWriter.cpp
    src/report/MapsComparison.cpp
    src/report/OutputPaths.cpp
    src/report/ResultAggregation.cpp
    src/report/SimulationReportWriter.cpp
    src/simulation/ComboEnumeration.cpp
    src/simulation/SimulationManager.cpp
    src/simulation/SimulationRunFactoryImpl.cpp
    src/simulation/SimulationRunImpl.cpp
    # UserCommon has no makefile (spec); its sources are compiled by each consumer.
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/src/ErrorLogger.cpp
)
target_include_directories(simulator_core PUBLIC
    src
    common_simulator/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/include
)
target_link_libraries(simulator_core PUBLIC
    common::common
    yaml-cpp::yaml-cpp
    TinyNPY::TinyNPY
    Threads::Threads
)
drone_warnings(simulator_core)

add_executable(simulator_322526864_323020347 src/main.cpp)
target_link_libraries(simulator_322526864_323020347 PRIVATE simulator_core ${CMAKE_DL_LIBS})
# Export the registration-constructor symbols (defined in Registration.cpp) so that
# dlopen'd plugin .so files resolve them against this executable at load time.
set_target_properties(simulator_322526864_323020347 PROPERTIES ENABLE_EXPORTS ON)
drone_warnings(simulator_322526864_323020347)

if(DRONE_BUILD_TESTS)
    find_package(GTest CONFIG REQUIRED)

    # Fixture plugins exercising the loader's count-delta protocol against real .so files.
    add_library(test_algo_ok SHARED tests/plugins/test_algo_ok.cpp)
    add_library(test_algo_none SHARED tests/plugins/test_algo_none.cpp)
    add_library(test_algo_double SHARED tests/plugins/test_algo_double.cpp)
    add_library(test_algo_ctor_throws SHARED tests/plugins/test_algo_ctor_throws.cpp)
    add_library(test_mc_ok SHARED tests/plugins/test_mc_ok.cpp)
    add_library(test_mc_error SHARED tests/plugins/test_mc_error.cpp)
    add_library(test_mc_ctor_throws SHARED tests/plugins/test_mc_ctor_throws.cpp)
    add_library(test_mc_lifecycle SHARED tests/plugins/test_mc_lifecycle.cpp)
    foreach(fixture test_algo_ok test_algo_none test_algo_double test_algo_ctor_throws test_mc_ok
            test_mc_error test_mc_ctor_throws test_mc_lifecycle)
        set_target_properties(${fixture} PROPERTIES PREFIX "")
        target_link_libraries(${fixture} PRIVATE common::common)
        drone_warnings(${fixture})
    endforeach()

    add_executable(simulator_tests
        tests/components/args_parser_test.cpp
        tests/components/map3d_test.cpp
        tests/components/maps_comparison_test.cpp
        tests/components/output_paths_test.cpp
        tests/components/mock_gps_movement_test.cpp
        tests/components/mock_lidar_test.cpp
        tests/components/plugin_manager_test.cpp
        tests/components/result_aggregation_test.cpp
        tests/components/simulation_manager_test.cpp
        tests/components/simulation_run_test.cpp
        tests/components/task_executor_test.cpp
        tests/integration/integration_test.cpp
    )
    target_include_directories(simulator_tests PRIVATE tests)
    target_link_libraries(simulator_tests PRIVATE
        simulator_core
        ${CMAKE_DL_LIBS}
        GTest::gtest
        GTest::gtest_main
        GTest::gmock
    )
    # The test binary dlopens fixture plugins, so it must export the registration ctors too.
    set_target_properties(simulator_tests PROPERTIES ENABLE_EXPORTS ON)
    drone_warnings(simulator_tests)
    add_dependencies(simulator_tests test_algo_ok test_algo_none test_algo_double
        test_algo_ctor_throws test_mc_ok test_mc_error test_mc_ctor_throws test_mc_lifecycle)

    # Hermetic fixture locations — tests never depend on the current working directory.
    target_compile_definitions(simulator_tests PRIVATE
        DRONE_TEST_INPUTS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../inputs"
        DRONE_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data"
        DRONE_TEST_PLUGIN_DIR="$<TARGET_FILE_DIR:test_algo_ok>"
    )
    # Real-plugin end-to-end cases run only under the root superbuild, where the plugin targets
    # exist (a standalone Simulator+tests build simply compiles them out).
    if(TARGET Algorithm_322526864_323020347 AND TARGET MissionControl_322526864_323020347)
        target_compile_definitions(simulator_tests PRIVATE
            DRONE_TEST_REAL_ALGO_SO="$<TARGET_FILE:Algorithm_322526864_323020347>"
            DRONE_TEST_REAL_MC_SO="$<TARGET_FILE:MissionControl_322526864_323020347>"
        )
        add_dependencies(simulator_tests
            Algorithm_322526864_323020347 MissionControl_322526864_323020347)
    endif()
endif()
