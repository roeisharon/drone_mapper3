#pragma once

// gmock doubles of the staff interfaces used by the MissionControl tests (ported from the ex2
// per-interface mock headers into one file). All doubles live in mc_tests so they can never be
// confused with the Simulator's concrete Mock* classes (MockGPS etc.), which are production code.

#include <Common/IDroneMovement.h>
#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IMappingAlgorithm.h>
#include <Common/IMutableMap3D.h>

#include <gmock/gmock.h>

#include <utility>

namespace mc_tests {

// Inject into DroneControl/MissionControl tests to return controlled scan results without running
// real ray-marching geometry.
class MockILidar : public common::ILidar {
public:
    MOCK_METHOD(common::types::LidarScanResult, scan, (common::Orientation), (const, override));
    MOCK_METHOD(common::types::LidarConfigData, config, (), (const, override));
};

// Inject to provide controlled position and heading without a real GPS device.
class MockIGPS : public common::IGPS {
public:
    MOCK_METHOD(common::Position3D, position, (), (const, override));
    MOCK_METHOD(common::Orientation, heading, (), (const, override));
};

// Inject to verify clamping behaviour and to simulate movement failures (return success=false)
// or ex3 physical collisions (Throw std::runtime_error) without touching real GPS state.
class MockIDroneMovement : public common::IDroneMovement {
public:
    MOCK_METHOD(common::types::MovementResult, rotate,
                (common::types::RotationDirection, common::HorizontalAngle), (override));
    MOCK_METHOD(common::types::MovementResult, advance, (common::PhysicalLength), (override));
    MOCK_METHOD(common::types::MovementResult, elevate, (common::PhysicalLength), (override));
};

// The ex3 IMappingAlgorithm base has no default constructor — it binds the shared dependencies
// struct — so the mock forwards one. (The ex2 default-constructible variant with an internal dummy
// map is gone: tests now supply real config objects plus the output-map double.)
class MockIMappingAlgorithm : public common::IMappingAlgorithm {
public:
    explicit MockIMappingAlgorithm(common::MappingAlgorithmDependencies dependencies)
        : common::IMappingAlgorithm(std::move(dependencies)) {}

    MOCK_METHOD(common::types::MappingStepCommand, nextStep,
                (const common::types::DroneState&, const common::types::LidarScanResult*),
                (override));
};

// Inject as the output map to verify set()/save() calls and to script per-position occupancy
// (atVoxel) without writing real .npy files.
class MockIMutableMap3D : public common::IMutableMap3D {
public:
    MOCK_METHOD(common::types::VoxelOccupancy, atVoxel, (const common::Position3D&),
                (const, override));
    MOCK_METHOD(common::types::MapConfig, getMapConfig, (), (const, override));
    MOCK_METHOD(bool, isInBounds, (const common::Position3D&), (const, override));
    MOCK_METHOD(void, set, (const common::Position3D&, common::types::VoxelOccupancy), (override));
    MOCK_METHOD(void, save, (const std::filesystem::path&), (const, override));
};

} // namespace mc_tests
