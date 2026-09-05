#pragma once

// gmock doubles of the staff interfaces used by the Simulator's component tests.

#include <Common/IMappingAlgorithm.h>
#include <Common/IMissionControl.h>
#include <Common/IMutableMap3D.h>
#include <Simulator/ISimulationRun.h>
#include <Simulator/ISimulationRunFactory.h>

#include <gmock/gmock.h>

#include <utility>

namespace sim_tests {

class MockIMappingAlgorithm : public common::IMappingAlgorithm {
public:
    explicit MockIMappingAlgorithm(common::MappingAlgorithmDependencies dependencies)
        : common::IMappingAlgorithm(std::move(dependencies)) {}

    MOCK_METHOD(common::types::MappingStepCommand, nextStep,
                (const common::types::DroneState&, const common::types::LidarScanResult*),
                (override));
};

class MockIMissionControl : public common::IMissionControl {
public:
    MOCK_METHOD(common::types::MissionRunResult, runMission, (), (override));
};

class MockIMutableMap3D : public common::IMutableMap3D {
public:
    MOCK_METHOD(common::types::VoxelOccupancy, atVoxel, (const common::Position3D&),
                (const, override));
    MOCK_METHOD(common::types::MapConfig, getMapConfig, (), (const, override));
    MOCK_METHOD(bool, isInBounds, (const common::Position3D&), (const, override));
    MOCK_METHOD(void, set, (const common::Position3D&, common::types::VoxelOccupancy), (override));
    MOCK_METHOD(void, save, (const std::filesystem::path&), (const, override));
};

class MockISimulationRun : public simulator::ISimulationRun {
public:
    MOCK_METHOD(simulator::types::SimulationResult, run, (), (override));
};

class MockISimulationRunFactory : public simulator::ISimulationRunFactory {
public:
    MOCK_METHOD(std::unique_ptr<simulator::ISimulationRun>, create,
                (const simulator::types::SimulationConfigData&,
                 const common::types::MissionConfigData&,
                 const common::types::DroneConfigData&,
                 const common::types::LidarConfigData&,
                 const std::filesystem::path&),
                (override));
};

} // namespace sim_tests
