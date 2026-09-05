#pragma once

#include <Common/IDroneMovement.h>
#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IMap3D.h>
#include <Common/IMappingAlgorithm.h>
#include <Common/IMissionControl.h>
#include <Common/IMutableMap3D.h>
#include <Simulator/ISimulationRun.h>

#include <filesystem>
#include <memory>

namespace simulator {

// One simulation run = one fully-isolated object graph (maps, mocks, one algorithm instance, one
// mission-control instance — the latter two created fresh from the plugin factories). Owning
// everything per-run is what makes runs independent, which threading (Phase 5) relies on.
//
// Member order is a lifetime contract: mission_control_ is declared last so it is destroyed FIRST
// (it references the algorithm, mocks and maps); mapping_algorithm_ second (it references the
// output map). Never reorder these members.
class SimulationRunImpl final : public ISimulationRun {
public:
    // owned_hidden_map may be NULL: in shared-cache mode (threading) the hidden map is owned by
    // the driver's HiddenMapCache, which outlives every run; hidden_map is the always-valid view
    // either way.
    SimulationRunImpl(std::unique_ptr<const common::IMap3D> owned_hidden_map,
                      const common::IMap3D& hidden_map,
                      std::unique_ptr<common::IMutableMap3D> output_map,
                      std::unique_ptr<common::IGPS> gps,
                      std::unique_ptr<common::IDroneMovement> movement,
                      std::unique_ptr<common::ILidar> lidar,
                      std::unique_ptr<common::IMappingAlgorithm> mapping_algorithm,
                      std::unique_ptr<common::IMissionControl> mission_control,
                      types::SimulationConfigData simulation_config,
                      common::types::MissionConfigData mission_config,
                      std::filesystem::path output_map_file);

    [[nodiscard]] types::SimulationResult run() override;

private:
    std::unique_ptr<const common::IMap3D> owned_hidden_map_; // may be null (see ctor)
    const common::IMap3D& hidden_map_;
    std::unique_ptr<common::IMutableMap3D> output_map_;
    std::unique_ptr<common::IGPS> gps_;
    std::unique_ptr<common::IDroneMovement> movement_;
    std::unique_ptr<common::ILidar> lidar_;
    std::unique_ptr<common::IMappingAlgorithm> mapping_algorithm_;
    std::unique_ptr<common::IMissionControl> mission_control_;
    // Retained so run() can return the configs and output path in SimulationResult.
    types::SimulationConfigData simulation_config_;
    common::types::MissionConfigData mission_config_;
    std::filesystem::path output_map_file_;
};

} // namespace simulator
