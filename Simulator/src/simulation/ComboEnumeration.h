#pragma once

#include <Simulator/SimulationTypes.h>

#include <cstddef>
#include <string>
#include <vector>

namespace simulator {

// One combination of the composition's cartesian product (groups → missions → drones → lidars),
// referenced by INDEX into the composition so tasks stay cheap and share the parsed data
// read-only. bounds_invalid is precomputed (pure) so no task needs shared validation state.
struct ComboSpec {
    std::size_t group_index = 0;
    std::size_t mission_index = 0;
    std::size_t drone_index = 0;
    std::size_t lidar_index = 0;
    bool bounds_invalid = false;
};

// Enumerates the canonical run order every consumer relies on (reports, per-run file naming,
// same_results grouping). Used by BOTH the sequential SimulationManager and the parallel task
// executor, so the two can never drift apart.
[[nodiscard]] std::vector<ComboSpec>
enumerateCombos(const types::SimulationCompositionData& composition);

// Indexed accessors into the composition for one combo.
[[nodiscard]] const types::SimulationConfigData&
simConfigOf(const types::SimulationCompositionData& composition, const ComboSpec& combo);
[[nodiscard]] const common::types::MissionConfigData&
missionConfigOf(const types::SimulationCompositionData& composition, const ComboSpec& combo);
[[nodiscard]] const common::types::DroneConfigData&
droneConfigOf(const types::SimulationCompositionData& composition, const ComboSpec& combo);
[[nodiscard]] const common::types::LidarConfigData&
lidarConfigOf(const types::SimulationCompositionData& composition, const ComboSpec& combo);

// Builds a failed-scenario SimulationResult: score -1 (the error sentinel) carrying the configs
// and an error code + message so the reports show WHY the scenario failed. Shared by the manager
// and the task executor.
[[nodiscard]] types::SimulationResult
makeErrorResult(const types::SimulationConfigData& simulation,
                const common::types::MissionConfigData& mission,
                const std::string& code,
                const std::string& message);

} // namespace simulator
