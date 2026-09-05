#include "simulation/ComboEnumeration.h"

#include <Common/Units.h>

#include <tuple>

namespace simulator {

using namespace common;

namespace {

// Mission boundaries are invalid when any axis is inverted (min > max). An absent block parses to
// a default all-zero MappingBounds (min == max), which is NOT inverted, so absent bounds are valid.
bool missionBoundsInverted(const common::types::MappingBounds& b) {
    return b.min_x > b.max_x || b.min_y > b.max_y || b.min_height > b.max_height;
}

} // namespace

std::vector<ComboSpec> enumerateCombos(const types::SimulationCompositionData& composition) {
    std::vector<ComboSpec> combos;
    for (std::size_t g = 0; g < composition.simulation_mission_groups.size(); ++g) {
        const auto& missions = std::get<1>(composition.simulation_mission_groups[g]);
        for (std::size_t m = 0; m < missions.size(); ++m) {
            const bool bounds_invalid = missionBoundsInverted(missions[m].mission_bounds);
            for (std::size_t d = 0; d < composition.drone_configs.size(); ++d) {
                for (std::size_t l = 0; l < composition.lidar_configs.size(); ++l) {
                    combos.push_back(ComboSpec{g, m, d, l, bounds_invalid});
                }
            }
        }
    }
    return combos;
}

const types::SimulationConfigData&
simConfigOf(const types::SimulationCompositionData& composition, const ComboSpec& combo) {
    return std::get<0>(composition.simulation_mission_groups[combo.group_index]);
}

const common::types::MissionConfigData&
missionConfigOf(const types::SimulationCompositionData& composition, const ComboSpec& combo) {
    return std::get<1>(composition.simulation_mission_groups[combo.group_index])[combo.mission_index];
}

const common::types::DroneConfigData&
droneConfigOf(const types::SimulationCompositionData& composition, const ComboSpec& combo) {
    return composition.drone_configs[combo.drone_index];
}

const common::types::LidarConfigData&
lidarConfigOf(const types::SimulationCompositionData& composition, const ComboSpec& combo) {
    return composition.lidar_configs[combo.lidar_index];
}

types::SimulationResult makeErrorResult(const types::SimulationConfigData& simulation,
                                        const common::types::MissionConfigData& mission,
                                        const std::string& code,
                                        const std::string& message) {
    types::SimulationResult r;
    r.simulation_config = simulation;
    r.mission_config    = mission;
    r.resolution_request_status = types::ResolutionRequestStatus::Ignored;
    r.mission_score = -1.0; // sentinel: this scenario errored and was not scored
    if (!code.empty()) {
        r.mission_results.push_back(common::types::MissionRunResult{
            common::types::MissionRunStatus::Error, 0, {{code, message}}});
    }
    return r;
}

} // namespace simulator
