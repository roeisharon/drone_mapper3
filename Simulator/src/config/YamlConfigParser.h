#pragma once

#include <Common/Types.h>
#include <Common/Units.h>
#include <Simulator/SimulationTypes.h>
#include <UserCommon/ErrorLogger.h>

#include <filesystem>
#include <vector>

namespace simulator::config {

namespace uc = user_common_322526864_323020347;

// Side-band carriers for source .yaml paths. The frozen skeleton data types
// (SimulationCompositionData, MissionConfigData, …) do not record which file each sub-config came
// from, but the report writer must emit those paths in the score reports. The parser hands
// them back through these structs. None duplicate a skeleton type — they hold only what the frozen
// types cannot.
struct CompositionGroupPaths {
    std::filesystem::path              simulation_config;  // the group's simulation_config file
    std::vector<std::filesystem::path> mission_configs;    // aligned with the group's mission list
};

// Source .yaml paths for a whole composition, index-aligned with the returned
// SimulationCompositionData (groups ↔ simulation_mission_groups, drone_configs ↔ drone_configs,
// lidar_configs ↔ lidar_configs).
struct CompositionPaths {
    std::vector<CompositionGroupPaths> groups;
    std::vector<std::filesystem::path> drone_configs;
    std::vector<std::filesystem::path> lidar_configs;
};

// Parsers load a YAML file into a struct. Missing required fields are logged, not thrown.
// Accepts flat documents or a top-level key (e.g., 'drone_config').

// Parses a drone_config YAML file into DroneConfigData.
common::types::DroneConfigData parseDroneConfig(const std::filesystem::path& path,
                                                uc::ErrorLogger& logger);

// Parses a lidar_config YAML file into LidarConfigData.
common::types::LidarConfigData parseLidarConfig(const std::filesystem::path& path,
                                                uc::ErrorLogger& logger);

// Parses a mission_config YAML file into MissionConfigData.
common::types::MissionConfigData parseMissionConfig(const std::filesystem::path& path,
                                                    uc::ErrorLogger& logger);

// Parses a simulation_config YAML file into SimulationConfigData. map_filename resolves relative
// to this file's own directory; when the file does not exist there and fallback_base_dir is given
// (the composition file's directory — the ex3 skeleton inputs spell map paths composition-relative),
// the fallback resolution is used instead. Both layouts are thereby supported.
simulator::types::SimulationConfigData
parseSimulationConfig(const std::filesystem::path& path,
                      uc::ErrorLogger& logger,
                      const std::filesystem::path* fallback_base_dir = nullptr);

// Parses a simulation_compositions YAML into SimulationCompositionData. Extracts paired
// sim/mission configs, plus global drone and lidar configs. Accepts flat or wrapped
// documents. Sub-config paths resolve relative to this file's directory. Skips invalid
// simulation entries (logs error); returns an empty struct if the file is unreadable.
// If out_paths is provided, it captures the index-aligned source YAML paths.
simulator::types::SimulationCompositionData
parseSimulationCompositions(const std::filesystem::path& path,
                            uc::ErrorLogger& logger,
                            CompositionPaths* out_paths = nullptr);

} // namespace simulator::config
