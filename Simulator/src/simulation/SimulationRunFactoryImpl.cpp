#include "simulation/SimulationRunFactoryImpl.h"

#include "map/Map3DImpl.h"
#include "mocks/MockGPS.h"
#include "mocks/MockLidar.h"
#include "mocks/MockMovement.h"
#include "simulation/ResolutionRequest.h"
#include "simulation/SimulationRunImpl.h"

#include <TinyNPY.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace simulator {

using namespace common;

namespace {

// Loads a 3-D NPY file. Throws std::runtime_error carrying the tinynpy error string when the file
// is missing or malformed.
NpyArray loadNpyArray(const std::filesystem::path& path) {
    NpyArray map;
    const std::string path_string = path.string();
    const char* error = map.LoadNPY(path_string.c_str());
    if (error != nullptr) {
        throw std::runtime_error("Failed to load NPY file '" + path_string + "': " + error);
    }
    return map;
}

// True when the mission supplied real bounds (not the all-zero default produced when the mission
// YAML has no `boundaries` block). An all-zero MappingBounds means "no restriction — map everything".
bool missionBoundsSpecified(const common::types::MappingBounds& b) {
    const auto z_x = 0.0 * x_extent[cm];
    const auto z_y = 0.0 * y_extent[cm];
    const auto z_h = 0.0 * z_extent[cm];
    return !(b.min_x == z_x && b.max_x == z_x &&
             b.min_y == z_y && b.max_y == z_y &&
             b.min_height == z_h && b.max_height == z_h);
}

// Per-axis intersection of two boundary boxes.
common::types::MappingBounds intersectBounds(const common::types::MappingBounds& a,
                                             const common::types::MappingBounds& b) {
    return common::types::MappingBounds{
        std::max(a.min_x, b.min_x),           std::min(a.max_x, b.max_x),
        std::max(a.min_y, b.min_y),           std::min(a.max_y, b.max_y),
        std::max(a.min_height, b.min_height), std::min(a.max_height, b.max_height),
    };
}

} // namespace

SimulationRunFactoryImpl::SimulationRunFactoryImpl(
    const common::MappingAlgorithmFactory& algorithm_factory,
    const common::MissionControlFactory& mission_control_factory,
    std::string run_file_prefix,
    bool verbose,
    std::size_t first_run_index,
    const common::IMap3D* preloaded_hidden_map)
    : algorithm_factory_(algorithm_factory),
      mission_control_factory_(mission_control_factory),
      run_file_prefix_(std::move(run_file_prefix)),
      verbose_(verbose),
      next_run_index_(first_run_index),
      preloaded_hidden_map_(preloaded_hidden_map) {}

common::types::MapConfig SimulationRunFactoryImpl::outputMapConfig(
    const common::types::MapConfig& hidden_config,
    const common::types::MappingBounds& mission_bounds) {
    // No mission bounds → output map spans the full hidden extent.
    if (!missionBoundsSpecified(mission_bounds)) {
        return hidden_config;
    }
    // Narrow the output map to (hidden ∩ mission). The offset places array index 0 at the lower
    // corner of the intersection, so atVoxel/set/isInBounds (and therefore the planner and scans)
    // treat everything outside the bounded region as OutOfBounds.
    const common::types::MappingBounds isect = intersectBounds(hidden_config.boundaries, mission_bounds);
    common::types::MapConfig cfg = hidden_config;
    cfg.boundaries = isect;
    cfg.offset = Position3D{-isect.min_x, -isect.min_y, -isect.min_height};
    return cfg;
}

Position3D SimulationRunFactoryImpl::snapToCellCenter(const Position3D& position,
                                                      const common::types::MapConfig& config) {
    const double res = config.resolution.numerical_value_in(cm);
    if (res <= 0.0) {
        return position; // degenerate config: nothing to snap to
    }
    const double ox = config.offset.x.numerical_value_in(cm);
    const double oy = config.offset.y.numerical_value_in(cm);
    const double oz = config.offset.z.numerical_value_in(cm);
    // Cell index = floor((world + offset) / res); the cell centre is (index + 0.5) * res - offset.
    const auto centre = [res](double world, double off) {
        const double index = std::floor((world + off) / res);
        return (index + 0.5) * res - off;
    };
    return Position3D{
        centre(position.x.numerical_value_in(cm), ox) * x_extent[cm],
        centre(position.y.numerical_value_in(cm), oy) * y_extent[cm],
        centre(position.z.numerical_value_in(cm), oz) * z_extent[cm],
    };
}

std::unique_ptr<ISimulationRun>
SimulationRunFactoryImpl::create(const types::SimulationConfigData& simulation_config,
                                 const common::types::MissionConfigData& mission_config,
                                 const common::types::DroneConfigData& drone_config,
                                 const common::types::LidarConfigData& lidar_config,
                                 const std::filesystem::path& output_path) {
    // Hidden map: the shared read-only cache entry when one was injected (the threaded path —
    // loaded once by the driver, shared across every concurrent run of the group), otherwise
    // loaded from the NPY named in the simulation config. MappingBounds is left default because
    // Map3DImpl derives the real spatial extent from the NPY shape plus offset+resolution.
    std::unique_ptr<const Map3DImpl> owned_hidden_map;
    const common::IMap3D* hidden_map = preloaded_hidden_map_;
    if (hidden_map == nullptr) {
        const common::types::MapConfig hidden_map_config{
            common::types::MappingBounds{},
            simulation_config.map_offset,
            simulation_config.map_resolution,
        };
        const NpyArray npy = loadNpyArray(simulation_config.map_filename);
        owned_hidden_map = std::make_unique<const Map3DImpl>(npy, hidden_map_config);
        hidden_map = owned_hidden_map.get();
    }

    // Output map: sized from the hidden map's RESOLVED config (its boundaries/offset come from
    // the NPY shape, not the zeroed placeholder above), narrowed by mission bounds. Phase 8b:
    // an ACCEPTED output_mapping_resolution_factor scales the output map's resolution — the
    // whole downstream stack (planner grid, cell-aligned moves, scan writes, comparison) reads
    // the resolution from this one config, so the coarser mission needs no other change.
    const common::types::MapConfig hidden_resolved = hidden_map->getMapConfig();
    common::types::MapConfig output_map_config =
        outputMapConfig(hidden_resolved, mission_config.mission_bounds);
    const double res_factor =
        acceptedResolutionFactor(mission_config.output_mapping_resolution_factor);
    if (res_factor > 1.0) {
        output_map_config.resolution = res_factor * output_map_config.resolution;
    }
    auto output_map = std::make_unique<Map3DImpl>(output_map_config);

    // GPS starts at the configured position SNAPPED to its output-grid cell centre, so the drone
    // begins exactly on the planner grid; altitude angle is 0 (drone level).
    auto gps = std::make_unique<MockGPS>(
        snapToCellCenter(simulation_config.initial_drone_position, output_map_config),
        Orientation{simulation_config.initial_angle, 0.0 * altitude_angle[deg]},
        mission_config.gps_resolution);
    // MockMovement validates moves against the hidden ground-truth map using the drone's radius, so
    // a drone that does not fit physically cannot move into the space.
    auto movement = std::make_unique<MockMovement>(*gps, *hidden_map, drone_config.radius);
    // MockLidar ray-marches against the hidden map using the live GPS pose.
    auto lidar = std::make_unique<MockLidar>(lidar_config, *hidden_map, *gps);

    // Plugin instances: created FRESH from the registered factories for every run (spec: factories
    // are cheap, never cache instances). The algorithm plans off the output map only — it never
    // sees the hidden map.
    auto mapping_algorithm = algorithm_factory_(common::MappingAlgorithmDependencies{
        .mission_config = mission_config,
        .lidar_config = lidar_config,
        .drone_config = drone_config,
        .output_map = *output_map,
    });
    if (!mapping_algorithm) {
        throw std::runtime_error("algorithm factory returned null");
    }
    ++algorithm_instances_created_;

    // Output file naming: <prefix>_run<N>_output.npy, where <prefix> is the component's .so stem
    // and N counts this factory's create() calls (canonical run order for the pairing).
    const auto run_idx = next_run_index_++;
    const std::string stem =
        run_file_prefix_ + "_run" + std::to_string(run_idx) + "_output";
    const std::filesystem::path output_map_file = output_path / (stem + ".npy");

    auto mission_control = mission_control_factory_(common::MissionControlDependencies{
        .mission_config = mission_config,
        .drone_config = drone_config,
        .lidar = *lidar,
        .gps = *gps,
        .movement = *movement,
        .output_map = *output_map,
        .mapping_algorithm = *mapping_algorithm,
        .output_map_file = output_map_file,
        .verbose = verbose_,
    });
    if (!mission_control) {
        throw std::runtime_error("mission control factory returned null");
    }
    ++mission_control_instances_created_;

    return std::make_unique<SimulationRunImpl>(
        std::move(owned_hidden_map),
        *hidden_map,
        std::move(output_map),
        std::move(gps),
        std::move(movement),
        std::move(lidar),
        std::move(mapping_algorithm),
        std::move(mission_control),
        simulation_config,
        mission_config,
        output_map_file);
}

} // namespace simulator
