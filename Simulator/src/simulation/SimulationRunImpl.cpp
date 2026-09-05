#include "simulation/SimulationRunImpl.h"

#include "report/MapsComparison.h"
#include "simulation/ResolutionRequest.h"

#include <Common/Units.h>
#include <UserCommon/ErrorLogger.h>

#include <exception>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace simulator {

using namespace common;
namespace uc = user_common_322526864_323020347;

namespace {

// True when the mission supplied real bounds (not the all-zero default = no restriction).
bool missionBoundsSpecified(const common::types::MappingBounds& b) {
    const auto zx = 0.0 * x_extent[cm];
    const auto zy = 0.0 * y_extent[cm];
    const auto zh = 0.0 * z_extent[cm];
    return !(b.min_x == zx && b.max_x == zx &&
             b.min_y == zy && b.max_y == zy &&
             b.min_height == zh && b.max_height == zh);
}

// World position inside the half-open box [min, max) on every axis.
bool withinBounds(const Position3D& p, const common::types::MappingBounds& b) {
    return p.x >= b.min_x && p.x < b.max_x &&
           p.y >= b.min_y && p.y < b.max_y &&
           p.z >= b.min_height && p.z < b.max_height;
}

// Validates the drone's start position; returns an error code (or nullopt when valid). Order matters:
// the map check gates the others (atVoxel is only meaningful in-bounds), and the mission-bounds check
// precedes the obstacle check so a start far outside the region reports the region error, not an
// incidental obstacle hit.
std::optional<std::string> startPositionError(const IMap3D& hidden_map,
                                              const common::types::MappingBounds& mission_bounds,
                                              const Position3D& start) {
    if (!hidden_map.isInBounds(start)) {
        return "START_OUTSIDE_MAP";
    }
    if (missionBoundsSpecified(mission_bounds) && !withinBounds(start, mission_bounds)) {
        return "START_OUTSIDE_MISSION_BOUNDS";
    }
    if (hidden_map.atVoxel(start) == common::types::VoxelOccupancy::Occupied) {
        return "START_IN_OBSTACLE";
    }
    return std::nullopt;
}

} // namespace

SimulationRunImpl::SimulationRunImpl(std::unique_ptr<const IMap3D> owned_hidden_map,
                                     const IMap3D& hidden_map,
                                     std::unique_ptr<IMutableMap3D> output_map,
                                     std::unique_ptr<IGPS> gps,
                                     std::unique_ptr<IDroneMovement> movement,
                                     std::unique_ptr<ILidar> lidar,
                                     std::unique_ptr<IMappingAlgorithm> mapping_algorithm,
                                     std::unique_ptr<IMissionControl> mission_control,
                                     types::SimulationConfigData simulation_config,
                                     common::types::MissionConfigData mission_config,
                                     std::filesystem::path output_map_file)
    : owned_hidden_map_(std::move(owned_hidden_map)),
      hidden_map_(hidden_map),
      output_map_(std::move(output_map)),
      gps_(std::move(gps)),
      movement_(std::move(movement)),
      lidar_(std::move(lidar)),
      mapping_algorithm_(std::move(mapping_algorithm)),
      mission_control_(std::move(mission_control)),
      simulation_config_(std::move(simulation_config)),
      mission_config_(std::move(mission_config)),
      output_map_file_(std::move(output_map_file)) {
    // owned_hidden_map_ is legitimately null in shared-cache mode; hidden_map_ is a reference and
    // therefore always valid.
    if (!output_map_ ||
        !gps_ ||
        !movement_ ||
        !lidar_ ||
        !mapping_algorithm_ ||
        !mission_control_) {
        throw std::invalid_argument("SimulationRunImpl requires injected dependencies.");
    }
}

// Drives one simulation run end to end:
//   1. validate the start position (a bad start fails only this run, with a clear code),
//   2. runMission() (its exceptions isolated so a mission failure still yields a scored result),
//   3. resolution-request status (Phase 1: always Accepted; real classification lands in Phase 2),
//   4. score the output map against the hidden map (skipped on mission Error → sentinel -1),
//   5. assemble and return the SimulationResult.
// The error log lives beside the output map (same stem, .log extension) so a human finds both in
// the same results directory. ErrorLogger creates the file lazily — nothing is written unless an
// error is actually logged.
types::SimulationResult SimulationRunImpl::run() {
    const std::filesystem::path error_log_path =
        output_map_file_.parent_path() / (output_map_file_.stem().string() + ".log");
    uc::ErrorLogger logger{error_log_path};

    // Resolution-request classification applies to the run regardless of how it ends — it
    // describes how the REQUEST was handled, not the mission outcome.
    const types::ResolutionRequestStatus res_status =
        classifyResolutionRequest(mission_config_.output_mapping_resolution_factor);
    if (res_status == types::ResolutionRequestStatus::IgnoredTooSmall) {
        try {
            logger.log("RESOLUTION_IGNORED_TOO_SMALL",
                       "output_mapping_resolution_factor < 1 is not supported; using the native "
                       "map resolution");
        } catch (...) {}
    } else if (res_status == types::ResolutionRequestStatus::Ignored) {
        try {
            logger.log("RESOLUTION_IGNORED",
                       "non-integer output_mapping_resolution_factor cannot tile the native "
                       "grid; using the native map resolution");
        } catch (...) {}
    }

    if (const auto start_error =
            startPositionError(hidden_map_, mission_config_.mission_bounds,
                               simulation_config_.initial_drone_position)) {
        try { logger.log(*start_error, "invalid start position"); } catch (...) {}
        return types::SimulationResult{
            simulation_config_,
            mission_config_,
            res_status,
            {common::types::MissionRunResult{
                common::types::MissionRunStatus::Error, 0, {{*start_error, "invalid start position"}}}},
            output_map_file_,
            output_map_->getMapConfig(),
            -1.0
        };
    }

    try {
        common::types::MissionRunResult mission_result;
        try {
            mission_result = mission_control_->runMission();
        } catch (const std::exception& e) {
            // A mission failure is logged at the point it occurs and converted into an error-scored
            // result rather than propagating and leaving the run half-assembled.
            try { logger.log("MISSION_EXCEPTION", e.what()); } catch (...) {}
            return types::SimulationResult{
                simulation_config_,
                mission_config_,
                res_status,
                {common::types::MissionRunResult{
                    common::types::MissionRunStatus::Error, 0,
                    {{"MISSION_EXCEPTION", e.what()}}}},
                output_map_file_,
                output_map_->getMapConfig(),
                -1.0
            };
        }

        // Log every drone-level error accumulated during the step loop (the mission still reached a
        // terminal status, so these are non-fatal and do not abort scoring).
        for (const auto& err : mission_result.errors) {
            logger.log(err.code, err.message);
        }

        // A mission Error means the output map may be in an unknown state — skip comparison and
        // return the sentinel -1. Completed and MaxSteps both stopped cleanly, so the map is worth
        // scoring even if mapping was incomplete.
        double score = -1.0;
        if (mission_result.status != common::types::MissionRunStatus::Error) {
            const auto scores = MapsComparison::compare(hidden_map_, {output_map_.get()});
            score = scores.empty() ? -1.0 : scores[0];
        }

        return types::SimulationResult{
            simulation_config_,
            mission_config_,
            res_status,
            {mission_result},
            output_map_file_,
            output_map_->getMapConfig(),
            score
        };

    } catch (const std::exception&) {
        // Backstop for unexpected exceptions from logger I/O or post-mission code: return a minimal
        // error result (score -1) rather than letting the process crash.
        return types::SimulationResult{
            simulation_config_,
            mission_config_,
            res_status,
            {},
            output_map_file_,
            {},
            -1.0
        };
    }
}

} // namespace simulator
