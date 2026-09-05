#include "simulation/SimulationManager.h"

#include "report/OutputPaths.h"
#include "simulation/ComboEnumeration.h"

#include <Common/Units.h>
#include <UserCommon/ErrorLogger.h>

#include <exception>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace simulator {

using namespace common;
namespace uc = user_common_322526864_323020347;

SimulationManager::SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory,
                                     std::filesystem::path manager_log_file)
    : run_factory_(std::move(run_factory)), manager_log_file_(std::move(manager_log_file)) {
    if (!run_factory_) {
        throw std::invalid_argument("SimulationManager requires a run factory.");
    }
}

types::SimulationManagerReport SimulationManager::run(
    const types::SimulationCompositionData& composition,
    const std::filesystem::path& output_path) {
    std::filesystem::create_directories(output_path); // idempotent

    const std::string utc = utcTimestamp();
    std::vector<types::SimulationResult> runs;

    // Factory/boundary failures are logged immediately; the file is created lazily on first log().
    uc::ErrorLogger logger{manager_log_file_};

    // The canonical run order is produced by the shared enumeration (also used by the parallel
    // task executor, so the sequential and threaded paths can never drift apart).
    // Failure handling:
    //  - Single scenario failure: scores -1 with an error code; the loop continues.
    //  - Group failure (e.g. map load fails): fails the entire group — remaining combinations get
    //    -1 with the same error, without calling the factory. Other groups proceed.
    //  - Invalid mission bounds: fails all combinations for that mission with
    //    MISSION_BOUNDARY_INVALID (-1) without calling the factory.
    bool        group_failed = false;
    std::size_t failed_group = 0;
    std::string group_code;    // error code shared by every auto-filled scenario in a dead group
    std::string group_message; // human-readable cause (e.g. the map-load error text)
    std::size_t last_logged_bounds_combo = static_cast<std::size_t>(-1);

    const auto combos = enumerateCombos(composition);
    for (const ComboSpec& combo : combos) {
        const auto& simulation = simConfigOf(composition, combo);
        const auto& mission    = missionConfigOf(composition, combo);

        if (group_failed && combo.group_index != failed_group) {
            group_failed = false; // a new group starts with a clean slate
        }
        if (group_failed) {
            runs.push_back(makeErrorResult(simulation, mission, group_code, group_message));
            continue;
        }
        if (combo.bounds_invalid) {
            // Log once per (group, mission), matching the pre-refactor behavior.
            const std::size_t mission_key = combo.group_index * 1000 + combo.mission_index;
            if (mission_key != last_logged_bounds_combo) {
                last_logged_bounds_combo = mission_key;
                try { logger.log("MISSION_BOUNDARY_INVALID", "mission boundaries are inverted"); }
                catch (...) {}
            }
            runs.push_back(makeErrorResult(simulation, mission, "MISSION_BOUNDARY_INVALID",
                                           "mission boundaries are inverted"));
            continue;
        }
        try {
            auto run_ptr = run_factory_->create(simulation, mission,
                                                droneConfigOf(composition, combo),
                                                lidarConfigOf(composition, combo), output_path);
            runs.push_back(run_ptr->run());
        } catch (const std::exception& e) {
            // Run setup failed (commonly a bad/missing map file). Treat as group-wide: record the
            // code now and reuse it for the rest of the group.
            group_failed  = true;
            failed_group  = combo.group_index;
            group_code    = "FACTORY_ERROR";
            group_message = e.what();
            try { logger.log(group_code, group_message); } catch (...) {}
            runs.push_back(makeErrorResult(simulation, mission, group_code, group_message));
        }
    }

    return types::SimulationManagerReport{
        composition.composition_file,
        utc,                   // generated_at_utc
        "output_map_accuracy", // metric (fixed per spec)
        {0.0, 100.0},          // score_range (fixed per spec)
        -1,                    // error_score (fixed per spec)
        runs,
    };
}

} // namespace simulator
