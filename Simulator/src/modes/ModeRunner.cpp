#include "modes/ModeRunner.h"

#include "config/YamlConfigParser.h"
#include "exec/TaskExecutor.h"
#include "map/HiddenMapCache.h"
#include "plugins/PluginManager.h"
#include "report/ComparativeReportWriter.h"
#include "report/CompetitiveReportWriter.h"
#include "report/OutputPaths.h"
#include "report/ResultAggregation.h"
#include "report/SimulationReportWriter.h"
#include "simulation/ComboEnumeration.h"
#include "simulation/SimulationRunFactoryImpl.h"

#include <UserCommon/ErrorLogger.h>

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <optional>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace simulator {

namespace uc = user_common_322526864_323020347;

namespace {

// Lists the regular *.so files in dir, sorted by filename for a deterministic load/report order.
// Returns nullopt (with a message on stderr) when the folder cannot be traversed.
std::optional<std::vector<std::filesystem::path>> listSoFiles(const std::filesystem::path& dir) {
    std::vector<std::filesystem::path> files;
    std::error_code ec;
    for (std::filesystem::directory_iterator it{dir, ec}, end; it != end && !ec;
         it.increment(ec)) {
        if (it->is_regular_file(ec) && it->path().extension() == ".so") {
            files.push_back(it->path());
        }
    }
    if (ec) {
        std::cerr << "Error: cannot traverse folder '" << dir.string() << "': " << ec.message()
                  << "\n";
        return std::nullopt;
    }
    std::sort(files.begin(), files.end(),
              [](const auto& a, const auto& b) { return a.filename() < b.filename(); });
    return files;
}

// Replays the canonical combo order over the parser's source-path records, producing RunPaths
// index-aligned with the combo enumeration.
std::vector<RunPaths> enumerateRunPaths(const config::CompositionPaths& paths,
                                        const types::SimulationCompositionData& composition) {
    std::vector<RunPaths> result;
    if (paths.groups.size() != composition.simulation_mission_groups.size()) {
        return result; // misaligned (parser skipped entries unevenly) → writer falls back to empty paths
    }
    for (std::size_t g = 0; g < paths.groups.size(); ++g) {
        const auto& missions = std::get<1>(composition.simulation_mission_groups[g]);
        if (paths.groups[g].mission_configs.size() != missions.size()) {
            return {};
        }
        for (std::size_t m = 0; m < missions.size(); ++m) {
            for (const auto& drone_path : paths.drone_configs) {
                for (const auto& lidar_path : paths.lidar_configs) {
                    result.push_back(RunPaths{paths.groups[g].simulation_config,
                                              paths.groups[g].mission_configs[m],
                                              drone_path, lidar_path});
                }
            }
        }
    }
    return result;
}

// The component stem used as the per-run file prefix ("MissionControl_1_2" from
// "MissionControl_1_2.so").
std::string stemOf(const std::string& so_filename) {
    return std::filesystem::path{so_filename}.stem().string();
}

// One task's outcome: the run result plus whether the varying component's factory produced an
// instance (feeds the executed-at-all classification post-join).
struct TaskResult {
    types::SimulationResult result;
    bool instance_created = false;
};

// THE "could not be executed at all" PREDICATE (forum taxonomy, deliberately isolated here):
// a component that loaded but whose factory never produced a single instance across its whole
// run matrix (threw or returned null every time) joins the report's `errors:` list alongside
// unloadable components. A component that executed at least one run stays in the results — its
// individual failures appear as -1 scores in its per-run reports, never in `errors:`.
bool executedAtAll(const std::vector<TaskResult>& results, std::size_t begin, std::size_t count) {
    for (std::size_t i = begin; i < begin + count; ++i) {
        if (results[i].instance_created) {
            return true;
        }
    }
    return false;
}

// Everything both modes share once the mode-specific pieces are named. fixed/varying terminology:
// comparative fixes the algorithm and varies the mission controls; competition fixes the mission
// control and varies the algorithms.
struct ModeSetup {
    bool varying_is_algorithm = false;        // competition mode
    std::filesystem::path varying_folder;     // scanned for .so files
    std::filesystem::path fixed_so;           // the single fixed-side plugin
    std::string folder_arg_name;              // for error messages ("mission_control_folder"/...)
    std::string fixed_arg_name;               // for error messages ("algorithm"/"mission_control")
    std::string output_prefix;                // "comparative_results_" / "competition_"
};

int runMode(const ParsedArgs& args, const ModeSetup& setup) {
    // 1. Enumerate the varying-side candidates. Zero .so files is a CLI-level error per the spec
    //    ("a folder that has zero files of the desired usage").
    const auto so_files = listSoFiles(setup.varying_folder);
    if (!so_files) {
        return 1;
    }
    if (so_files->empty()) {
        printUsage(std::cerr);
        std::cerr << "Error: " << setup.folder_arg_name << " '" << setup.varying_folder.string()
                  << "' contains no .so files\n";
        return 1;
    }

    // 2. Load the fixed-side plugin. Its failure means the whole invocation cannot run (it is a
    //    mandatory file argument → usage + error, finish). PluginManager is declared before
    //    anything that uses the factories, so its dlclose runs after every factory and instance
    //    is gone.
    PluginManager plugins;
    {
        std::string error;
        const bool loaded = setup.varying_is_algorithm
                                ? plugins.loadMissionControl(setup.fixed_so, error)
                                : plugins.loadAlgorithm(setup.fixed_so, error);
        if (!loaded) {
            printUsage(std::cerr);
            std::cerr << "Error: cannot load " << setup.fixed_arg_name << " '"
                      << setup.fixed_so.string() << "': " << error << "\n";
            return 1;
        }
    }

    // 3. Create the timestamped results folder directly under the varying-side folder (spec).
    const auto out_dir = createOutputFolder(setup.varying_folder, setup.output_prefix);
    if (!out_dir) {
        return 1; // createOutputFolder already printed the error
    }

    uc::ErrorLogger main_logger{*out_dir / "main_errors.log"};

    // 4. (Phase 8c bonus — grouped-by-plugin lifecycle.) Varying-side plugins are NOT loaded up
    //    front: each is loaded once when its group starts, runs its whole matrix, and is
    //    unloaded before the next group's load — see step 6.
    std::vector<std::string> errors;

    // 5. Parse the composition once, enumerate the canonical combo order, load every hidden map
    //    ONCE into the shared read-only cache, and capture ONE timestamp for every report —
    //    all still on the main thread.
    config::CompositionPaths comp_paths;
    const auto composition =
        config::parseSimulationCompositions(args.simulation_yaml, main_logger, &comp_paths);
    if (composition.simulation_mission_groups.empty()) {
        std::cerr << "Error: no valid simulations found in '" << args.simulation_yaml.string()
                  << "' (see " << (*out_dir / "main_errors.log").string() << ")\n";
        return 1;
    }
    const auto run_paths = enumerateRunPaths(comp_paths, composition);
    const auto combos = enumerateCombos(composition);
    const auto map_cache = HiddenMapCache::build(composition);
    const std::string utc = utcTimestamp();

    // 6. Grouped-by-plugin execution (Phase 8c bonus). Each varying-side plugin is its own
    //    GROUP: dlopen once (main thread, before this group's workers exist — the registrar
    //    stays lock-free by the same timeline invariant as before), fan its whole combo matrix
    //    out across the worker pool, join, aggregate — and then unload it (factory destroyed,
    //    ONE dlclose) before the next group's load. No .so is ever loaded twice, none stays
    //    mapped after its last run, and every created instance died inside its own task before
    //    the join. The fixed-side plugin is needed by every group, so it loads once up front
    //    (step 2) and stays until the end. Single-threaded and threaded runs share this exact
    //    path (workerThreads()==0 just runs the group inline).
    //    Within a group: task = combo index; results are pre-allocated with exactly ONE writer
    //    per slot; distribution is an atomic index; composition, combos, cache and factories
    //    are only ever READ by workers — no locks exist anywhere.
    const std::size_t per_component = combos.size();
    std::vector<ComponentAggregate> aggregates;

    for (const auto& so : *so_files) {
        const std::string name = so.filename().string();

        // Load this group's plugin (once, ever).
        PluginManager group_plugins;
        {
            std::string error;
            const bool loaded = setup.varying_is_algorithm
                                    ? group_plugins.loadAlgorithm(so, error)
                                    : group_plugins.loadMissionControl(so, error);
            if (!loaded) {
                errors.push_back(name);
                try { main_logger.log("PLUGIN_LOAD_ERROR", so.string() + ": " + error); }
                catch (...) {}
                continue;
            }
        }
        const common::MappingAlgorithmFactory& algorithm_factory =
            setup.varying_is_algorithm ? group_plugins.algorithms().front().factory
                                       : plugins.algorithms().front().factory;
        const common::MissionControlFactory& mission_control_factory =
            setup.varying_is_algorithm ? plugins.missionControls().front().factory
                                       : group_plugins.missionControls().front().factory;

        std::vector<TaskResult> results(per_component);
        const std::string prefix = stemOf(name);

        const auto execute_task = [&](std::size_t k) {
            const ComboSpec& combo = combos[k];
            const auto& simulation = simConfigOf(composition, combo);
            const auto& mission    = missionConfigOf(composition, combo);
            TaskResult& out = results[k];

            // Pre-run failures go to the run's own log file (the same stem its outputs use); no
            // log file is ever shared between tasks.
            uc::ErrorLogger run_logger{
                *out_dir / (prefix + "_run" + std::to_string(k) + "_output.log")};

            try {
                if (combo.bounds_invalid) {
                    out.result = makeErrorResult(simulation, mission, "MISSION_BOUNDARY_INVALID",
                                                 "mission boundaries are inverted");
                    try { run_logger.log("MISSION_BOUNDARY_INVALID",
                                         "mission boundaries are inverted"); }
                    catch (...) {}
                    return;
                }
                const auto& map_entry = map_cache.forGroup(combo.group_index);
                if (map_entry.map == nullptr) {
                    // The group's hidden map failed to load (recorded once at cache build): every
                    // combo of the group fails with the same code — the sequential group-failure
                    // semantics without shared mutable state.
                    out.result = makeErrorResult(simulation, mission, "FACTORY_ERROR",
                                                 map_entry.error);
                    try { run_logger.log("FACTORY_ERROR", map_entry.error); } catch (...) {}
                    return;
                }

                // A throwaway factory per task: run numbering preset from the combo index
                // (identical file names to the sequential path), the shared map injected,
                // instances created FRESH from the plugin factories (spec: never cache
                // instances — recreating from the factory is NOT reloading the .so).
                SimulationRunFactoryImpl factory{algorithm_factory,
                                                 mission_control_factory,
                                                 prefix, args.verbose, k, map_entry.map};
                auto run = factory.create(simulation, mission,
                                          droneConfigOf(composition, combo),
                                          lidarConfigOf(composition, combo), *out_dir);
                out.instance_created = setup.varying_is_algorithm
                                           ? factory.algorithmInstancesCreated() > 0
                                           : factory.missionControlInstancesCreated() > 0;
                out.result = run->run();
            } catch (const std::exception& e) {
                out.result = makeErrorResult(simulation, mission, "FACTORY_ERROR", e.what());
                try { run_logger.log("FACTORY_ERROR", e.what()); } catch (...) {}
            } catch (...) {
                // Workers must never leak an exception (std::terminate); the catch-all keeps the
                // "Simulator never crashes except on a plugin crash" promise on every thread.
                out.result = makeErrorResult(simulation, mission, "FACTORY_ERROR",
                                             "unknown failure");
            }
        };

        TaskExecutor::runAll(per_component, args.workerThreads(), execute_task);

        // 7 (per group), post-join, main thread only: classify executed-at-all, write the
        // per-run score reports for executed components (the classification needs the whole
        // matrix, which no single worker can know), and aggregate.
        if (!executedAtAll(results, 0, per_component)) {
            errors.push_back(name);
            try {
                main_logger.log("PLUGIN_EXECUTE_ERROR",
                                name + ": loaded but could not be executed at all");
            } catch (...) {}
            continue; // group_plugins unloads on scope exit — the .so never comes back
        }

        std::vector<types::SimulationResult> runs;
        runs.reserve(per_component);
        for (std::size_t k = 0; k < per_component; ++k) {
            runs.push_back(std::move(results[k].result));
        }
        for (std::size_t k = 0; k < runs.size(); ++k) {
            const RunPaths paths = (k < run_paths.size()) ? run_paths[k] : RunPaths{};
            SimulationReportWriter::writeRunReport(
                runs[k], paths, composition.composition_file, utc,
                *out_dir / (prefix + "_run" + std::to_string(k) + "_result.yaml"));
        }
        aggregates.push_back(aggregateComponent(name, runs));
        // group_plugins goes out of scope HERE: its factory is destroyed first, then the one
        // dlclose — with every instance long dead (tasks joined above). Unload-when-unused.
    }

    // Errors sorted for deterministic output regardless of whether a component failed at load or
    // at execute time.
    std::sort(errors.begin(), errors.end());
    if (setup.varying_is_algorithm) {
        CompetitiveReportWriter::write(*out_dir / "competitive_report.yaml",
                                       composition.composition_file,
                                       plugins.missionControls().front().name,
                                       utc,
                                       aggregates,
                                       errors);
    } else {
        ComparativeReportWriter::write(*out_dir / "comparative_report.yaml",
                                       composition.composition_file,
                                       setup.varying_folder,
                                       utc,
                                       aggregates,
                                       errors);
    }

    std::cout << (setup.varying_is_algorithm ? "Competition" : "Comparative") << " run finished: "
              << aggregates.size() << (setup.varying_is_algorithm ? " algorithm(s), "
                                                                  : " mission control(s), ")
              << per_component << " run(s) each. Results in " << out_dir->string() << "\n";
    return 0;
}

} // namespace

int runComparativeMode(const ParsedArgs& args) {
    ModeSetup setup;
    setup.varying_is_algorithm = false;
    setup.varying_folder = args.mission_control_folder;
    setup.fixed_so = args.algorithm_so;
    setup.folder_arg_name = "mission_control_folder";
    setup.fixed_arg_name = "algorithm";
    setup.output_prefix = "comparative_results_";
    return runMode(args, setup);
}

int runCompetitionMode(const ParsedArgs& args) {
    ModeSetup setup;
    setup.varying_is_algorithm = true;
    setup.varying_folder = args.algorithms_folder;
    setup.fixed_so = args.mission_control_so;
    setup.folder_arg_name = "algorithms_folder";
    setup.fixed_arg_name = "mission_control";
    setup.output_prefix = "competition_";
    return runMode(args, setup);
}

} // namespace simulator
