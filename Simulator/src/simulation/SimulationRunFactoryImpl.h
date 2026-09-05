#pragma once

#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>
#include <Simulator/ISimulationRunFactory.h>

#include <cstddef>
#include <string>

namespace simulator {

// Builds one fully-wired SimulationRunImpl per create() call. The plugin factories (whose lambdas
// live inside the dlopen'd .so files) are held by reference — they are owned by the PluginManager's
// plugin records, which outlive every factory and run by the mode driver's scoping.
//
// Per the spec, the plugin factories are invoked freshly for every run (instances are never
// cached); only the .so files themselves stay loaded.
class SimulationRunFactoryImpl final : public ISimulationRunFactory {
public:
    // run_file_prefix names this pairing's output files (the component .so stem), fulfilling the
    // spec's "unique name per file, easy to relate to the mission that created it".
    // first_run_index presets the run numbering: the task executor builds one throwaway factory
    // per task with the combo index here, so concurrent tasks of the same component produce the
    // same file names the sequential path would — with zero shared mutable state.
    // preloaded_hidden_map (non-owning, may be null): when set, create() uses this shared
    // read-only map instead of loading the NPY itself (the HiddenMapCache path).
    SimulationRunFactoryImpl(const common::MappingAlgorithmFactory& algorithm_factory,
                             const common::MissionControlFactory& mission_control_factory,
                             std::string run_file_prefix,
                             bool verbose,
                             std::size_t first_run_index = 0,
                             const common::IMap3D* preloaded_hidden_map = nullptr);

    [[nodiscard]] std::unique_ptr<ISimulationRun>
    create(const types::SimulationConfigData& simulation_config,
           const common::types::MissionConfigData& mission_config,
           const common::types::DroneConfigData& drone_config,
           const common::types::LidarConfigData& lidar_config,
           const std::filesystem::path& output_path) override;

    // Computes the output map's config from the resolved hidden-map config and the mission bounds.
    // When the mission specifies bounds, the output map is narrowed to (hidden extent ∩ mission
    // bounds) so the drone only plans/scans/moves inside the bounded region; when it doesn't, the
    // output map matches the full hidden extent. Pure + static for testing.
    [[nodiscard]] static common::types::MapConfig
    outputMapConfig(const common::types::MapConfig& hidden_config,
                    const common::types::MappingBounds& mission_bounds);

    // Snaps a world position to the centre of the grid cell that contains it, under the given map
    // config (resolution + offset). The drone starts here so its position coincides with a planner
    // grid-cell centre — the first move then lands cleanly on the grid instead of drifting from a
    // sub-cell origin. Pure + static for testing.
    [[nodiscard]] static common::Position3D
    snapToCellCenter(const common::Position3D& position, const common::types::MapConfig& config);

    // How many plugin instances each factory successfully produced across all create() calls.
    // These feed the reports' "could not be executed at all" predicate (forum taxonomy): a
    // component whose count stayed 0 after its whole matrix — its factory threw or returned null
    // every time — belongs in the report's `errors:` list, not in the results.
    [[nodiscard]] std::size_t algorithmInstancesCreated() const noexcept {
        return algorithm_instances_created_;
    }
    [[nodiscard]] std::size_t missionControlInstancesCreated() const noexcept {
        return mission_control_instances_created_;
    }

private:
    const common::MappingAlgorithmFactory& algorithm_factory_;
    const common::MissionControlFactory& mission_control_factory_;
    std::string run_file_prefix_;
    bool verbose_;
    // Instance state, deliberately NOT a function-local static (the ex2 grading review flagged
    // static state as a hazard): every factory instance numbers its own runs from 0 in creation
    // order, which is the canonical run order for its pairing.
    std::size_t next_run_index_ = 0;
    const common::IMap3D* preloaded_hidden_map_ = nullptr;
    std::size_t algorithm_instances_created_ = 0;
    std::size_t mission_control_instances_created_ = 0;
};

} // namespace simulator
