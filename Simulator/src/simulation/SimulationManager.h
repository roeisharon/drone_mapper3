#pragma once

#include <Simulator/ISimulation.h>
#include <Simulator/ISimulationRunFactory.h>

#include <filesystem>
#include <memory>

namespace simulator {

// Runs one full composition (the cartesian product groups × missions × drones × lidars) for ONE
// (algorithm, mission control) pairing — the run factory it owns is already bound to that pairing.
// The mode drivers sit above this class and iterate pairings; per-run and cross-component report
// files are written there, not here.
class SimulationManager final : public ISimulation {
public:
    // manager_log_file: where factory/boundary failures for this pairing are logged (each pairing
    // gets its own file so no log is ever shared between concurrent managers later on).
    SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory,
                      std::filesystem::path manager_log_file);

    [[nodiscard]] types::SimulationManagerReport run(
        const types::SimulationCompositionData& composition,
        const std::filesystem::path& output_path) override;

private:
    std::unique_ptr<ISimulationRunFactory> run_factory_;
    std::filesystem::path manager_log_file_;
};

} // namespace simulator
