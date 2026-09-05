// Fixture plugin: registers cleanly (delta 1) but its constructor ALWAYS throws — the algorithm
// can never be instantiated, so in competition mode it "could not be executed at all" (forum
// taxonomy case b): errors list, no per-run files.

#include <Common/IMappingAlgorithm.h>
#include <Common/MappingAlgorithmRegistration.h>

#include <stdexcept>
#include <utility>

namespace TestAlgoCtorThrows_0_0 {

class TestAlgoCtorThrows final : public common::IMappingAlgorithm {
public:
    explicit TestAlgoCtorThrows(common::MappingAlgorithmDependencies dependencies)
        : IMappingAlgorithm(std::move(dependencies)) {
        throw std::runtime_error("fixture algorithm refuses to be constructed");
    }

    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState& /*state*/,
        const common::types::LidarScanResult* /*latest_scan*/) override {
        common::types::MappingStepCommand command;
        command.status = common::types::AlgorithmStatus::Finished;
        return command;
    }
};

REGISTER_MAPPING_ALGORITHM(TestAlgoCtorThrows);

} // namespace TestAlgoCtorThrows_0_0
