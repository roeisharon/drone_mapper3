// Fixture plugin: registers EXACTLY ONE mapping-algorithm factory (the well-behaved case for the
// loader's count-delta protocol). The algorithm finishes immediately.

#include <Common/IMappingAlgorithm.h>
#include <Common/MappingAlgorithmRegistration.h>

#include <utility>

namespace TestAlgoOk_0_0 {

class TestAlgoOk final : public common::IMappingAlgorithm {
public:
    explicit TestAlgoOk(common::MappingAlgorithmDependencies dependencies)
        : IMappingAlgorithm(std::move(dependencies)) {}

    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState& /*state*/,
        const common::types::LidarScanResult* /*latest_scan*/) override {
        common::types::MappingStepCommand command;
        command.status = common::types::AlgorithmStatus::Finished;
        return command;
    }
};

REGISTER_MAPPING_ALGORITHM(TestAlgoOk);

} // namespace TestAlgoOk_0_0
