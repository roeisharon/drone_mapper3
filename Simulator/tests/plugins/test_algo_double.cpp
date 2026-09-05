// Fixture plugin: registers TWO mapping-algorithm factories (the loader must classify a
// registration delta of 2 as a load error and destroy BOTH entries before dlclosing).

#include <Common/IMappingAlgorithm.h>
#include <Common/MappingAlgorithmRegistration.h>

#include <utility>

namespace TestAlgoDouble_0_0 {

class DoubleA final : public common::IMappingAlgorithm {
public:
    explicit DoubleA(common::MappingAlgorithmDependencies dependencies)
        : IMappingAlgorithm(std::move(dependencies)) {}
    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState& /*state*/,
        const common::types::LidarScanResult* /*latest_scan*/) override {
        common::types::MappingStepCommand command;
        command.status = common::types::AlgorithmStatus::Finished;
        return command;
    }
};

class DoubleB final : public common::IMappingAlgorithm {
public:
    explicit DoubleB(common::MappingAlgorithmDependencies dependencies)
        : IMappingAlgorithm(std::move(dependencies)) {}
    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState& /*state*/,
        const common::types::LidarScanResult* /*latest_scan*/) override {
        common::types::MappingStepCommand command;
        command.status = common::types::AlgorithmStatus::Finished;
        return command;
    }
};

REGISTER_MAPPING_ALGORITHM(DoubleA);
REGISTER_MAPPING_ALGORITHM(DoubleB);

} // namespace TestAlgoDouble_0_0
