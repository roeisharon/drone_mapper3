// Fixture plugin: registers cleanly (delta 1) but its constructor ALWAYS throws — the factory can
// never produce an instance, so the component loads yet "could not be executed at all" (forum
// taxonomy case b): it must appear in the report's errors list and produce no per-run files.

#include <Common/IMissionControl.h>
#include <Common/MissionControlRegistration.h>

#include <stdexcept>

namespace TestMcCtorThrows_0_0 {

class TestMcCtorThrows final : public common::IMissionControl {
public:
    explicit TestMcCtorThrows(common::MissionControlDependencies /*dependencies*/) {
        throw std::runtime_error("fixture mission control refuses to be constructed");
    }

    [[nodiscard]] common::types::MissionRunResult runMission() override {
        return {common::types::MissionRunStatus::Error, 0, {}};
    }
};

REGISTER_MISSION_CONTROL(TestMcCtorThrows);

} // namespace TestMcCtorThrows_0_0
