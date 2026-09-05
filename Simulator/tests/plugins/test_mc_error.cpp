// Fixture plugin: a mission control that loads and EXECUTES, but every mission ends with an Error
// status. Per the forum taxonomy this component must stay in results_summary (with -1 scores in
// its per-run reports) and must NOT appear in the report's errors list.

#include <Common/IMissionControl.h>
#include <Common/MissionControlRegistration.h>

#include <filesystem>
#include <utility>

namespace TestMcError_0_0 {

class TestMcError final : public common::IMissionControl {
public:
    explicit TestMcError(common::MissionControlDependencies dependencies)
        : output_map_(dependencies.output_map),
          output_map_file_(std::move(dependencies.output_map_file)) {}

    [[nodiscard]] common::types::MissionRunResult runMission() override {
        output_map_.save(output_map_file_); // save-always contract holds even for failing missions
        return {common::types::MissionRunStatus::Error, 1,
                {{"MC_ALWAYS_FAILS", "fixture mission control always reports an error"}}};
    }

private:
    common::IMutableMap3D& output_map_;
    std::filesystem::path output_map_file_;
};

REGISTER_MISSION_CONTROL(TestMcError);

} // namespace TestMcError_0_0
