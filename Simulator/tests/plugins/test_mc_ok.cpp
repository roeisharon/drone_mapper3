// Fixture plugin: registers exactly one mission control that completes immediately after saving
// the output map (honoring the save-always contract so end-to-end fixture runs produce files).

#include <Common/IMissionControl.h>
#include <Common/MissionControlRegistration.h>

#include <filesystem>
#include <utility>

namespace TestMcOk_0_0 {

class TestMcOk final : public common::IMissionControl {
public:
    explicit TestMcOk(common::MissionControlDependencies dependencies)
        : output_map_(dependencies.output_map),
          output_map_file_(std::move(dependencies.output_map_file)) {}

    [[nodiscard]] common::types::MissionRunResult runMission() override {
        output_map_.save(output_map_file_);
        return {common::types::MissionRunStatus::Completed, 0, {}};
    }

private:
    common::IMutableMap3D& output_map_;
    std::filesystem::path output_map_file_;
};

REGISTER_MISSION_CONTROL(TestMcOk);

} // namespace TestMcOk_0_0
