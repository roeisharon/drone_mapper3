// Fixture plugin instrumenting the Phase-8c grouped lifecycle: a static probe appends
// "load:<so-name>" when the library is mapped (static init at dlopen) and "unload:<so-name>"
// when it is unmapped (static destruction at dlclose) to the file named by the
// DRONE_LIFECYCLE_LOG environment variable. The .so name comes from dladdr on the probe's own
// address, so byte-copies of this fixture staged under different names log their own identity.
// Behaviorally the registered mission control is test_mc_ok: completes immediately, saves.

#include <Common/IMissionControl.h>
#include <Common/MissionControlRegistration.h>

#include <dlfcn.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>

namespace TestMcLifecycle_0_0 {

namespace {

void appendEvent(const char* event) {
    const char* log_path = std::getenv("DRONE_LIFECYCLE_LOG");
    if (log_path == nullptr) {
        return;
    }
    Dl_info info{};
    std::string so_name = "unknown";
    if (dladdr(reinterpret_cast<const void*>(&appendEvent), &info) != 0 &&
        info.dli_fname != nullptr) {
        so_name = std::filesystem::path{info.dli_fname}.filename().string();
    }
    std::ofstream out{log_path, std::ios::app};
    out << event << ":" << so_name << "\n";
}

struct LifecycleProbe {
    LifecycleProbe() { appendEvent("load"); }
    ~LifecycleProbe() { appendEvent("unload"); }
};

[[maybe_unused]] const LifecycleProbe probe{};

} // namespace

class TestMcLifecycle final : public common::IMissionControl {
public:
    explicit TestMcLifecycle(common::MissionControlDependencies dependencies)
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

REGISTER_MISSION_CONTROL(TestMcLifecycle);

} // namespace TestMcLifecycle_0_0
