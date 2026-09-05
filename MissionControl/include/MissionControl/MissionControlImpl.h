#pragma once

#include <Common/IMissionControl.h>
#include <Common/MissionControlFactory.h>
#include <MissionControl/DroneControlImpl.h>

#include <filesystem>

namespace mission_control_322526864_323020347 {

// Drives one mission: steps the drone controller up to max_steps times, classifies the terminal
// status, always saves the output map, and (iff -verbose was passed) writes a verbose progress
// file next to it. Creates its OWN drone controller from the injected dependencies — the ex3
// contract ("mission will create its own drone controller"); the Simulator never sees it.
// Deliberately holds NO hidden map: the ex2 class stored one it never used, which is impossible
// (and wrong) for a real mission controller — dropped in the port.
class MissionControlImpl final : public common::IMissionControl {
public:
    explicit MissionControlImpl(common::MissionControlDependencies dependencies);

    [[nodiscard]] common::types::MissionRunResult runMission() override;

private:
    common::types::MissionConfigData mission_;
    common::IMutableMap3D& output_map_;
    std::filesystem::path output_map_file_;
    bool verbose_;
    DroneControlImpl drone_control_;
};

} // namespace mission_control_322526864_323020347
