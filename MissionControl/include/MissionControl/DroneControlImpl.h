#pragma once

#include <Common/IDroneMovement.h>
#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IMappingAlgorithm.h>
#include <Common/IMutableMap3D.h>
#include <MissionControl/IDroneControl.h>

#include <optional>

namespace mission_control_322526864_323020347 {

// The mission's own step engine: asks the algorithm for a command, executes it (clamped), scans,
// applies the scan to the output map, and classifies failures. Implements the PRIVATE staff
// interface mission_control::IDroneControl — per the staff structuring doc, drone control is the
// MissionControl project's responsibility, invisible to the Simulator.
//
// Phase 8a (bonus): implements the common-issues table's OPTIONAL error-handling rows as guard
// functions around the core step flow — see step()'s section comments for the row-by-row map.
class DroneControlImpl final : public mission_control::IDroneControl {
public:
    DroneControlImpl(common::types::DroneConfigData drone,
                     common::types::MissionConfigData mission,
                     common::ILidar& lidar,
                     common::IGPS& gps,
                     common::IDroneMovement& movement,
                     common::IMutableMap3D& output_map,
                     common::IMappingAlgorithm& mapping_algorithm);

    [[nodiscard]] common::types::DroneStepResult step() override;
    [[nodiscard]] common::types::DroneState state() const override;

private:
    // Outcome of one executed movement command (or chunk): the driver's result plus the
    // predicted destination of a translation (used by collision classification and row 12).
    struct MoveExecution {
        common::types::MovementResult result{true, {}};
        std::optional<common::Position3D> predicted_dest{};
    };

    // Row 11: validate the step's opening GPS read; substitute the internal estimate when the
    // reading is impossible/out of bounds, throw FatalMissionError when both are.
    [[nodiscard]] common::Position3D validatedStartPosition();

    // Executes one movement command (rows 2, 8, 10 + the row-5 collision classification):
    // whole-cell clamping, mission-bounds amendment, map-bounds refusal, and over-limit
    // splitting (the remainder is stored in pending_movement_).
    [[nodiscard]] MoveExecution executeMovement(const common::types::MovementCommand& mv,
                                                const common::Position3D& pos,
                                                const common::Orientation& heading);

    // Row 6: performs a requested scan with bounded in-step retries on empty results; stores
    // and applies a non-empty scan, counts consecutive all-empty steps and escalates.
    void performScan(const std::optional<common::Orientation>& scan_orientation);

    // Row 12: after a successful translation, confirm the GPS produced a physically possible
    // reading (finite), re-reading a bounded number of times. False → the step reports Error.
    [[nodiscard]] bool postMoveGpsPlausible(const common::Position3D& predicted_dest);

    common::types::DroneConfigData drone_;
    common::types::MissionConfigData mission_;
    common::ILidar& lidar_;
    common::IGPS& gps_;
    common::IDroneMovement& movement_;
    common::IMutableMap3D& output_map_;
    common::IMappingAlgorithm& mapping_algorithm_;
    // The previous step's scan, handed to the algorithm on the next call. Semantics (pinned by
    // tests — ex2 grading bug DRO10 flipped them): the last scan PERSISTS until replaced by a
    // newer NON-EMPTY one; a step without a scan does not clear it, and neither does a
    // malfunctioning empty scan (row 6). nullptr only before the first successful scan.
    std::optional<common::types::LidarScanResult> latest_scan_result_{};
    std::size_t step_index_ = 0;

    // ---- Phase 8a optional-rows state ----
    // Consecutive-fault counters ("throw after N tries"); each resets on its success path.
    std::size_t consecutive_invalid_commands_ = 0;  // row 3
    std::size_t consecutive_noop_commands_ = 0;     // row 4
    std::size_t consecutive_empty_scans_ = 0;       // row 6
    std::size_t consecutive_movement_failures_ = 0; // row 7
    // Row 8: the remainder of an over-limit movement, executed as its own subsequent step(s)
    // without re-consulting the algorithm ("separate the movement into 2 or more commands,
    // each as a separate step").
    std::optional<common::types::MovementCommand> pending_movement_{};
    // Rows 11/12: dead-reckoning fallback — the last position known to be valid (a good GPS
    // read, or the predicted destination of a successfully executed translation).
    std::optional<common::Position3D> internal_position_{};
};

} // namespace mission_control_322526864_323020347
