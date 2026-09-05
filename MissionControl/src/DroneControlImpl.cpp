#include <MissionControl/DroneControlImpl.h>

#include <MissionControl/FatalCollisionError.h>
#include <MissionControl/ScanResultToVoxels.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <string>
#include <utility>

namespace mission_control_322526864_323020347 {

using namespace common;

namespace {

// N of the common-issues table's "throw an exception after N tries" (its value is explicitly
// left to the implementation): the SAME kind of fault on this many CONSECUTIVE occasions
// escalates from graceful handling to FatalMissionError / step Error. 3 keeps recovery cheap
// while bounding how long a broken component can burn the step budget.
constexpr std::size_t kMaxConsecutiveFaults = 3;
// Row 6: total lidar attempts within one step before the scan counts as an empty-scan fault.
constexpr int kLidarTriesPerStep = 3;
// Row 12: total GPS reads after a movement before the reading counts as impossible.
constexpr int kGpsReadTries = 3;

// Benign refusal messages: the movement did not happen but the mission is healthy — the step
// returns Continue and the planner's stuck-detection reroutes.
constexpr const char* kObstacleRefusal = "DRONE_HITS_OBSTACLE";
constexpr const char* kOutOfBoundsIgnored = "OUT_OF_BOUNDS_IGNORED";

// Clamps a requested move to a whole number of grid cells, strictly bounded by the
// per-command max limit. Returns the distance in cm. Distances are safely floored
// (never rounded up) to guarantee the move never exceeds the requested or maximum
// distance, ensuring it always aligns with cell centers. A small epsilon prevents
// float truncation errors on exact multiples. Degrades to a continuous clamp if res <= 0.
double cellAlignedDistanceCm(double requested_cm, double max_cm, double res_cm) {
    if (res_cm <= 0.0) {
        return std::min(requested_cm, max_cm);
    }
    constexpr double kCellEps = 1e-6;
    const long long req_cells = static_cast<long long>(std::floor(requested_cm / res_cm + kCellEps));
    const long long max_cells = static_cast<long long>(std::floor(max_cm / res_cm + kCellEps));
    const long long cells = std::min(req_cells, std::max<long long>(0, max_cells));
    return static_cast<double>(cells) * res_cm;
}

[[nodiscard]] bool isFinitePosition(const Position3D& p) {
    return std::isfinite(p.x.numerical_value_in(cm)) &&
           std::isfinite(p.y.numerical_value_in(cm)) &&
           std::isfinite(p.z.numerical_value_in(cm));
}

// Row 3: a command whose values cannot be executed at all. Distinct from an over-limit value
// (row 8: split) and from a merely useless-but-legal one. Elevate may be negative (descend);
// Advance may not; Rotate's angle is a magnitude with the direction in its own field.
[[nodiscard]] bool movementValuesInvalid(const types::MovementCommand& mv) {
    const double dist_cm = mv.distance.numerical_value_in(cm);
    const double angle_deg = mv.angle.numerical_value_in(deg);
    switch (mv.type) {
    case types::MovementCommandType::Hover:
        return false;
    case types::MovementCommandType::Rotate:
        return !std::isfinite(angle_deg) || angle_deg < 0.0;
    case types::MovementCommandType::Advance:
        return !std::isfinite(dist_cm) || dist_cm < 0.0;
    case types::MovementCommandType::Elevate:
        return !std::isfinite(dist_cm);
    }
    return true; // out-of-range enum value from a faulty plugin is invalid too
}

// Row 10 helper: mission bounds are honored only when they describe a real box; a
// default-constructed config (all zeros, used by unit fixtures and bound-less setups) means
// "no mission bounds" rather than "nowhere is legal".
[[nodiscard]] bool missionBoundsValid(const types::MappingBounds& b) {
    return b.max_x > b.min_x && b.max_y > b.min_y && b.max_height > b.min_height;
}

[[nodiscard]] bool inMissionBounds(const types::MappingBounds& b, const Position3D& p) {
    return p.x >= b.min_x && p.x <= b.max_x &&
           p.y >= b.min_y && p.y <= b.max_y &&
           p.z >= b.min_height && p.z <= b.max_height;
}

} // namespace

DroneControlImpl::DroneControlImpl(types::DroneConfigData drone,
                                   types::MissionConfigData mission,
                                   ILidar& lidar,
                                   IGPS& gps,
                                   IDroneMovement& movement,
                                   IMutableMap3D& output_map,
                                   IMappingAlgorithm& mapping_algorithm)
    : drone_(std::move(drone)),
      mission_(std::move(mission)),
      lidar_(lidar),
      gps_(gps),
      movement_(movement),
      output_map_(output_map),
      mapping_algorithm_(mapping_algorithm) {}

// Row 11: the GPS says we are somewhere impossible (non-finite) or outside the map. Compare
// with the internal dead-reckoning estimate: if that is still valid, IGNORE the reading and fly
// on the estimate; if both are out of bounds the mission has genuinely lost the drone — throw.
Position3D DroneControlImpl::validatedStartPosition() {
    const Position3D read = gps_.position();
    if (isFinitePosition(read) && output_map_.isInBounds(read)) {
        internal_position_ = read; // healthy read refreshes the estimate
        return read;
    }
    if (internal_position_.has_value() && output_map_.isInBounds(*internal_position_)) {
        return *internal_position_;
    }
    throw FatalMissionError(
        "GPS_OUT_OF_BOUNDS: GPS reading is invalid and no valid internal estimate exists");
}

// Row 6: an empty scan vector is a lidar malfunction (the simulator's lidar always reports the
// center beam, with "never returns" encoded as max distance — an empty vector cannot mean open
// space). Retry within the step; a step whose every try came back empty counts one fault, and
// the stored latest scan is NOT clobbered by the malfunction.
void DroneControlImpl::performScan(const std::optional<Orientation>& scan_orientation) {
    if (!scan_orientation.has_value()) {
        return; // no scan requested; the stored scan persists (DRO10 semantics)
    }
    types::LidarScanResult new_scan;
    for (int attempt = 0; attempt < kLidarTriesPerStep; ++attempt) {
        new_scan = lidar_.scan(*scan_orientation);
        if (!new_scan.empty()) {
            break;
        }
    }
    if (new_scan.empty()) {
        if (++consecutive_empty_scans_ >= kMaxConsecutiveFaults) {
            throw FatalMissionError("LIDAR_FAILURE: empty scan on " +
                                    std::to_string(kMaxConsecutiveFaults) +
                                    " consecutive scanning steps");
        }
        return;
    }
    consecutive_empty_scans_ = 0;
    latest_scan_result_ = new_scan;
    const Position3D scan_pos = gps_.position();
    const Orientation scan_heading = gps_.heading();
    ScanResultToVoxels::applyToMap(output_map_, scan_pos, scan_heading, new_scan,
                                   lidar_.config());
}

// Row 12: a movement was executed, so the GPS must place us SOMEWHERE physically possible.
// Non-finite coordinates cannot be a real pose — re-read a bounded number of times before the
// step reports Error. (A finite but out-of-bounds reading is row 11's business on the next
// step's validation.) A healthy read refreshes the internal estimate; the predicted
// destination already did when the movement succeeded.
bool DroneControlImpl::postMoveGpsPlausible(const Position3D& predicted_dest) {
    internal_position_ = predicted_dest;
    for (int attempt = 0; attempt < kGpsReadTries; ++attempt) {
        const Position3D read = gps_.position();
        if (isFinitePosition(read)) {
            if (output_map_.isInBounds(read)) {
                internal_position_ = read;
            }
            return true;
        }
    }
    return false;
}

// Executes one movement command (or a pending chunk of one). Guard order, outermost first:
//   row 3 already filtered invalid values before this is called;
//   whole-cell clamp to the per-command limit (planner and execution share the footprint);
//   row 10 — amend a translation that would exit the MISSION bounds (shorten to the longest
//            whole-cell prefix that stays inside);
//   row 8  — an over-limit request executes its first chunk now and stores the remainder in
//            pending_movement_, executed on subsequent steps as separate commands;
//   row 2  — a chunk whose destination leaves the MAP is ignored entirely: the driver is never
//            called and any pending remainder is dropped;
//   row 5  — the driver throwing is classified: destination KNOWN Occupied → FatalCollisionError
//            (fatal), otherwise discovery → refused-move semantics.
DroneControlImpl::MoveExecution DroneControlImpl::executeMovement(
    const types::MovementCommand& mv, const Position3D& pos, const Orientation& heading) {
    MoveExecution out;
    const double res_cm = output_map_.getMapConfig().resolution.numerical_value_in(cm);
    bool known_occupied_destination = false;

    try {
        switch (mv.type) {
        case types::MovementCommandType::Hover:
            break; // no-op; drone stays in place
        case types::MovementCommandType::Rotate: {
            const double req_deg = mv.angle.numerical_value_in(deg);
            const double max_deg = drone_.max_rotate.numerical_value_in(deg);
            const double chunk_deg = std::min(req_deg, max_deg);
            out.result = movement_.rotate(mv.rotation, chunk_deg * deg);
            // Row 8 (rotation): the rest of an over-limit turn continues next step.
            if (out.result.success && req_deg - chunk_deg > 1e-9) {
                pending_movement_ = types::MovementCommand{
                    types::MovementCommandType::Rotate, mv.rotation,
                    (req_deg - chunk_deg) * deg, 0.0 * cm};
            }
            break;
        }
        case types::MovementCommandType::Advance: {
            const double req_cm = mv.distance.numerical_value_in(cm);
            const double heading_rad =
                heading.horizontal.numerical_value_in(deg) * std::numbers::pi / 180.0;
            const double dir_x = std::cos(heading_rad);
            const double dir_y = std::sin(heading_rad);
            const auto destAt = [&](double d_cm) {
                return Position3D{pos.x + d_cm * dir_x * x_extent[cm],
                                  pos.y + d_cm * dir_y * y_extent[cm], pos.z};
            };
            // Whole-cell total the algorithm asked for…
            double desired_cm = cellAlignedDistanceCm(req_cm, req_cm, res_cm);
            // …row 10: shortened to the longest prefix that stays inside the mission bounds.
            if (missionBoundsValid(mission_.mission_bounds) && res_cm > 0.0) {
                while (desired_cm > 0.0 &&
                       !inMissionBounds(mission_.mission_bounds, destAt(desired_cm))) {
                    desired_cm -= res_cm;
                }
                desired_cm = std::max(desired_cm, 0.0);
            }
            const double chunk_cm =
                cellAlignedDistanceCm(desired_cm, drone_.max_advance.numerical_value_in(cm),
                                      res_cm);
            if (chunk_cm <= 0.0) {
                break; // fully amended away (row 10) or sub-cell: legal no-op
            }
            out.predicted_dest = destAt(chunk_cm);
            // Row 2: the chunk would leave the map — ignore it, never call the driver.
            if (!output_map_.isInBounds(*out.predicted_dest)) {
                pending_movement_.reset();
                out.predicted_dest.reset();
                out.result = types::MovementResult{false, kOutOfBoundsIgnored};
                break;
            }
            known_occupied_destination =
                output_map_.atVoxel(*out.predicted_dest) == types::VoxelOccupancy::Occupied;
            out.result = movement_.advance(chunk_cm * cm);
            // Row 8: the remainder becomes its own command on subsequent steps.
            if (out.result.success && desired_cm - chunk_cm > 1e-9) {
                pending_movement_ = types::MovementCommand{
                    types::MovementCommandType::Advance, mv.rotation, 0.0 * deg,
                    (desired_cm - chunk_cm) * cm};
            }
            break;
        }
        case types::MovementCommandType::Elevate: {
            // Elevate can be negative (descend); clamp the magnitude to whole cells, keep the sign.
            const double req_cm = mv.distance.numerical_value_in(cm);
            const double sign = (req_cm >= 0.0) ? 1.0 : -1.0;
            const auto destAt = [&](double mag_cm) {
                return Position3D{pos.x, pos.y, pos.z + sign * mag_cm * z_extent[cm]};
            };
            double desired_cm = cellAlignedDistanceCm(std::abs(req_cm), std::abs(req_cm), res_cm);
            if (missionBoundsValid(mission_.mission_bounds) && res_cm > 0.0) {
                while (desired_cm > 0.0 &&
                       !inMissionBounds(mission_.mission_bounds, destAt(desired_cm))) {
                    desired_cm -= res_cm;
                }
                desired_cm = std::max(desired_cm, 0.0);
            }
            const double chunk_cm =
                cellAlignedDistanceCm(desired_cm, drone_.max_elevate.numerical_value_in(cm),
                                      res_cm);
            if (chunk_cm <= 0.0) {
                break;
            }
            out.predicted_dest = destAt(chunk_cm);
            if (!output_map_.isInBounds(*out.predicted_dest)) {
                pending_movement_.reset();
                out.predicted_dest.reset();
                out.result = types::MovementResult{false, kOutOfBoundsIgnored};
                break;
            }
            known_occupied_destination =
                output_map_.atVoxel(*out.predicted_dest) == types::VoxelOccupancy::Occupied;
            out.result = movement_.elevate(sign * chunk_cm * cm);
            if (out.result.success && desired_cm - chunk_cm > 1e-9) {
                pending_movement_ = types::MovementCommand{
                    types::MovementCommandType::Elevate, mv.rotation, 0.0 * deg,
                    sign * (desired_cm - chunk_cm) * cm};
            }
            break;
        }
        } // switch
    } catch (const std::exception& e) {
        pending_movement_.reset(); // a throwing driver invalidates any planned continuation
        if (known_occupied_destination) {
            throw FatalCollisionError(
                "DRONE_COLLISION: movement into a cell already known to be Occupied ("
                + std::string(e.what()) + ")");
        }
        // Discovery: the ground truth blocked a move into space we had not confirmed.
        // Same handling as a refused move; the planner reroutes via its stuck-detector.
        out.predicted_dest.reset();
        out.result = types::MovementResult{false, kObstacleRefusal};
    }
    return out;
}

types::DroneStepResult DroneControlImpl::step() {
    try {
        // 1. Read the current GPS pose (row 11 validates it / substitutes the estimate).
        const Position3D pos = validatedStartPosition();
        const Orientation heading = gps_.heading();

        // 2. Real-collision check: if the drone's current cell is Occupied it is actually INSIDE
        //    an obstacle — the movement layer's prevention failed or an inconsistency occurred.
        //    This is FATAL: a distinct DRONE_COLLISION error stops the mission and scores the
        //    scenario -1 (a move merely REFUSED before entering an obstacle stays non-fatal).
        if (output_map_.atVoxel(pos) == types::VoxelOccupancy::Occupied) {
            return {types::DroneStepStatus::Error, "DRONE_COLLISION"};
        }

        // 2b. The drone physically occupies this cell and it is provably not Occupied, so mark it
        //     Empty. This guarantees every traversed cell is mapped even in open space, where the
        //     forward beams miss (no surface) and would otherwise leave the path Unmapped.
        output_map_.set(pos, types::VoxelOccupancy::Empty);

        // 2c. Row 8 continuation: a previous over-limit command left a remainder — this step
        //     executes the next chunk as its own command, WITHOUT consulting the algorithm
        //     ("2 or more commands, each as a separate step"). No scan: none was requested for
        //     the continuation.
        if (pending_movement_.has_value()) {
            const types::MovementCommand chunk = *pending_movement_;
            pending_movement_.reset(); // executeMovement re-arms it if a remainder remains
            const MoveExecution exec = executeMovement(chunk, pos, heading);
            ++step_index_;
            if (!exec.result.success) {
                pending_movement_.reset(); // a failed chunk drops the rest of the plan
                if (exec.result.message == kObstacleRefusal ||
                    exec.result.message == kOutOfBoundsIgnored) {
                    return {types::DroneStepStatus::Continue, {}};
                }
                if (++consecutive_movement_failures_ >= kMaxConsecutiveFaults) {
                    throw FatalMissionError(
                        "MOVEMENT_DRIVER_FAILURE: " +
                        std::to_string(kMaxConsecutiveFaults) +
                        " consecutive failures (" + exec.result.message + ")");
                }
                return {types::DroneStepStatus::Error, exec.result.message};
            }
            consecutive_movement_failures_ = 0;
            if (exec.predicted_dest.has_value() && !postMoveGpsPlausible(*exec.predicted_dest)) {
                return {types::DroneStepStatus::Error, "GPS_IMPOSSIBLE_COORDINATES"};
            }
            return {types::DroneStepStatus::Continue, {}};
        }

        // 3. Ask the algorithm what to do, passing the previous step's scan (nullptr before the
        //    first scan; a stored scan persists until replaced — see the header note).
        const types::LidarScanResult* scan_ptr =
            latest_scan_result_.has_value() ? &*latest_scan_result_ : nullptr;
        const types::DroneState current_state{pos, heading, step_index_};
        const types::MappingStepCommand cmd = mapping_algorithm_.nextStep(current_state, scan_ptr);

        // 4. Algorithm completion → Completed (both finish statuses are terminal-success).
        if (cmd.status == types::AlgorithmStatus::Finished ||
            cmd.status == types::AlgorithmStatus::FinishedWithUnmappableVoxels) {
            ++step_index_;
            return {types::DroneStepStatus::Completed, {}};
        }

        // 4b. Row 4: an empty command — no movement AND no scan while still Working ("NOOP") —
        //     is tolerated and retried; N consecutive NOOPs mean the algorithm is stuck-broken.
        if (!cmd.movement.has_value() && !cmd.scan_orientation.has_value()) {
            if (++consecutive_noop_commands_ >= kMaxConsecutiveFaults) {
                throw FatalMissionError("FAULTY_ALGORITHM_NOOP: empty command on " +
                                        std::to_string(kMaxConsecutiveFaults) +
                                        " consecutive steps");
            }
            ++step_index_;
            return {types::DroneStepStatus::Continue, {}};
        }
        consecutive_noop_commands_ = 0;

        // 4c. Row 3: a movement whose values cannot be executed (NaN/Inf, negative magnitudes).
        //     The movement is skipped and the algorithm simply gets asked again next step; a
        //     requested scan still runs (harmless information). N consecutive → fatal.
        if (cmd.movement.has_value() && movementValuesInvalid(*cmd.movement)) {
            if (++consecutive_invalid_commands_ >= kMaxConsecutiveFaults) {
                throw FatalMissionError("FAULTY_ALGORITHM_INVALID_COMMAND: invalid values on " +
                                        std::to_string(kMaxConsecutiveFaults) +
                                        " consecutive steps");
            }
            performScan(cmd.scan_orientation);
            ++step_index_;
            return {types::DroneStepStatus::Continue, {}};
        }
        consecutive_invalid_commands_ = 0;

        // 5. Execute the requested movement (rows 2/5/8/10 inside executeMovement).
        MoveExecution exec;
        if (cmd.movement.has_value()) {
            exec = executeMovement(*cmd.movement, pos, heading);
        }

        // 6. Perform the scan (if requested) from the post-movement pose, then apply it to the
        //    output map (row 6 inside performScan).
        performScan(cmd.scan_orientation);

        // 7. Advance the step counter (every executed step counts, including the first — the exact
        //    values are pinned by tests; steps feed the grading-visible total_steps).
        //    Benign refusals (obstacle discovery, out-of-map ignore) → Continue: the drone stayed
        //    put and the planner reroutes. A genuine driver failure returns Error (row-9 logging)
        //    and, on the Nth consecutive occurrence, escalates to fatal (row 7).
        ++step_index_;
        if (!exec.result.success) {
            if (exec.result.message == kObstacleRefusal ||
                exec.result.message == kOutOfBoundsIgnored) {
                return {types::DroneStepStatus::Continue, {}};
            }
            if (++consecutive_movement_failures_ >= kMaxConsecutiveFaults) {
                throw FatalMissionError("MOVEMENT_DRIVER_FAILURE: " +
                                        std::to_string(kMaxConsecutiveFaults) +
                                        " consecutive failures (" + exec.result.message + ")");
            }
            return {types::DroneStepStatus::Error, exec.result.message};
        }
        consecutive_movement_failures_ = 0;

        // 7b. Row 12: a translation happened — the GPS must report a possible pose.
        if (exec.predicted_dest.has_value() && !postMoveGpsPlausible(*exec.predicted_dest)) {
            return {types::DroneStepStatus::Error, "GPS_IMPOSSIBLE_COORDINATES"};
        }
        return {types::DroneStepStatus::Continue, {}};

    } catch (const FatalMissionError&) {
        throw; // the one exception family that must escape (rows 3/4/5/6/7/11 escalations)
    } catch (const std::exception& e) {
        return {types::DroneStepStatus::Error,
                std::string("DroneControlImpl::step exception: ") + e.what()};
    }
}

types::DroneState DroneControlImpl::state() const {
    return types::DroneState{gps_.position(), gps_.heading(), step_index_};
}

} // namespace mission_control_322526864_323020347
