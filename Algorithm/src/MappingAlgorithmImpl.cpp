#include <Algorithm/MappingAlgorithmImpl.h>

#include <Common/MappingAlgorithmRegistration.h>
#include <UserCommon/DroneFootprint.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>
#include <queue>
#include <unordered_map>
#include <vector>

namespace algorithm_322526864_323020347 {

using namespace common;
namespace uc = user_common_322526864_323020347;

namespace {

// Minimum heading difference (degrees) that triggers a Rotate command rather than an Advance.
constexpr double kAngleTolDeg = 0.5;

// Hard cap on BFS expansions per nextStep() call — the second half of the termination guarantee
// (the structural cell box is the first). Far above any realistic bounded-map cell count; it only
// ever fires if something is deeply wrong, in which case finishing beats hanging.
constexpr std::size_t kMaxBfsExpansions = 1'000'000;

// Fallback half-width (in cells) of the structural box when the map config is degenerate.
constexpr long long kFallbackBoxRadius = 10'000;

// Normalises an angle difference to (-180, +180].
double normaliseDiff(double d) noexcept {
    while (d >  180.0) d -= 360.0;
    while (d < -180.0) d += 360.0;
    return d;
}

} // namespace

std::size_t MappingAlgorithmImpl::GridCellHash::operator()(const GridCell& c) const noexcept {
    std::size_t h = std::hash<int>{}(c.x);
    h ^= std::hash<int>{}(c.y) + 0x9e3779b9u + (h << 6) + (h >> 2);
    h ^= std::hash<int>{}(c.z) + 0x9e3779b9u + (h << 6) + (h >> 2);
    return h;
}

double MappingAlgorithmImpl::stepCm() const noexcept {
    const double step = output_map_.getMapConfig().resolution.numerical_value_in(cm);
    return (step > 0.0) ? step : 1.0;
}

MappingAlgorithmImpl::GridCell
MappingAlgorithmImpl::worldToGrid(const Position3D& pos) const noexcept {
    const double res = stepCm();
    const auto& config = output_map_.getMapConfig();
    const double ox = config.offset.x.numerical_value_in(cm);
    const double oy = config.offset.y.numerical_value_in(cm);
    const double oz = config.offset.z.numerical_value_in(cm);
    return {
        static_cast<int>(std::floor((pos.x.numerical_value_in(cm) + ox) / res)),
        static_cast<int>(std::floor((pos.y.numerical_value_in(cm) + oy) / res)),
        static_cast<int>(std::floor((pos.z.numerical_value_in(cm) + oz) / res)),
    };
}

Position3D MappingAlgorithmImpl::gridToWorld(const GridCell& cell) const noexcept {
    const double res = stepCm();
    const auto& config = output_map_.getMapConfig();
    const double ox = config.offset.x.numerical_value_in(cm);
    const double oy = config.offset.y.numerical_value_in(cm);
    const double oz = config.offset.z.numerical_value_in(cm);
    return {
        ((static_cast<double>(cell.x) + 0.5) * res - ox) * cm,
        ((static_cast<double>(cell.y) + 0.5) * res - oy) * cm,
        ((static_cast<double>(cell.z) + 0.5) * res - oz) * cm,
    };
}

bool MappingAlgorithmImpl::isNavigable(const GridCell& cell) const noexcept {
    // Navigable if inside the map and not a confirmed obstacle.
    //  - Unmapped: Passable (optimistic exploration).
    //  - OutOfBounds: Blocked.
    //  - PotentiallyOccupied: Passable. Represents near-field lidar uncertainty, not a hard wall.
    //    Blocking it strands large drones. The movement mock provides the real collision guard
    //    against the ground-truth map, allowing subsequent scans to resolve the uncertainty.
    const Position3D center = gridToWorld(cell);
    const types::VoxelOccupancy v = output_map_.atVoxel(center);
    if (v == types::VoxelOccupancy::Occupied ||
        v == types::VoxelOccupancy::OutOfBounds) {
        return false;
    }
    // The cell itself is free, but the drone is a sphere of drone_config_.radius — it can only
    // occupy this cell if its full footprint clears every CONFIRMED obstacle. This stops the planner
    // routing a large drone through gaps a point-sized drone could slip through. With a zero radius
    // this reduces to the single-cell test above.
    return uc::footprintFits(output_map_, center, drone_config_.radius,
                             [](types::VoxelOccupancy occ) {
                                 return occ == types::VoxelOccupancy::Occupied;
                             });
}

MappingAlgorithmImpl::CellBounds
MappingAlgorithmImpl::structuralCellBounds(const GridCell& current) const noexcept {
    // Computed from config VALUES only — deliberately independent of atVoxel()/isNavigable(), so
    // BFS termination survives any navigability bug (ex2 grading bug ALG28: a predicate forced to
    // `true` removed the OutOfBounds stop and the search expanded forever).
    const auto& config = output_map_.getMapConfig();
    const double res = stepCm();
    const auto axis = [res](double min_cm, double max_cm, double off_cm,
                            long long& lo, long long& hi) {
        lo = static_cast<long long>(std::floor((min_cm + off_cm) / res));
        hi = static_cast<long long>(std::ceil((max_cm + off_cm) / res)) - 1;
    };
    CellBounds b;
    axis(config.boundaries.min_x.numerical_value_in(cm),
         config.boundaries.max_x.numerical_value_in(cm),
         config.offset.x.numerical_value_in(cm), b.lo_x, b.hi_x);
    axis(config.boundaries.min_y.numerical_value_in(cm),
         config.boundaries.max_y.numerical_value_in(cm),
         config.offset.y.numerical_value_in(cm), b.lo_y, b.hi_y);
    axis(config.boundaries.min_height.numerical_value_in(cm),
         config.boundaries.max_height.numerical_value_in(cm),
         config.offset.z.numerical_value_in(cm), b.lo_z, b.hi_z);

    // Degenerate config (empty/inverted box on any axis) → a generous fixed box around the drone,
    // so the search is ALWAYS bounded, even against a map that reports nothing sensible.
    if (b.hi_x < b.lo_x || b.hi_y < b.lo_y || b.hi_z < b.lo_z) {
        b.lo_x = current.x - kFallbackBoxRadius; b.hi_x = current.x + kFallbackBoxRadius;
        b.lo_y = current.y - kFallbackBoxRadius; b.hi_y = current.y + kFallbackBoxRadius;
        b.lo_z = current.z - kFallbackBoxRadius; b.hi_z = current.z + kFallbackBoxRadius;
    }
    return b;
}

types::MappingStepCommand MappingAlgorithmImpl::nextStep(
    const types::DroneState& state,
    const types::LidarScanResult* /*latest_scan*/)
{
    const GridCell current = worldToGrid(state.position);
    visited_.insert(current);

    // 1. STUCK DETECTOR — if the previous tick commanded a physical move and the drone did not
    //    change cell, it hit a blind-spot wall: blacklist the cell it tried to enter. An
    //    intentional non-move (rotate, scan hold) is exempt via last_was_rotate_.
    const bool did_not_move =
        last_position_cell_.has_value() && *last_position_cell_ == current;
    if (did_not_move && !last_was_rotate_ && last_immediate_target_.has_value()) {
        blacklist_.insert(*last_immediate_target_);
    }

    // 2. VOLUMETRIC SWEEPING
    // Alternate Lidar pitch to map floor and ceiling seamlessly while moving
    const int tick = static_cast<int>(visited_.size());
    double pitch = 0.0;
    if (tick % 3 == 0) pitch = 30.0;
    else if (tick % 3 == 1) pitch = -30.0;

    Orientation dynamic_scan{
        0.0 * horizontal_angle[deg],
        pitch * altitude_angle[deg],
    };

    // 3. TRUE FRONTIER BFS (structurally bounded — see structuralCellBounds)
    const CellBounds box = structuralCellBounds(current);
    const auto inBox = [&box](const GridCell& c) noexcept {
        return c.x >= box.lo_x && c.x <= box.hi_x &&
               c.y >= box.lo_y && c.y <= box.hi_y &&
               c.z >= box.lo_z && c.z <= box.hi_z;
    };

    std::unordered_set<GridCell, GridCellHash> bfs_visited;
    std::queue<GridCell> q;
    std::unordered_map<GridCell, GridCell, GridCellHash> parent;

    q.push(current);
    bfs_visited.insert(current);

    std::optional<GridCell> target_frontier = std::nullopt;
    std::size_t expansions = 0;

    while (!q.empty()) {
        if (++expansions > kMaxBfsExpansions) {
            break; // hard termination backstop; best-effort finish beats hanging a worker thread
        }
        GridCell c = q.front();
        q.pop();

        const auto c_occ = output_map_.atVoxel(gridToWorld(c));

        // If the cell we pulled is Unmapped, it is our destination!
        if (c != current && c_occ == types::VoxelOccupancy::Unmapped) {
            target_frontier = c;
            break;
        }

        const GridCell neighbors[6] = {
            {c.x + 1, c.y, c.z}, {c.x - 1, c.y, c.z},
            {c.x, c.y + 1, c.z}, {c.x, c.y - 1, c.z},
            {c.x, c.y, c.z + 1}, {c.x, c.y, c.z - 1}
        };

        for (const auto& nb : neighbors) {
            // Structural bound FIRST: never leave the map's cell box, no matter what the
            // navigability predicate says about the cell.
            if (!inBox(nb)) continue;
            // The blacklist is deliberately PERMANENT. Phase 8d measured the alternative —
            // re-opening a blacklisted cell once the map shows it Empty — and it regressed the
            // full provided matrix hard (avg 87.3 → 77.0, total steps 22688 → 54373): probed
            // blind-spot walls sit inside the lidar's z_min blind zone, get optimistically
            // over-written as Empty by near-field evidence, and a re-validating planner walks
            // into the same wall repeatedly. A physically-confirmed refusal outranks map
            // evidence that cannot see the cell.
            if (bfs_visited.count(nb) || blacklist_.count(nb)) continue;

            // Enqueue only cells the drone can physically occupy: in-bounds, Empty/Unmapped, and
            // with the full spherical footprint clearing known obstacles (isNavigable). This keeps
            // the planned path width-aware — a large drone never routes through a gap too narrow
            // for its body, while a point-sized drone behaves exactly as before.
            if (isNavigable(nb)) {
                bfs_visited.insert(nb);
                parent[nb] = c;
                q.push(nb);
            }
        }
    }

    if (!target_frontier) {
        return {std::nullopt, dynamic_scan, types::AlgorithmStatus::Finished};
    }

    // 4. PATH EXTRACTION & COMPRESSION
    std::vector<GridCell> path;
    GridCell curr = *target_frontier;
    while (curr != current) {
        path.push_back(curr);
        if (parent.find(curr) == parent.end()) break;
        curr = parent[curr];
    }
    std::reverse(path.begin(), path.end());

    GridCell immediate_step = path[0];
    GridCell target_cell = immediate_step;

    // Scan-before-enter requires an observable leading edge (radius >= z_min).
    // If the new footprint falls inside the lidar's blind zone, requiring confirmation
    // would stall the drone. Instead, it enters optimistically; any collisions
    // remain non-fatal and are mapped on the next scan.
    const double radius_cm = drone_config_.radius.numerical_value_in(cm);
    const double z_min_cm  = lidar_config_.z_min.numerical_value_in(cm);
    const bool confirm_before_enter = radius_cm > 0.0 && radius_cm >= z_min_cm;

    // Jump ahead through a straight run of already-verified-Empty cells. The run is ALWAYS
    // capped to the per-command move limit (whole cells within max_advance / max_elevate): a
    // valid algorithm never issues a command beyond the drone's limits (common-issues row 1),
    // and target_cell is then the cell the drone ACTUALLY lands on — so the leading-edge check
    // and the planner's own bookkeeping match exactly what the move enters. (Before Phase 8a
    // the cap applied only when confirmation was active, silently relying on DroneControl's
    // clamp for the rest.)
    if (path.size() > 1) {
        const int dx = path[0].x - current.x;
        const int dy = path[0].y - current.y;
        const int dz = path[0].z - current.z;

        std::size_t jump_limit = 5;
        {
            const double max_move_cm = (dz != 0)
                ? drone_config_.max_elevate.numerical_value_in(cm)
                : drone_config_.max_advance.numerical_value_in(cm);
            const auto max_cells = static_cast<std::size_t>(
                std::max<long long>(1, static_cast<long long>(std::floor(max_move_cm / stepCm()))));
            jump_limit = std::min(jump_limit, max_cells);
        }

        for (std::size_t i = 1; i < std::min(path.size(), jump_limit); ++i) {
            if (path[i].x - path[i - 1].x == dx &&
                path[i].y - path[i - 1].y == dy &&
                path[i].z - path[i - 1].z == dz &&
                output_map_.atVoxel(gridToWorld(path[i])) == types::VoxelOccupancy::Empty) {
                target_cell = path[i];
            } else {
                break;
            }
        }
    }

    // 5. EXECUTE COMMAND
    // Bounded best-effort leading-edge confirmation. When observable (radius >= z_min),
    // the drone scans voxels newly entering its footprint at target_cell.
    // After kMaxScanAttempts, it enters optimistically to prevent stranding.
    // Unobservable leading edges are never gated.
    constexpr int kMaxScanAttempts = 1;
    const bool leading_unconfirmed = confirm_before_enter &&
        !uc::leadingFootprintClear(output_map_, gridToWorld(current), gridToWorld(target_cell),
                                   drone_config_.radius);

    // Records this tick's state for the stuck-detector to read next tick. rotate_flag=true marks
    // an intentional non-move (rotate or scan-before-enter hold) so a legitimate stay-in-place is
    // not mistaken for a blind-wall stall.
    const auto pushState = [&](bool rotate_flag) {
        last_immediate_target_ = immediate_step;
        last_was_rotate_ = rotate_flag;
        last_position_cell_ = current;
    };

    // Decides whether to hold and scan (instead of moving) this tick, consuming the per-cell scan
    // budget. Returns false — proceed with the move — when the leading edge is confirmed,
    // unobservable, or the budget is spent. Never blacklists: a frontier we could not confirm is
    // entered optimistically, not permanently abandoned.
    const auto takeScanHold = [&]() -> bool {
        if (!leading_unconfirmed) {
            pending_scan_cell_.reset();
            scan_attempts_ = 0;
            return false;
        }
        const bool same = pending_scan_cell_ && *pending_scan_cell_ == target_cell;
        const int attempts = same ? scan_attempts_ : 0;
        if (attempts >= kMaxScanAttempts) {
            return false; // best effort spent → enter optimistically
        }
        pending_scan_cell_ = target_cell;
        scan_attempts_ = attempts + 1;
        return true;
    };

    if (target_cell.z != current.z) {
        const double dz_cm = static_cast<double>(target_cell.z - current.z) * stepCm();
        if (takeScanHold()) {
            // The forward beams never see straight up/down, so aim vertically to reveal the cell
            // the drone is about to rise/descend into before committing the elevate.
            const double aim_pitch = (dz_cm > 0.0) ? 90.0 : -90.0;
            pushState(/*rotate_flag=*/true);
            return {std::nullopt,
                    Orientation{0.0 * horizontal_angle[deg], aim_pitch * altitude_angle[deg]},
                    types::AlgorithmStatus::Working};
        }

        const double elevate_pitch = (dz_cm > 0.0) ? 45.0 : -45.0;
        Orientation elevate_scan{0.0 * horizontal_angle[deg], elevate_pitch * altitude_angle[deg]};
        pushState(/*rotate_flag=*/false);
        return {
            types::MovementCommand{
                types::MovementCommandType::Elevate,
                types::RotationDirection::Left,
                0.0 * deg,
                dz_cm * cm,
            },
            elevate_scan,
            types::AlgorithmStatus::Working,
        };
    }

    const double cx = state.position.x.numerical_value_in(cm);
    const double cy = state.position.y.numerical_value_in(cm);
    const Position3D next_world = gridToWorld(target_cell);
    const double ndx = next_world.x.numerical_value_in(cm) - cx;
    const double ndy = next_world.y.numerical_value_in(cm) - cy;
    const double step_dist = std::sqrt(ndx * ndx + ndy * ndy);

    double desired_heading = std::atan2(ndy, ndx) * 180.0 / std::numbers::pi;
    if (desired_heading < 0.0) desired_heading += 360.0;

    const double cur_heading = state.heading.horizontal.numerical_value_in(deg);
    const double diff = normaliseDiff(desired_heading - cur_heading);

    if (std::abs(diff) > kAngleTolDeg) {
        // Rotate to face the target first; the scan along the way reveals the leading edge in that
        // direction, so by the time the drone is aligned the cells ahead are usually confirmed.
        const types::RotationDirection dir =
            diff > 0.0 ? types::RotationDirection::Left : types::RotationDirection::Right;
        // Never beyond the drone's own limit (row 1); 45° stays the planning cap so provided
        // configs behave exactly as before (their max_rotate is >= 45). A non-positive
        // max_rotate is an unconfigured/degenerate value, treated as unlimited.
        const double max_rotate_deg = drone_config_.max_rotate.numerical_value_in(deg);
        const double rotate_cap = (max_rotate_deg > 0.0) ? std::min(45.0, max_rotate_deg) : 45.0;
        const double safe_rotate_amount = std::min(std::abs(diff), rotate_cap);
        pushState(/*rotate_flag=*/true);
        return {
            types::MovementCommand{
                types::MovementCommandType::Rotate,
                dir,
                safe_rotate_amount * deg,
                0.0 * cm,
            },
            dynamic_scan,
            types::AlgorithmStatus::Working,
        };
    }

    // Facing the target: best-effort confirm the leading edge before advancing (a level forward
    // scan reveals exactly the cells the move enters). Falls through to Advance when confirmed,
    // unobservable, or the scan budget is spent.
    if (takeScanHold()) {
        pushState(/*rotate_flag=*/true);
        return {std::nullopt,
                Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                types::AlgorithmStatus::Working};
    }
    pushState(/*rotate_flag=*/false);
    return {
        types::MovementCommand{
            types::MovementCommandType::Advance,
            types::RotationDirection::Left,
            0.0 * deg,
            step_dist * cm,
        },
        dynamic_scan,
        types::AlgorithmStatus::Working,
    };
}

// Registers this class's factory with the Simulator's registrar when the .so is loaded.
// Must stay at namespace scope with an unqualified class name (the macro token-pastes it).
REGISTER_MAPPING_ALGORITHM(MappingAlgorithmImpl);

} // namespace algorithm_322526864_323020347
