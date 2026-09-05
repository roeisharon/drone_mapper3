#pragma once

#include <Common/IMappingAlgorithm.h>

#include <cstddef>
#include <optional>
#include <unordered_set>

namespace algorithm_322526864_323020347 {

// BFS-based 3-D mapping algorithm (the competition entry, ported from assignment 2).
//
// Exploration model:
//   - Grid resolution matches the output map.
//   - Optimistic BFS targets Unmapped cells (obstacles and out-of-bounds are ignored).
//   - Scan-before-enter: the drone only moves into confirmed Empty cells where its leading edge is
//     observable; Unmapped target cells trigger a stationary directional scan first.
//   - nextStep() is a reactive state machine returning commands based on live map/drone state.
//   - Scans are requested on every active step to build the map continuously.
//   - Returns AlgorithmStatus::Finished when the BFS frontier is exhausted.
//
// Termination hardening (ex3, from the ex2 grading review): the BFS is additionally bounded by a
// STRUCTURAL cell box derived from the output map's config values alone, plus a hard cap on
// expansions per nextStep(). Termination therefore never depends on what atVoxel() returns — a
// bug (or mutation) in the navigability predicate can degrade mapping quality, but can no longer
// make nextStep() run forever.
class MappingAlgorithmImpl final : public common::IMappingAlgorithm {
public:
    // Inherit the base-class constructor (single MappingAlgorithmDependencies parameter).
    using common::IMappingAlgorithm::IMappingAlgorithm;

    // Returns the next MappingStepCommand. latest_scan is nullptr on the first call and may
    // be nullptr on any step; the output map (output_map_) is the authoritative source.
    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState& state,
        const common::types::LidarScanResult* latest_scan) override;

private:
    // Integer-quantized 3-D grid coordinate; one unit equals the output map's resolution.
    struct GridCell {
        int x = 0;
        int y = 0;
        int z = 0;
        bool operator==(const GridCell& o) const noexcept {
            return x == o.x && y == o.y && z == o.z;
        }
    };

    struct GridCellHash {
        std::size_t operator()(const GridCell& c) const noexcept;
    };

    // Inclusive per-axis cell-index box the BFS may never leave.
    struct CellBounds {
        long long lo_x = 0, hi_x = 0;
        long long lo_y = 0, hi_y = 0;
        long long lo_z = 0, hi_z = 0;
    };

    using CellSet = std::unordered_set<GridCell, GridCellHash>;

    // Exploration state.
    CellSet visited_;   // cells the drone has physically occupied
    // Blacklist of probed blind-spot walls (see stuck detector). NOT permanent (Phase 8d): a
    // cell later proven Empty by a scan is dropped from the blacklist when the BFS meets it —
    // the refusal was transient, so the cell is reachable again.
    CellSet blacklist_;

    // Stuck-detector state from the previous tick (Phase 8d: named members — this was a 3-slot
    // reuse of a queue in the ex2 port). rotate flag = the previous command was an intentional
    // non-move (rotate or scan hold), so staying put was expected, not a blind-wall stall.
    std::optional<GridCell> last_immediate_target_;
    std::optional<GridCell> last_position_cell_;
    bool last_was_rotate_ = false;

    // Scan-before-enter bookkeeping: the cell a bodied drone is currently trying to confirm before
    // entering, and how many consecutive scans it has spent on it. Re-scanning a fixed orientation
    // reveals nothing new, so after a small cap the algorithm enters optimistically rather than
    // scanning forever. Unused for point drones (radius 0), which stay optimistic.
    std::optional<GridCell> pending_scan_cell_;
    int scan_attempts_ = 0;

    // Grid step size (cm) from the output map's resolution.
    [[nodiscard]] double stepCm() const noexcept;
    // World position -> nearest grid cell.
    [[nodiscard]] GridCell worldToGrid(const common::Position3D& pos) const noexcept;
    // Grid cell -> its world-space centre position.
    [[nodiscard]] common::Position3D gridToWorld(const GridCell& cell) const noexcept;
    // True iff the cell is in-bounds, not a known obstacle, AND the drone's spherical footprint
    // (drone_config_.radius) fits there without overlapping a confirmed obstacle.
    [[nodiscard]] bool isNavigable(const GridCell& cell) const noexcept;
    // The structural BFS box: the output map's cell range computed from its config VALUES only
    // (boundaries/offset/resolution — never atVoxel). Degenerate configs fall back to a generous
    // fixed box around `current`, so the search stays bounded in every case.
    [[nodiscard]] CellBounds structuralCellBounds(const GridCell& current) const noexcept;
};

} // namespace algorithm_322526864_323020347
