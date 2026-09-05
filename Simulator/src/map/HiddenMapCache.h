#pragma once

#include "map/Map3DImpl.h"

#include <Simulator/SimulationTypes.h>

#include <cstddef>
#include <string>
#include <vector>

namespace simulator {

// One hidden (ground-truth) map per simulation group, loaded ONCE on the main thread before any
// worker exists and shared read-only across every concurrent run of that group. Concurrent
// atVoxel() reads of an immutable Map3DImpl are data-race-free; ownership is plain unique_ptr
// because the cache's lifetime strictly dominates all workers (the driver joins before the cache
// dies) — no shared_ptr needed, per the assignment's ownership rules.
//
// Load failures are RECORDED, not thrown: every task of a dead group turns the stored error into
// its own FACTORY_ERROR result, preserving the sequential manager's per-group failure semantics
// without any shared mutable state.
class HiddenMapCache {
public:
    struct Entry {
        const common::IMap3D* map = nullptr; // null → error holds the load-failure text
        std::string error;
        common::types::MapConfig resolved_config; // the loaded map's real config (valid iff map)
    };

    // Loads every group's map (main thread only).
    [[nodiscard]] static HiddenMapCache build(const types::SimulationCompositionData& composition);

    [[nodiscard]] const Entry& forGroup(std::size_t group_index) const {
        return entries_[group_index];
    }

private:
    std::vector<Entry> entries_;
    std::vector<std::unique_ptr<const Map3DImpl>> storage_;
};

} // namespace simulator
