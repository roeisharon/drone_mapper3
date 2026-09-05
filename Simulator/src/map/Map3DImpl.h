#pragma once

#include <TinyNPY.h>

#include <Common/IMutableMap3D.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <vector>

namespace simulator {

// Voxel-map implementation backing both the hidden (ground-truth) map and the per-run output map.
// Simulation-only per the staff structuring doc: in a real deployment maps would come from real
// sensors/services, so this lives in the Simulator project, not in common.
class Map3DImpl final : public common::IMutableMap3D {
public:
    // Output-map constructor: an empty map sized from the config's boundaries/resolution, with
    // every voxel Unmapped. set() fills voxels in as the drone scans.
    explicit Map3DImpl(const common::types::MapConfig& map_config);

    // What an NPY file being loaded represents — decides how the negative mapping-state bytes
    // are treated (forum clarification): they are ILLEGAL in a ground-truth simulation input map
    // (they describe mapping results, not ground truth) but VALID in a map the simulation produced.
    enum class NpySource {
        GroundTruth,  // strict: Unmapped/OutOfBounds/PotentiallyOccupied bytes throw
        MappedOutput, // lenient: they decode back to their VoxelOccupancy values
    };

    // NPY-loading constructor: dimensions come from the (non-empty, 3-D, 1-byte-per-voxel) NPY
    // array; boundaries are recomputed from shape + offset + resolution. The NPY bytes are
    // copied into owned storage — the array is not retained (ex2 kept a vestigial shared_ptr;
    // dropped per the "prefer unique_ptr / no unneeded sharing" rule).
    // Positive block ids normalize to Occupied with no log required (forum clarification; ex2
    // behavior).
    // Under GroundTruth (the default — every production load site loads ground truth), the
    // negative mapping-state values throw std::invalid_argument naming all distinct offenders;
    // the load sites' existing error handling turns that into a per-group failure.
    Map3DImpl(const NpyArray& npy, const common::types::MapConfig& map_config,
              NpySource source = NpySource::GroundTruth);

    [[nodiscard]] common::types::VoxelOccupancy atVoxel(const common::Position3D& pos) const override;
    [[nodiscard]] common::types::MapConfig getMapConfig() const override;
    [[nodiscard]] bool isInBounds(const common::Position3D& pos) const override;

    // Mutable map methods
    void set(const common::Position3D& pos, common::types::VoxelOccupancy value) override;
    void save(const std::filesystem::path& output_path) const override;

private:
    // Maps a world position to a flat array index, or nullopt when out of bounds.
    // Shared by atVoxel, set, and isInBounds so the index math lives in one place.
    [[nodiscard]] std::optional<std::size_t> flatIndexFor(const common::Position3D& pos) const;

    common::types::MapConfig config_;
    // Voxel occupancy as int8 (one byte per voxel), row-major (C) order.
    // int8 round-trips every VoxelOccupancy value, including PotentiallyOccupied (-3).
    std::vector<std::int8_t> data_;
    std::size_t x_size_ = 0;
    std::size_t y_size_ = 0;
    std::size_t z_size_ = 0;
};

} // namespace simulator
