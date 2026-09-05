#include <gtest/gtest.h>

#include <MissionControl/ScanResultToVoxels.h>

#include <Common/IMutableMap3D.h>
#include <Common/Units.h>

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <limits>
#include <map>
#include <tuple>

// Tests for mission_control_322526864_323020347::ScanResultToVoxels::applyToMap, pinning the
// behaviours the ex2 grading mutations flipped without being caught:
//   DRO11 — an out-of-bounds scan origin must write NOTHING;
//   DRO07 — evidence priority Occupied > Empty > PotentiallyOccupied (no downgrades);
//   DRO12 — the Occupied endpoint is written ONLY for a real hit (a miss produces no Occupied).
//
// Geometry: 10 cm voxels, scan origin at (5,5,5) = the centre of cell (0,0,0), drone heading
// {0°,0°} and a single beam at relative angle {0°,0°} → the beam runs straight along +X, so the
// sampled cells are exactly (0,0,0), (1,0,0), ... — deterministic assertions per cell index.
namespace {

using namespace common;

using mission_control_322526864_323020347::ScanResultToVoxels;

// A minimal std::map-backed mutable map (the TestMap pattern, plus set()). Cells outside the
// cubic world bounds [min_cm, max_cm) report OutOfBounds / isInBounds false; unset cells are
// Unmapped. Counts every set() call so "writes nothing" is directly observable.
class FakeMutableMap final : public IMutableMap3D {
public:
    FakeMutableMap(double res_cm, double min_cm, double max_cm)
        : res_cm_(res_cm), min_cm_(min_cm), max_cm_(max_cm) {}

    [[nodiscard]] common::types::VoxelOccupancy atVoxel(const Position3D& pos) const override {
        if (!isInBounds(pos)) return common::types::VoxelOccupancy::OutOfBounds;
        const auto it = data_.find(posToKey(pos));
        if (it == data_.end()) return common::types::VoxelOccupancy::Unmapped;
        return it->second;
    }

    [[nodiscard]] common::types::MapConfig getMapConfig() const override {
        return common::types::MapConfig{
            common::types::MappingBounds{
                min_cm_ * x_extent[cm], max_cm_ * x_extent[cm],
                min_cm_ * y_extent[cm], max_cm_ * y_extent[cm],
                min_cm_ * z_extent[cm], max_cm_ * z_extent[cm]},
            Position3D{},
            res_cm_ * cm};
    }

    [[nodiscard]] bool isInBounds(const Position3D& pos) const override {
        const double x = pos.x.numerical_value_in(cm);
        const double y = pos.y.numerical_value_in(cm);
        const double z = pos.z.numerical_value_in(cm);
        return x >= min_cm_ && x < max_cm_ &&
               y >= min_cm_ && y < max_cm_ &&
               z >= min_cm_ && z < max_cm_;
    }

    void set(const Position3D& pos, common::types::VoxelOccupancy value) override {
        ++set_calls_;
        data_[posToKey(pos)] = value;
    }

    void save(const std::filesystem::path&) const override {} // not needed by these tests

    // Test helpers ------------------------------------------------------------------------------

    // Occupancy of grid cell (ix, iy, iz); Unmapped when never written.
    [[nodiscard]] common::types::VoxelOccupancy occAtCell(int ix, int iy, int iz) const {
        const auto it = data_.find(Key{ix, iy, iz});
        if (it == data_.end()) return common::types::VoxelOccupancy::Unmapped;
        return it->second;
    }

    // Seeds a cell directly (bypasses the set() counter — test arrangement, not an observation).
    void seedCell(int ix, int iy, int iz, common::types::VoxelOccupancy value) {
        data_[Key{ix, iy, iz}] = value;
    }

    [[nodiscard]] std::size_t setCalls() const { return set_calls_; }

    // Number of cells currently holding the given occupancy — lets a test assert "no Occupied
    // anywhere", not just "not at the endpoint" (DRO12).
    [[nodiscard]] std::size_t countCellsWith(common::types::VoxelOccupancy value) const {
        std::size_t n = 0;
        for (const auto& [key, occ] : data_) {
            (void)key;
            if (occ == value) ++n;
        }
        return n;
    }

private:
    using Key = std::tuple<int, int, int>;

    Key posToKey(const Position3D& p) const {
        return {
            static_cast<int>(std::floor(p.x.numerical_value_in(cm) / res_cm_)),
            static_cast<int>(std::floor(p.y.numerical_value_in(cm) / res_cm_)),
            static_cast<int>(std::floor(p.z.numerical_value_in(cm) / res_cm_)),
        };
    }

    std::map<Key, common::types::VoxelOccupancy> data_;
    double res_cm_;
    double min_cm_;
    double max_cm_;
    std::size_t set_calls_ = 0;
};

Position3D posCm(double x, double y, double z) {
    return {x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}

// Beam direction is drone heading + relative hit angle; both {0,0} → straight +X.
Orientation levelHeading() {
    return {0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};
}

common::types::LidarHit hitAt(double distance_cm) {
    return {distance_cm * cm, Orientation{}};
}

common::types::LidarHit missHit() {
    return {std::numeric_limits<double>::max() * cm, Orientation{}};
}

common::types::LidarHit zeroDistanceHit() {
    return {0.0 * cm, Orientation{}};
}

// z_min 30 cm (PotentiallyOccupied near segment), z_max 50 cm (miss Empty range).
common::types::LidarConfigData lidarConfig(double z_min_cm = 30.0, double z_max_cm = 50.0) {
    return {z_min_cm * cm, z_max_cm * cm, 1.0 * cm, 1};
}

} // namespace

// DRO11: a scan whose origin is out of bounds cannot be attributed to map cells — applyToMap
// must return without a single set() call, even though the scan carries a normal hit that would
// otherwise write a whole Empty path plus an Occupied endpoint.
TEST(ScanResultToVoxels, OutOfBoundsOriginWritesNothing) {
    FakeMutableMap map{10.0, 0.0, 200.0};
    const common::types::LidarScanResult scan{hitAt(100.0)};

    ScanResultToVoxels::applyToMap(map, posCm(-50.0, 5.0, 5.0), levelHeading(), scan,
                                   lidarConfig());

    EXPECT_EQ(map.setCalls(), 0u) << "an out-of-bounds origin must not write any voxel";
}

// DRO07: conflicting observations resolve by evidence strength Occupied > Empty >
// PotentiallyOccupied — upgrades happen, downgrades never do.
TEST(ScanResultToVoxels, PriorityOrderOccupiedOverEmptyOverPotentially) {
    using V = common::types::VoxelOccupancy;
    FakeMutableMap map{10.0, 0.0, 200.0};
    // z_max 150 so a miss's Empty ray reaches past the endpoint cell of the earlier hit.
    const auto cfg = lidarConfig(30.0, 150.0);

    map.seedCell(2, 0, 0, V::PotentiallyOccupied); // on the Empty path of the coming hit
    map.seedCell(10, 0, 0, V::Empty);              // the coming hit's endpoint cell

    // A normal hit at 100 cm: Empty path through cell 2, Occupied endpoint at cell 10.
    ScanResultToVoxels::applyToMap(map, posCm(5, 5, 5), levelHeading(),
                                   common::types::LidarScanResult{hitAt(100.0)}, cfg);
    EXPECT_EQ(map.occAtCell(2, 0, 0), V::Empty)
        << "PotentiallyOccupied IS upgraded by an Empty observation";
    EXPECT_EQ(map.occAtCell(10, 0, 0), V::Occupied)
        << "a previously-Empty cell IS upgraded by an Occupied observation";

    // A miss: Empty along the ray out to z_max=150, sweeping over the Occupied cell 10.
    ScanResultToVoxels::applyToMap(map, posCm(5, 5, 5), levelHeading(),
                                   common::types::LidarScanResult{missHit()}, cfg);
    EXPECT_EQ(map.occAtCell(10, 0, 0), V::Occupied)
        << "an Occupied cell is NOT downgraded by an Empty observation";

    // A zero-distance hit: PotentiallyOccupied over the near segment (cells 0..3), which the
    // previous scans confirmed Empty.
    ScanResultToVoxels::applyToMap(map, posCm(5, 5, 5), levelHeading(),
                                   common::types::LidarScanResult{zeroDistanceHit()}, cfg);
    for (int ix = 0; ix <= 3; ++ix) {
        EXPECT_EQ(map.occAtCell(ix, 0, 0), V::Empty)
            << "an Empty cell is NOT downgraded to PotentiallyOccupied (cell " << ix << ")";
    }
}

// A miss (max-double distance) marks the ray Empty out to z_max — and produces NO Occupied
// anywhere (DRO12: the endpoint write is exclusive to real hits).
TEST(ScanResultToVoxels, MissMarksEmptyToZMax) {
    using V = common::types::VoxelOccupancy;
    FakeMutableMap map{10.0, 0.0, 200.0};

    ScanResultToVoxels::applyToMap(map, posCm(5, 5, 5), levelHeading(),
                                   common::types::LidarScanResult{missHit()}, lidarConfig());

    // z_max 50: samples run x = 5..55 → cells 0..5 Empty; cell 6 untouched.
    for (int ix = 0; ix <= 5; ++ix) {
        EXPECT_EQ(map.occAtCell(ix, 0, 0), V::Empty) << "cell " << ix << " on the missed ray";
    }
    EXPECT_EQ(map.occAtCell(6, 0, 0), V::Unmapped) << "beyond z_max stays unmapped";
    EXPECT_EQ(map.countCellsWith(V::Occupied), 0u)
        << "a miss must not mark ANY cell Occupied (DRO12)";
}

// A zero-distance hit (obstacle closer than z_min, exact voxel unknown) marks the near segment
// PotentiallyOccupied out to z_min — and nothing Occupied.
TEST(ScanResultToVoxels, ZeroDistanceMarksPotentiallyOccupiedToZMin) {
    using V = common::types::VoxelOccupancy;
    FakeMutableMap map{10.0, 0.0, 200.0};

    ScanResultToVoxels::applyToMap(map, posCm(5, 5, 5), levelHeading(),
                                   common::types::LidarScanResult{zeroDistanceHit()},
                                   lidarConfig());

    // z_min 30: samples run x = 5..35 → cells 0..3 PotentiallyOccupied; cell 4 untouched.
    for (int ix = 0; ix <= 3; ++ix) {
        EXPECT_EQ(map.occAtCell(ix, 0, 0), V::PotentiallyOccupied)
            << "cell " << ix << " in the blind near segment";
    }
    EXPECT_EQ(map.occAtCell(4, 0, 0), V::Unmapped) << "beyond z_min stays unmapped";
    EXPECT_EQ(map.countCellsWith(V::Occupied), 0u)
        << "a zero-distance hit must not mark any cell Occupied";
}

// A normal hit empties the path up to the hit and marks exactly the endpoint cell Occupied.
TEST(ScanResultToVoxels, NormalHitMarksEndpointOccupied) {
    using V = common::types::VoxelOccupancy;
    FakeMutableMap map{10.0, 0.0, 200.0};

    ScanResultToVoxels::applyToMap(map, posCm(5, 5, 5), levelHeading(),
                                   common::types::LidarScanResult{hitAt(100.0)},
                                   lidarConfig());

    // Hit at 100 cm from (5,5,5) along +X → endpoint (105,5,5) = cell 10.
    for (int ix = 0; ix <= 9; ++ix) {
        EXPECT_EQ(map.occAtCell(ix, 0, 0), V::Empty) << "path cell " << ix << " before the hit";
    }
    EXPECT_EQ(map.occAtCell(10, 0, 0), V::Occupied) << "the hit endpoint cell";
    EXPECT_EQ(map.countCellsWith(V::Occupied), 1u)
        << "exactly the endpoint is Occupied — nowhere else";
}
