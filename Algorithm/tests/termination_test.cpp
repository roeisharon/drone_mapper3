// Termination guarantees under adversarial/lying maps — the regression suite for ex2 grading bug
// ALG28 (a navigability predicate forced to `true` unbounded the BFS and timed out the component
// suite) and ALG29 (everything blocked). nextStep() must return, promptly, no matter what the map
// claims.

#include <Algorithm/MappingAlgorithmImpl.h>

#include <gtest/gtest.h>

#include <chrono>

namespace {

using algorithm_322526864_323020347::MappingAlgorithmImpl;
using namespace common; // Units symbols; data types stay fully qualified

// A map that reports a FIXED occupancy for every position — including positions far outside its
// declared boundaries (a "lying" map: it never returns OutOfBounds, which is exactly the ALG28
// failure surface).
class UniformLyingMap final : public common::IMap3D {
public:
    UniformLyingMap(common::types::VoxelOccupancy value, double extent_cm)
        : value_(value) {
        config_.resolution = 10.0 * cm;
        config_.boundaries = {0.0 * x_extent[cm], extent_cm * x_extent[cm],
                              0.0 * y_extent[cm], extent_cm * y_extent[cm],
                              0.0 * z_extent[cm], extent_cm * z_extent[cm]};
    }

    [[nodiscard]] common::types::VoxelOccupancy atVoxel(const Position3D&) const override {
        return value_;
    }
    [[nodiscard]] common::types::MapConfig getMapConfig() const override { return config_; }
    [[nodiscard]] bool isInBounds(const Position3D&) const override { return true; } // lies

private:
    common::types::VoxelOccupancy value_;
    common::types::MapConfig config_{};
};

common::types::DroneState centerState() {
    return common::types::DroneState{
        {15.0 * x_extent[cm], 15.0 * y_extent[cm], 15.0 * z_extent[cm]},
        {0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
        0};
}

common::types::MappingStepCommand stepOnce(const common::IMap3D& map, double& elapsed_seconds) {
    const common::types::MissionConfigData mission{};
    const common::types::LidarConfigData lidar{};
    const common::types::DroneConfigData drone{}; // point drone
    MappingAlgorithmImpl algorithm{
        common::MappingAlgorithmDependencies{mission, lidar, drone, map}};

    const auto start = std::chrono::steady_clock::now();
    const auto command = algorithm.nextStep(centerState(), nullptr);
    elapsed_seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    return command;
}

TEST(MappingAlgorithmTermination, TerminationAllEmptyLyingMapFinishes) {
    // ALG28 analog with sane declared bounds: every voxel reads Empty everywhere (never
    // OutOfBounds), so ex2's only BFS stop is gone. The structural cell box must bound the search:
    // 30x30x30 cells → exhaustive sweep, then Finished, fast.
    const UniformLyingMap map{common::types::VoxelOccupancy::Empty, 300.0};
    double elapsed = 0.0;
    const auto command = stepOnce(map, elapsed);
    EXPECT_EQ(command.status, common::types::AlgorithmStatus::Finished);
    EXPECT_LT(elapsed, 10.0);
}

TEST(MappingAlgorithmTermination, TerminationDegenerateBoundsHitsExpansionCap) {
    // The worst case: the map also declares NO usable boundaries (zero box), so the structural
    // bound falls back to a ±10k-cell box and the 1M-expansion cap is the active guarantee. This
    // is the direct regression test for the ex2 component-suite timeout.
    const UniformLyingMap map{common::types::VoxelOccupancy::Empty, 0.0};
    double elapsed = 0.0;
    const auto command = stepOnce(map, elapsed);
    EXPECT_EQ(command.status, common::types::AlgorithmStatus::Finished);
    EXPECT_LT(elapsed, 120.0); // generous for an unoptimized build; the point is: it RETURNS
}

TEST(MappingAlgorithmTermination, TerminationAllOccupiedFinishesImmediately) {
    // ALG29 analog: everything reads Occupied → nothing is navigable → the frontier is empty on
    // the first expansion round → immediate Finished.
    const UniformLyingMap map{common::types::VoxelOccupancy::Occupied, 300.0};
    double elapsed = 0.0;
    const auto command = stepOnce(map, elapsed);
    EXPECT_EQ(command.status, common::types::AlgorithmStatus::Finished);
    EXPECT_LT(elapsed, 5.0);
}

TEST(MappingAlgorithmTermination, TerminationAllUnmappedProducesWorkPromptly) {
    // Sanity inverse: a fully Unmapped (but bounded) world is all frontier — the algorithm must
    // return a Working command quickly, not get lost.
    const UniformLyingMap map{common::types::VoxelOccupancy::Unmapped, 300.0};
    double elapsed = 0.0;
    const auto command = stepOnce(map, elapsed);
    EXPECT_EQ(command.status, common::types::AlgorithmStatus::Working);
    EXPECT_TRUE(command.movement.has_value() || command.scan_orientation.has_value());
    EXPECT_LT(elapsed, 5.0);
}

TEST(MappingAlgorithmTermination, TerminationRepeatedCallsStayBounded) {
    // A short mission-style loop against the lying all-Empty map: every call must keep returning
    // Finished without accumulating state that could unbound later searches.
    const UniformLyingMap map{common::types::VoxelOccupancy::Empty, 300.0};
    const common::types::MissionConfigData mission{};
    const common::types::LidarConfigData lidar{};
    const common::types::DroneConfigData drone{};
    MappingAlgorithmImpl algorithm{
        common::MappingAlgorithmDependencies{mission, lidar, drone, map}};

    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < 5; ++i) {
        const auto command = algorithm.nextStep(centerState(), nullptr);
        EXPECT_EQ(command.status, common::types::AlgorithmStatus::Finished);
    }
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    EXPECT_LT(elapsed, 30.0);
}

} // namespace
