#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <MissionControl/DroneControlImpl.h>
#include <MissionControl/FatalCollisionError.h>

#include "mocks/CommonMocks.h"

#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using common::cm;
using common::deg;

using mission_control_322526864_323020347::DroneControlImpl;
using mission_control_322526864_323020347::FatalCollisionError;

// Matches a PhysicalLength equal to expected_cm (within a tiny tolerance).
MATCHER_P(CmEq, expected_cm, "physical length == " + std::to_string(expected_cm) + " cm") {
    return std::abs(arg.numerical_value_in(cm) - expected_cm) <= 1e-6;
}

// Matches a HorizontalAngle equal to expected_deg.
MATCHER_P(DegEq, expected_deg, "angle == " + std::to_string(expected_deg) + " deg") {
    return std::abs(arg.numerical_value_in(deg) - expected_deg) <= 1e-6;
}

// Drone config with the per-command movement limits the clamping tests rely on.
common::types::DroneConfigData makeConfig(double max_rotate_deg  = 45.0,
                                          double max_advance_cm = 50.0,
                                          double max_elevate_cm = 40.0) {
    common::types::DroneConfigData cfg;
    cfg.radius      = 30.0 * cm;
    cfg.max_rotate  = max_rotate_deg * deg;
    cfg.max_advance = max_advance_cm * cm;
    cfg.max_elevate = max_elevate_cm * cm;
    return cfg;
}

// A MapConfig with non-zero resolution so ScanResultToVoxels::applyToMap actually walks the
// beam (it sub-steps at 0.1×resolution and bails if resolution is 0).
common::types::MapConfig mapConfig10() {
    using namespace common;
    return types::MapConfig{
        types::MappingBounds{0.0 * x_extent[cm], 200.0 * x_extent[cm],
                             0.0 * y_extent[cm], 200.0 * y_extent[cm],
                             0.0 * z_extent[cm], 200.0 * z_extent[cm]},
        Position3D{}, 10.0 * cm};
}

common::Orientation forwardOrientation() {
    return {0.0 * common::horizontal_angle[common::deg],
            0.0 * common::altitude_angle[common::deg]};
}

// A Working command carrying a movement and (optionally) a forward scan request.
common::types::MappingStepCommand workingCmd(common::types::MovementCommand mv,
                                             bool with_scan = true) {
    return {mv,
            with_scan ? std::optional<common::Orientation>{forwardOrientation()}
                      : std::optional<common::Orientation>{std::nullopt},
            common::types::AlgorithmStatus::Working};
}

common::types::MappingStepCommand advanceStepCmd(double dist_cm, bool with_scan = true) {
    return workingCmd({common::types::MovementCommandType::Advance,
                       common::types::RotationDirection::Left, 0.0 * deg, dist_cm * cm},
                      with_scan);
}

common::types::MappingStepCommand rotateStepCmd(common::types::RotationDirection dir,
                                                double angle_deg, bool with_scan = true) {
    return workingCmd({common::types::MovementCommandType::Rotate,
                       dir, angle_deg * deg, 0.0 * cm}, with_scan);
}

common::types::MappingStepCommand elevateStepCmd(double dist_cm, bool with_scan = true) {
    return workingCmd({common::types::MovementCommandType::Elevate,
                       common::types::RotationDirection::Left, 0.0 * deg, dist_cm * cm},
                      with_scan);
}

common::types::MappingStepCommand finishedCmd() {
    return {std::nullopt, std::nullopt, common::types::AlgorithmStatus::Finished};
}

common::types::MappingStepCommand finishedWithUnmappableCmd() {
    return {std::nullopt, std::nullopt,
            common::types::AlgorithmStatus::FinishedWithUnmappableVoxels};
}

common::types::MappingStepCommand hoverWorkingCmd() {
    return {common::types::MovementCommand{common::types::MovementCommandType::Hover},
            forwardOrientation(), common::types::AlgorithmStatus::Working};
}

// A healthy single-hit scan: the simulator's lidar always reports at least the center beam, so
// the fixture default is non-empty; row-6 (empty = malfunction) tests override it explicitly.
common::types::LidarScanResult oneHitScan(double distance_cm = 5.0) {
    common::types::LidarHit hit;
    hit.distance = distance_cm * cm;
    hit.angle    = common::Orientation{};
    return {hit};
}

common::Position3D posCm(double x, double y, double z) {
    return {x * common::x_extent[cm], y * common::y_extent[cm], z * common::z_extent[cm]};
}

// Mission bounds x∈[0,100], y/z∈[0,200] — smaller than the map on x so row-10 amendment has
// something to amend against. Degenerate (all-zero) bounds mean "no mission bounds".
common::types::MissionConfigData missionWithBounds() {
    common::types::MissionConfigData m;
    m.mission_bounds = common::types::MappingBounds{
        0.0 * common::x_extent[cm], 100.0 * common::x_extent[cm],
        0.0 * common::y_extent[cm], 200.0 * common::y_extent[cm],
        0.0 * common::z_extent[cm], 200.0 * common::z_extent[cm]};
    return m;
}

} // namespace

class DroneControl : public ::testing::Test {
protected:
    void SetUp() override {
        using namespace ::testing;
        using common::types::VoxelOccupancy;
        using common::types::MovementResult;
        using common::Position3D;
        using common::Orientation;

        ON_CALL(gps_, position()).WillByDefault(Return(Position3D{}));  // origin
        ON_CALL(gps_, heading()).WillByDefault(Return(Orientation{}));  // facing east (0°)

        // Output map: every cell Unmapped (not Occupied), in-bounds, with a real resolution so
        // applyToMap proceeds. isInBounds is mocked true so the converter does not early-return.
        ON_CALL(output_map_, atVoxel(_)).WillByDefault(Return(VoxelOccupancy::Unmapped));
        ON_CALL(output_map_, isInBounds(_)).WillByDefault(Return(true));
        ON_CALL(output_map_, getMapConfig()).WillByDefault(Return(mapConfig10()));

        ON_CALL(lidar_, scan(_)).WillByDefault(Return(oneHitScan()));

        ON_CALL(movement_, rotate(_, _)).WillByDefault(Return(MovementResult{true, {}}));
        ON_CALL(movement_, advance(_)).WillByDefault(Return(MovementResult{true, {}}));
        ON_CALL(movement_, elevate(_)).WillByDefault(Return(MovementResult{true, {}}));

        ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(advanceStepCmd(10.0)));

        drone_ = std::make_unique<DroneControlImpl>(
            makeConfig(), common::types::MissionConfigData{},
            lidar_, gps_, movement_, output_map_, *algo_);
    }

    ::testing::NiceMock<mc_tests::MockILidar>         lidar_;
    ::testing::NiceMock<mc_tests::MockIGPS>           gps_;
    ::testing::NiceMock<mc_tests::MockIDroneMovement> movement_;
    ::testing::NiceMock<mc_tests::MockIMutableMap3D>  output_map_;
    // Configs the algorithm mock's base class binds to (declared before the mock itself).
    common::types::MissionConfigData algo_mission_cfg_{};
    common::types::LidarConfigData   algo_lidar_cfg_{};
    common::types::DroneConfigData   algo_drone_cfg_{};
    ::testing::NiceMock<mc_tests::MockIMappingAlgorithm> algo_raw_{
        common::MappingAlgorithmDependencies{algo_mission_cfg_, algo_lidar_cfg_,
                                             algo_drone_cfg_, output_map_}};
    mc_tests::MockIMappingAlgorithm* algo_{&algo_raw_};
    std::unique_ptr<DroneControlImpl> drone_;
};

// Step status

// A normal working step returns Continue.
TEST_F(DroneControl, StepReturnsContinueOnNormalStep) {
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Continue);
}

// AlgorithmStatus::Finished → DroneStepStatus::Completed.
TEST_F(DroneControl, StepReturnsCompletedWhenAlgorithmFinished) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(finishedCmd()));
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Completed);
}

// FinishedWithUnmappableVoxels also maps to Completed (both finish statuses are terminal-success).
TEST_F(DroneControl, StepReturnsCompletedWhenAlgorithmFinishedWithUnmappable) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(finishedWithUnmappableCmd()));
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Completed);
}

// A failed advance surfaces as a step Error carrying the movement message.
TEST_F(DroneControl, StepReturnsErrorWhenMovementFails) {
    using namespace ::testing;
    ON_CALL(movement_, advance(_))
        .WillByDefault(Return(common::types::MovementResult{false, "motor_stall"}));
    const auto result = drone_->step();
    EXPECT_EQ(result.status, common::types::DroneStepStatus::Error);
    EXPECT_EQ(result.message, "motor_stall");
}

// A failed rotate surfaces as a step Error.
TEST_F(DroneControl, StepReturnsErrorWhenRotateFails) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _))
        .WillByDefault(Return(rotateStepCmd(common::types::RotationDirection::Left, 10.0)));
    ON_CALL(movement_, rotate(_, _))
        .WillByDefault(Return(common::types::MovementResult{false, "servo_error"}));
    const auto result = drone_->step();
    EXPECT_EQ(result.status, common::types::DroneStepStatus::Error);
    EXPECT_EQ(result.message, "servo_error");
}

// A failed elevate surfaces as a step Error.
TEST_F(DroneControl, StepReturnsErrorWhenElevateFails) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(elevateStepCmd(10.0)));
    ON_CALL(movement_, elevate(_))
        .WillByDefault(Return(common::types::MovementResult{false, "prop_fault"}));
    const auto result = drone_->step();
    EXPECT_EQ(result.status, common::types::DroneStepStatus::Error);
    EXPECT_EQ(result.message, "prop_fault");
}

// A collision refusal (DRONE_HITS_OBSTACLE) is routine exploration, not a fatal fault: the step
// continues so the planner can reroute. This is what distinguishes it from a genuine movement fault
// (motor_stall etc.), which still errors above.
TEST_F(DroneControl, StepContinuesWhenMovementRefusedByCollision) {
    using namespace ::testing;
    ON_CALL(movement_, advance(_))
        .WillByDefault(Return(common::types::MovementResult{false, "DRONE_HITS_OBSTACLE"}));
    const auto result = drone_->step();
    // Collision PREVENTION: the move was refused before entering the obstacle, so the drone stays put
    // and exploration continues. Non-fatal, no error message — distinct from an actual collision.
    EXPECT_EQ(result.status, common::types::DroneStepStatus::Continue);
    EXPECT_TRUE(result.message.empty());
}

// Obstacle detection

// Current cell actually Occupied → a real collision → fatal DRONE_COLLISION error.
TEST_F(DroneControl, StepReturnsCollisionErrorWhenCurrentPositionIsOccupied) {
    using namespace ::testing;
    ON_CALL(output_map_, atVoxel(_)).WillByDefault(Return(common::types::VoxelOccupancy::Occupied));
    const auto result = drone_->step();
    EXPECT_EQ(result.status, common::types::DroneStepStatus::Error);
    EXPECT_EQ(result.message, "DRONE_COLLISION");
}

// The obstacle early-return does not advance the step index.
TEST_F(DroneControl, ObstacleErrorDoesNotIncrementStepIndex) {
    using namespace ::testing;
    ON_CALL(output_map_, atVoxel(_)).WillByDefault(Return(common::types::VoxelOccupancy::Occupied));
    EXPECT_EQ(drone_->state().step_index, 0u);
    (void)drone_->step();
    EXPECT_EQ(drone_->state().step_index, 0u);
}

// The obstacle early-return happens before any lidar/algorithm call.
TEST_F(DroneControl, ObstacleErrorDoesNotCallLidarOrAlgorithm) {
    using namespace ::testing;
    ON_CALL(output_map_, atVoxel(_)).WillByDefault(Return(common::types::VoxelOccupancy::Occupied));
    EXPECT_CALL(lidar_, scan(_)).Times(0);
    EXPECT_CALL(*algo_, nextStep(_, _)).Times(0);
    (void)drone_->step();
}

// The current (provably free) cell is marked Empty via set() before anything else writes.
TEST_F(DroneControl, StepMarksCurrentCellEmptyInOutputMap) {
    using namespace ::testing;
    using common::types::VoxelOccupancy;
    using common::Position3D;
    const Position3D here{1.0 * common::x_extent[cm], 2.0 * common::y_extent[cm],
                          3.0 * common::z_extent[cm]};
    ON_CALL(gps_, position()).WillByDefault(Return(here));
    // Permit the scan's own beam writes; require the current cell's Empty mark specifically.
    EXPECT_CALL(output_map_, set(_, _)).Times(AnyNumber());
    EXPECT_CALL(output_map_, set(Truly([](const Position3D& p) {
                                     return p.x.numerical_value_in(cm) == 1.0 &&
                                            p.y.numerical_value_in(cm) == 2.0 &&
                                            p.z.numerical_value_in(cm) == 3.0;
                                 }),
                                 VoxelOccupancy::Empty)).Times(AtLeast(1));
    (void)drone_->step();
}

// Algorithm / lidar interaction

// The algorithm is consulted exactly once per step.
TEST_F(DroneControl, StepCallsAlgorithmBeforeLidar) {
    using namespace ::testing;
    EXPECT_CALL(*algo_, nextStep(_, _)).Times(1).WillOnce(Return(advanceStepCmd(10.0)));
    (void)drone_->step();
}

// No scan_orientation in the command → lidar is not called.
TEST_F(DroneControl, StepCallsLidarOnlyWhenScanOrientationRequested) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(advanceStepCmd(10.0, /*with_scan=*/false)));
    EXPECT_CALL(lidar_, scan(_)).Times(0);
    (void)drone_->step();
}

// The lidar is scanned with the orientation the algorithm requested.
TEST_F(DroneControl, StepCallsLidarScanWithAlgorithmOrientation) {
    using namespace ::testing;
    EXPECT_CALL(lidar_, scan(Truly([](const common::Orientation& o) {
        return o.horizontal.numerical_value_in(deg) == 0.0;
    }))).Times(1);
    (void)drone_->step();
}

// A lidar hit is written into the output map as Occupied (via ScanResultToVoxels::applyToMap).
TEST_F(DroneControl, StepWritesOccupiedToOutputMapWhenLidarHits) {
    using namespace ::testing;
    common::types::LidarHit hit;
    hit.distance = 5.0 * cm;
    hit.angle    = common::Orientation{};
    ON_CALL(lidar_, scan(_)).WillByDefault(Return(common::types::LidarScanResult{hit}));
    // Permit the Empty path-clearing writes; require at least one Occupied (the hit endpoint).
    // The catch-all is declared first so the specific Occupied expectation is matched first.
    EXPECT_CALL(output_map_, set(_, _)).Times(AnyNumber());
    EXPECT_CALL(output_map_, set(_, common::types::VoxelOccupancy::Occupied)).Times(AtLeast(1));
    (void)drone_->step();
}

// An empty scan writes no Occupied voxels — only the drone's own cell is recorded Empty.
TEST_F(DroneControl, StepWithEmptyScanWritesNoOccupied) {
    using namespace ::testing;
    using common::types::VoxelOccupancy;
    ON_CALL(lidar_, scan(_)).WillByDefault(Return(common::types::LidarScanResult{}));
    EXPECT_CALL(output_map_, set(_, VoxelOccupancy::Empty)).Times(AtLeast(1));
    EXPECT_CALL(output_map_, set(_, VoxelOccupancy::Occupied)).Times(0);
    (void)drone_->step();
}

// The lidar's own config() (ILidar::config()) reaches applyToMap: with a large z_max a miss empties
// a long ray (many Empty writes), which would not happen with the default zero-config. Confirms
// step() reads the config from the injected lidar rather than a stale/zero default.
TEST_F(DroneControl, LidarConfigFromLidarIsUsedByScanConversion) {
    using namespace ::testing;
    using common::types::VoxelOccupancy;
    ON_CALL(lidar_, config())
        .WillByDefault(Return(common::types::LidarConfigData{0.0 * cm, 50.0 * cm, 1.0 * cm, 1}));
    // A miss: max-double distance → applyToMap empties the ray out to z_max (50 cm at 1 cm steps).
    common::types::LidarHit miss;
    miss.distance = std::numeric_limits<double>::max() * cm;
    ON_CALL(lidar_, scan(_)).WillByDefault(Return(common::types::LidarScanResult{miss}));
    EXPECT_CALL(output_map_, set(_, VoxelOccupancy::Empty)).Times(AtLeast(10));
    (void)drone_->step();
}

// latest_scan threading

// The first step passes nullptr as latest_scan (no prior scan exists).
TEST_F(DroneControl, FirstCallPassesNullScanToAlgorithm) {
    using namespace ::testing;
    const common::types::LidarScanResult* captured =
        reinterpret_cast<const common::types::LidarScanResult*>(0x1);
    ON_CALL(*algo_, nextStep(_, _))
        .WillByDefault([&](const common::types::DroneState&,
                           const common::types::LidarScanResult* scan) {
            captured = scan;
            return finishedCmd();
        });
    (void)drone_->step();
    EXPECT_EQ(captured, nullptr);
}

// The second step passes the scan captured on the first step (non-null).
TEST_F(DroneControl, SecondCallPassesScanResultFromFirstScan) {
    using namespace ::testing;
    common::types::LidarHit hit;
    hit.distance = 5.0 * cm;
    ON_CALL(lidar_, scan(_)).WillByDefault(Return(common::types::LidarScanResult{hit}));
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(advanceStepCmd(10.0)));
    (void)drone_->step(); // first step scans → latest_scan stored

    const common::types::LidarScanResult* second = nullptr;
    EXPECT_CALL(*algo_, nextStep(_, _))
        .WillOnce([&](const common::types::DroneState&,
                      const common::types::LidarScanResult* p) {
            second = p;
            return finishedCmd();
        });
    (void)drone_->step();
    EXPECT_NE(second, nullptr);
}

// Ex3 (ex2-grading missed bug DRO10): the stored scan PERSISTS across scanless steps — a step that
// commands no scan hands the algorithm the previous scan, not nullptr.
TEST_F(DroneControl, LatestScanPersistsAcrossScanlessSteps) {
    using namespace ::testing;
    common::types::LidarHit hit;
    hit.distance = 5.0 * cm;
    hit.angle    = common::Orientation{};
    ON_CALL(lidar_, scan(_)).WillByDefault(Return(common::types::LidarScanResult{hit}));

    const common::types::LidarScanResult* second = nullptr;
    const common::types::LidarScanResult* third  = nullptr;
    EXPECT_CALL(*algo_, nextStep(_, _))
        // Step 1 commands a scan → the 1-hit result is stored.
        .WillOnce(Return(advanceStepCmd(10.0, /*with_scan=*/true)))
        // Step 2 receives that scan and commands NO scan.
        .WillOnce([&](const common::types::DroneState&,
                      const common::types::LidarScanResult* p) {
            second = p;
            return advanceStepCmd(10.0, /*with_scan=*/false);
        })
        // Step 3 must STILL receive the step-1 scan (not nullptr).
        .WillOnce([&](const common::types::DroneState&,
                      const common::types::LidarScanResult* p) {
            third = p;
            return finishedCmd();
        });
    (void)drone_->step();
    (void)drone_->step();
    (void)drone_->step();

    ASSERT_NE(second, nullptr);
    ASSERT_EQ(second->size(), 1u);
    EXPECT_NEAR((*second)[0].distance.numerical_value_in(cm), 5.0, 1e-9);
    ASSERT_NE(third, nullptr) << "a scanless step must not clear the stored scan";
    ASSERT_EQ(third->size(), 1u);
    EXPECT_NEAR((*third)[0].distance.numerical_value_in(cm), 5.0, 1e-9);
}

// Clamping

TEST_F(DroneControl, AdvanceNotClampedWhenBelowMax) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(advanceStepCmd(30.0)));
    EXPECT_CALL(movement_, advance(CmEq(30.0))).Times(1);
    (void)drone_->step();
}

TEST_F(DroneControl, AdvanceClampedWhenAboveMax) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(advanceStepCmd(200.0)));
    EXPECT_CALL(movement_, advance(CmEq(50.0))).Times(1);
    (void)drone_->step();
}

TEST_F(DroneControl, AdvanceAtExactMaxNotClamped) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(advanceStepCmd(50.0)));
    EXPECT_CALL(movement_, advance(CmEq(50.0))).Times(1);
    (void)drone_->step();
}

TEST_F(DroneControl, RotateNotClampedWhenBelowMax) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _))
        .WillByDefault(Return(rotateStepCmd(common::types::RotationDirection::Left, 20.0)));
    EXPECT_CALL(movement_, rotate(common::types::RotationDirection::Left, DegEq(20.0))).Times(1);
    (void)drone_->step();
}

TEST_F(DroneControl, RotateClampedWhenAboveMax) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _))
        .WillByDefault(Return(rotateStepCmd(common::types::RotationDirection::Right, 90.0)));
    EXPECT_CALL(movement_, rotate(common::types::RotationDirection::Right, DegEq(45.0))).Times(1);
    (void)drone_->step();
}

TEST_F(DroneControl, ElevateNotClampedWhenBelowMax) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(elevateStepCmd(20.0)));
    EXPECT_CALL(movement_, elevate(CmEq(20.0))).Times(1);
    (void)drone_->step();
}

TEST_F(DroneControl, ElevateClampedWhenAboveMax) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(elevateStepCmd(80.0)));
    EXPECT_CALL(movement_, elevate(CmEq(40.0))).Times(1);
    (void)drone_->step();
}

// A sub-cell request is floored to whole cells (never rounded up), so it never exceeds the requested
// magnitude, and the descent sign is preserved: -15cm at 10cm resolution → 1 cell = -10cm (not -20).
TEST_F(DroneControl, ElevateNegativeSignPreserved) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(elevateStepCmd(-15.0)));
    EXPECT_CALL(movement_, elevate(CmEq(-10.0))).Times(1);
    (void)drone_->step();
}

// floor (not round): a request between cell multiples is truncated DOWN so the executed move never
// exceeds the requested/validated distance. 25cm at 10cm resolution → 2 cells = 20cm (round would give 30).
TEST_F(DroneControl, AdvanceFlooredToWholeCellsNeverExceedsRequest) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(advanceStepCmd(25.0)));
    EXPECT_CALL(movement_, advance(CmEq(20.0))).Times(1);
    (void)drone_->step();
}

TEST_F(DroneControl, ElevateNegativeClampedSignPreserved) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(elevateStepCmd(-80.0)));
    EXPECT_CALL(movement_, elevate(CmEq(-40.0))).Times(1);
    (void)drone_->step();
}

// Translations are clamped to WHOLE cells so they land on cell centres even when the per-command
// limit is not a multiple of the resolution. With max_advance 25cm and res 10cm, the largest
// whole-cell move is 2 cells = 20cm (a continuous clamp would have allowed sub-cell 25).
TEST_F(DroneControl, AdvanceClampedToWholeCellsWhenMaxNotResolutionMultiple) {
    using namespace ::testing;
    drone_ = std::make_unique<DroneControlImpl>(
        makeConfig(45.0, 25.0), common::types::MissionConfigData{},
        lidar_, gps_, movement_, output_map_, *algo_);
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(advanceStepCmd(200.0)));
    EXPECT_CALL(movement_, advance(CmEq(20.0))).Times(1);
    (void)drone_->step();
}

// Same for elevate: max_elevate 25cm, res 10cm → 2 whole cells = 20cm.
TEST_F(DroneControl, ElevateClampedToWholeCellsWhenMaxNotResolutionMultiple) {
    using namespace ::testing;
    drone_ = std::make_unique<DroneControlImpl>(
        makeConfig(45.0, 50.0, 25.0), common::types::MissionConfigData{},
        lidar_, gps_, movement_, output_map_, *algo_);
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(elevateStepCmd(200.0)));
    EXPECT_CALL(movement_, elevate(CmEq(20.0))).Times(1);
    (void)drone_->step();
}

// No-movement commands

// A Hover command does not invoke any movement API and still returns Continue.
TEST_F(DroneControl, HoverMovementDoesNotCallMovementAPI) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(hoverWorkingCmd()));
    EXPECT_CALL(movement_, advance(_)).Times(0);
    EXPECT_CALL(movement_, rotate(_, _)).Times(0);
    EXPECT_CALL(movement_, elevate(_)).Times(0);
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Continue);
}

// A command with no movement field does not invoke any movement API.
TEST_F(DroneControl, CommandWithNoMovementDoesNotCallMovementAPI) {
    using namespace ::testing;
    const common::types::MappingStepCommand no_move{
        std::nullopt, forwardOrientation(), common::types::AlgorithmStatus::Working};
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(no_move));
    EXPECT_CALL(movement_, advance(_)).Times(0);
    EXPECT_CALL(movement_, rotate(_, _)).Times(0);
    EXPECT_CALL(movement_, elevate(_)).Times(0);
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Continue);
}

// Step index tracking

TEST_F(DroneControl, StepIndexStartsAtZero) {
    EXPECT_EQ(drone_->state().step_index, 0u);
}

TEST_F(DroneControl, StepIndexIncrementsAfterContinue) {
    (void)drone_->step();
    EXPECT_EQ(drone_->state().step_index, 1u);
}

TEST_F(DroneControl, StepIndexIncrementsAfterCompleted) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(finishedCmd()));
    (void)drone_->step();
    EXPECT_EQ(drone_->state().step_index, 1u);
}

TEST_F(DroneControl, StepIndexIncrementsEvenOnMovementFailure) {
    using namespace ::testing;
    ON_CALL(movement_, advance(_))
        .WillByDefault(Return(common::types::MovementResult{false, "err"}));
    (void)drone_->step();
    EXPECT_EQ(drone_->state().step_index, 1u);
}

TEST_F(DroneControl, StepIndexAccumulatesAcrossMultipleCalls) {
    (void)drone_->step();
    (void)drone_->step();
    (void)drone_->step();
    EXPECT_EQ(drone_->state().step_index, 3u);
}

// state()

TEST_F(DroneControl, StateReflectsCurrentGpsPosition) {
    using namespace ::testing;
    const common::Position3D expected{7.0 * common::x_extent[cm],
                                      3.0 * common::y_extent[cm],
                                      1.0 * common::z_extent[cm]};
    ON_CALL(gps_, position()).WillByDefault(Return(expected));
    const auto s = drone_->state();
    EXPECT_DOUBLE_EQ(s.position.x.numerical_value_in(cm), 7.0);
    EXPECT_DOUBLE_EQ(s.position.y.numerical_value_in(cm), 3.0);
    EXPECT_DOUBLE_EQ(s.position.z.numerical_value_in(cm), 1.0);
}

TEST_F(DroneControl, StateReflectsCurrentGpsHeading) {
    using namespace ::testing;
    const common::Orientation expected{30.0 * deg, {}};
    ON_CALL(gps_, heading()).WillByDefault(Return(expected));
    EXPECT_DOUBLE_EQ(drone_->state().heading.horizontal.numerical_value_in(deg), 30.0);
}

TEST_F(DroneControl, StateStepIndexMatchesCallCount) {
    (void)drone_->step();
    (void)drone_->step();
    EXPECT_EQ(drone_->state().step_index, 2u);
}

// Step index / state passed to the algorithm

TEST_F(DroneControl, NextStepReceivesStepIndexZeroOnFirstCall) {
    using namespace ::testing;
    EXPECT_CALL(*algo_, nextStep(Field(&common::types::DroneState::step_index, 0u), _))
        .Times(1).WillOnce(Return(finishedCmd()));
    (void)drone_->step();
}

TEST_F(DroneControl, NextStepReceivesIncrementedStepIndexOnLaterCalls) {
    using namespace ::testing;
    common::types::DroneState third_call_state;
    EXPECT_CALL(*algo_, nextStep(_, _))
        .WillOnce(Return(advanceStepCmd(10.0)))
        .WillOnce(Return(advanceStepCmd(10.0)))
        .WillOnce(DoAll(SaveArg<0>(&third_call_state), Return(finishedCmd())));
    (void)drone_->step();
    (void)drone_->step();
    (void)drone_->step();
    EXPECT_EQ(third_call_state.step_index, 2u);
}

TEST_F(DroneControl, NextStepReceivesCurrentPosition) {
    using namespace ::testing;
    const common::Position3D test_pos{5.0 * common::x_extent[cm],
                                      3.0 * common::y_extent[cm],
                                      2.0 * common::z_extent[cm]};
    ON_CALL(gps_, position()).WillByDefault(Return(test_pos));
    EXPECT_CALL(*algo_, nextStep(Truly([](const common::types::DroneState& s) {
                                     return std::abs(s.position.x.numerical_value_in(cm) - 5.0) < 1e-9 &&
                                            std::abs(s.position.y.numerical_value_in(cm) - 3.0) < 1e-9 &&
                                            std::abs(s.position.z.numerical_value_in(cm) - 2.0) < 1e-9;
                                 }),
                                 _)).Times(1).WillOnce(Return(finishedCmd()));
    (void)drone_->step();
}

// Ex2-grading missed bug DRO09: the step_index seen by the algorithm is EXACT — 0 on the very first
// nextStep call, then 1, then 2 — matched against state() after each step.
TEST_F(DroneControl, StepIndexExactFromZero) {
    using namespace ::testing;
    std::vector<std::size_t> seen;
    ON_CALL(*algo_, nextStep(_, _))
        .WillByDefault([&](const common::types::DroneState& s,
                           const common::types::LidarScanResult*) {
            seen.push_back(s.step_index);
            return advanceStepCmd(10.0);
        });

    EXPECT_EQ(drone_->state().step_index, 0u);
    (void)drone_->step();
    EXPECT_EQ(drone_->state().step_index, 1u);
    (void)drone_->step();
    EXPECT_EQ(drone_->state().step_index, 2u);

    ASSERT_EQ(seen.size(), 2u);
    EXPECT_EQ(seen[0], 0u) << "the FIRST nextStep call must see step_index 0";
    EXPECT_EQ(seen[1], 1u);
}

// Ex3 collision classification: movement throws but the commanded destination was Unmapped →
// exploration discovery, not a fault. The step returns Continue (like a refused move) and the
// drone did not move.
TEST_F(DroneControl, MovementThrowUnknownDestContinues) {
    using namespace ::testing;
    // Default map: every cell (including the destination) reads Unmapped.
    ON_CALL(*algo_, nextStep(_, _))
        .WillByDefault(Return(advanceStepCmd(10.0, /*with_scan=*/false)));
    EXPECT_CALL(movement_, advance(_))
        .Times(1)
        .WillOnce(Throw(std::runtime_error("physical collision in simulator")));
    const auto result = drone_->step();
    EXPECT_EQ(result.status, common::types::DroneStepStatus::Continue)
        << "a throw into UNMAPPED space is discovery, not a fatal fault";
    EXPECT_TRUE(result.message.empty());
    EXPECT_EQ(drone_->state().step_index, 1u) << "the step still counts";
}

// Ex3 collision classification: movement throws AND the commanded destination was already KNOWN
// Occupied in the output map → a valid algorithm never commands that, so step() itself throws
// FatalCollisionError. Current cell (5,5,5) reads Empty; Advance 10cm at heading 0 → destination
// (15,5,5), which reads Occupied.
TEST_F(DroneControl, MovementThrowKnownOccupiedDestThrowsFatal) {
    using namespace ::testing;
    const common::Position3D here{5.0 * common::x_extent[cm], 5.0 * common::y_extent[cm],
                                  5.0 * common::z_extent[cm]};
    ON_CALL(gps_, position()).WillByDefault(Return(here));
    // Occupancy by position: the destination cell (x = 15) is Occupied, everything else Empty —
    // so the current-cell fatal check passes and only the destination is a known obstacle.
    ON_CALL(output_map_, atVoxel(_))
        .WillByDefault([](const common::Position3D& p) {
            return (p.x.numerical_value_in(cm) > 10.0)
                       ? common::types::VoxelOccupancy::Occupied
                       : common::types::VoxelOccupancy::Empty;
        });
    ON_CALL(*algo_, nextStep(_, _))
        .WillByDefault(Return(advanceStepCmd(10.0, /*with_scan=*/false)));
    ON_CALL(movement_, advance(_))
        .WillByDefault(Throw(std::runtime_error("physical collision in simulator")));

    EXPECT_THROW(static_cast<void>(drone_->step()), FatalCollisionError);
}

// ---------------------------------------------------------------------------------------------
// Phase 8a bonus — the common-issues table's OPTIONAL error-handling rows.
// Filter: --gtest_filter=DroneControl.ErrorHandling*
// N (kMaxConsecutiveFaults) = 3 throughout: fault, fault, fatal-on-third.
// ---------------------------------------------------------------------------------------------

using mission_control_322526864_323020347::FatalMissionError;

// Row 2: a movement whose destination leaves the MAP is ignored entirely — the driver is never
// called and the step continues (the planner reroutes).
TEST_F(DroneControl, ErrorHandlingRow2_OutOfMapMovementIgnoredWithoutCallingDriver) {
    using namespace ::testing;
    // The map ends at x=5: the start (origin) is in bounds, the 10cm-advance destination is not.
    ON_CALL(output_map_, isInBounds(_))
        .WillByDefault([](const common::Position3D& p) {
            return p.x.numerical_value_in(cm) < 5.0;
        });
    ON_CALL(*algo_, nextStep(_, _))
        .WillByDefault(Return(advanceStepCmd(10.0, /*with_scan=*/false)));
    EXPECT_CALL(movement_, advance(_)).Times(0);
    const auto result = drone_->step();
    EXPECT_EQ(result.status, common::types::DroneStepStatus::Continue);
    EXPECT_TRUE(result.message.empty());
}

// Row 3: a command with unexecutable values (NaN) skips the movement and continues — the
// algorithm simply gets asked again.
TEST_F(DroneControl, ErrorHandlingRow3_InvalidCommandSkipsMovementAndContinues) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _))
        .WillByDefault(Return(advanceStepCmd(std::numeric_limits<double>::quiet_NaN(),
                                             /*with_scan=*/false)));
    EXPECT_CALL(movement_, advance(_)).Times(0);
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Continue);
}

// Row 3 escalation: the third CONSECUTIVE invalid command is fatal.
TEST_F(DroneControl, ErrorHandlingRow3_ThreeConsecutiveInvalidCommandsThrowFatal) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _))
        .WillByDefault(Return(advanceStepCmd(-5.0, /*with_scan=*/false))); // negative = invalid
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Continue);
    EXPECT_THROW(static_cast<void>(drone_->step()), FatalMissionError);
}

// Row 3 reset: a valid command in between resets the streak — no throw over 5 steps.
TEST_F(DroneControl, ErrorHandlingRow3_ValidCommandResetsInvalidStreak) {
    using namespace ::testing;
    const auto bad = advanceStepCmd(std::numeric_limits<double>::quiet_NaN(), false);
    EXPECT_CALL(*algo_, nextStep(_, _))
        .WillOnce(Return(bad))
        .WillOnce(Return(bad))
        .WillOnce(Return(advanceStepCmd(10.0, false)))
        .WillOnce(Return(bad))
        .WillOnce(Return(bad));
    for (int i = 0; i < 5; ++i) {
        EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Continue);
    }
}

// Row 4: an empty command (no movement AND no scan while Working) is tolerated and retried.
TEST_F(DroneControl, ErrorHandlingRow4_NoopCommandContinues) {
    using namespace ::testing;
    const common::types::MappingStepCommand noop{std::nullopt, std::nullopt,
                                                 common::types::AlgorithmStatus::Working};
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(noop));
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Continue);
}

// Row 4 escalation: three consecutive NOOPs mean the algorithm is stuck-broken → fatal.
TEST_F(DroneControl, ErrorHandlingRow4_ThreeConsecutiveNoopsThrowFatal) {
    using namespace ::testing;
    const common::types::MappingStepCommand noop{std::nullopt, std::nullopt,
                                                 common::types::AlgorithmStatus::Working};
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(noop));
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Continue);
    EXPECT_THROW(static_cast<void>(drone_->step()), FatalMissionError);
}

// Row 6: an empty scan (a lidar malfunction — the simulator's lidar always reports the center
// beam) is retried within the step, kLidarTriesPerStep = 3 attempts total.
TEST_F(DroneControl, ErrorHandlingRow6_EmptyScanRetriedWithinStep) {
    using namespace ::testing;
    EXPECT_CALL(lidar_, scan(_))
        .Times(3)
        .WillRepeatedly(Return(common::types::LidarScanResult{}));
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Continue);
}

// Row 6: a malfunctioning empty scan must NOT clobber the stored (good) latest scan.
TEST_F(DroneControl, ErrorHandlingRow6_EmptyScanDoesNotClobberStoredScan) {
    using namespace ::testing;
    EXPECT_CALL(lidar_, scan(_))
        .WillOnce(Return(oneHitScan()))                          // step 1: good scan, stored
        .WillRepeatedly(Return(common::types::LidarScanResult{})); // step 2: 3 empty tries
    const common::types::LidarScanResult* third = nullptr;
    EXPECT_CALL(*algo_, nextStep(_, _))
        .WillOnce(Return(advanceStepCmd(10.0, /*with_scan=*/true)))
        .WillOnce(Return(advanceStepCmd(10.0, /*with_scan=*/true)))
        .WillOnce([&](const common::types::DroneState&,
                      const common::types::LidarScanResult* p) {
            third = p;
            return finishedCmd();
        });
    (void)drone_->step();
    (void)drone_->step();
    (void)drone_->step();
    ASSERT_NE(third, nullptr) << "the empty scan must not clear the stored scan";
    ASSERT_EQ(third->size(), 1u);
}

// Row 6 escalation: a third consecutive all-empty scanning step is fatal.
TEST_F(DroneControl, ErrorHandlingRow6_ThreeConsecutiveEmptyScanStepsThrowFatal) {
    using namespace ::testing;
    ON_CALL(lidar_, scan(_)).WillByDefault(Return(common::types::LidarScanResult{}));
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Continue);
    EXPECT_THROW(static_cast<void>(drone_->step()), FatalMissionError);
}

// Row 7 escalation: the first two driver failures surface as step Errors (row-9 logging keeps
// the mission alive); the third consecutive one is fatal.
TEST_F(DroneControl, ErrorHandlingRow7_ThirdConsecutiveDriverFailureThrowsFatal) {
    using namespace ::testing;
    ON_CALL(movement_, advance(_))
        .WillByDefault(Return(common::types::MovementResult{false, "motor_stall"}));
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Error);
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Error);
    EXPECT_THROW(static_cast<void>(drone_->step()), FatalMissionError);
}

// Row 7 reset: a successful movement in between resets the streak.
TEST_F(DroneControl, ErrorHandlingRow7_SuccessResetsFailureStreak) {
    using namespace ::testing;
    const common::types::MovementResult fail{false, "motor_stall"};
    const common::types::MovementResult ok{true, {}};
    EXPECT_CALL(movement_, advance(_))
        .WillOnce(Return(fail))
        .WillOnce(Return(fail))
        .WillOnce(Return(ok))
        .WillOnce(Return(fail))
        .WillOnce(Return(fail));
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Error);
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Error);
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Error);
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Error);
}

// Row 8: an over-limit advance is split into per-step chunks — 120cm with max_advance 50 and
// resolution 10 executes as 50 + 50 + 20 across three steps, consulting the algorithm only once.
TEST_F(DroneControl, ErrorHandlingRow8_OverLimitAdvanceSplitsAcrossSteps) {
    using namespace ::testing;
    EXPECT_CALL(*algo_, nextStep(_, _))
        .WillOnce(Return(advanceStepCmd(120.0, /*with_scan=*/false)))
        .WillOnce(Return(finishedCmd()));
    {
        InSequence seq;
        EXPECT_CALL(movement_, advance(CmEq(50.0))).Times(2)
            .WillRepeatedly(Return(common::types::MovementResult{true, {}}));
        EXPECT_CALL(movement_, advance(CmEq(20.0))).Times(1)
            .WillOnce(Return(common::types::MovementResult{true, {}}));
    }
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Continue);  // algo: 50
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Continue);  // pending: 50
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Continue);  // pending: 20
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Completed); // algo again
    EXPECT_EQ(drone_->state().step_index, 4u) << "each chunk is its own counted step";
}

// Row 8 (rotation): 100° right with max_rotate 45 → 45 + 45 + 10 across three steps.
TEST_F(DroneControl, ErrorHandlingRow8_OverLimitRotateSplitsAcrossSteps) {
    using namespace ::testing;
    EXPECT_CALL(*algo_, nextStep(_, _))
        .WillOnce(Return(rotateStepCmd(common::types::RotationDirection::Right, 100.0,
                                       /*with_scan=*/false)))
        .WillOnce(Return(finishedCmd()));
    {
        InSequence seq;
        EXPECT_CALL(movement_, rotate(common::types::RotationDirection::Right, DegEq(45.0)))
            .Times(2)
            .WillRepeatedly(Return(common::types::MovementResult{true, {}}));
        EXPECT_CALL(movement_, rotate(common::types::RotationDirection::Right, DegEq(10.0)))
            .Times(1)
            .WillOnce(Return(common::types::MovementResult{true, {}}));
    }
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Completed);
}

// Row 8: a failed chunk drops the remainder — control returns to the algorithm on the next step.
TEST_F(DroneControl, ErrorHandlingRow8_FailedChunkDropsRemainder) {
    using namespace ::testing;
    EXPECT_CALL(*algo_, nextStep(_, _))
        .WillOnce(Return(advanceStepCmd(120.0, /*with_scan=*/false)))
        .WillOnce(Return(finishedCmd()));
    EXPECT_CALL(movement_, advance(_))
        .WillOnce(Return(common::types::MovementResult{true, {}}))       // chunk 1: 50, ok
        .WillOnce(Return(common::types::MovementResult{false, "motor_stall"})); // chunk 2 fails
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Continue);
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Error);
    // The 20cm remainder is gone: the next step consults the algorithm (which finishes).
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Completed);
}

// Row 10: an advance that would exit the MISSION bounds is amended to the longest whole-cell
// prefix that stays inside: from x=85 toward x_max=100, a 50cm request becomes 10cm (dest 95).
TEST_F(DroneControl, ErrorHandlingRow10_AdvanceAmendedToStayInMissionBounds) {
    using namespace ::testing;
    drone_ = std::make_unique<DroneControlImpl>(
        makeConfig(), missionWithBounds(), lidar_, gps_, movement_, output_map_, *algo_);
    ON_CALL(gps_, position()).WillByDefault(Return(posCm(85.0, 50.0, 50.0)));
    ON_CALL(*algo_, nextStep(_, _))
        .WillByDefault(Return(advanceStepCmd(50.0, /*with_scan=*/false)));
    EXPECT_CALL(movement_, advance(CmEq(10.0))).Times(1);
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Continue);
}

// Row 10: at the boundary itself every prefix exits, so the movement is amended away entirely —
// a legal no-op, the driver is never called.
TEST_F(DroneControl, ErrorHandlingRow10_AdvanceAtBoundaryBecomesNoop) {
    using namespace ::testing;
    drone_ = std::make_unique<DroneControlImpl>(
        makeConfig(), missionWithBounds(), lidar_, gps_, movement_, output_map_, *algo_);
    ON_CALL(gps_, position()).WillByDefault(Return(posCm(95.0, 50.0, 50.0)));
    ON_CALL(*algo_, nextStep(_, _))
        .WillByDefault(Return(advanceStepCmd(50.0, /*with_scan=*/false)));
    EXPECT_CALL(movement_, advance(_)).Times(0);
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Continue);
}

// Row 11: an out-of-bounds GPS reading is IGNORED when the internal estimate is still valid —
// the step proceeds on the estimate (the algorithm sees the estimated position).
TEST_F(DroneControl, ErrorHandlingRow11_OobGpsIgnoredUsingInternalEstimate) {
    using namespace ::testing;
    // The map ends at x=500; (999,…) is a bogus out-of-bounds reading.
    ON_CALL(output_map_, isInBounds(_))
        .WillByDefault([](const common::Position3D& p) {
            return p.x.numerical_value_in(cm) < 500.0;
        });
    EXPECT_CALL(gps_, position())
        .WillOnce(Return(posCm(0.0, 0.0, 0.0)))    // step 1 start: healthy
        .WillOnce(Return(posCm(10.0, 0.0, 0.0)))   // step 1 post-move: healthy → estimate = 10
        .WillOnce(Return(posCm(999.0, 0.0, 0.0))); // step 2 start: bogus → estimate used
    common::types::DroneState seen{};
    EXPECT_CALL(*algo_, nextStep(_, _))
        .WillOnce(Return(advanceStepCmd(10.0, /*with_scan=*/false)))
        .WillOnce(DoAll(SaveArg<0>(&seen), Return(finishedCmd())));
    (void)drone_->step();
    (void)drone_->step();
    EXPECT_NEAR(seen.position.x.numerical_value_in(cm), 10.0, 1e-9)
        << "the bogus GPS reading must be ignored in favor of the internal estimate";
}

// Row 11 escalation: reading AND estimate both out of bounds → the drone is genuinely lost.
TEST_F(DroneControl, ErrorHandlingRow11_BothOutOfBoundsThrowsFatal) {
    using namespace ::testing;
    ON_CALL(output_map_, isInBounds(_)).WillByDefault(Return(false));
    EXPECT_CALL(*algo_, nextStep(_, _)).Times(0);
    EXPECT_THROW(static_cast<void>(drone_->step()), FatalMissionError);
}

// Row 12: after an executed movement a non-finite GPS reading is re-read (3 tries total); if it
// never becomes possible the step reports Error — the mission logs it and continues (row 9).
TEST_F(DroneControl, ErrorHandlingRow12_ImpossibleGpsAfterMovementRetriesThenErrors) {
    using namespace ::testing;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_CALL(gps_, position())
        .WillOnce(Return(posCm(0.0, 0.0, 0.0)))  // step start: healthy
        .WillRepeatedly(Return(posCm(nan, nan, nan)));
    ON_CALL(*algo_, nextStep(_, _))
        .WillByDefault(Return(advanceStepCmd(10.0, /*with_scan=*/false)));
    const auto result = drone_->step();
    EXPECT_EQ(result.status, common::types::DroneStepStatus::Error);
    EXPECT_EQ(result.message, "GPS_IMPOSSIBLE_COORDINATES");
}

// Row 12 recovery: a good reading on a retry keeps the step healthy.
TEST_F(DroneControl, ErrorHandlingRow12_GpsRecoversOnRetry) {
    using namespace ::testing;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_CALL(gps_, position())
        .WillOnce(Return(posCm(0.0, 0.0, 0.0)))   // step start
        .WillOnce(Return(posCm(nan, nan, nan)))   // post-move try 1: impossible
        .WillOnce(Return(posCm(10.0, 0.0, 0.0))); // post-move try 2: recovered
    ON_CALL(*algo_, nextStep(_, _))
        .WillByDefault(Return(advanceStepCmd(10.0, /*with_scan=*/false)));
    EXPECT_EQ(drone_->step().status, common::types::DroneStepStatus::Continue);
}
