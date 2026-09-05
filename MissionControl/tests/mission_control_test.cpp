#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <MissionControl/FatalCollisionError.h>
#include <MissionControl/MissionControlImpl.h>

#include "mocks/CommonMocks.h"

#include <cstddef>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

// Redesigned from the ex2 suite: ex2 injected a mocked IDroneControl, which is impossible in ex3 —
// MissionControlImpl is built from common::MissionControlDependencies and creates its own REAL
// DroneControlImpl internally. So the mission loop is driven end-to-end through the staff-interface
// mocks (lidar/gps/movement/output map) with the algorithm mock scripting the mission.
using namespace common;

namespace mc = mission_control_322526864_323020347;

namespace {

// A Working command that hovers (no movement API call, no scan).
common::types::MappingStepCommand workingHover() {
    common::types::MappingStepCommand cmd;
    cmd.movement = common::types::MovementCommand{}; // Hover: the drone stays put
    cmd.status   = common::types::AlgorithmStatus::Working;
    return cmd;
}

// A Working command that advances by `distance` (no scan).
common::types::MappingStepCommand workingAdvance(common::PhysicalLength distance) {
    common::types::MovementCommand mv;
    mv.type     = common::types::MovementCommandType::Advance;
    mv.distance = distance;
    common::types::MappingStepCommand cmd;
    cmd.movement = mv;
    cmd.status   = common::types::AlgorithmStatus::Working;
    return cmd;
}

// A terminal command: Finished by default, or FinishedWithUnmappableVoxels.
common::types::MappingStepCommand finishedCommand(
    common::types::AlgorithmStatus status = common::types::AlgorithmStatus::Finished) {
    common::types::MappingStepCommand cmd;
    cmd.status = status;
    return cmd;
}

common::types::MovementResult moveOk() { return {true, {}}; }
common::types::MovementResult moveError(const std::string& msg) { return {false, msg}; }

std::filesystem::path defaultOutputPath() {
    return std::filesystem::temp_directory_path() / "mission_ctrl_test_output.npy";
}

} // namespace

// Fixture named "MissionControl" (kept from ex2) so --gtest_filter=MissionControl.* works; at
// global scope so it does not shadow mc::MissionControlImpl.
//
// World defaults: the drone sits at (5,5,5) cm heading 0° in a 0..100 cm map of 10 cm voxels whose
// cells all read Empty, every move succeeds, the lidar sees nothing, and the algorithm hovers
// forever. Individual tests override exactly the piece of the world they exercise.
class MissionControl : public ::testing::Test {
protected:
    void SetUp() override {
        using namespace ::testing;

        mission_cfg_.max_steps      = 10;
        mission_cfg_.gps_resolution = 10.0 * cm;

        drone_cfg_.radius      = 5.0 * cm;
        drone_cfg_.max_rotate  = 90.0 * deg;
        drone_cfg_.max_advance = 100.0 * cm;
        drone_cfg_.max_elevate = 100.0 * cm;

        // Empty (not Unmapped) everywhere so the drone never starts inside a wall and the
        // known-Occupied fatal classification stays off unless a test arms it.
        ON_CALL(output_map_, atVoxel(_))
            .WillByDefault(Return(common::types::VoxelOccupancy::Empty));
        ON_CALL(output_map_, getMapConfig()).WillByDefault(Return(mapConfig()));
        ON_CALL(output_map_, isInBounds(_)).WillByDefault(Return(true));

        ON_CALL(gps_, position()).WillByDefault(Return(startPosition()));
        ON_CALL(gps_, heading()).WillByDefault(Return(Orientation{0.0 * deg, 0.0 * deg}));

        ON_CALL(lidar_, scan(_)).WillByDefault(Return(common::types::LidarScanResult{}));
        ON_CALL(lidar_, config()).WillByDefault(Return(lidar_cfg_));

        ON_CALL(movement_, rotate(_, _)).WillByDefault(Return(moveOk()));
        ON_CALL(movement_, advance(_)).WillByDefault(Return(moveOk()));
        ON_CALL(movement_, elevate(_)).WillByDefault(Return(moveOk()));

        ON_CALL(algorithm_, nextStep(_, _)).WillByDefault(Return(workingHover()));
    }

    // 10 cm voxels, zero offset, 0..100 cm on each axis.
    static common::types::MapConfig mapConfig() {
        return common::types::MapConfig{
            common::types::MappingBounds{
                0.0 * x_extent[cm], 100.0 * x_extent[cm],
                0.0 * y_extent[cm], 100.0 * y_extent[cm],
                0.0 * z_extent[cm], 100.0 * z_extent[cm]},
            Position3D{},
            10.0 * cm};
    }

    // A fixed in-bounds start; with heading 0°, Advance 10cm predicts destination (15,5,5).
    static Position3D startPosition() {
        return {5.0 * x_extent[cm], 5.0 * y_extent[cm], 5.0 * z_extent[cm]};
    }

    std::unique_ptr<mc::MissionControlImpl> makeMission(
        std::size_t max_steps,
        std::filesystem::path path = defaultOutputPath(),
        bool verbose               = false) {
        mission_cfg_.max_steps = max_steps;
        return std::make_unique<mc::MissionControlImpl>(common::MissionControlDependencies{
            mission_cfg_, drone_cfg_, lidar_, gps_, movement_, output_map_,
            algorithm_, std::move(path), verbose});
    }

    common::types::MissionConfigData mission_cfg_{};
    common::types::DroneConfigData   drone_cfg_{};
    common::types::LidarConfigData   lidar_cfg_{};

    ::testing::NiceMock<mc_tests::MockILidar>         lidar_;
    ::testing::NiceMock<mc_tests::MockIGPS>           gps_;
    ::testing::NiceMock<mc_tests::MockIDroneMovement> movement_;
    ::testing::NiceMock<mc_tests::MockIMutableMap3D>  output_map_;
    // Declared after the configs and output_map_ its base class binds to.
    ::testing::NiceMock<mc_tests::MockIMappingAlgorithm> algorithm_{
        common::MappingAlgorithmDependencies{mission_cfg_, lidar_cfg_, drone_cfg_, output_map_}};
};

// Termination status ---------------------------------------------------------

// Finished on the very first step → Completed with exactly one step (the completing step() call
// still counts: runMission increments steps before it inspects the result) and no errors.
TEST_F(MissionControl, FinishedOnFirstStepCompletes) {
    using namespace ::testing;
    EXPECT_CALL(algorithm_, nextStep(_, _)).WillOnce(Return(finishedCommand()));
    const auto result = makeMission(10)->runMission();
    EXPECT_EQ(result.status, common::types::MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 1u);
    EXPECT_TRUE(result.errors.empty());
}

// FinishedWithUnmappableVoxels is terminal-success too → Completed.
TEST_F(MissionControl, FinishedWithUnmappableAlsoCompletes) {
    using namespace ::testing;
    EXPECT_CALL(algorithm_, nextStep(_, _))
        .WillOnce(Return(finishedCommand(
            common::types::AlgorithmStatus::FinishedWithUnmappableVoxels)));
    const auto result = makeMission(10)->runMission();
    EXPECT_EQ(result.status, common::types::MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 1u);
    EXPECT_TRUE(result.errors.empty());
}

// The algorithm never finishes (Working+Hover forever) → the loop exhausts cleanly at exactly
// max_steps with status MaxSteps and no errors (MIS17/18: max_steps used exactly as configured).
TEST_F(MissionControl, MaxStepsReached) {
    const auto result = makeMission(5)->runMission();
    EXPECT_EQ(result.status, common::types::MissionRunStatus::MaxSteps);
    EXPECT_EQ(result.steps, 5u);
    EXPECT_TRUE(result.errors.empty());
}

// The algorithm is consulted exactly once per step — max_steps times on a full run.
TEST_F(MissionControl, AlgorithmCalledExactlyMaxStepsTimes) {
    using namespace ::testing;
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(6).WillRepeatedly(Return(workingHover()));
    (void)makeMission(6)->runMission();
}

// Working for 3 steps then Finished → Completed with steps == 4 (the finishing step counts).
TEST_F(MissionControl, StepsCountExact) {
    using namespace ::testing;
    EXPECT_CALL(algorithm_, nextStep(_, _))
        .WillOnce(Return(workingHover()))
        .WillOnce(Return(workingHover()))
        .WillOnce(Return(workingHover()))
        .WillOnce(Return(finishedCommand()));
    const auto result = makeMission(10)->runMission();
    EXPECT_EQ(result.status, common::types::MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 4u);
}

// max_steps == 0 → the loop body never runs (no drone step, no algorithm call) → MaxSteps with
// zero steps; the output map is still saved (the save-always invariant holds even here).
TEST_F(MissionControl, MaxStepsZeroRunsNoStepsAndStillSaves) {
    using namespace ::testing;
    EXPECT_CALL(algorithm_, nextStep(_, _)).Times(0);
    EXPECT_CALL(output_map_, save(_)).Times(1);
    const auto result = makeMission(0)->runMission();
    EXPECT_EQ(result.status, common::types::MissionRunStatus::MaxSteps);
    EXPECT_EQ(result.steps, 0u);
    EXPECT_TRUE(result.errors.empty());
}

// Error handling -------------------------------------------------------------

// A movement failure (a real fault, not a collision refusal) is recorded as one DRONE_ERROR entry
// and the mission continues: errors never break the loop, and Completed outranks earlier errors.
TEST_F(MissionControl, MovementErrorRecordedAndContinues) {
    using namespace ::testing;
    EXPECT_CALL(algorithm_, nextStep(_, _))
        .WillOnce(Return(workingAdvance(10.0 * cm)))
        .WillOnce(Return(workingAdvance(10.0 * cm)))
        .WillOnce(Return(finishedCommand()));
    EXPECT_CALL(movement_, advance(_))
        .WillOnce(Return(moveError("SOME_ERROR")))
        .WillOnce(Return(moveOk()));
    const auto result = makeMission(10)->runMission();
    EXPECT_EQ(result.status, common::types::MissionRunStatus::Completed)
        << "status priority is Completed > Error";
    EXPECT_EQ(result.steps, 3u);
    ASSERT_EQ(result.errors.size(), 1u);
    EXPECT_EQ(result.errors[0].code, "DRONE_ERROR");
    EXPECT_EQ(result.errors[0].message, "SOME_ERROR");
}

// Ported from ex2 StatusIsCompletedEvenWithEarlierErrors: an error on the very first step does not
// prevent a later Completed from winning the status classification.
TEST_F(MissionControl, CompletedTakesPriorityOverEarlierErrors) {
    using namespace ::testing;
    EXPECT_CALL(algorithm_, nextStep(_, _))
        .WillOnce(Return(workingAdvance(10.0 * cm)))
        .WillOnce(Return(workingHover()))
        .WillOnce(Return(finishedCommand()));
    EXPECT_CALL(movement_, advance(_)).WillOnce(Return(moveError("glitch")));
    const auto result = makeMission(10)->runMission();
    EXPECT_EQ(result.status, common::types::MissionRunStatus::Completed);
    ASSERT_EQ(result.errors.size(), 1u);
    EXPECT_EQ(result.errors[0].message, "glitch");
}

// Ported from ex2 MultipleErrorsAllCaptured: non-consecutive failures are all recorded in order,
// the loop still runs the full budget, and the exhausted-with-errors run classifies as Error.
TEST_F(MissionControl, MultipleErrorsAllCaptured) {
    using namespace ::testing;
    EXPECT_CALL(algorithm_, nextStep(_, _))
        .Times(4)
        .WillRepeatedly(Return(workingAdvance(10.0 * cm)));
    EXPECT_CALL(movement_, advance(_))
        .WillOnce(Return(moveError("err_step1")))
        .WillOnce(Return(moveOk()))
        .WillOnce(Return(moveOk()))
        .WillOnce(Return(moveError("err_step4")));
    const auto result = makeMission(4)->runMission();
    EXPECT_EQ(result.status, common::types::MissionRunStatus::Error);
    EXPECT_EQ(result.steps, 4u);
    ASSERT_EQ(result.errors.size(), 2u);
    EXPECT_EQ(result.errors[0].message, "err_step1");
    EXPECT_EQ(result.errors[1].message, "err_step4");
}

// Step errors do not shorten the loop: with failures interleaved below the row-7 escalation
// threshold (a success resets the consecutive counter), the run uses its whole step budget and
// ends Error with one entry per failed step.
TEST_F(MissionControl, ErrorsDoNotShortenTheLoop) {
    using namespace ::testing;
    ON_CALL(algorithm_, nextStep(_, _)).WillByDefault(Return(workingAdvance(10.0 * cm)));
    EXPECT_CALL(movement_, advance(_))
        .WillOnce(Return(moveError("E")))
        .WillOnce(Return(moveError("E")))
        .WillOnce(Return(common::types::MovementResult{true, {}}))
        .WillOnce(Return(moveError("E")));
    const auto result = makeMission(4)->runMission();
    EXPECT_EQ(result.status, common::types::MissionRunStatus::Error);
    EXPECT_EQ(result.steps, 4u);
    EXPECT_EQ(result.errors.size(), 3u);
}

// Phase 8a row 7: the THIRD consecutive driver failure escalates to FatalMissionError, which
// propagates out of runMission — after the output map was saved (the save-always invariant).
TEST_F(MissionControl, ThreeConsecutiveDriverFailuresAbortFatally) {
    using namespace ::testing;
    ON_CALL(algorithm_, nextStep(_, _)).WillByDefault(Return(workingAdvance(10.0 * cm)));
    ON_CALL(movement_, advance(_)).WillByDefault(Return(moveError("E")));
    EXPECT_CALL(output_map_, save(_)).Times(1);
    EXPECT_THROW((void)makeMission(10)->runMission(), mc::FatalMissionError);
}

// A move refused with the DRONE_HITS_OBSTACLE convention (returned false, the ex2-style movement
// driver) is collision PREVENTION: the step continues with no error entry, so a subsequent Finish
// still yields a clean Completed run.
TEST_F(MissionControl, RefusedMoveIsNotAnError) {
    using namespace ::testing;
    EXPECT_CALL(algorithm_, nextStep(_, _))
        .WillOnce(Return(workingAdvance(10.0 * cm)))
        .WillOnce(Return(finishedCommand()));
    EXPECT_CALL(movement_, advance(_)).WillOnce(Return(moveError("DRONE_HITS_OBSTACLE")));
    const auto result = makeMission(10)->runMission();
    EXPECT_EQ(result.status, common::types::MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 2u);
    EXPECT_TRUE(result.errors.empty());
}

// Collision classification ---------------------------------------------------

// The drone's CURRENT cell reads Occupied → the drone is actually inside an obstacle. The first
// step reports DRONE_COLLISION and the mission breaks immediately (Checkpoint-C semantics kept
// from ex2): steps == 1, status Error, the collision recorded — it must not spin to max_steps.
TEST_F(MissionControl, DroneCollisionBreaksLoop) {
    using namespace ::testing;
    ON_CALL(output_map_, atVoxel(_))
        .WillByDefault(Return(common::types::VoxelOccupancy::Occupied));
    const auto result = makeMission(10)->runMission();
    EXPECT_EQ(result.status, common::types::MissionRunStatus::Error);
    EXPECT_EQ(result.steps, 1u) << "the mission must stop on the first real collision, not run on";
    ASSERT_EQ(result.errors.size(), 1u);
    EXPECT_EQ(result.errors[0].code, "DRONE_ERROR");
    EXPECT_EQ(result.errors[0].message, "DRONE_COLLISION");
}

// Row-5 fatal path: the movement layer throws AND the commanded destination was already KNOWN
// Occupied in the output map (current cell (5,5,5) Empty; Advance 10cm at heading 0° → destination
// (15,5,5) Occupied). runMission lets FatalCollisionError propagate — but only AFTER saving the
// output map (the MIS14 save-always invariant covers the fatal path too).
TEST_F(MissionControl, FatalCollisionPropagatesAndSaves) {
    using namespace ::testing;
    ON_CALL(output_map_, atVoxel(_))
        .WillByDefault([](const common::Position3D& pos) {
            return (pos.x.numerical_value_in(cm) > 10.0)
                       ? common::types::VoxelOccupancy::Occupied
                       : common::types::VoxelOccupancy::Empty;
        });
    ON_CALL(algorithm_, nextStep(_, _)).WillByDefault(Return(workingAdvance(10.0 * cm)));
    ON_CALL(movement_, advance(_))
        .WillByDefault(Throw(std::runtime_error("DRONE_HITS_OBSTACLE: physical collision")));
    EXPECT_CALL(output_map_, save(_)).Times(1);
    EXPECT_THROW((void)makeMission(10)->runMission(), mc::FatalCollisionError);
}

// The same movement throw with an UNMAPPED destination is exploration discovery, not a fault: the
// mission does not throw, the step continues without an error entry, and the run completes.
TEST_F(MissionControl, UnknownSpaceCollisionIsNonFatal) {
    using namespace ::testing;
    ON_CALL(output_map_, atVoxel(_))
        .WillByDefault([](const common::Position3D& pos) {
            return (pos.x.numerical_value_in(cm) > 10.0)
                       ? common::types::VoxelOccupancy::Unmapped
                       : common::types::VoxelOccupancy::Empty;
        });
    EXPECT_CALL(algorithm_, nextStep(_, _))
        .WillOnce(Return(workingAdvance(10.0 * cm)))
        .WillOnce(Return(finishedCommand()));
    ON_CALL(movement_, advance(_))
        .WillByDefault(Throw(std::runtime_error("DRONE_HITS_OBSTACLE: physical collision")));

    common::types::MissionRunResult result;
    EXPECT_NO_THROW(result = makeMission(10)->runMission());
    EXPECT_EQ(result.status, common::types::MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 2u);
    EXPECT_TRUE(result.errors.empty()) << "a throw into unmapped space is not a DRONE_ERROR";
}

// save() behaviour (MIS14: the output map is ALWAYS saved exactly once) --------

// save() is called exactly once on a Completed mission.
TEST_F(MissionControl, MapSavedOnCompletion) {
    using namespace ::testing;
    ON_CALL(algorithm_, nextStep(_, _)).WillByDefault(Return(finishedCommand()));
    EXPECT_CALL(output_map_, save(_)).Times(1);
    (void)makeMission(10)->runMission();
}

// save() is called exactly once on a MaxSteps mission.
TEST_F(MissionControl, SaveCalledOnceOnMaxSteps) {
    using namespace ::testing;
    EXPECT_CALL(output_map_, save(_)).Times(1);
    (void)makeMission(3)->runMission();
}

// save() is still called exactly once when the mission ends in Error (here: the immediate
// DRONE_COLLISION break) — the invariant the graders mutate.
TEST_F(MissionControl, MapSavedOnError) {
    using namespace ::testing;
    ON_CALL(output_map_, atVoxel(_))
        .WillByDefault(Return(common::types::VoxelOccupancy::Occupied));
    EXPECT_CALL(output_map_, save(_)).Times(1);
    const auto result = makeMission(3)->runMission();
    EXPECT_EQ(result.status, common::types::MissionRunStatus::Error);
}

// save() receives the exact output path supplied through the dependencies.
TEST_F(MissionControl, SaveCalledWithExactOutputPath) {
    using namespace ::testing;
    const std::filesystem::path expected =
        std::filesystem::temp_directory_path() / "specific_mission_output.npy";
    ON_CALL(algorithm_, nextStep(_, _)).WillByDefault(Return(finishedCommand()));
    EXPECT_CALL(output_map_, save(expected)).Times(1);
    (void)makeMission(1, expected)->runMission();
}

// Verbose output --------------------------------------------------------------

// The `<stem>_verbose.txt` file next to the output map exists after runMission IFF verbose was
// requested (spec: verbose files exist iff -verbose was passed). save() is a mocked no-op, so any
// file that appears was written by MissionControlImpl itself.
TEST_F(MissionControl, VerboseFileIffFlag) {
    using namespace ::testing;
    namespace fs = std::filesystem;
    const fs::path dir = fs::temp_directory_path() / "mc_322526864_323020347_verbose_test";
    fs::remove_all(dir);
    fs::create_directories(dir);

    ON_CALL(algorithm_, nextStep(_, _)).WillByDefault(Return(finishedCommand()));

    (void)makeMission(3, dir / "with_verbose.npy", /*verbose=*/true)->runMission();
    EXPECT_TRUE(fs::exists(dir / "with_verbose_verbose.txt"))
        << "-verbose must produce <stem>_verbose.txt next to the output map";

    (void)makeMission(3, dir / "no_verbose.npy", /*verbose=*/false)->runMission();
    EXPECT_FALSE(fs::exists(dir / "no_verbose_verbose.txt"))
        << "without -verbose no verbose file may be written";

    fs::remove_all(dir);
}
