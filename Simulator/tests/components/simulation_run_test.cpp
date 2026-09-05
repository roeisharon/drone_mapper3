// SimulationRun + SimulationRunFactory under the ex3 factory-injection model: the plugin factories
// are plain std::functions, so the tests inject lambdas producing gmock instances — no dlopen
// needed to unit-test the run layer. Also pins the resolution-request classification (SIM20).

#include "map/Map3DImpl.h"
#include "mocks/CommonMocks.h"
#include "simulation/SimulationRunFactoryImpl.h"
#include "simulation/SimulationRunImpl.h"

#include <TinyNPY.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <string>
#include <utility>

namespace {

using ::testing::NiceMock;
using ::testing::Return;
using ::testing::Throw;

using sim_tests::MockIMappingAlgorithm;
using sim_tests::MockIMissionControl;
using namespace common; // Units symbols; data types stay fully qualified

// The 5x5x5 fixture map with a single Occupied voxel at cell (4,4,4). At resolution 10 cm the
// world extent is a 50 cm cube and the occupied cell spans (40..50) on each axis.
std::filesystem::path fixtureMapPath() {
    return std::filesystem::path{DRONE_TEST_DATA_DIR} / "single_voxel_x4_y4_z4.npy";
}

simulator::types::SimulationConfigData makeSimConfig(const Position3D& start) {
    simulator::types::SimulationConfigData cfg;
    cfg.map_filename = fixtureMapPath();
    cfg.map_resolution = 10.0 * cm;
    cfg.initial_drone_position = start;
    cfg.initial_angle = 0.0 * horizontal_angle[deg];
    return cfg;
}

common::types::MissionConfigData makeMission(double resolution_factor = 1.0) {
    common::types::MissionConfigData mission;
    mission.max_steps = 20;
    mission.gps_resolution = 10.0 * cm;
    mission.output_mapping_resolution_factor = resolution_factor;
    return mission;
}

class SimulationRunTest : public ::testing::Test {
protected:
    void SetUp() override {
        out_dir_ = std::filesystem::temp_directory_path() /
                   ("sim_run_test_" +
                    std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()));
        std::filesystem::remove_all(out_dir_);
        std::filesystem::create_directories(out_dir_);
    }

    // Builds a factory whose algorithm is an inert mock and whose mission control runs the given
    // scripted behavior.
    template <typename McScript>
    std::unique_ptr<simulator::SimulationRunFactoryImpl> makeFactory(McScript&& script,
                                                                     bool verbose = false) {
        algorithm_factory_ = [](common::MappingAlgorithmDependencies deps) {
            return std::make_unique<NiceMock<MockIMappingAlgorithm>>(std::move(deps));
        };
        mission_control_factory_ =
            [this, script](common::MissionControlDependencies deps)
            -> std::unique_ptr<common::IMissionControl> {
            last_output_map_file_ = deps.output_map_file;
            last_verbose_ = deps.verbose;
            auto mc = std::make_unique<NiceMock<MockIMissionControl>>();
            script(*mc);
            return mc;
        };
        return std::make_unique<simulator::SimulationRunFactoryImpl>(
            algorithm_factory_, mission_control_factory_, "T", verbose);
    }

    static common::types::MissionRunResult completedResult(std::size_t steps) {
        return {common::types::MissionRunStatus::Completed, steps, {}};
    }

    std::filesystem::path out_dir_;
    common::MappingAlgorithmFactory algorithm_factory_;
    common::MissionControlFactory mission_control_factory_;
    std::filesystem::path last_output_map_file_;
    bool last_verbose_ = false;
};

TEST_F(SimulationRunTest, HappyPathReturnsScoredResult) {
    auto factory = makeFactory([](MockIMissionControl& mc) {
        ON_CALL(mc, runMission()).WillByDefault(Return(completedResult(3)));
    });
    auto run = factory->create(makeSimConfig({5.0 * x_extent[cm], 5.0 * y_extent[cm],
                                              5.0 * z_extent[cm]}),
                               makeMission(), {}, {}, out_dir_);
    const auto result = run->run();

    ASSERT_EQ(result.mission_results.size(), 1u);
    EXPECT_EQ(result.mission_results.front().status, common::types::MissionRunStatus::Completed);
    EXPECT_EQ(result.mission_results.front().steps, 3u);
    // The mocked mission never mapped anything: an all-Unmapped output map scores exactly 0
    // (scored, NOT the -1 error sentinel).
    EXPECT_DOUBLE_EQ(result.mission_score, 0.0);
    EXPECT_EQ(result.resolution_request_status,
              simulator::types::ResolutionRequestStatus::Accepted);
    EXPECT_EQ(result.output_map_file, out_dir_ / "T_run0_output.npy");
    EXPECT_TRUE(last_verbose_ == false);
}

TEST_F(SimulationRunTest, MissionErrorStatusSkipsScoring) {
    auto factory = makeFactory([](MockIMissionControl& mc) {
        ON_CALL(mc, runMission())
            .WillByDefault(Return(common::types::MissionRunResult{
                common::types::MissionRunStatus::Error, 5, {{"SOME_CODE", "boom"}}}));
    });
    auto run = factory->create(makeSimConfig({5.0 * x_extent[cm], 5.0 * y_extent[cm],
                                              5.0 * z_extent[cm]}),
                               makeMission(), {}, {}, out_dir_);
    const auto result = run->run();
    // SIM16 invariant: an Error mission is never comparison-scored — always the -1 sentinel.
    EXPECT_DOUBLE_EQ(result.mission_score, -1.0);
    EXPECT_EQ(result.mission_results.front().status, common::types::MissionRunStatus::Error);
}

TEST_F(SimulationRunTest, MissionExceptionBecomesErrorResult) {
    auto factory = makeFactory([](MockIMissionControl& mc) {
        ON_CALL(mc, runMission()).WillByDefault(Throw(std::runtime_error("mission blew up")));
    });
    auto run = factory->create(makeSimConfig({5.0 * x_extent[cm], 5.0 * y_extent[cm],
                                              5.0 * z_extent[cm]}),
                               makeMission(), {}, {}, out_dir_);
    const auto result = run->run();
    EXPECT_DOUBLE_EQ(result.mission_score, -1.0);
    ASSERT_EQ(result.mission_results.size(), 1u);
    ASSERT_FALSE(result.mission_results.front().errors.empty());
    EXPECT_EQ(result.mission_results.front().errors.front().code, "MISSION_EXCEPTION");
}

TEST_F(SimulationRunTest, StartOutsideMapFailsWithCode) {
    auto factory = makeFactory([](MockIMissionControl&) {});
    auto run = factory->create(makeSimConfig({1000.0 * x_extent[cm], 1000.0 * y_extent[cm],
                                              1000.0 * z_extent[cm]}),
                               makeMission(), {}, {}, out_dir_);
    const auto result = run->run();
    EXPECT_DOUBLE_EQ(result.mission_score, -1.0);
    ASSERT_FALSE(result.mission_results.empty());
    ASSERT_FALSE(result.mission_results.front().errors.empty());
    EXPECT_EQ(result.mission_results.front().errors.front().code, "START_OUTSIDE_MAP");
}

TEST_F(SimulationRunTest, StartInObstacleFailsWithCode) {
    auto factory = makeFactory([](MockIMissionControl&) {});
    // Cell (4,4,4) of the fixture map is the occupied voxel: world (45,45,45) is inside it.
    auto run = factory->create(makeSimConfig({45.0 * x_extent[cm], 45.0 * y_extent[cm],
                                              45.0 * z_extent[cm]}),
                               makeMission(), {}, {}, out_dir_);
    const auto result = run->run();
    EXPECT_EQ(result.mission_results.front().errors.front().code, "START_IN_OBSTACLE");
}

TEST_F(SimulationRunTest, StartOutsideMissionBoundsFailsWithCode) {
    auto mission = makeMission();
    mission.mission_bounds = {20.0 * x_extent[cm], 40.0 * x_extent[cm],
                              20.0 * y_extent[cm], 40.0 * y_extent[cm],
                              20.0 * z_extent[cm], 40.0 * z_extent[cm]};
    auto factory = makeFactory([](MockIMissionControl&) {});
    auto run = factory->create(makeSimConfig({5.0 * x_extent[cm], 5.0 * y_extent[cm],
                                              5.0 * z_extent[cm]}),
                               mission, {}, {}, out_dir_);
    const auto result = run->run();
    EXPECT_EQ(result.mission_results.front().errors.front().code,
              "START_OUTSIDE_MISSION_BOUNDS");
}

TEST_F(SimulationRunTest, ResolutionFactorOneIsAccepted) {
    auto factory = makeFactory([](MockIMissionControl& mc) {
        ON_CALL(mc, runMission()).WillByDefault(Return(completedResult(1)));
    });
    auto run = factory->create(makeSimConfig({5.0 * x_extent[cm], 5.0 * y_extent[cm],
                                              5.0 * z_extent[cm]}),
                               makeMission(1.0), {}, {}, out_dir_);
    EXPECT_EQ(run->run().resolution_request_status,
              simulator::types::ResolutionRequestStatus::Accepted);
}

TEST_F(SimulationRunTest, ResolutionFactorBelowOneIsIgnoredTooSmall) {
    // The SIM20 rule that cost ex2 an "obsolete bug" penalty.
    auto factory = makeFactory([](MockIMissionControl& mc) {
        ON_CALL(mc, runMission()).WillByDefault(Return(completedResult(1)));
    });
    auto run = factory->create(makeSimConfig({5.0 * x_extent[cm], 5.0 * y_extent[cm],
                                              5.0 * z_extent[cm]}),
                               makeMission(0.5), {}, {}, out_dir_);
    EXPECT_EQ(run->run().resolution_request_status,
              simulator::types::ResolutionRequestStatus::IgnoredTooSmall);
}

// Phase 8b: an INTEGER factor above one is Accepted — and the output map is genuinely built at
// the scaled resolution (the whole planner/scan/comparison stack reads it from this config).
TEST_F(SimulationRunTest, ResolutionIntegerFactorAboveOneIsAccepted) {
    auto factory = makeFactory([](MockIMissionControl& mc) {
        ON_CALL(mc, runMission()).WillByDefault(Return(completedResult(1)));
    });
    auto run = factory->create(makeSimConfig({5.0 * x_extent[cm], 5.0 * y_extent[cm],
                                              5.0 * z_extent[cm]}),
                               makeMission(2.0), {}, {}, out_dir_);
    const auto result = run->run();
    EXPECT_EQ(result.resolution_request_status,
              simulator::types::ResolutionRequestStatus::Accepted);
    // Base map resolution is 10cm → the produced output map is a 20cm grid.
    EXPECT_DOUBLE_EQ(result.output_map_config.resolution.numerical_value_in(cm), 20.0);
}

// Phase 8b: a non-integer factor cannot tile the native grid — Ignored, native resolution kept.
TEST_F(SimulationRunTest, ResolutionNonIntegerFactorIsIgnored) {
    auto factory = makeFactory([](MockIMissionControl& mc) {
        ON_CALL(mc, runMission()).WillByDefault(Return(completedResult(1)));
    });
    auto run = factory->create(makeSimConfig({5.0 * x_extent[cm], 5.0 * y_extent[cm],
                                              5.0 * z_extent[cm]}),
                               makeMission(1.5), {}, {}, out_dir_);
    const auto result = run->run();
    EXPECT_EQ(result.resolution_request_status,
              simulator::types::ResolutionRequestStatus::Ignored);
    EXPECT_DOUBLE_EQ(result.output_map_config.resolution.numerical_value_in(cm), 10.0);
}

TEST_F(SimulationRunTest, ConsecutiveCreatesProduceUniqueOutputFiles) {
    auto factory = makeFactory([](MockIMissionControl& mc) {
        ON_CALL(mc, runMission()).WillByDefault(Return(completedResult(1)));
    });
    const auto sim = makeSimConfig({5.0 * x_extent[cm], 5.0 * y_extent[cm], 5.0 * z_extent[cm]});
    auto run0 = factory->create(sim, makeMission(), {}, {}, out_dir_);
    auto run1 = factory->create(sim, makeMission(), {}, {}, out_dir_);
    const auto r0 = run0->run();
    const auto r1 = run1->run();
    EXPECT_EQ(r0.output_map_file, out_dir_ / "T_run0_output.npy");
    EXPECT_EQ(r1.output_map_file, out_dir_ / "T_run1_output.npy");
}

TEST_F(SimulationRunTest, NullAlgorithmFactoryResultThrows) {
    algorithm_factory_ = [](common::MappingAlgorithmDependencies) {
        return std::unique_ptr<common::IMappingAlgorithm>{};
    };
    mission_control_factory_ = [](common::MissionControlDependencies) {
        return std::unique_ptr<common::IMissionControl>{};
    };
    simulator::SimulationRunFactoryImpl factory{algorithm_factory_, mission_control_factory_,
                                                "T", false};
    EXPECT_THROW(
        (void)factory.create(makeSimConfig({5.0 * x_extent[cm], 5.0 * y_extent[cm],
                                            5.0 * z_extent[cm]}),
                             makeMission(), {}, {}, out_dir_),
        std::runtime_error);
}

TEST_F(SimulationRunTest, VerboseFlagReachesMissionControlDependencies) {
    auto factory = makeFactory(
        [](MockIMissionControl& mc) {
            ON_CALL(mc, runMission()).WillByDefault(Return(completedResult(1)));
        },
        /*verbose=*/true);
    auto run = factory->create(makeSimConfig({5.0 * x_extent[cm], 5.0 * y_extent[cm],
                                              5.0 * z_extent[cm]}),
                               makeMission(), {}, {}, out_dir_);
    EXPECT_TRUE(last_verbose_);
    EXPECT_EQ(last_output_map_file_, out_dir_ / "T_run0_output.npy");
}

TEST_F(SimulationRunTest, FactoryCountsCreatedInstances) {
    auto factory = makeFactory([](MockIMissionControl& mc) {
        ON_CALL(mc, runMission()).WillByDefault(Return(completedResult(1)));
    });
    const auto sim = makeSimConfig({5.0 * x_extent[cm], 5.0 * y_extent[cm], 5.0 * z_extent[cm]});
    (void)factory->create(sim, makeMission(), {}, {}, out_dir_);
    (void)factory->create(sim, makeMission(), {}, {}, out_dir_);
    EXPECT_EQ(factory->algorithmInstancesCreated(), 2u);
    EXPECT_EQ(factory->missionControlInstancesCreated(), 2u);
}

TEST_F(SimulationRunTest, ThrowingMissionControlFactoryLeavesCountAtZero) {
    // The "could not be executed at all" signal: a component whose factory never yields an
    // instance keeps its counter at 0 across the whole matrix (the algorithm side, created first,
    // still counts its successes).
    algorithm_factory_ = [](common::MappingAlgorithmDependencies deps) {
        return std::make_unique<NiceMock<MockIMappingAlgorithm>>(std::move(deps));
    };
    mission_control_factory_ =
        [](common::MissionControlDependencies) -> std::unique_ptr<common::IMissionControl> {
        throw std::runtime_error("fixture factory always throws");
    };
    simulator::SimulationRunFactoryImpl factory{algorithm_factory_, mission_control_factory_,
                                                "T", false};
    const auto sim = makeSimConfig({5.0 * x_extent[cm], 5.0 * y_extent[cm], 5.0 * z_extent[cm]});
    EXPECT_THROW((void)factory.create(sim, makeMission(), {}, {}, out_dir_), std::runtime_error);
    EXPECT_THROW((void)factory.create(sim, makeMission(), {}, {}, out_dir_), std::runtime_error);
    EXPECT_EQ(factory.missionControlInstancesCreated(), 0u);
    EXPECT_EQ(factory.algorithmInstancesCreated(), 2u);
}

// ---- Pure-function factory helpers (ported concept from the ex2 factory tests) ----

TEST(SimulationRunFactory, OutputMapConfigWithoutBoundsKeepsHiddenConfig) {
    common::types::MapConfig hidden;
    hidden.resolution = 10.0 * cm;
    hidden.boundaries = {0.0 * x_extent[cm], 50.0 * x_extent[cm], 0.0 * y_extent[cm],
                         50.0 * y_extent[cm], 0.0 * z_extent[cm], 50.0 * z_extent[cm]};
    const auto out = simulator::SimulationRunFactoryImpl::outputMapConfig(hidden, {});
    EXPECT_EQ(out.boundaries.min_x, hidden.boundaries.min_x);
    EXPECT_EQ(out.boundaries.max_x, hidden.boundaries.max_x);
    EXPECT_EQ(out.offset.x, hidden.offset.x);
}

TEST(SimulationRunFactory, OutputMapConfigNarrowsToMissionBounds) {
    common::types::MapConfig hidden;
    hidden.resolution = 10.0 * cm;
    hidden.boundaries = {0.0 * x_extent[cm], 50.0 * x_extent[cm], 0.0 * y_extent[cm],
                         50.0 * y_extent[cm], 0.0 * z_extent[cm], 50.0 * z_extent[cm]};
    common::types::MappingBounds mission{10.0 * x_extent[cm], 30.0 * x_extent[cm],
                                         10.0 * y_extent[cm], 30.0 * y_extent[cm],
                                         10.0 * z_extent[cm], 30.0 * z_extent[cm]};
    const auto out = simulator::SimulationRunFactoryImpl::outputMapConfig(hidden, mission);
    EXPECT_EQ(out.boundaries.min_x, 10.0 * x_extent[cm]);
    EXPECT_EQ(out.boundaries.max_x, 30.0 * x_extent[cm]);
    // Index 0 sits at the intersection's lower corner.
    EXPECT_EQ(out.offset.x, -10.0 * x_extent[cm]);
    EXPECT_EQ(out.offset.z, -10.0 * z_extent[cm]);
}

TEST(SimulationRunFactory, SnapToCellCenterCentersWithinCell) {
    common::types::MapConfig cfg;
    cfg.resolution = 10.0 * cm;
    const auto snapped = simulator::SimulationRunFactoryImpl::snapToCellCenter(
        {12.0 * x_extent[cm], 17.0 * y_extent[cm], 23.0 * z_extent[cm]}, cfg);
    EXPECT_EQ(snapped.x, 15.0 * x_extent[cm]);
    EXPECT_EQ(snapped.y, 15.0 * y_extent[cm]);
    EXPECT_EQ(snapped.z, 25.0 * z_extent[cm]);
}

TEST(SimulationRunFactory, SnapToCellCenterHonorsOffset) {
    common::types::MapConfig cfg;
    cfg.resolution = 10.0 * cm;
    cfg.offset = {5.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    // World x=12 with offset 5 → index floor(17/10)=1 → centre (1.5*10)-5 = 10.
    const auto snapped = simulator::SimulationRunFactoryImpl::snapToCellCenter(
        {12.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]}, cfg);
    EXPECT_EQ(snapped.x, 10.0 * x_extent[cm]);
}

} // namespace
