// SimulationManager: canonical cartesian enumeration (matrix completeness — the MAN21 lesson),
// failure tiers (inverted bounds, group-wide factory failure), pass-through of run results in
// order, and the pinned report header constants (the MAN27 lesson).

#include "mocks/CommonMocks.h"
#include "simulation/SimulationManager.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace {

using ::testing::_;
using ::testing::Invoke;
using ::testing::NiceMock;

using sim_tests::MockISimulationRun;
using sim_tests::MockISimulationRunFactory;
using namespace common; // Units symbols; data types stay fully qualified

simulator::types::SimulationCompositionData makeComposition(int groups,
                                                            const std::vector<int>& missions_per_group,
                                                            int drones, int lidars) {
    simulator::types::SimulationCompositionData comp;
    comp.composition_file = "unit_test_composition.yaml";
    for (int g = 0; g < groups; ++g) {
        simulator::types::SimulationConfigData sim;
        std::vector<common::types::MissionConfigData> missions(
            static_cast<std::size_t>(missions_per_group[static_cast<std::size_t>(g)]));
        comp.simulation_mission_groups.emplace_back(sim, std::move(missions));
    }
    comp.drone_configs.resize(static_cast<std::size_t>(drones));
    comp.lidar_configs.resize(static_cast<std::size_t>(lidars));
    return comp;
}

std::filesystem::path tempOut(const char* name) {
    const auto dir = std::filesystem::temp_directory_path() / "sim_manager_test" / name;
    std::filesystem::create_directories(dir);
    return dir;
}

// A factory mock whose created runs return an incrementing score, recording the call count.
class CountingFactory {
public:
    std::unique_ptr<NiceMock<MockISimulationRunFactory>> build() {
        auto factory = std::make_unique<NiceMock<MockISimulationRunFactory>>();
        ON_CALL(*factory, create(_, _, _, _, _))
            .WillByDefault(Invoke([this](const simulator::types::SimulationConfigData&,
                                         const common::types::MissionConfigData&,
                                         const common::types::DroneConfigData&,
                                         const common::types::LidarConfigData&,
                                         const std::filesystem::path&) {
                const int index = calls_++;
                auto run = std::make_unique<NiceMock<MockISimulationRun>>();
                ON_CALL(*run, run()).WillByDefault(Invoke([index] {
                    simulator::types::SimulationResult r;
                    r.mission_score = static_cast<double>(index);
                    r.mission_results.push_back(common::types::MissionRunResult{
                        common::types::MissionRunStatus::Completed,
                        static_cast<std::size_t>(index), {}});
                    return r;
                }));
                return run;
            }));
        return factory;
    }

    int calls() const { return calls_; }

private:
    int calls_ = 0;
};

TEST(SimulationManager, NullFactoryThrows) {
    EXPECT_THROW(simulator::SimulationManager(nullptr, "unused.log"), std::invalid_argument);
}

TEST(SimulationManager, RunsFullCartesianProduct) {
    // 2 groups (1 + 2 missions) x 2 drones x 2 lidars = 12 runs — no combination may be dropped
    // (the MAN21 mutation dropped one lidar case; this asserts the exact count AND factory calls).
    CountingFactory counting;
    auto factory = counting.build();
    simulator::SimulationManager manager{std::move(factory),
                                         tempOut("cartesian") / "manager_errors.log"};
    const auto comp = makeComposition(2, {1, 2}, 2, 2);
    const auto report = manager.run(comp, tempOut("cartesian"));

    EXPECT_EQ(report.runs.size(), 12u);
    EXPECT_EQ(counting.calls(), 12);
}

TEST(SimulationManager, ResultsKeepCanonicalOrder) {
    CountingFactory counting;
    auto factory = counting.build();
    simulator::SimulationManager manager{std::move(factory),
                                         tempOut("order") / "manager_errors.log"};
    const auto report = manager.run(makeComposition(1, {2}, 2, 1), tempOut("order"));
    ASSERT_EQ(report.runs.size(), 4u);
    for (std::size_t i = 0; i < report.runs.size(); ++i) {
        EXPECT_DOUBLE_EQ(report.runs[i].mission_score, static_cast<double>(i));
    }
}

TEST(SimulationManager, InvertedMissionBoundsSkipFactoryAndScoreMinusOne) {
    CountingFactory counting;
    auto factory = counting.build();
    simulator::SimulationManager manager{std::move(factory),
                                         tempOut("bounds") / "manager_errors.log"};
    auto comp = makeComposition(1, {1}, 1, 1);
    auto& mission = std::get<1>(comp.simulation_mission_groups[0])[0];
    mission.mission_bounds.min_x = 10.0 * x_extent[cm];
    mission.mission_bounds.max_x = 0.0 * x_extent[cm]; // inverted

    const auto report = manager.run(comp, tempOut("bounds"));
    ASSERT_EQ(report.runs.size(), 1u);
    EXPECT_EQ(counting.calls(), 0);
    EXPECT_DOUBLE_EQ(report.runs[0].mission_score, -1.0);
    ASSERT_FALSE(report.runs[0].mission_results.empty());
    EXPECT_EQ(report.runs[0].mission_results.front().errors.front().code,
              "MISSION_BOUNDARY_INVALID");
}

TEST(SimulationManager, FactoryThrowFailsRestOfGroupOnly) {
    auto factory = std::make_unique<NiceMock<MockISimulationRunFactory>>();
    int calls = 0;
    ON_CALL(*factory, create(_, _, _, _, _))
        .WillByDefault(Invoke([&calls](const simulator::types::SimulationConfigData&,
                                       const common::types::MissionConfigData&,
                                       const common::types::DroneConfigData&,
                                       const common::types::LidarConfigData&,
                                       const std::filesystem::path&)
                                  -> std::unique_ptr<simulator::ISimulationRun> {
            ++calls;
            if (calls == 1) {
                throw std::runtime_error("bad map file");
            }
            auto run = std::make_unique<NiceMock<MockISimulationRun>>();
            ON_CALL(*run, run()).WillByDefault(Invoke([] {
                simulator::types::SimulationResult r;
                r.mission_score = 42.0;
                return r;
            }));
            return run;
        }));

    simulator::SimulationManager manager{std::move(factory),
                                         tempOut("groupfail") / "manager_errors.log"};
    // Group A: 2 missions x 1 x 1 = 2 combos; group B: 1 combo.
    const auto report = manager.run(makeComposition(2, {2, 1}, 1, 1), tempOut("groupfail"));

    ASSERT_EQ(report.runs.size(), 3u);
    // Group A: first combo threw → FACTORY_ERROR; the second is auto-filled WITHOUT another call.
    EXPECT_DOUBLE_EQ(report.runs[0].mission_score, -1.0);
    EXPECT_EQ(report.runs[0].mission_results.front().errors.front().code, "FACTORY_ERROR");
    EXPECT_DOUBLE_EQ(report.runs[1].mission_score, -1.0);
    EXPECT_EQ(report.runs[1].mission_results.front().errors.front().code, "FACTORY_ERROR");
    // Group B proceeded normally.
    EXPECT_DOUBLE_EQ(report.runs[2].mission_score, 42.0);
    EXPECT_EQ(calls, 2); // 1 throwing call for group A, 1 successful for group B
}

TEST(SimulationManager, ReportHeaderConstantsArePinned) {
    // The MAN22/MAN27 mutations corrupted the summary/score_range on errors — the header
    // constants must be independent of run outcomes.
    CountingFactory counting;
    auto factory = counting.build();
    simulator::SimulationManager manager{std::move(factory),
                                         tempOut("header") / "manager_errors.log"};
    auto comp = makeComposition(1, {1}, 1, 1);
    auto& mission = std::get<1>(comp.simulation_mission_groups[0])[0];
    mission.mission_bounds.min_x = 10.0 * x_extent[cm];
    mission.mission_bounds.max_x = 0.0 * x_extent[cm]; // force an errored run

    const auto report = manager.run(comp, tempOut("header"));
    EXPECT_EQ(report.composition_file, std::filesystem::path{"unit_test_composition.yaml"});
    EXPECT_EQ(report.metric, "output_map_accuracy");
    EXPECT_DOUBLE_EQ(std::get<0>(report.score_range), 0.0);
    EXPECT_DOUBLE_EQ(std::get<1>(report.score_range), 100.0);
    EXPECT_EQ(report.error_score, -1);
    EXPECT_FALSE(report.generated_at_utc.empty());
}

} // namespace
