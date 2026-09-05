// The isolated aggregation/grouping policies of the final reports — the direct descendants of the
// ex2 grading bugs MAN22/MAN27 (aggregation over mixed error/success runs was untested there).

#include "report/ResultAggregation.h"

#include <gtest/gtest.h>

#include <cstddef>
#include <string>
#include <vector>

namespace {

using simulator::ComponentAggregate;
using simulator::aggregateComponent;
using simulator::groupBySameResults;
using simulator::runOutcome;
using simulator::sameResults;

simulator::types::SimulationResult makeRun(double score, std::size_t steps) {
    simulator::types::SimulationResult r;
    r.mission_score = score;
    r.mission_results.push_back(
        common::types::MissionRunResult{common::types::MissionRunStatus::Completed, steps, {}});
    return r;
}

simulator::types::SimulationResult makeErrorRunWithoutMissionResult() {
    simulator::types::SimulationResult r;
    r.mission_score = -1.0;
    return r; // e.g. a factory failure: no mission ever ran
}

ComponentAggregate makeAggregate(std::string name,
                                 const std::vector<std::pair<double, std::size_t>>& outcomes) {
    std::vector<simulator::types::SimulationResult> runs;
    runs.reserve(outcomes.size());
    for (const auto& [score, steps] : outcomes) {
        runs.push_back(makeRun(score, steps));
    }
    return aggregateComponent(std::move(name), runs);
}

TEST(ResultAggregation, RunOutcomeExtractsScoreAndSteps) {
    const auto outcome = runOutcome(makeRun(87.5, 123));
    EXPECT_DOUBLE_EQ(outcome.score, 87.5);
    EXPECT_EQ(outcome.steps, 123u);
}

TEST(ResultAggregation, RunOutcomeOfFactoryFailureHasZeroSteps) {
    const auto outcome = runOutcome(makeErrorRunWithoutMissionResult());
    EXPECT_DOUBLE_EQ(outcome.score, -1.0);
    EXPECT_EQ(outcome.steps, 0u);
}

TEST(ResultAggregation, TotalsExcludeErrorRuns) {
    // Forum-binding: -1 scores are excluded from total_score; the isolated policy also excludes
    // those runs' steps from total_steps.
    std::vector<simulator::types::SimulationResult> runs;
    runs.push_back(makeRun(90.0, 100));
    runs.push_back(makeRun(-1.0, 555)); // errored run: neither score nor steps may count
    runs.push_back(makeRun(10.0, 50));
    const auto agg = aggregateComponent("a.so", runs);

    EXPECT_DOUBLE_EQ(agg.total_score, 100.0);
    EXPECT_EQ(agg.total_steps, 150u);
    ASSERT_EQ(agg.outcomes.size(), 3u); // the per-run outcome vector keeps ALL runs, errors included
    EXPECT_DOUBLE_EQ(agg.outcomes[1].score, -1.0);
}

TEST(ResultAggregation, AllErrorRunsGiveZeroTotals) {
    std::vector<simulator::types::SimulationResult> runs;
    runs.push_back(makeRun(-1.0, 10));
    runs.push_back(makeErrorRunWithoutMissionResult());
    const auto agg = aggregateComponent("a.so", runs);
    EXPECT_DOUBLE_EQ(agg.total_score, 0.0);
    EXPECT_EQ(agg.total_steps, 0u);
}

TEST(ResultAggregation, SameResultsRequiresIdenticalVectors) {
    const auto a = makeAggregate("a.so", {{90.0, 10}, {80.0, 20}});
    const auto b = makeAggregate("b.so", {{90.0, 10}, {80.0, 20}});
    const auto c_score = makeAggregate("c.so", {{90.0, 10}, {81.0, 20}});
    const auto d_steps = makeAggregate("d.so", {{90.0, 10}, {80.0, 21}});
    const auto e_len = makeAggregate("e.so", {{90.0, 10}});

    EXPECT_TRUE(sameResults(a, b));
    EXPECT_FALSE(sameResults(a, c_score));
    EXPECT_FALSE(sameResults(a, d_steps)); // equal scores, different steps → different groups
    EXPECT_FALSE(sameResults(a, e_len));
}

TEST(ResultAggregation, EqualTotalsDifferentRunsStaySeparate) {
    // The spec's own example: identical total_score can still be two groups.
    const auto a = makeAggregate("a.so", {{50.0, 10}, {50.0, 10}});
    const auto b = makeAggregate("b.so", {{60.0, 10}, {40.0, 10}}); // same total (100), other runs
    EXPECT_FALSE(sameResults(a, b));
}

TEST(ResultAggregation, GroupsSortedBySizeDescendingThenName) {
    std::vector<ComponentAggregate> comps;
    comps.push_back(makeAggregate("a.so", {{10.0, 1}}));  // group X
    comps.push_back(makeAggregate("b.so", {{20.0, 2}}));  // group Y
    comps.push_back(makeAggregate("c.so", {{10.0, 1}}));  // group X
    comps.push_back(makeAggregate("d.so", {{30.0, 3}}));  // group Z

    const auto groups = groupBySameResults(comps);
    ASSERT_EQ(groups.size(), 3u);
    // Largest group first (a+c); the two singletons tie-break by name (b before d).
    ASSERT_EQ(groups[0].size(), 2u);
    EXPECT_EQ(groups[0][0]->name, "a.so");
    EXPECT_EQ(groups[0][1]->name, "c.so");
    EXPECT_EQ(groups[1].front()->name, "b.so");
    EXPECT_EQ(groups[2].front()->name, "d.so");
}

TEST(ResultAggregation, EmptyComponentsGiveNoGroups) {
    EXPECT_TRUE(groupBySameResults({}).empty());
}

TEST(ResultAggregation, CompetitionOrderScoreDescThenStepsAsc) {
    std::vector<ComponentAggregate> comps;
    comps.push_back(makeAggregate("slow_winner.so", {{90.0, 200}}));  // score 90, steps 200
    comps.push_back(makeAggregate("fast_winner.so", {{90.0, 100}}));  // score 90, steps 100
    comps.push_back(makeAggregate("best.so", {{95.0, 500}}));         // score 95
    comps.push_back(makeAggregate("worst.so", {{10.0, 1}}));          // score 10

    const auto ordered = simulator::orderByScoreDescStepsAsc(comps);
    ASSERT_EQ(ordered.size(), 4u);
    EXPECT_EQ(ordered[0]->name, "best.so");        // highest score first, steps irrelevant
    EXPECT_EQ(ordered[1]->name, "fast_winner.so"); // equal scores → fewer steps first
    EXPECT_EQ(ordered[2]->name, "slow_winner.so");
    EXPECT_EQ(ordered[3]->name, "worst.so");
}

TEST(ResultAggregation, CompetitionOrderExactTieBreaksByName) {
    std::vector<ComponentAggregate> comps;
    comps.push_back(makeAggregate("b.so", {{50.0, 10}}));
    comps.push_back(makeAggregate("a.so", {{50.0, 10}}));
    const auto ordered = simulator::orderByScoreDescStepsAsc(comps);
    ASSERT_EQ(ordered.size(), 2u);
    EXPECT_EQ(ordered[0]->name, "a.so");
    EXPECT_EQ(ordered[1]->name, "b.so");
}

} // namespace
