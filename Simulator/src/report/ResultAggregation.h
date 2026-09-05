#pragma once

#include <Simulator/SimulationTypes.h>

#include <cstddef>
#include <string>
#include <vector>

namespace simulator {

// The published metrics of one simulation run, as they appear in the final reports.
struct RunOutcome {
    double      score = -1.0; // -1 = error sentinel
    std::size_t steps = 0;
};

// Everything the mode reports need to know about one loaded component (mission control or
// algorithm) after all of its runs finished.
struct ComponentAggregate {
    std::string             name;      // the component's .so filename (e.g. "MissionControl_x_y.so")
    std::vector<RunOutcome> outcomes;  // canonical run order (identical for every component)
    double                  total_score = 0.0;
    std::size_t             total_steps = 0;
};

// Extracts the published (score, steps) outcome of one run.
[[nodiscard]] RunOutcome runOutcome(const types::SimulationResult& run);

// AGGREGATION POLICY (isolated here on purpose — see the plan's open questions):
//  - total_score sums only runs with score != -1 (forum clarification: exclude error scores).
//  - total_steps likewise sums only non-error runs' steps. The forum answered scores only; treating
//    steps consistently is our documented temporary assumption, changeable in this one function.
[[nodiscard]] ComponentAggregate aggregateComponent(std::string name,
                                                    const std::vector<types::SimulationResult>& runs);

// SAME-RESULTS POLICY (temporary assumption, isolated here): two components have "same results"
// iff their canonical per-run (score, steps) vectors are identical index-by-index. The spec's
// example shows equal total_score with different total_steps in separate groups, which this
// definition reproduces. If the staff later clarify a different criterion (e.g. output-map
// equality), only this function changes.
[[nodiscard]] bool sameResults(const ComponentAggregate& a, const ComponentAggregate& b);

// Groups components by sameResults(). Groups are ordered by size descending (spec sort rule),
// ties broken by the first member's name ascending for deterministic output; members keep their
// input order (callers pass components in sorted-by-name load order).
[[nodiscard]] std::vector<std::vector<const ComponentAggregate*>>
groupBySameResults(const std::vector<ComponentAggregate>& components);

// COMPETITION ORDERING POLICY (spec: "sorted by score descending, then by steps ascending";
// name ascending breaks exact ties for deterministic output).
[[nodiscard]] std::vector<const ComponentAggregate*>
orderByScoreDescStepsAsc(const std::vector<ComponentAggregate>& components);

} // namespace simulator
