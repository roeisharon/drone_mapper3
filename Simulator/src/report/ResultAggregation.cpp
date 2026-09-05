#include "report/ResultAggregation.h"

#include <algorithm>
#include <utility>

namespace simulator {

RunOutcome runOutcome(const types::SimulationResult& run) {
    RunOutcome outcome;
    outcome.score = run.mission_score;
    if (!run.mission_results.empty()) {
        outcome.steps = run.mission_results.front().steps;
    }
    return outcome;
}

ComponentAggregate aggregateComponent(std::string name,
                                      const std::vector<types::SimulationResult>& runs) {
    ComponentAggregate agg;
    agg.name = std::move(name);
    agg.outcomes.reserve(runs.size());
    for (const auto& run : runs) {
        const RunOutcome outcome = runOutcome(run);
        agg.outcomes.push_back(outcome);
        if (outcome.score >= 0.0) {
            agg.total_score += outcome.score;
            agg.total_steps += outcome.steps;
        }
    }
    return agg;
}

bool sameResults(const ComponentAggregate& a, const ComponentAggregate& b) {
    if (a.outcomes.size() != b.outcomes.size()) {
        return false;
    }
    for (std::size_t i = 0; i < a.outcomes.size(); ++i) {
        // Exact double equality is intended: identical behaviour produces bit-identical scores
        // (same inputs through the same scoring code); anything else is a genuine difference.
        if (a.outcomes[i].score != b.outcomes[i].score ||
            a.outcomes[i].steps != b.outcomes[i].steps) {
            return false;
        }
    }
    return true;
}

std::vector<std::vector<const ComponentAggregate*>>
groupBySameResults(const std::vector<ComponentAggregate>& components) {
    std::vector<std::vector<const ComponentAggregate*>> groups;
    for (const auto& component : components) {
        bool placed = false;
        for (auto& group : groups) {
            if (sameResults(*group.front(), component)) {
                group.push_back(&component);
                placed = true;
                break;
            }
        }
        if (!placed) {
            groups.push_back({&component});
        }
    }

    // Spec sort rule: by number of agreeing members, descending. Tie-break by first member's name
    // for deterministic output across runs.
    std::stable_sort(groups.begin(), groups.end(),
                     [](const auto& a, const auto& b) {
                         if (a.size() != b.size()) return a.size() > b.size();
                         return a.front()->name < b.front()->name;
                     });
    return groups;
}

std::vector<const ComponentAggregate*>
orderByScoreDescStepsAsc(const std::vector<ComponentAggregate>& components) {
    std::vector<const ComponentAggregate*> ordered;
    ordered.reserve(components.size());
    for (const auto& component : components) {
        ordered.push_back(&component);
    }
    std::stable_sort(ordered.begin(), ordered.end(),
                     [](const ComponentAggregate* a, const ComponentAggregate* b) {
                         if (a->total_score != b->total_score) {
                             return a->total_score > b->total_score; // score descending
                         }
                         if (a->total_steps != b->total_steps) {
                             return a->total_steps < b->total_steps; // then steps ascending
                         }
                         return a->name < b->name; // deterministic exact-tie break
                     });
    return ordered;
}

} // namespace simulator
