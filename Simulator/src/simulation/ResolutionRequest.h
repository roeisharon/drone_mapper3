#pragma once

#include <Simulator/SimulationTypes.h>

#include <cmath>

namespace simulator {

// The single decision point for the mission's output_mapping_resolution_factor request, shared
// by the run factory (which sizes the output map) and the simulation run (which reports the
// status) so the two can never disagree.
//
// Phase 8b (bonus — different-resolution support):
//   factor == 1          → Accepted        (output map at the native map resolution)
//   integer factor  > 1  → Accepted        (output map produced at native_res × factor —
//                                           the previously-unclaimed ex2 bonus, real now)
//   factor < 1           → IgnoredTooSmall (finer than the simulation can source — the ex2
//                                           spec rule that ex2 grading bug SIM20 made core)
//   non-integer factor>1 → Ignored         (coarse cells must tile the native grid exactly;
//                                           a fractional multiple cannot)
// The parser defaults a missing factor to 1.
[[nodiscard]] inline types::ResolutionRequestStatus classifyResolutionRequest(double factor) {
    constexpr double kIntegerEps = 1e-9;
    if (factor < 1.0) {
        return types::ResolutionRequestStatus::IgnoredTooSmall;
    }
    if (std::abs(factor - std::round(factor)) <= kIntegerEps) {
        return types::ResolutionRequestStatus::Accepted;
    }
    return types::ResolutionRequestStatus::Ignored;
}

// The factor the output map is actually built with: the request when Accepted, 1 otherwise.
[[nodiscard]] inline double acceptedResolutionFactor(double factor) {
    return (classifyResolutionRequest(factor) == types::ResolutionRequestStatus::Accepted)
               ? std::round(factor)
               : 1.0;
}

} // namespace simulator
