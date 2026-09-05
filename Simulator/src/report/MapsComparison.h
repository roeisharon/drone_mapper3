#pragma once

#include <Common/IMap3D.h>
#include <Common/Types.h>

#include <vector>

namespace simulator {

class MapsComparison {
public:
    // Scores each target map against the authoritative origin (ground-truth) map; 100 = identical
    // over the scored region, 0 = nothing matches. A null target entry scores the error sentinel
    // -1.0 (defensive guard — a caller bug must not crash the comparison of the other targets).
    [[nodiscard]] static std::vector<double> compare(const common::IMap3D& origin,
                                                     const std::vector<const common::IMap3D*>& targets);
};

} // namespace simulator
