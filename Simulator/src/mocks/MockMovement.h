#pragma once

#include <Common/IDroneMovement.h>
#include <Common/IMap3D.h>

#include "mocks/MockGPS.h"

namespace simulator {

// Simulation-only movement driver. As a mock it holds the real (hidden) map, so it is the one
// component that can physically detect a collision the drone's own sensors have not seen yet.
class MockMovement final : public common::IDroneMovement {
public:
    explicit MockMovement(MockGPS& gps);

    // Collision-aware overload: advance() and elevate() validate moves against the ground-truth
    // map. A move whose spherical footprint (drone_radius) would penetrate an Occupied voxel
    // THROWS std::runtime_error and leaves the drone in place — the common-issues table's
    // Mandatory row 5: the movement mock is the one component holding the real map, so it is the
    // detector; the drone controller and simulation run are the targets that catch and classify.
    // The plain constructor disables collision checking (always succeeds).
    MockMovement(MockGPS& gps, const common::IMap3D& hidden_map, common::PhysicalLength drone_radius);

    common::types::MovementResult rotate(common::types::RotationDirection direction,
                                         common::HorizontalAngle angle) override;
    common::types::MovementResult advance(common::PhysicalLength distance) override;
    common::types::MovementResult elevate(common::PhysicalLength distance) override;

private:
    // Returns true if the drone's footprint fits at dest in the hidden map (always true when no
    // hidden map was injected). Shared by advance() and elevate().
    [[nodiscard]] bool destinationClear(const common::Position3D& dest) const;

    MockGPS& gps_;
    const common::IMap3D* hidden_map_ = nullptr; // null → collision checking disabled
    common::PhysicalLength drone_radius_{};
};

} // namespace simulator
