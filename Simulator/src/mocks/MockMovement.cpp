#include "mocks/MockMovement.h"

#include <UserCommon/DroneFootprint.h>

#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>

namespace simulator {

using namespace common;
namespace uc = user_common_322526864_323020347;

MockMovement::MockMovement(MockGPS& gps) : gps_(gps) {}

MockMovement::MockMovement(MockGPS& gps, const IMap3D& hidden_map, PhysicalLength drone_radius)
    : gps_(gps), hidden_map_(&hidden_map), drone_radius_(drone_radius) {}

bool MockMovement::destinationClear(const Position3D& dest) const {
    if (hidden_map_ == nullptr) {
        return true; // collision checking disabled (plain ctor)
    }
    // The hidden map is ground truth: only solid (Occupied) voxels block the sphere.
    return uc::footprintFits(*hidden_map_, dest, drone_radius_,
                             [](common::types::VoxelOccupancy occ) {
                                 return occ == common::types::VoxelOccupancy::Occupied;
                             });
}

common::types::MovementResult MockMovement::rotate(common::types::RotationDirection direction,
                                                   HorizontalAngle angle) {
    const Orientation current = gps_.heading();
    // Left rotation adds the angle; right rotation subtracts it.
    const HorizontalAngle signed_angle =
        (direction == common::types::RotationDirection::Left) ? angle : -angle;
    // Wrap to [0, 360) so heading never accumulates past ±360°.
    double raw = std::fmod((current.horizontal + signed_angle).numerical_value_in(deg), 360.0);
    if (raw < 0.0) raw += 360.0;
    gps_.setHeading(Orientation{raw * deg, current.altitude});
    return common::types::MovementResult{true, {}};
}

common::types::MovementResult MockMovement::advance(PhysicalLength distance) {
    const Position3D pos = gps_.position();
    // Project the distance onto the X-Y plane using the current horizontal heading:
    // heading 0° → +X, heading 90° → +Y. std::cos/std::sin take radians, so convert.
    const double heading_rad =
        gps_.heading().horizontal.numerical_value_in(deg) * std::numbers::pi / 180.0;
    const double dist_cm = distance.numerical_value_in(cm);
    const double dx = dist_cm * std::cos(heading_rad);
    const double dy = dist_cm * std::sin(heading_rad);
    const Position3D dest{
        pos.x + dx * x_extent[cm],
        pos.y + dy * y_extent[cm],
        pos.z, // advance never changes altitude
    };
    // Physical collision: the drone's body would penetrate solid geometry. Throw (Mandatory
    // row 5) — the drone stays put; the drone controller classifies faulty-command vs discovery.
    if (!destinationClear(dest)) {
        throw std::runtime_error(
            "DRONE_HITS_OBSTACLE: advance to (" + std::to_string(dest.x.numerical_value_in(cm)) +
            ", " + std::to_string(dest.y.numerical_value_in(cm)) + ", " +
            std::to_string(dest.z.numerical_value_in(cm)) + ") cm blocked by occupied voxel");
    }
    gps_.setPosition(dest);
    return common::types::MovementResult{true, {}};
}

common::types::MovementResult MockMovement::elevate(PhysicalLength distance) {
    // Purely vertical: a negative distance descends. No clamping here — clamping is
    // the drone controller's responsibility.
    const Position3D pos = gps_.position();
    const Position3D dest{
        pos.x,
        pos.y,
        pos.z + distance.numerical_value_in(cm) * z_extent[cm],
    };
    // Physical collision with a floor/ceiling/solid: throw (Mandatory row 5); the drone stays put.
    if (!destinationClear(dest)) {
        throw std::runtime_error(
            "DRONE_HITS_OBSTACLE: elevate to height " +
            std::to_string(dest.z.numerical_value_in(cm)) + " cm blocked by occupied voxel");
    }
    gps_.setPosition(dest);
    return common::types::MovementResult{true, {}};
}

} // namespace simulator
