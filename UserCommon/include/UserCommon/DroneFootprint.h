#pragma once

#include <Common/IMap3D.h>
#include <Common/Units.h>

#include <cmath>

namespace user_common_322526864_323020347 {

// Contained import of the staff types for this header's geometry code — the same pattern the
// staff's own MissionControl/IDroneControl.h uses (`using namespace common;` inside its namespace).
using namespace common;

// Tests if a spherical drone (radius at center) intersects any map voxels where is_blocking() is true.
// Used for path planning (blocks on Occupied/PotentiallyOccupied) and movement legality (blocks on Occupied).
// Strict penetration required: distance(center, voxel-AABB) < radius. Touching a wall face fits,
// allowing a drone of diameter D to navigate a corridor of width D. A radius of 0 reduces to a single-cell test.
template <typename BlockingPred>
[[nodiscard]] bool footprintFits(const IMap3D& map,
                                 const Position3D& center,
                                 PhysicalLength radius,
                                 BlockingPred is_blocking) {
    const types::MapConfig cfg = map.getMapConfig();
    const double res = cfg.resolution.numerical_value_in(cm);
    if (res <= 0.0) {
        return true; // degenerate map: nothing meaningful to test against
    }

    const double r = radius.numerical_value_in(cm);
    if (r <= 0.0) {
        return !is_blocking(map.atVoxel(center)); // point drone: only its own voxel matters
    }

    const double cx = center.x.numerical_value_in(cm);
    const double cy = center.y.numerical_value_in(cm);
    const double cz = center.z.numerical_value_in(cm);
    const double ox = cfg.offset.x.numerical_value_in(cm);
    const double oy = cfg.offset.y.numerical_value_in(cm);
    const double oz = cfg.offset.z.numerical_value_in(cm);

    // World coordinate -> array index uses floor((world + offset) / res), matching Map3DImpl. The
    // sphere can only reach cells whose index lies within the [center - r, center + r] box.
    const auto cellIndex = [res](double world, double off) {
        return static_cast<long long>(std::floor((world + off) / res));
    };
    const long long lo_x = cellIndex(cx - r, ox), hi_x = cellIndex(cx + r, ox);
    const long long lo_y = cellIndex(cy - r, oy), hi_y = cellIndex(cy + r, oy);
    const long long lo_z = cellIndex(cz - r, oz), hi_z = cellIndex(cz + r, oz);

    // Strict-penetration threshold; the tiny epsilon keeps an exactly-touching face from counting.
    constexpr double kEpsCm = 1e-6;
    const double thresh = r - kEpsCm;
    const double thresh_sq = thresh * thresh;

    // Distance from a coordinate to a cell's [lo, hi] span on one axis (0 when inside).
    const auto axisGap = [](double c, double lo, double hi) {
        if (c < lo) return lo - c;
        if (c > hi) return c - hi;
        return 0.0;
    };

    for (long long ix = lo_x; ix <= hi_x; ++ix) {
        const double cell_lo_x = static_cast<double>(ix) * res - ox;
        const double dx = axisGap(cx, cell_lo_x, cell_lo_x + res);
        for (long long iy = lo_y; iy <= hi_y; ++iy) {
            const double cell_lo_y = static_cast<double>(iy) * res - oy;
            const double dy = axisGap(cy, cell_lo_y, cell_lo_y + res);
            for (long long iz = lo_z; iz <= hi_z; ++iz) {
                const double cell_lo_z = static_cast<double>(iz) * res - oz;
                const double dz = axisGap(cz, cell_lo_z, cell_lo_z + res);

                if (dx * dx + dy * dy + dz * dz >= thresh_sq) {
                    continue; // this cell lies entirely outside the sphere
                }
                // Sample the cell centre so atVoxel resolves to exactly (ix, iy, iz).
                const Position3D sample{
                    (cell_lo_x + 0.5 * res) * x_extent[cm],
                    (cell_lo_y + 0.5 * res) * y_extent[cm],
                    (cell_lo_z + 0.5 * res) * z_extent[cm],
                };
                if (is_blocking(map.atVoxel(sample))) {
                    return false;
                }
            }
        }
    }
    return true;
}

// Leading-edge scan-before-enter test. Returns true if all newly-entered voxels
// (within `radius` of `dest` but not `from`) are confirmed Empty. Shared voxels
// are skipped as they are already known clear. Fails if any new voxel is Unmapped
// (needs scanning) or Occupied. A point drone (radius 0) just checks the dest voxel.
[[nodiscard]] inline bool leadingFootprintClear(const IMap3D& map,
                                                const Position3D& from,
                                                const Position3D& dest,
                                                PhysicalLength radius) {
    const types::MapConfig cfg = map.getMapConfig();
    const double res = cfg.resolution.numerical_value_in(cm);
    if (res <= 0.0) {
        return true; // degenerate map: nothing meaningful to test against
    }
    const double r = radius.numerical_value_in(cm);
    if (r <= 0.0) {
        return map.atVoxel(dest) == types::VoxelOccupancy::Empty; // point drone: only the dest voxel
    }

    const double ox = cfg.offset.x.numerical_value_in(cm);
    const double oy = cfg.offset.y.numerical_value_in(cm);
    const double oz = cfg.offset.z.numerical_value_in(cm);

    const double dx_c = dest.x.numerical_value_in(cm);
    const double dy_c = dest.y.numerical_value_in(cm);
    const double dz_c = dest.z.numerical_value_in(cm);
    const double fx_c = from.x.numerical_value_in(cm);
    const double fy_c = from.y.numerical_value_in(cm);
    const double fz_c = from.z.numerical_value_in(cm);

    const auto cellIndex = [res](double world, double off) {
        return static_cast<long long>(std::floor((world + off) / res));
    };
    const long long lo_x = cellIndex(dx_c - r, ox), hi_x = cellIndex(dx_c + r, ox);
    const long long lo_y = cellIndex(dy_c - r, oy), hi_y = cellIndex(dy_c + r, oy);
    const long long lo_z = cellIndex(dz_c - r, oz), hi_z = cellIndex(dz_c + r, oz);

    constexpr double kEpsCm = 1e-6;
    const double thresh = r - kEpsCm;
    const double thresh_sq = thresh * thresh;

    const auto axisGap = [](double c, double lo, double hi) {
        if (c < lo) return lo - c;
        if (c > hi) return c - hi;
        return 0.0;
    };

    for (long long ix = lo_x; ix <= hi_x; ++ix) {
        const double cell_lo_x = static_cast<double>(ix) * res - ox;
        const double dxd = axisGap(dx_c, cell_lo_x, cell_lo_x + res);
        const double dxf = axisGap(fx_c, cell_lo_x, cell_lo_x + res);
        for (long long iy = lo_y; iy <= hi_y; ++iy) {
            const double cell_lo_y = static_cast<double>(iy) * res - oy;
            const double dyd = axisGap(dy_c, cell_lo_y, cell_lo_y + res);
            const double dyf = axisGap(fy_c, cell_lo_y, cell_lo_y + res);
            for (long long iz = lo_z; iz <= hi_z; ++iz) {
                const double cell_lo_z = static_cast<double>(iz) * res - oz;
                const double dzd = axisGap(dz_c, cell_lo_z, cell_lo_z + res);
                const double dzf = axisGap(fz_c, cell_lo_z, cell_lo_z + res);

                if (dxd * dxd + dyd * dyd + dzd * dzd >= thresh_sq) {
                    continue; // outside the destination footprint → not entered by this move
                }
                if (dxf * dxf + dyf * dyf + dzf * dzf < thresh_sq) {
                    continue; // already inside the current footprint → reused, known clear
                }
                // Newly-entered voxel: it must be confirmed Empty for the move to be safe to commit.
                const Position3D sample{
                    (cell_lo_x + 0.5 * res) * x_extent[cm],
                    (cell_lo_y + 0.5 * res) * y_extent[cm],
                    (cell_lo_z + 0.5 * res) * z_extent[cm],
                };
                if (map.atVoxel(sample) != types::VoxelOccupancy::Empty) {
                    return false;
                }
            }
        }
    }
    return true;
}

} // namespace user_common_322526864_323020347
