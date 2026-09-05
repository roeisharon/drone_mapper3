#pragma once

#include <Common/IMutableMap3D.h>
#include <Common/Types.h>

namespace mission_control_322526864_323020347 {

// Converts one LiDAR scan into output-map voxel writes. Lives in the MissionControl project: its
// sole consumer is DroneControlImpl (the mission's own mapping loop), so it is not UserCommon.
class ScanResultToVoxels {
public:
    // Applies a LiDAR scan directly to the output map.
    // The converter writes only scan observation states:
    // Occupied, Empty, and PotentiallyOccupied.
    static void applyToMap(common::IMutableMap3D& output_map,
                           const common::Position3D& scan_origin,
                           const common::Orientation& drone_heading,
                           const common::types::LidarScanResult& scan,
                           const common::types::LidarConfigData& lidar_config);
};

} // namespace mission_control_322526864_323020347
