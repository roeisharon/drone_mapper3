#pragma once

#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IMap3D.h>

namespace simulator {

// Simulation-only lidar: ray-marches beams against the hidden ground-truth map (which a real
// lidar obviously cannot hold — the staff structuring doc places exactly this kind of fake here).
class MockLidar final : public common::ILidar {
public:
    MockLidar(common::types::LidarConfigData config, const common::IMap3D& map,
              const common::IGPS& gps);

    [[nodiscard]] common::types::LidarScanResult scan(
        common::Orientation scan_orientation) const override;
    [[nodiscard]] common::types::LidarConfigData config() const override;

private:
    [[nodiscard]] common::PhysicalLength traceBeam(const common::Orientation& beam) const;

    common::types::LidarConfigData config_;
    const common::IMap3D& map_;
    const common::IGPS& gps_;
};

} // namespace simulator
