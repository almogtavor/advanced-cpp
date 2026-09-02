#pragma once

#include <MissionControl/IDroneControl.h>
#include <Common/IDroneMovement.h>
#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IMappingAlgorithm.h>
#include <Common/IMutableMap3D.h>

namespace mission_control_323084962_212223036 {

using namespace common;

class DroneControlImpl final : public mission_control::IDroneControl {
public:
    DroneControlImpl(types::DroneConfigData drone,
                     types::MissionConfigData mission,
                     ILidar& lidar,
                     IGPS& gps,
                     IDroneMovement& movement,
                     IMutableMap3D& output_map,
                     IMappingAlgorithm& mapping_algorithm);

    [[nodiscard]] types::DroneStepResult step() override;
    [[nodiscard]] types::DroneState state() const override;

private:
    types::DroneConfigData drone_;
    types::MissionConfigData mission_;
    ILidar& lidar_;
    IGPS& gps_;
    IDroneMovement& movement_;
    IMutableMap3D& output_map_;
    IMappingAlgorithm& mapping_algorithm_;
    std::size_t step_index_ = 0;
    // Most recent LiDAR scan, forwarded to the algorithm on the next step.
    types::LidarScanResult last_scan_{};
    bool has_scan_ = false;
};

} // namespace mission_control_323084962_212223036
