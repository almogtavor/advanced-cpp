#pragma once

#include "sensors/LidarFrame.h"
#include "types/Position.h"
#include "units/Units.h"

namespace drone {

// Lidar sensor interface seen by the drone. The drone treats this as a
// black box; in the simulator it is implemented by LidarMock which knows
// the building truth.
class ILidarSensor {
public:
    virtual ~ILidarSensor() = default;

    // Performs a scan offset by the given angles relative to the drone's
    // current orientation. xy_angle = 0 means "straight ahead in the
    // current yaw direction"; pitch = 0 means "horizontal".
    virtual LidarFrame scan(units::Angle xy_offset,
                            units::Angle pitch_offset) = 0;
};

class IPositionSensor {
public:
    virtual ~IPositionSensor() = default;
    virtual Position get_position() const = 0;
    virtual units::Angle get_yaw() const = 0;
};

} // namespace drone
