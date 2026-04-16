#pragma once

#include "config/DroneConfig.h"
#include "types/Position.h"
#include "units/Units.h"
#include "world/BuildingTruth.h"

namespace drone {

// Shared state behind the three mocks. The lidar mock, position mock and
// movement mock all reference the same MockWorld so that movement commands
// update the position that the lidar will then read from.
//
// The drone is unaware of MockWorld. The interfaces it sees are the three
// abstract sensor/driver classes.
struct MockWorld {
    BuildingTruth truth{};
    DroneConfig   drone_config{};
    Position      position{};
    units::Angle  yaw{0 * units::deg};
    bool          collided{false};
};

} // namespace drone
