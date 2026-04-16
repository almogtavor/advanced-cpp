#pragma once

#include "mocks/MockWorld.h"
#include "sensors/ILidarSensor.h"  // brings in IPositionSensor

namespace drone {

class PositionMock : public IPositionSensor {
public:
    explicit PositionMock(const MockWorld& world) : world_(world) {}

    Position get_position() const override { return world_.position; }
    units::Angle get_yaw() const override  { return world_.yaw; }

private:
    const MockWorld& world_;
};

} // namespace drone
