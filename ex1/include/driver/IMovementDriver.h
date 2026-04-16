#pragma once

#include "units/Units.h"

namespace drone {

enum class RotateDirection { Left, Right };
enum class MoveResult      { Ok, Collision, OutOfRange, Clamped };

// Movement driver interface seen by the drone. The drone never directly
// touches the simulated world; all motion goes through this interface.
class IMovementDriver {
public:
    virtual ~IMovementDriver() = default;

    virtual MoveResult rotate(RotateDirection dir, units::Angle amount) = 0;
    virtual MoveResult advance(units::Length amount) = 0;
    virtual MoveResult elevate(units::Length amount) = 0;
};

} // namespace drone
