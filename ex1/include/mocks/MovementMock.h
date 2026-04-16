#pragma once

#include "driver/IMovementDriver.h"
#include "mocks/MockWorld.h"

namespace drone {

// Movement mock that updates the simulated drone position. The mock checks
// the building truth along the requested motion path. If the path passes
// through an occupied voxel, the move is reported as a collision and the
// drone is left in its previous position so that the simulator can stop
// gracefully.
class MovementMock : public IMovementDriver {
public:
    explicit MovementMock(MockWorld& world) : world_(world) {}

    MoveResult rotate(RotateDirection dir, units::Angle amount) override;
    MoveResult advance(units::Length amount) override;
    MoveResult elevate(units::Length amount) override;

private:
    MoveResult try_translate(double dx_cm, double dy_cm, double dz_cm);

    MockWorld& world_;
};

} // namespace drone
