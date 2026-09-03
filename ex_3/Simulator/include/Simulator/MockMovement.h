#pragma once

#include <Common/IDroneMovement.h>
#include <Common/IMap3D.h>
#include <Simulator/MockGPS.h>

namespace simulator {

using namespace common;
// Optional implementation for the 
class MockMovement final : public IDroneMovement {
public:
    // Changed from assignment 2: the mock also holds the hidden truth map.
    // MissionControl no longer sees that map, so a move that puts the drone's
    // centre inside an obstacle is reported here as a failed MovementResult.
    MockMovement(MockGPS& gps, const IMap3D& hidden_map);

    common::types::MovementResult rotate(common::types::RotationDirection direction, HorizontalAngle angle) override;
    common::types::MovementResult advance(PhysicalLength distance) override;
    common::types::MovementResult elevate(PhysicalLength distance) override;

private:
    MockGPS& gps_;
    const IMap3D& hidden_map_;
};

} // namespace simulator
