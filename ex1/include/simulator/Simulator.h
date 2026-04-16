#pragma once

#include <string>

#include "config/DroneConfig.h"
#include "config/MissionConfig.h"
#include "drone/Drone.h"
#include "mocks/LidarMock.h"
#include "mocks/MockWorld.h"
#include "mocks/MovementMock.h"
#include "mocks/PositionMock.h"
#include "world/BuildingMap.h"
#include "world/BuildingTruth.h"

namespace drone {

// Holds the result of one full simulator run.
struct SimulationReport {
    long total_in_bounds_cells{0};
    long correct_cells{0};
    long incorrect_cells{0};
    long unmapped_cells{0};
    int  command_count{0};
    bool drone_collided{false};
    double score{0.0};
};

// Top-level orchestrator. Owns the building truth, mock world, drone
// known map and Drone instance, and runs the main "ask drone for command"
// loop until the drone reports Finished.
class Simulator {
public:
    Simulator(BuildingTruth truth,
              DroneConfig   drone_cfg,
              MissionConfig mission);

    SimulationReport run(int max_commands = 100000);

    const BuildingMap& known_map() const { return known_map_; }
    const BuildingTruth& truth()    const { return world_.truth; }

private:
    SimulationReport score_against_truth() const;

    MissionConfig mission_;
    MockWorld     world_{};
    BuildingMap   known_map_{};

    PositionMock pos_mock_;
    LidarMock    lidar_mock_;
    MovementMock move_mock_;

    Drone drone_;
};

} // namespace drone
