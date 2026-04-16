#pragma once

#include <utility>
#include <vector>

#include "types/Position.h"
#include "units/Units.h"

namespace drone {

// Per-mission parameters. Loaded from mission_config.txt.
struct MissionConfig {
    // Drone's initial position.
    Position start{};

    // Boundary polygon in the (X, Y) plane. Vertices are in cm.
    std::vector<std::pair<units::Length, units::Length>> boundary_polygon{};

    // Vertical extent of the mapping volume.
    units::Length height_min{0 * units::cm};
    units::Length height_max{300 * units::cm};

    // Required precision (decimal places) of the output map. In our
    // discretized voxel representation we use these values to round
    // continuous coordinates when querying the map.
    int xy_decimal_places     {0};
    int height_decimal_places {0};

    // Recharge stations. Always empty in exercise 1.
    std::vector<Position> recharge_positions{};
};

} // namespace drone
