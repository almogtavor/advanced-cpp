#pragma once

#include <vector>

#include "types/Position.h"
#include "units/Units.h"

namespace drone {

// Per-mission parameters. Loaded from mission_config.txt.
struct MissionConfig {
    // Drone's initial position.
    Position start{};

    // Drone's initial XY-angle. 0 = east (+X), 90 = south (+Y),
    // 180 = west (-X), 270 = north (-Y). Optional in mission_config.txt;
    // defaults to 0.
    units::Angle start_yaw{0 * units::deg};

    // Bounded rectangle in the (X, Y) plane.
    units::Length min_x{0   * units::cm};
    units::Length max_x{100 * units::cm};
    units::Length min_y{0   * units::cm};
    units::Length max_y{100 * units::cm};

    // Vertical extent of the mapping volume.
    units::Length height_min{0   * units::cm};
    units::Length height_max{300 * units::cm};

    // Required mapping resolution, expressed as the number of decimal
    // places after the dot in meter-based output. cell_size_cm =
    // 100 / 10^N. The simulator validates this matches the supported
    // resolution (== truth map cell size); a mismatch is a fatal error
    // per the assignment's resolution clarification.
    int xy_resolution_decimals{1};
    int height_resolution_decimals{1};

    // Recharge stations. Always empty in exercise 1.
    std::vector<Position> recharge_positions{};
};

} // namespace drone
