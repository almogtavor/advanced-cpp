#pragma once

#include <vector>

#include "types/Position.h"
#include "units/Units.h"

namespace drone {

// Per-mission parameters. Loaded from mission_config.txt.
struct MissionConfig {
    // Drone's initial position.
    Position start{};

    // Bounded rectangle in the (X, Y) plane.
    units::Length min_x{0   * units::cm};
    units::Length max_x{100 * units::cm};
    units::Length min_y{0   * units::cm};
    units::Length max_y{100 * units::cm};

    // Vertical extent of the mapping volume.
    units::Length height_min{0   * units::cm};
    units::Length height_max{300 * units::cm};

    // Required mapping resolution. The simulator validates that these match
    // the supported resolution (== truth map cell size); a mismatch is a
    // fatal error per the assignment's resolution clarification.
    units::Length xy_resolution{10 * units::cm};
    units::Length height_resolution{10 * units::cm};

    // Recharge stations. Always empty in exercise 1.
    std::vector<Position> recharge_positions{};
};

} // namespace drone
