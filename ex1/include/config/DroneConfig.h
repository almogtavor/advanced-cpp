#pragma once

#include "units/Units.h"

namespace drone {

// Capabilities of the drone hardware. Loaded from drone_config.txt.
//
// Lidar geometry follows the assignment specification:
//   - Circle 0 is the central beam, always emitted.
//   - Outer circles 1..FOVC-1 are concentric rings around the central beam.
//     Each circle k has 4^k evenly-spaced beams.
//   - The radius of circle k at distance Z-min is k*D.
//   - Z-min is the minimum measurable distance; Z-max is the maximum range.
struct DroneConfig {
    // Smallest passage the drone is willing to attempt to enter.
    units::Length min_passage_width  {30 * units::cm};
    units::Length min_passage_length {30 * units::cm};
    units::Length min_passage_height {50 * units::cm};

    // Lidar geometry (per assignment specification).
    units::Length lidar_z_min {20 * units::cm};   // Z-min
    units::Length lidar_z_max {120 * units::cm};  // Z-max
    units::Length lidar_d     {2.5 * units::cm};  // circle spacing at Z-min
    int           lidar_fovc  {5};                // number of beam circles (>=1)

    // Maximum movement allowed in a single command.
    units::Angle  max_rotate_per_cmd  {45 * units::deg};
    units::Length max_advance_per_cmd {100 * units::cm};
    units::Length max_elevate_per_cmd {100 * units::cm};
};

} // namespace drone
