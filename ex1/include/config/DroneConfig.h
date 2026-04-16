#pragma once

#include "units/Units.h"

namespace drone {

// Capabilities of the drone hardware. Loaded from drone_config.txt.
struct DroneConfig {
    // Smallest passage the drone is willing to attempt to enter.
    units::Length min_passage_width  {30 * units::cm};
    units::Length min_passage_length {30 * units::cm};
    units::Length min_passage_height {50 * units::cm};

    // Lidar sensor capabilities.
    units::Angle  lidar_fov          {60 * units::deg};
    units::Length lidar_min_range    {10 * units::cm};
    units::Length lidar_max_range    {500 * units::cm};

    // Lidar resolution: side length of one scan cell at two reference
    // distances. Linear interpolation is used in between.
    units::Length lidar_res_dist_a   {100 * units::cm};
    units::Length lidar_res_side_a   {5 * units::cm};
    units::Length lidar_res_dist_b   {500 * units::cm};
    units::Length lidar_res_side_b   {25 * units::cm};

    // Maximum movement allowed in a single command.
    units::Angle  max_rotate_per_cmd  {45 * units::deg};
    units::Length max_advance_per_cmd {100 * units::cm};
    units::Length max_elevate_per_cmd {100 * units::cm};
};

} // namespace drone
