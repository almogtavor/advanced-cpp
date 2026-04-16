#pragma once

#include <vector>

#include "units/Units.h"

namespace drone {

// A single lidar scan result. The cells form a square matrix covering the
// FOV cone. cell.distance values follow the assignment specification:
//   >0  -> distance to the closest hit, in cm
//   -1  -> nothing detected within the effective range
//   -2  -> something detected closer than the minimum range
struct LidarFrame {
    int side{0};                       // matrix is side x side
    units::Angle fov{};                // total angular extent
    units::Angle yaw_offset{};         // yaw at which the scan was taken
    units::Angle pitch_offset{};       // pitch at which the scan was taken
    std::vector<double> distance_cm{}; // length = side*side, row-major

    double at(int row, int col) const { return distance_cm[row * side + col]; }
};

} // namespace drone
