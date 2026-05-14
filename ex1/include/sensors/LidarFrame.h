#pragma once

#include <vector>

#include "units/Units.h"

namespace drone {

// A single beam emitted by the lidar.
//
// circle / index identify the beam in the spec's geometry:
//   - circle 0 is the central beam (always one beam, index = 0).
//   - circle k (k > 0) has 4^k beams evenly spaced around the ring.
//
// (azimuth, elevation) is the direction the beam was fired in, expressed
// relative to the lidar centerline (i.e., the centerline points at the
// drone yaw + the scan's xy/pitch offset; azimuth = 0 and elevation = 0
// describe Circle 0).
//
// distance_cm follows the assignment's encoding for *reported* values:
//   >0   distance to the closest hit, in cm
//    0   something was detected closer than Z-min (too close to measure)
//   -1   no hit within Z-max (kept so we can return one record per beam,
//        as allowed by the assignment's "lidar returns info on all beams"
//        clarification for Ex1)
struct LidarBeam {
    int          circle{0};
    int          index{0};
    units::Angle azimuth{};   // angle around the lidar centerline (0..360)
    units::Angle elevation{}; // cone half-angle from the centerline (>= 0)
    double       distance_cm{-1.0};
};

// Result of a single lidar scan. yaw_offset and pitch_offset are the
// requested offsets relative to the drone's current orientation at the
// time of the scan (passed to ILidarSensor::scan).
struct LidarFrame {
    units::Angle yaw_offset{};
    units::Angle pitch_offset{};
    units::Length z_min{0 * units::cm};
    units::Length z_max{0 * units::cm};
    std::vector<LidarBeam> beams{};
};

} // namespace drone
