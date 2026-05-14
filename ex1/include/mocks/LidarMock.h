#pragma once

#include "mocks/MockWorld.h"
#include "sensors/ILidarSensor.h"

namespace drone {

// Lidar mock that ray-casts against the building truth voxel grid.
//
// Geometry per assignment specification:
//   - Circle 0: the single central beam.
//   - Circle k (k = 1..FOVC-1): a ring of 4^k evenly-spaced beams at
//     cone half-angle atan(k*D / Z-min) from the centerline.
//
// Per the Ex1 clarification, the mock returns one record per emitted
// beam (including beams that did not hit anything), so the drone can
// reason about blind spots between rings rather than between hits.
class LidarMock : public ILidarSensor {
public:
    explicit LidarMock(const MockWorld& world) : world_(world) {}

    LidarFrame scan(units::Angle xy_offset,
                    units::Angle pitch_offset) override;

private:
    // Cast a single normalized ray and return its reported distance in
    // cm, using the encoding documented on LidarBeam::distance_cm.
    double cast_ray(double x_cm, double y_cm, double z_cm,
                    double dx, double dy, double dz,
                    double z_min_cm, double z_max_cm) const;

    const MockWorld& world_;
};

} // namespace drone
