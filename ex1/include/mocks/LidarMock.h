#pragma once

#include "mocks/MockWorld.h"
#include "sensors/ILidarSensor.h"

namespace drone {

// Lidar mock that ray-casts against the building truth voxel grid.
//
// The number of rays is derived from the lidar resolution at the maximum
// effective distance, so that the matrix actually reflects the configured
// resolution. Each ray is stepped through the voxel grid until either the
// max range is exceeded or an occupied voxel is hit.
class LidarMock : public ILidarSensor {
public:
    explicit LidarMock(const MockWorld& world) : world_(world) {}

    LidarFrame scan(units::Angle xy_offset,
                    units::Angle pitch_offset) override;

private:
    int compute_grid_side() const;
    double cast_ray(double x_cm, double y_cm, double z_cm,
                    double dx, double dy, double dz) const;

    const MockWorld& world_;
};

} // namespace drone
