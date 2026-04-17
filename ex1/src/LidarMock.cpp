#include "mocks/LidarMock.h"

#include <algorithm>
#include <cmath>

namespace drone {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

int LidarMock::compute_grid_side() const {
    // Resolution is the side length of one scan cell at a given distance.
    // We linearly interpolate between the two reference points and pick
    // the resolution at lidar_max_range, then divide the visible square
    // side at that distance by it to obtain the grid side count.
    const auto& cfg = world_.drone_config;
    const double d1 = cfg.lidar_res_dist_a.numerical_value_in(units::cm);
    const double s1 = cfg.lidar_res_side_a.numerical_value_in(units::cm);
    const double d2 = cfg.lidar_res_dist_b.numerical_value_in(units::cm);
    const double s2 = cfg.lidar_res_side_b.numerical_value_in(units::cm);
    const double max_d = cfg.lidar_max_range.numerical_value_in(units::cm);

    double res_at_max;
    if (d2 == d1) {
        res_at_max = s1;
    } else {
        const double t = (max_d - d1) / (d2 - d1);
        res_at_max = s1 + (s2 - s1) * t;
    }
    if (res_at_max <= 0.0) res_at_max = 1.0;

    const double half_fov_rad = units::to_rad(cfg.lidar_fov) / 2.0;
    const double visible_side = 2.0 * max_d * std::tan(half_fov_rad);
    int side = static_cast<int>(std::round(visible_side / res_at_max));
    if (side < 3) side = 3;        // ensure at least a tiny grid
    if (side % 2 == 0) ++side;     // odd so there is a true center ray
    if (side > 51) side = 51;      // sanity cap to keep ex1 cheap
    return side;
}

double LidarMock::cast_ray(double x_cm, double y_cm, double z_cm,
                           double dx, double dy, double dz) const {
    const auto& grid = world_.truth.grid();
    const double cs   = grid.cell_size().numerical_value_in(units::cm);
    const double max_d = world_.drone_config.lidar_max_range.numerical_value_in(units::cm);

    // Step in small increments. Half a cell side keeps us from skipping
    // thin walls without slowing the simulation too much.
    const double step = cs * 0.5;
    double t = 0.0;
    while (t <= max_d) {
        const double cx = x_cm + dx * t;
        const double cy = y_cm + dy * t;
        const double cz = z_cm + dz * t;
        const Cell c = grid.cell_at(Position{
            cx * units::cm, cy * units::cm, cz * units::cm});
        if (!grid.in_bounds(c)) return -1.0; // ran past the world
        if (grid.get(c) == voxel::kOccupied) {
            return t; // hit at this distance
        }
        t += step;
    }
    return -1.0; // nothing within range
}

LidarFrame LidarMock::scan(units::Angle xy_offset, units::Angle pitch_offset) {
    LidarFrame frame;
    const auto& cfg = world_.drone_config;
    frame.fov = cfg.lidar_fov;
    frame.yaw_offset = xy_offset;
    frame.pitch_offset = pitch_offset;
    frame.side = compute_grid_side();
    frame.distance_cm.assign(static_cast<std::size_t>(frame.side) * frame.side, -1.0);

    // Direction the lidar is pointing in (yaw + pitch in degrees).
    const double yaw_deg   = units::normalized(world_.yaw + xy_offset).numerical_value_in(units::deg);
    const double pitch_deg = pitch_offset.numerical_value_in(units::deg);
    const double yaw_rad   = yaw_deg * kPi / 180.0;
    const double pitch_rad = pitch_deg * kPi / 180.0;

    const double half_fov = units::to_rad(cfg.lidar_fov) / 2.0;
    const double min_d    = cfg.lidar_min_range.numerical_value_in(units::cm);

    // Build local right/up vectors so we can offset rays inside the cone.
    const double fwd_x = std::cos(pitch_rad) * std::cos(yaw_rad);
    const double fwd_y = std::cos(pitch_rad) * std::sin(yaw_rad);
    const double fwd_z = std::sin(pitch_rad);

    const double right_x = -std::sin(yaw_rad);
    const double right_y =  std::cos(yaw_rad);
    const double right_z =  0.0;

    const double up_x = -std::sin(pitch_rad) * std::cos(yaw_rad);
    const double up_y = -std::sin(pitch_rad) * std::sin(yaw_rad);
    const double up_z =  std::cos(pitch_rad);

    const double origin_x = world_.position.x.numerical_value_in(units::cm);
    const double origin_y = world_.position.y.numerical_value_in(units::cm);
    const double origin_z = world_.position.z.numerical_value_in(units::cm);

    const int side = frame.side;
    const int center = side / 2;
    for (int row = 0; row < side; ++row) {
        for (int col = 0; col < side; ++col) {
            const double v_frac = (row - center) / static_cast<double>(center);
            const double h_frac = (col - center) / static_cast<double>(center);
            const double a_h = h_frac * half_fov;
            const double a_v = v_frac * half_fov;

            // Combine forward, right, up to build the ray direction.
            const double dx = fwd_x + right_x * std::tan(a_h) + up_x * std::tan(a_v);
            const double dy = fwd_y + right_y * std::tan(a_h) + up_y * std::tan(a_v);
            const double dz = fwd_z + right_z * std::tan(a_h) + up_z * std::tan(a_v);
            const double len = std::sqrt(dx*dx + dy*dy + dz*dz);
            const double nx = dx / len;
            const double ny = dy / len;
            const double nz = dz / len;

            const double hit = cast_ray(origin_x, origin_y, origin_z, nx, ny, nz);
            double reported;
            if (hit < 0.0)        reported = -1.0;          // nothing in range
            else if (hit < min_d) reported = -2.0;          // too close
            else                  reported = hit;
            frame.distance_cm[static_cast<std::size_t>(row * side + col)] = reported;
        }
    }
    return frame;
}

} // namespace drone
