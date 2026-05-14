#include "mocks/LidarMock.h"

#include <cmath>

namespace drone {

namespace {
constexpr double kPi = 3.14159265358979323846;

// Number of beams in circle k (k=0 -> 1, k=1 -> 4, k=2 -> 16, ...).
int beams_on_circle(int circle_index) {
    int n = 1;
    for (int i = 0; i < circle_index; ++i) n *= 4;
    return n;
}
} // namespace

double LidarMock::cast_ray(double x_cm, double y_cm, double z_cm,
                           double dx, double dy, double dz,
                           double z_min_cm, double z_max_cm) const {
    const auto& grid = world_.truth.grid();
    const double cs = grid.cell_size().numerical_value_in(units::cm);
    // Step at half a cell to avoid skipping thin walls.
    const double step = cs * 0.5;

    for (double t = step; t <= z_max_cm; t += step) {
        const double cx = x_cm + dx * t;
        const double cy = y_cm + dy * t;
        const double cz = z_cm + dz * t;
        const Cell c = grid.cell_at(Position{
            cx * units::cm, cy * units::cm, cz * units::cm});
        if (!grid.in_bounds(c)) return -1.0;
        if (grid.get(c) == voxel::kOccupied) {
            return (t < z_min_cm) ? 0.0 : t;
        }
    }
    return -1.0;
}

LidarFrame LidarMock::scan(units::Angle xy_offset, units::Angle pitch_offset) {
    const auto& cfg = world_.drone_config;
    LidarFrame frame;
    frame.yaw_offset   = xy_offset;
    frame.pitch_offset = pitch_offset;
    frame.z_min        = cfg.lidar_z_min;
    frame.z_max        = cfg.lidar_z_max;

    if (cfg.lidar_fovc <= 0) return frame;

    const double z_min_cm = cfg.lidar_z_min.numerical_value_in(units::cm);
    const double z_max_cm = cfg.lidar_z_max.numerical_value_in(units::cm);
    const double d_cm     = cfg.lidar_d.numerical_value_in(units::cm);

    // Absolute centerline yaw/pitch in radians.
    const double yaw_rad = units::to_rad(
        units::normalized(world_.yaw + xy_offset));
    const double pitch_rad = units::to_rad(pitch_offset);

    // Local axes for the cone: forward, right, up.
    const double fwd_x = std::cos(pitch_rad) * std::cos(yaw_rad);
    const double fwd_y = std::cos(pitch_rad) * std::sin(yaw_rad);
    const double fwd_z = std::sin(pitch_rad);
    const double right_x = -std::sin(yaw_rad);
    const double right_y =  std::cos(yaw_rad);
    const double up_x = -std::sin(pitch_rad) * std::cos(yaw_rad);
    const double up_y = -std::sin(pitch_rad) * std::sin(yaw_rad);
    const double up_z =  std::cos(pitch_rad);

    const double ox = world_.position.x.numerical_value_in(units::cm);
    const double oy = world_.position.y.numerical_value_in(units::cm);
    const double oz = world_.position.z.numerical_value_in(units::cm);

    auto emit = [&](int circle, int idx, double azim_deg, double elev_deg,
                    double dx, double dy, double dz) {
        const double len = std::sqrt(dx*dx + dy*dy + dz*dz);
        const double nx = dx / len, ny = dy / len, nz = dz / len;
        LidarBeam b;
        b.circle      = circle;
        b.index       = idx;
        b.azimuth     = azim_deg * units::deg;
        b.elevation   = elev_deg * units::deg;
        b.distance_cm = cast_ray(ox, oy, oz, nx, ny, nz, z_min_cm, z_max_cm);
        frame.beams.push_back(b);
    };

    // Circle 0: central beam.
    emit(0, 0, 0.0, 0.0, fwd_x, fwd_y, fwd_z);

    // Circles 1..FOVC-1: ring of 4^k beams at half-angle atan(k*D / Z-min).
    for (int k = 1; k < cfg.lidar_fovc; ++k) {
        const int n = beams_on_circle(k);
        const double ring_radius_cm = k * d_cm;
        const double cone_half = std::atan2(ring_radius_cm, z_min_cm); // radians
        const double tan_half  = std::tan(cone_half);
        for (int i = 0; i < n; ++i) {
            const double theta = (2.0 * kPi * i) / static_cast<double>(n);
            const double h_off = std::cos(theta);  // along right axis
            const double v_off = std::sin(theta);  // along up axis
            const double dx = fwd_x + right_x * tan_half * h_off + up_x * tan_half * v_off;
            const double dy = fwd_y + right_y * tan_half * h_off + up_y * tan_half * v_off;
            const double dz = fwd_z +                                   up_z * tan_half * v_off;
            const double azim_deg = theta * 180.0 / kPi;
            const double elev_deg = cone_half * 180.0 / kPi;
            emit(k, i, azim_deg, elev_deg, dx, dy, dz);
        }
    }

    return frame;
}

} // namespace drone
