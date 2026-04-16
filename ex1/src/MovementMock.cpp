#include "mocks/MovementMock.h"

#include <algorithm>
#include <cmath>

namespace drone {

MoveResult MovementMock::rotate(RotateDirection dir, units::Angle amount) {
    // Clamp to the configured maximum and remember whether we clamped.
    const double max_deg = world_.drone_config.max_rotate_per_cmd.in_deg();
    double requested = amount.in_deg();
    bool clamped = false;
    if (std::abs(requested) > max_deg) {
        requested = std::copysign(max_deg, requested);
        clamped = true;
    }
    const double signed_deg = (dir == RotateDirection::Right) ? requested : -requested;
    world_.yaw = (world_.yaw + units::Angle(signed_deg)).normalized();
    return clamped ? MoveResult::Clamped : MoveResult::Ok;
}

MoveResult MovementMock::try_translate(double dx_cm, double dy_cm, double dz_cm) {
    const auto& grid = world_.truth.grid();
    const double cs = grid.cell_size().in_cm();
    const double dist = std::sqrt(dx_cm*dx_cm + dy_cm*dy_cm + dz_cm*dz_cm);
    if (dist == 0.0) return MoveResult::Ok;

    // Walk along the segment in half-cell increments and check each step.
    const int steps = std::max(1, static_cast<int>(std::ceil(dist / (cs * 0.5))));
    const double sx = dx_cm / steps;
    const double sy = dy_cm / steps;
    const double sz = dz_cm / steps;

    double x = world_.position.x.in_cm();
    double y = world_.position.y.in_cm();
    double z = world_.position.z.in_cm();

    for (int i = 1; i <= steps; ++i) {
        const double nx = x + sx;
        const double ny = y + sy;
        const double nz = z + sz;
        const Cell c = grid.cell_at(Position{
            units::Length(nx), units::Length(ny), units::Length(nz)});
        if (grid.in_bounds(c) && grid.get(c) == voxel::kOccupied) {
            world_.collided = true;
            return MoveResult::Collision;
        }
        x = nx; y = ny; z = nz;
    }

    // Snap to the nearest 0.01 cm to prevent cumulative floating-point
    // drift along axis-aligned moves (sin/cos of pi/2 etc. are not
    // exactly zero in IEEE 754, which can push the position across a
    // cell boundary after many moves).
    auto snap = [](double v) { return std::round(v * 100.0) / 100.0; };
    world_.position = Position{
        units::Length(snap(x)), units::Length(snap(y)), units::Length(snap(z))};
    return MoveResult::Ok;
}

MoveResult MovementMock::advance(units::Length amount) {
    // Clamp to max_advance_per_cmd.
    const double max_cm = world_.drone_config.max_advance_per_cmd.in_cm();
    double req = amount.in_cm();
    bool clamped = false;
    if (std::abs(req) > max_cm) {
        req = std::copysign(max_cm, req);
        clamped = true;
    }
    const double yaw_rad = world_.yaw.in_rad();
    const double dx = req * std::cos(yaw_rad);
    const double dy = req * std::sin(yaw_rad);
    auto r = try_translate(dx, dy, 0.0);
    if (r == MoveResult::Ok && clamped) return MoveResult::Clamped;
    return r;
}

MoveResult MovementMock::elevate(units::Length amount) {
    const double max_cm = world_.drone_config.max_elevate_per_cmd.in_cm();
    double req = amount.in_cm();
    bool clamped = false;
    if (std::abs(req) > max_cm) {
        req = std::copysign(max_cm, req);
        clamped = true;
    }
    auto r = try_translate(0.0, 0.0, req);
    if (r == MoveResult::Ok && clamped) return MoveResult::Clamped;
    return r;
}

} // namespace drone
