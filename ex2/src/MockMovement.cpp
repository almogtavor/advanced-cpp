#include <drone_mapper/MockMovement.h>

#include <drone_mapper/MapGeometry.h>

#include <cmath>
#include <numbers>

namespace drone_mapper {

namespace {
constexpr double kDegToRad = std::numbers::pi / 180.0;
} // namespace

MockMovement::MockMovement(MockGPS& gps) : gps_(gps) {}

types::MovementResult MockMovement::rotate(types::RotationDirection direction, HorizontalAngle angle) {
    // Convention: heading 0=east, 90=south, 180=west, 270=north. A Right turn
    // increases the heading angle (east -> south), a Left turn decreases it.
    const Orientation current = gps_.heading();
    const HorizontalAngle signed_angle =
        (direction == types::RotationDirection::Right) ? angle : -angle;
    gps_.setHeading(Orientation{current.horizontal + signed_angle, current.altitude});
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::advance(PhysicalLength distance) {
    // Advance moves horizontally along the current heading.
    const Orientation heading = gps_.heading();
    const Position3D pos = gps_.position();
    const double h_rad = geom::hdeg(heading.horizontal) * kDegToRad;
    const double d = geom::lcm(distance);
    gps_.setPosition(Position3D{
        geom::xlen(geom::xcm(pos.x) + d * std::cos(h_rad)),
        geom::ylen(geom::ycm(pos.y) + d * std::sin(h_rad)),
        pos.z,
    });
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::elevate(PhysicalLength distance) {
    // Elevate changes height only; the distance may be negative to descend.
    const Position3D pos = gps_.position();
    gps_.setPosition(Position3D{pos.x, pos.y, geom::zlen(geom::zcm(pos.z) + geom::lcm(distance))});
    return types::MovementResult{true, {}};
}

} // namespace drone_mapper
