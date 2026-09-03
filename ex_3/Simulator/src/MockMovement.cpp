#include <Simulator/MockMovement.h>

#include <UserCommon/ErrorCodes.h>
#include <UserCommon/MapGeometry.h>

#include <cmath>
#include <numbers>

namespace simulator {

using namespace common;
using namespace user_common_323084962_212223036;

namespace {
constexpr double kDegToRad = std::numbers::pi / 180.0;
} // namespace

MockMovement::MockMovement(MockGPS& gps, const IMap3D& hidden_map)
    : gps_(gps), hidden_map_(hidden_map) {}

namespace {

// A crash is the drone driving its centre into an obstacle. The spherical body
// is kept clear of walls by the mapping algorithm's clearance, so a move is not
// failed merely because the body scratches a neighbouring voxel.
[[nodiscard]] common::types::MovementResult collisionCheck(const common::IMap3D& map,
                                                           const common::Position3D& pos) {
    if (map.atVoxel(pos) == common::types::VoxelOccupancy::Occupied) {
        return common::types::MovementResult{
            false, user_common_323084962_212223036::kDroneHitsObstacleMessage};
    }
    return common::types::MovementResult{true, {}};
}

} // namespace

common::types::MovementResult MockMovement::rotate(common::types::RotationDirection direction, HorizontalAngle angle) {
    // Convention: heading 0=east, 90=south, 180=west, 270=north. A Right turn
    // increases the heading angle (east -> south), a Left turn decreases it.
    const Orientation current = gps_.heading();
    const HorizontalAngle signed_angle =
        (direction == common::types::RotationDirection::Right) ? angle : -angle;
    gps_.setHeading(Orientation{current.horizontal + signed_angle, current.altitude});
    return common::types::MovementResult{true, {}};
}

common::types::MovementResult MockMovement::advance(PhysicalLength distance) {
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
    return collisionCheck(hidden_map_, gps_.position());
}

common::types::MovementResult MockMovement::elevate(PhysicalLength distance) {
    // Elevate changes height only; the distance may be negative to descend.
    const Position3D pos = gps_.position();
    gps_.setPosition(Position3D{pos.x, pos.y, geom::zlen(geom::zcm(pos.z) + geom::lcm(distance))});
    return collisionCheck(hidden_map_, gps_.position());
}

} // namespace simulator
