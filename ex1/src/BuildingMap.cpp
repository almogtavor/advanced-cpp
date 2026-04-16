#include "world/BuildingMap.h"

namespace drone {

BuildingMap::BuildingMap(const MissionConfig& mission,
                         units::Length cell_size,
                         Position origin,
                         int nx, int ny, int nz)
    : grid_(cell_size, origin, nx, ny, nz, voxel::kOutOfBounds) {
    // Walk all cells, marking the in-mission ones as kUnmapped.
    for (int z = 0; z < nz; ++z) {
        for (int y = 0; y < ny; ++y) {
            for (int x = 0; x < nx; ++x) {
                const Cell c{x, y, z};
                const Position center = grid_.center_of(c);
                const bool in_height =
                    center.z >= mission.height_min &&
                    center.z <= mission.height_max;
                if (!in_height) continue;
                if (!mission.boundary_polygon.empty()) {
                    const bool inside = point_in_polygon(
                        center.x.in_cm(), center.y.in_cm(),
                        mission.boundary_polygon);
                    if (!inside) continue;
                }
                grid_.set(c, voxel::kUnmapped);
            }
        }
    }
}

int8_t BuildingMap::get(Position p) const {
    return grid_.get(grid_.cell_at(p));
}

void BuildingMap::set(Position p, int8_t v) {
    grid_.set(grid_.cell_at(p), v);
}

bool BuildingMap::point_in_polygon(
    double x, double y,
    const std::vector<std::pair<units::Length, units::Length>>& poly) {
    // Standard ray casting; count intersections with horizontal ray to +X.
    bool inside = false;
    const std::size_t n = poly.size();
    if (n < 3) return true;
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
        const double xi = poly[i].first.in_cm();
        const double yi = poly[i].second.in_cm();
        const double xj = poly[j].first.in_cm();
        const double yj = poly[j].second.in_cm();
        const bool crosses =
            ((yi > y) != (yj > y)) &&
            (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi);
        if (crosses) inside = !inside;
    }
    return inside;
}

} // namespace drone
