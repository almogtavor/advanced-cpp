#include "world/BuildingMap.h"

namespace drone {

BuildingMap::BuildingMap(const MissionConfig& mission,
                         units::Length cell_size,
                         Position origin,
                         int nx, int ny, int nz)
    : grid_(cell_size, origin, nx, ny, nz, voxel::kOutOfBounds) {
    for (int z = 0; z < nz; ++z) {
        for (int y = 0; y < ny; ++y) {
            for (int x = 0; x < nx; ++x) {
                const Cell c{x, y, z};
                const Position center = grid_.center_of(c);
                const bool in_height =
                    center.z >= mission.height_min &&
                    center.z <= mission.height_max;
                const bool in_xy =
                    center.x >= mission.min_x && center.x <= mission.max_x &&
                    center.y >= mission.min_y && center.y <= mission.max_y;
                if (in_height && in_xy) grid_.set(c, voxel::kUnmapped);
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

} // namespace drone
