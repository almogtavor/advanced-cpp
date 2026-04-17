#include "world/VoxelGrid.h"

#include <cmath>

namespace drone {

VoxelGrid::VoxelGrid(units::Length cell_size,
                     Position origin,
                     int nx, int ny, int nz,
                     int8_t fill_value)
    : cell_size_(cell_size),
      origin_(origin),
      nx_(nx),
      ny_(ny),
      nz_(nz),
      data_(static_cast<std::size_t>(nx) * ny * nz, fill_value) {}

bool VoxelGrid::in_bounds(Cell c) const {
    return c.x >= 0 && c.x < nx_ &&
           c.y >= 0 && c.y < ny_ &&
           c.z >= 0 && c.z < nz_;
}

Cell VoxelGrid::cell_at(Position p) const {
    const double cs = cell_size_.numerical_value_in(units::cm);
    const double dx = (p.x - origin_.x).numerical_value_in(units::cm) / cs;
    const double dy = (p.y - origin_.y).numerical_value_in(units::cm) / cs;
    const double dz = (p.z - origin_.z).numerical_value_in(units::cm) / cs;
    return Cell{
        static_cast<int>(std::floor(dx)),
        static_cast<int>(std::floor(dy)),
        static_cast<int>(std::floor(dz))};
}

Position VoxelGrid::center_of(Cell c) const {
    const double cs = cell_size_.numerical_value_in(units::cm);
    return Position{
        origin_.x + (c.x + 0.5) * cs * units::cm,
        origin_.y + (c.y + 0.5) * cs * units::cm,
        origin_.z + (c.z + 0.5) * cs * units::cm};
}

int8_t VoxelGrid::get(Cell c) const {
    if (!in_bounds(c)) return voxel::kOutOfBounds;
    return data_[index(c)];
}

void VoxelGrid::set(Cell c, int8_t v) {
    if (!in_bounds(c)) return;
    data_[index(c)] = v;
}

} // namespace drone
