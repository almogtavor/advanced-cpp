#pragma once

#include "world/VoxelGrid.h"

namespace drone {

// The "real" building. Used only by the mock sensors and by the scoring
// stage at the end of the simulation. The drone code itself never sees this.
class BuildingTruth {
public:
    BuildingTruth() = default;
    explicit BuildingTruth(VoxelGrid grid) : grid_(std::move(grid)) {}

    int8_t at(Cell c) const { return grid_.get(c); }
    int8_t at(Position p) const { return grid_.get(grid_.cell_at(p)); }

    bool is_occupied(Cell c) const { return at(c) == voxel::kOccupied; }
    bool is_empty(Cell c)    const { return at(c) == voxel::kEmpty; }

    const VoxelGrid& grid() const { return grid_; }
    VoxelGrid& grid() { return grid_; }

private:
    VoxelGrid grid_{};
};

} // namespace drone
