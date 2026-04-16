#pragma once

#include "config/MissionConfig.h"
#include "world/VoxelGrid.h"

namespace drone {

// The drone's accumulated knowledge map. Cells inside the mission polygon
// and within the height range start as kUnmapped (-1); cells outside the
// polygon or outside [height_min, height_max] start as kOutOfBounds (-2).
//
// During the mission the drone overwrites kUnmapped cells with kEmpty (0)
// or kOccupied (1) based on lidar observations.
class BuildingMap {
public:
    BuildingMap() = default;
    BuildingMap(const MissionConfig& mission,
                units::Length cell_size,
                Position origin,
                int nx, int ny, int nz);

    // Continuous-coordinate API matching the assignment requirement
    // ("API for setting and getting each {X, Y, Height}").
    int8_t get(Position p) const;
    void   set(Position p, int8_t v);

    // Direct cell-level API used internally by the drone.
    int8_t get_cell(Cell c) const { return grid_.get(c); }
    void   set_cell(Cell c, int8_t v) { grid_.set(c, v); }

    const VoxelGrid& grid() const { return grid_; }
    VoxelGrid& grid() { return grid_; }

private:
    static bool point_in_polygon(
        double x, double y,
        const std::vector<std::pair<units::Length, units::Length>>& poly);

    VoxelGrid grid_{};
};

} // namespace drone
