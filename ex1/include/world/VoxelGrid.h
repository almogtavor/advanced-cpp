#pragma once

#include <cstdint>
#include <vector>

#include "types/Position.h"
#include "units/Units.h"

namespace drone {

// Voxel value semantics, matching the assignment specification:
//   0  -> empty
//   1  -> occupied
//  -1  -> not mapped (drone could not reach / determine)
//  -2  -> outside required mapping boundaries
namespace voxel {
constexpr int8_t kEmpty       = 0;
constexpr int8_t kOccupied    = 1;
constexpr int8_t kUnmapped    = -1;
constexpr int8_t kOutOfBounds = -2;
} // namespace voxel

// A regular axis-aligned 3D voxel grid. The grid is defined by:
//   - an origin (lower-front-left corner of cell {0,0,0}), in cm
//   - a uniform cell side length, in cm
//   - the number of cells along each axis
//
// The grid is the shared coordinate frame for both the building "truth"
// map (used by the mock sensors) and the drone's accumulated knowledge map.
class VoxelGrid {
public:
    VoxelGrid() = default;
    VoxelGrid(units::Length cell_size,
              Position origin,
              int nx, int ny, int nz,
              int8_t fill_value);

    units::Length cell_size() const { return cell_size_; }
    Position origin()         const { return origin_; }
    int nx() const { return nx_; }
    int ny() const { return ny_; }
    int nz() const { return nz_; }

    bool in_bounds(Cell c) const;

    // Cell <-> continuous coordinate conversions.
    Cell     cell_at(Position p) const;
    Position center_of(Cell c) const;

    // Voxel access. Out-of-grid coordinates clamp to kOutOfBounds on read
    // and are silently ignored on write.
    int8_t get(Cell c) const;
    void   set(Cell c, int8_t v);

    // Range helpers used by the scoring routine.
    long total_cells() const { return static_cast<long>(nx_) * ny_ * nz_; }

private:
    int index(Cell c) const { return (c.z * ny_ + c.y) * nx_ + c.x; }

    units::Length cell_size_{1 * units::cm};
    Position origin_{};
    int nx_{0};
    int ny_{0};
    int nz_{0};
    std::vector<int8_t> data_{};
};

} // namespace drone
