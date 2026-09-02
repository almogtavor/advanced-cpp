#pragma once

// Small, header-only helpers shared by the map, the scan converter, the
// mapping algorithm, and the comparison utility. They translate between the
// strongly-typed mp-units coordinates and plain doubles in centimetres, and
// between world coordinates and integer voxel indices.
//
// Voxel model (matches the .npy layout, C-order, axis order X, Y, Z):
//   flat_index = (x * ny + y) * nz + z
//   world_corner(i)  = offset + i        * resolution
//   world_center(i)  = offset + (i + 0.5) * resolution
//   voxel_index(w)   = floor((w - offset) / resolution)

#include <Common/Types.h>
#include <Common/Units.h>

#include <cmath>
#include <cstdint>

namespace user_common_323084962_212223036::geom {

using namespace common;

// ---- unit <-> double (centimetres) ---------------------------------------

[[nodiscard]] inline double xcm(XLength v) { return v.force_numerical_value_in(cm); }
[[nodiscard]] inline double ycm(YLength v) { return v.force_numerical_value_in(cm); }
[[nodiscard]] inline double zcm(ZLength v) { return v.force_numerical_value_in(cm); }
[[nodiscard]] inline double lcm(PhysicalLength v) { return v.force_numerical_value_in(cm); }

[[nodiscard]] inline XLength xlen(double c) { return c * x_extent[cm]; }
[[nodiscard]] inline YLength ylen(double c) { return c * y_extent[cm]; }
[[nodiscard]] inline ZLength zlen(double c) { return c * z_extent[cm]; }
[[nodiscard]] inline PhysicalLength plen(double c) { return c * cm; }

[[nodiscard]] inline double hdeg(HorizontalAngle a) { return a.force_numerical_value_in(deg); }
[[nodiscard]] inline double adeg(AltitudeAngle a) { return a.force_numerical_value_in(deg); }
[[nodiscard]] inline HorizontalAngle hang(double d) { return d * horizontal_angle[deg]; }
[[nodiscard]] inline AltitudeAngle aang(double d) { return d * altitude_angle[deg]; }

// A voxel index triple.
struct VoxelIndex {
    long x = 0;
    long y = 0;
    long z = 0;
    friend bool operator==(const VoxelIndex& a, const VoxelIndex& b) {
        return a.x == b.x && a.y == b.y && a.z == b.z;
    }
    friend bool operator!=(const VoxelIndex& a, const VoxelIndex& b) { return !(a == b); }
};

// ---- world <-> voxel ------------------------------------------------------

[[nodiscard]] inline long indexFor(double world_cm, double offset_cm, double res_cm) {
    return static_cast<long>(std::floor((world_cm - offset_cm) / res_cm));
}

[[nodiscard]] inline double centerFor(long index, double offset_cm, double res_cm) {
    return offset_cm + (static_cast<double>(index) + 0.5) * res_cm;
}

[[nodiscard]] inline VoxelIndex worldToVoxel(const Position3D& pos, const types::MapConfig& cfg) {
    const double res = lcm(cfg.resolution);
    return VoxelIndex{
        indexFor(xcm(pos.x), xcm(cfg.offset.x), res),
        indexFor(ycm(pos.y), ycm(cfg.offset.y), res),
        indexFor(zcm(pos.z), zcm(cfg.offset.z), res),
    };
}

[[nodiscard]] inline Position3D voxelCenter(const VoxelIndex& idx, const types::MapConfig& cfg) {
    const double res = lcm(cfg.resolution);
    return Position3D{
        xlen(centerFor(idx.x, xcm(cfg.offset.x), res)),
        ylen(centerFor(idx.y, ycm(cfg.offset.y), res)),
        zlen(centerFor(idx.z, zcm(cfg.offset.z), res)),
    };
}

// Number of voxels spanning a MappingBounds axis at the given resolution.
[[nodiscard]] inline long spanVoxels(double min_cm, double max_cm, double res_cm) {
    if (res_cm <= 0.0) {
        return 0;
    }
    const long n = static_cast<long>(std::llround(std::ceil((max_cm - min_cm) / res_cm)));
    return n > 0 ? n : 0;
}

} // namespace user_common_323084962_212223036::geom
