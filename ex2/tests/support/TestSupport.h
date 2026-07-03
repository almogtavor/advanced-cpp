#pragma once

// Shared helpers for the component and integration tests: terse constructors
// for the strongly-typed units, in-memory map builders, and voxel counters.

#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MapBuilder.h>
#include <drone_mapper/MapGeometry.h>
#include <drone_mapper/Types.h>
#include <drone_mapper/Units.h>

#include <filesystem>
#include <memory>

namespace dmtest {

namespace dm = drone_mapper;
namespace t = drone_mapper::types;
namespace g = drone_mapper::geom;

// ---- unit constructors ----------------------------------------------------

[[nodiscard]] inline dm::Position3D pos(double x, double y, double z) {
    return dm::Position3D{g::xlen(x), g::ylen(y), g::zlen(z)};
}

[[nodiscard]] inline dm::Orientation orient(double horizontal, double altitude = 0.0) {
    return dm::Orientation{g::hang(horizontal), g::aang(altitude)};
}

[[nodiscard]] inline t::MappingBounds bounds(double xmin, double xmax,
                                             double ymin, double ymax,
                                             double zmin, double zmax) {
    return t::MappingBounds{g::xlen(xmin), g::xlen(xmax), g::ylen(ymin),
                            g::ylen(ymax), g::zlen(zmin), g::zlen(zmax)};
}

[[nodiscard]] inline t::MapConfig mapConfig(const t::MappingBounds& b,
                                            const dm::Position3D& offset,
                                            double res_cm) {
    t::MapConfig cfg;
    cfg.boundaries = b;
    cfg.offset = offset;
    cfg.resolution = g::plen(res_cm);
    return cfg;
}

// A config whose offset sits at the bounds minimum corner (the usual case).
[[nodiscard]] inline t::MapConfig mapConfigAt(const t::MappingBounds& b, double res_cm) {
    return mapConfig(b, pos(g::xcm(b.min_x), g::ycm(b.min_y), g::zcm(b.min_height)), res_cm);
}

// ---- map builders ---------------------------------------------------------

[[nodiscard]] inline std::unique_ptr<dm::Map3DImpl> makeMap(const t::MapConfig& cfg,
                                                            t::VoxelOccupancy fill) {
    return std::make_unique<dm::Map3DImpl>(dm::makeOccupancyGrid(cfg, fill), cfg);
}

// ---- inspection -----------------------------------------------------------

[[nodiscard]] inline long countValue(const dm::IMap3D& map, t::VoxelOccupancy value) {
    const t::MapConfig cfg = map.getMapConfig();
    const double res = g::lcm(cfg.resolution);
    const t::MappingBounds& b = cfg.boundaries;
    const long nx = g::spanVoxels(g::xcm(b.min_x), g::xcm(b.max_x), res);
    const long ny = g::spanVoxels(g::ycm(b.min_y), g::ycm(b.max_y), res);
    const long nz = g::spanVoxels(g::zcm(b.min_height), g::zcm(b.max_height), res);
    long count = 0;
    for (long x = 0; x < nx; ++x) {
        for (long y = 0; y < ny; ++y) {
            for (long z = 0; z < nz; ++z) {
                if (map.atVoxel(g::voxelCenter(g::VoxelIndex{x, y, z}, cfg)) == value) {
                    ++count;
                }
            }
        }
    }
    return count;
}

[[nodiscard]] inline std::filesystem::path dataDir() {
    return std::filesystem::path{DRONE_MAPPER_TEST_DATA_DIR};
}

[[nodiscard]] inline std::filesystem::path dataFile(const std::string& name) {
    return dataDir() / name;
}

} // namespace dmtest
