#include <drone_mapper/MapBuilder.h>

#include <drone_mapper/MapGeometry.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace drone_mapper {

ResolutionDecision decideResolution(const types::MissionConfigData& mission) {
    const double factor = mission.output_mapping_resolution_factor;
    const PhysicalLength gps = mission.gps_resolution;

    if (factor < 1.0) {
        return ResolutionDecision{gps, types::ResolutionRequestStatus::IgnoredTooSmall};
    }
    if (std::floor(factor) != factor) {
        return ResolutionDecision{gps, types::ResolutionRequestStatus::Ignored};
    }
    return ResolutionDecision{factor * gps, types::ResolutionRequestStatus::Accepted};
}

types::MapConfig makeOutputMapConfig(const types::MissionConfigData& mission) {
    const ResolutionDecision decision = decideResolution(mission);
    const types::MappingBounds& b = mission.mission_bounds;

    types::MapConfig config;
    config.boundaries = b;
    config.offset = Position3D{
        geom::xlen(geom::xcm(b.min_x)),
        geom::ylen(geom::ycm(b.min_y)),
        geom::zlen(geom::zcm(b.min_height)),
    };
    config.resolution = decision.resolution;
    return config;
}

types::MapConfig makeHiddenMapConfig(const types::SimulationConfigData& sim,
                                     const NpyArray& loaded) {
    const double res = geom::lcm(sim.map_resolution);
    const NpyArray::shape_t& shape = loaded.Shape();
    const double nx = shape.size() == 3 ? static_cast<double>(shape[0]) : 0.0;
    const double ny = shape.size() == 3 ? static_cast<double>(shape[1]) : 0.0;
    const double nz = shape.size() == 3 ? static_cast<double>(shape[2]) : 0.0;

    const double ox = geom::xcm(sim.map_offset.x);
    const double oy = geom::ycm(sim.map_offset.y);
    const double oz = geom::zcm(sim.map_offset.z);

    types::MapConfig config;
    config.offset = sim.map_offset;
    config.resolution = sim.map_resolution;
    config.boundaries = types::MappingBounds{
        geom::xlen(ox), geom::xlen(ox + nx * res),
        geom::ylen(oy), geom::ylen(oy + ny * res),
        geom::zlen(oz), geom::zlen(oz + nz * res),
    };
    return config;
}

std::shared_ptr<NpyArray> loadHiddenMapArray(const std::filesystem::path& path) {
    auto arr = std::make_shared<NpyArray>();
    const char* err = arr->LoadNPY(path.string());
    if (err != nullptr) {
        throw std::runtime_error("Failed to load map '" + path.string() + "': " + err);
    }
    if (arr->Shape().size() != 3) {
        throw std::runtime_error("Map '" + path.string() + "' must be a 3D array.");
    }
    if (arr->SizeValueBytes() != 1) {
        throw std::runtime_error("Map '" + path.string() + "' must use a 1-byte voxel type.");
    }
    // Normalise every voxel to Empty(0)/Occupied(1) so Map3DImpl can read the
    // raw bytes directly as VoxelOccupancy codes.
    std::int8_t* data = arr->Data<std::int8_t>();
    const std::size_t count = arr->NumValue();
    for (std::size_t i = 0; i < count; ++i) {
        data[i] = (data[i] != 0)
                      ? static_cast<std::int8_t>(types::VoxelOccupancy::Occupied)
                      : static_cast<std::int8_t>(types::VoxelOccupancy::Empty);
    }
    return arr;
}

std::shared_ptr<NpyArray> makeOccupancyGrid(const types::MapConfig& config,
                                            types::VoxelOccupancy fill) {
    const double res = geom::lcm(config.resolution);
    const types::MappingBounds& b = config.boundaries;
    const long nx = std::max<long>(1, geom::spanVoxels(geom::xcm(b.min_x), geom::xcm(b.max_x), res));
    const long ny = std::max<long>(1, geom::spanVoxels(geom::ycm(b.min_y), geom::ycm(b.max_y), res));
    const long nz = std::max<long>(1, geom::spanVoxels(geom::zcm(b.min_height), geom::zcm(b.max_height), res));

    NpyArray::shape_t shape{static_cast<std::size_t>(nx),
                            static_cast<std::size_t>(ny),
                            static_cast<std::size_t>(nz)};
    auto arr = std::make_shared<NpyArray>(shape, static_cast<std::size_t>(1), 'i');
    arr->Allocate();
    std::int8_t* data = arr->Data<std::int8_t>();
    std::fill(data, data + arr->NumValue(), static_cast<std::int8_t>(fill));
    return arr;
}

} // namespace drone_mapper
