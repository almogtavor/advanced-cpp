#include <drone_mapper/Map3DImpl.h>

#include <drone_mapper/MapGeometry.h>

#include <cstdint>
#include <stdexcept>
#include <utility>

namespace drone_mapper {

namespace {

// A voxel is stored as one signed byte whose value is exactly the
// types::VoxelOccupancy code. Hidden maps loaded from .npy files are
// normalised to Empty(0)/Occupied(1) by the loader before they reach here,
// so both hidden and output maps share this single interpretation.
[[nodiscard]] types::VoxelOccupancy decodeByte(std::int8_t raw) {
    switch (raw) {
    case static_cast<std::int8_t>(types::VoxelOccupancy::PotentiallyOccupied):
        return types::VoxelOccupancy::PotentiallyOccupied;
    case static_cast<std::int8_t>(types::VoxelOccupancy::OutOfBounds):
        return types::VoxelOccupancy::OutOfBounds;
    case static_cast<std::int8_t>(types::VoxelOccupancy::Unmapped):
        return types::VoxelOccupancy::Unmapped;
    case static_cast<std::int8_t>(types::VoxelOccupancy::Empty):
        return types::VoxelOccupancy::Empty;
    default:
        // Any non-zero byte (occupancy 1, or a raw block id from a Minecraft
        // style map that slipped through) counts as occupied.
        return types::VoxelOccupancy::Occupied;
    }
}

} // namespace

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr)
    : Map3DImpl(std::move(map_ptr), types::MapConfig{}) {}

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr, const types::MapConfig map_config)
    : map_(std::move(map_ptr)),
      config_(map_config) {
    if (!map_) {
        throw std::invalid_argument("Map3DImpl requires a valid map pointer.");
    }
}

types::VoxelOccupancy Map3DImpl::atVoxel(const Position3D& pos) const {
    if (map_->IsEmpty() || geom::lcm(config_.resolution) <= 0.0) {
        return types::VoxelOccupancy::Unmapped;
    }
    const NpyArray::shape_t& shape = map_->Shape();
    if (shape.size() != 3) {
        return types::VoxelOccupancy::Unmapped;
    }
    const long nx = static_cast<long>(shape[0]);
    const long ny = static_cast<long>(shape[1]);
    const long nz = static_cast<long>(shape[2]);

    const geom::VoxelIndex idx = geom::worldToVoxel(pos, config_);
    if (idx.x < 0 || idx.x >= nx || idx.y < 0 || idx.y >= ny || idx.z < 0 || idx.z >= nz) {
        return types::VoxelOccupancy::OutOfBounds;
    }
    const std::size_t flat = (static_cast<std::size_t>(idx.x) * ny + idx.y) * nz + idx.z;
    return decodeByte(map_->Data<std::int8_t>()[flat]);
}

types::MapConfig Map3DImpl::getMapConfig() const {
    return config_;
}

bool Map3DImpl::isInBounds(const Position3D& pos) const {
    const types::MappingBounds& b = config_.boundaries;
    const double x = geom::xcm(pos.x);
    const double y = geom::ycm(pos.y);
    const double z = geom::zcm(pos.z);
    return x >= geom::xcm(b.min_x) && x < geom::xcm(b.max_x) &&
           y >= geom::ycm(b.min_y) && y < geom::ycm(b.max_y) &&
           z >= geom::zcm(b.min_height) && z < geom::zcm(b.max_height);
}

void Map3DImpl::set(const Position3D& pos, types::VoxelOccupancy value) {
    if (map_->IsEmpty() || geom::lcm(config_.resolution) <= 0.0) {
        return;
    }
    const NpyArray::shape_t& shape = map_->Shape();
    if (shape.size() != 3) {
        return;
    }
    const long nx = static_cast<long>(shape[0]);
    const long ny = static_cast<long>(shape[1]);
    const long nz = static_cast<long>(shape[2]);

    const geom::VoxelIndex idx = geom::worldToVoxel(pos, config_);
    if (idx.x < 0 || idx.x >= nx || idx.y < 0 || idx.y >= ny || idx.z < 0 || idx.z >= nz) {
        return;
    }
    const std::size_t flat = (static_cast<std::size_t>(idx.x) * ny + idx.y) * nz + idx.z;
    map_->Data<std::int8_t>()[flat] = static_cast<std::int8_t>(value);
}

void Map3DImpl::save(const std::filesystem::path& output_path) const {
    if (output_path.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(output_path.parent_path(), ec);
    }
    if (map_->IsEmpty()) {
        throw std::runtime_error("Map3DImpl::save called on an empty map.");
    }
    const char* err = map_->SaveNPY(output_path.string());
    if (err != nullptr) {
        throw std::runtime_error(std::string("Failed to save output map: ") + err);
    }
}

} // namespace drone_mapper
