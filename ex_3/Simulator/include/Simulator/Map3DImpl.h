#pragma once

#include <TinyNPY.h>

#include <Common/IMutableMap3D.h>

#include <filesystem>
#include <memory>

namespace simulator {

using namespace common;

class Map3DImpl final : public IMutableMap3D {
public:
    Map3DImpl(std::shared_ptr<NpyArray> map_ptr);
    // Changed: added offset-aware construction for hidden maps loaded from NPY files.
    Map3DImpl(std::shared_ptr<NpyArray> map_ptr, const common::types::MapConfig map_config);

    [[nodiscard]] common::types::VoxelOccupancy atVoxel(const Position3D& pos) const override;
    // Changed: exposes boundaries, offset, and resolution as one map-owned configuration.
    [[nodiscard]] common::types::MapConfig getMapConfig() const override;
    [[nodiscard]] bool isInBounds(const Position3D& pos) const override;

    //Mutable map methods
    void set(const Position3D& pos, common::types::VoxelOccupancy value) override;
    void save(const std::filesystem::path& output_path) const override;

private:
    // Changed: shared ownership supports the new pointer-based storage member.
    std::shared_ptr<NpyArray> map_;
    // Changed: replaces standalone resolution_ so all map geometry stays together.
    common::types::MapConfig config_;
};

} // namespace simulator
