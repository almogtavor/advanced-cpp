#pragma once

#include <Common/IMappingAlgorithm.h>
#include <UserCommon/MapGeometry.h>

#include <deque>
#include <vector>

namespace algorithm_323084962_212223036 {

using namespace common;
using namespace user_common_323084962_212223036;

// Frontier-based exploration algorithm. It works on the output map's voxel
// grid: from the current voxel it emits a ring of scans, then plans a path
// through known-empty cells to the nearest cell bordering unmapped space,
// walks there one bounded command at a time, and repeats until no reachable
// frontier remains.
//
// The algorithm never issues a movement into a cell that the output map does
// not already know to be empty, so the drone only traverses proven-free space.
class MappingAlgorithmImpl_323084962_212223036 final : public IMappingAlgorithm {
public:
    using IMappingAlgorithm::IMappingAlgorithm;

    [[nodiscard]] types::MappingStepCommand nextStep(const types::DroneState& state,
                                                     const types::LidarScanResult* latest_scan) override;

private:
    enum class Phase {
        Scan,
        Plan,
        Move,
        Done,
    };

    void ensureInitialized();
    [[nodiscard]] types::VoxelOccupancy occAt(const geom::VoxelIndex& idx) const;
    [[nodiscard]] bool inGrid(const geom::VoxelIndex& idx) const;
    [[nodiscard]] bool traversable(const geom::VoxelIndex& idx) const;
    [[nodiscard]] bool hasClearance(const geom::VoxelIndex& idx) const;
    [[nodiscard]] bool isFrontier(const geom::VoxelIndex& idx) const;
    [[nodiscard]] bool anyUnmappedRemains() const;

    [[nodiscard]] std::vector<geom::VoxelIndex> planPath(const geom::VoxelIndex& start) const;
    void buildMoveQueue(const types::DroneState& state,
                        const std::vector<geom::VoxelIndex>& path);

    bool initialized_ = false;
    Phase phase_ = Phase::Scan;
    std::size_t scan_index_ = 0;
    std::deque<types::MovementCommand> move_queue_{};

    // Cached grid geometry (filled by ensureInitialized on the first step).
    types::MapConfig cfg_{};
    long nx_ = 0;
    long ny_ = 0;
    long nz_ = 0;
    long clearance_ = 0;
};

} // namespace algorithm_323084962_212223036
