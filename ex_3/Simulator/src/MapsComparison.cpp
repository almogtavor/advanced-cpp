#include <Simulator/MapsComparison.h>

#include <UserCommon/MapGeometry.h>

#include <algorithm>

namespace simulator {

using namespace common;
using namespace user_common_323084962_212223036;

namespace {

// Both a measured hit (Occupied) and an uncertain near-hit
// (PotentiallyOccupied) count as "there is something here" for scoring.
[[nodiscard]] bool isOccupiedLike(common::types::VoxelOccupancy v) {
    return v == common::types::VoxelOccupancy::Occupied ||
           v == common::types::VoxelOccupancy::PotentiallyOccupied;
}

// Voxel-by-voxel agreement over the origin map's grid, expressed as a
// percentage in [0, 100]. Identical maps score 100; maps that disagree on
// every voxel score 0.
[[nodiscard]] double scoreOne(const IMap3D& origin, const IMap3D& target) {
    const common::types::MapConfig cfg = origin.getMapConfig();
    const double res = geom::lcm(cfg.resolution);
    const common::types::MappingBounds& b = cfg.boundaries;
    const long nx = geom::spanVoxels(geom::xcm(b.min_x), geom::xcm(b.max_x), res);
    const long ny = geom::spanVoxels(geom::ycm(b.min_y), geom::ycm(b.max_y), res);
    const long nz = geom::spanVoxels(geom::zcm(b.min_height), geom::zcm(b.max_height), res);

    const long total = nx * ny * nz;
    if (total <= 0) {
        return 0.0;
    }

    long matches = 0;
    for (long ix = 0; ix < nx; ++ix) {
        for (long iy = 0; iy < ny; ++iy) {
            for (long iz = 0; iz < nz; ++iz) {
                const Position3D center = geom::voxelCenter(geom::VoxelIndex{ix, iy, iz}, cfg);
                if (isOccupiedLike(origin.atVoxel(center)) == isOccupiedLike(target.atVoxel(center))) {
                    ++matches;
                }
            }
        }
    }
    return 100.0 * static_cast<double>(matches) / static_cast<double>(total);
}

} // namespace

std::vector<double> MapsComparison::compare(const IMap3D& origin,
                                            const std::vector<IMap3D*> targets) {
    std::vector<double> scores;
    scores.reserve(targets.size());
    for (const IMap3D* target : targets) {
        if (target == nullptr) {
            scores.push_back(-1.0);
            continue;
        }
        scores.push_back(std::clamp(scoreOne(origin, *target), 0.0, 100.0));
    }
    return scores;
}

} // namespace simulator
