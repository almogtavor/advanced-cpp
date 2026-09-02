#include <Algorithm/MappingAlgorithmImpl.h>

#include <Common/MappingAlgorithmRegistration.h>

#include <Common/IMap3D.h>

#include <algorithm>
#include <cmath>
#include <queue>

namespace algorithm_323084962_212223036 {

using namespace common;
using namespace user_common_323084962_212223036;

namespace {

constexpr double kEps = 1e-6;

// Relative scan orientations (relative to the drone heading) that together
// sweep the drone's surroundings: four sideways cones plus straight up/down.
constexpr int kNumScans = 6;

[[nodiscard]] Orientation scanDir(std::size_t i) {
    switch (i) {
    case 0: return Orientation{geom::hang(0.0), geom::aang(0.0)}; // forward
    case 1: return Orientation{geom::hang(90.0), geom::aang(0.0)}; // right
    case 2: return Orientation{geom::hang(180.0), geom::aang(0.0)}; // back
    case 3: return Orientation{geom::hang(270.0), geom::aang(0.0)}; // left
    case 4: return Orientation{geom::hang(0.0), geom::aang(90.0)}; // up
    default: return Orientation{geom::hang(0.0), geom::aang(-90.0)}; // down
    }
}

// Smallest signed difference from -> to in (-180, 180].
[[nodiscard]] double signedDeltaDeg(double from, double to) {
    double d = std::fmod(to - from, 360.0); // leaving something in (-360, 360)
    if (d > 180.0) d -= 360.0;
    if (d <= -180.0) d += 360.0;
    return d;
}

// Absolute heading (deg) the drone must face to step into the neighbouring
// voxel offset by (dx, dy). the algorithm only moves to the four side-adjacent cells, never
// diagonally
[[nodiscard]] double headingFor(long dx, long dy) {
    if (dx == 1) return 0.0;
    if (dy == 1) return 90.0;
    if (dx == -1) return 180.0;
    return 270.0; // dy == -1
}

} // namespace

void MappingAlgorithmImpl_323084962_212223036::ensureInitialized() {
    if (initialized_) {
        return;
    }
    initialized_ = true;
    cfg_ = output_map_.getMapConfig();
    const double res = geom::lcm(cfg_.resolution);
    const types::MappingBounds& b = cfg_.boundaries;
    nx_ = geom::spanVoxels(geom::xcm(b.min_x), geom::xcm(b.max_x), res);
    ny_ = geom::spanVoxels(geom::ycm(b.min_y), geom::ycm(b.max_y), res);
    nz_ = geom::spanVoxels(geom::zcm(b.min_height), geom::zcm(b.max_height), res);
    // Obstacle clearance in whole voxels. Rounded up so the spherical drone
    // body never overlaps an occupied voxel even when its centre sits at a
    // cell corner: keeping ceil(radius/res) empty cells between the drone and
    // any obstacle guarantees the body (radius) cannot reach it.
    const double radius = geom::lcm(drone_config_.radius);
    if (res > 0.0 && radius > 0.0) {
        //for example if the drone has radius of 70cm and the voxel 50 then ceil(70 / 50) = 2 voxels
        clearance_ = static_cast<long>(std::ceil(radius / res));
    }
}

types::VoxelOccupancy MappingAlgorithmImpl_323084962_212223036::occAt(const geom::VoxelIndex& idx) const {
    return output_map_.atVoxel(geom::voxelCenter(idx, cfg_));
}

bool MappingAlgorithmImpl_323084962_212223036::inGrid(const geom::VoxelIndex& idx) const {
    return idx.x >= 0 && idx.x < nx_ && idx.y >= 0 && idx.y < ny_ && idx.z >= 0 && idx.z < nz_;
}

bool MappingAlgorithmImpl_323084962_212223036::hasClearance(const geom::VoxelIndex& idx) const {
    if (clearance_ <= 0) {
        return true;
    }
    for (long dz = -clearance_; dz <= clearance_; ++dz) {
        for (long dy = -clearance_; dy <= clearance_; ++dy) {
            for (long dx = -clearance_; dx <= clearance_; ++dx) {
                if (dx == 0 && dy == 0 && dz == 0) {
                    continue;
                }
                const geom::VoxelIndex n{idx.x + dx, idx.y + dy, idx.z + dz};
                if (inGrid(n) && occAt(n) == types::VoxelOccupancy::Occupied) {
                    return false;
                }
            }
        }
    }
    return true;
}

bool MappingAlgorithmImpl_323084962_212223036::traversable(const geom::VoxelIndex& idx) const {
    return inGrid(idx) && occAt(idx) == types::VoxelOccupancy::Empty && hasClearance(idx);
}

// The function asks: is this voxel on the edge of what I know?
// the voxel must be known empty & at least one of the 6 neighbours is unmapped. this is "frontier"
bool MappingAlgorithmImpl_323084962_212223036::isFrontier(const geom::VoxelIndex& idx) const {
    if (occAt(idx) != types::VoxelOccupancy::Empty) {
        return false;
    }
    // columns: i 0(east) 1(west) 2(north) 3(south) 4(up) 5(down)
    static const long dx[6] = {1, -1, 0, 0, 0, 0};
    static const long dy[6] = {0, 0, 1, -1, 0, 0};
    static const long dz[6] = {0, 0, 0, 0, 1, -1};
    for (int i = 0; i < 6; ++i) {
        const geom::VoxelIndex n{idx.x + dx[i], idx.y + dy[i], idx.z + dz[i]};
        if (inGrid(n) && occAt(n) == types::VoxelOccupancy::Unmapped) {
            return true;
        }
    }
    return false;
}

bool MappingAlgorithmImpl_323084962_212223036::anyUnmappedRemains() const {
    for (long x = 0; x < nx_; ++x) {
        for (long y = 0; y < ny_; ++y) {
            for (long z = 0; z < nz_; ++z) {
                if (occAt(geom::VoxelIndex{x, y, z}) == types::VoxelOccupancy::Unmapped) {
                    return true;
                }
            }
        }
    }
    return false;
}

// BFS over the output map's voxel grid (Nodes = voxels. Edges = traversable() 6 face-neighbours)
std::vector<geom::VoxelIndex> MappingAlgorithmImpl_323084962_212223036::planPath(const geom::VoxelIndex& start) const {
    std::vector<geom::VoxelIndex> path;
    if (!inGrid(start) || nx_ * ny_ * nz_ <= 0) {
        return path;
    }
    const auto flat = [&](const geom::VoxelIndex& v) {
        return (v.x * ny_ + v.y) * nz_ + v.z;
    };
    std::vector<long> parent(static_cast<std::size_t>(nx_ * ny_ * nz_), -1);

    std::queue<geom::VoxelIndex> q;
    q.push(start);
    parent[static_cast<std::size_t>(flat(start))] = flat(start);

    static const long dx[6] = {1, -1, 0, 0, 0, 0};
    static const long dy[6] = {0, 0, 1, -1, 0, 0};
    static const long dz[6] = {0, 0, 0, 0, 1, -1};

    geom::VoxelIndex goal{};
    bool found = false;
    while (!q.empty()) {
        const geom::VoxelIndex cur = q.front();
        q.pop();
        if (!(cur == start) && isFrontier(cur)) {
            goal = cur;
            found = true;
            break;
        }
        for (int i = 0; i < 6; ++i) {
            const geom::VoxelIndex n{cur.x + dx[i], cur.y + dy[i], cur.z + dz[i]};
            if (!inGrid(n) || parent[static_cast<std::size_t>(flat(n))] != -1) {
                continue;
            }
            if (!traversable(n)) {
                continue;
            }
            parent[static_cast<std::size_t>(flat(n))] = flat(cur);
            q.push(n);
        }
    }
    if (!found) {
        return path;
    }
    // Backtrack from the goal to the start using the parent array, then reverse the path so it goes from start to goal.
    // since if BFS found A(start) -> B -> C -> D(goal) then parent[D] = C, parent[C] = B, parent[B] = A and cur=D initially
    // We'll end with [D, C, B] - the route in reverse. std::reverse flips it to [B, C, D]
    for (geom::VoxelIndex cur = goal; !(cur == start);) {
        path.push_back(cur);
        const long p = parent[static_cast<std::size_t>(flat(cur))];
        cur = geom::VoxelIndex{p / (ny_ * nz_), (p / nz_) % ny_, p % nz_};
    }
    std::reverse(path.begin(), path.end());
    return path;
}

void MappingAlgorithmImpl_323084962_212223036::buildMoveQueue(const types::DroneState& state,
                                          const std::vector<geom::VoxelIndex>& path) {
    move_queue_.clear();
    const double res = geom::lcm(cfg_.resolution);
    const double max_rot = geom::hdeg(drone_config_.max_rotate);
    const double max_adv = geom::lcm(drone_config_.max_advance);
    const double max_ele = geom::lcm(drone_config_.max_elevate);

    double heading = geom::hdeg(state.heading.horizontal);
    geom::VoxelIndex prev = geom::worldToVoxel(state.position, cfg_);

    for (const geom::VoxelIndex& next : path) {
        const long ddx = next.x - prev.x;
        const long ddy = next.y - prev.y;
        const long ddz = next.z - prev.z;

        if (ddz != 0) {
            const double sign = ddz > 0 ? 1.0 : -1.0;
            double remaining = res;
            const double chunk_cap = max_ele > kEps ? max_ele : res;
            while (remaining > kEps) {
                const double chunk = std::min(remaining, chunk_cap);
                types::MovementCommand cmd;
                cmd.type = types::MovementCommandType::Elevate;
                cmd.distance = geom::plen(sign * chunk);
                move_queue_.push_back(cmd);
                remaining -= chunk;
            }
        } else {
            const double target = headingFor(ddx, ddy);
            double delta = signedDeltaDeg(heading, target);
            const double rot_cap = max_rot > kEps ? max_rot : std::abs(delta);
            while (std::abs(delta) > kEps) {
                const double chunk = std::min(std::abs(delta), rot_cap > kEps ? rot_cap : std::abs(delta));
                types::MovementCommand cmd;
                cmd.type = types::MovementCommandType::Rotate;
                cmd.rotation = delta > 0 ? types::RotationDirection::Right : types::RotationDirection::Left;
                cmd.angle = geom::hang(chunk);
                move_queue_.push_back(cmd);
                delta -= std::copysign(chunk, delta);
            }
            heading = target;

            double remaining = res;
            const double chunk_cap = max_adv > kEps ? max_adv : res;
            while (remaining > kEps) {
                const double chunk = std::min(remaining, chunk_cap);
                types::MovementCommand cmd;
                cmd.type = types::MovementCommandType::Advance;
                cmd.distance = geom::plen(chunk);
                move_queue_.push_back(cmd);
                remaining -= chunk;
            }
        }
        prev = next;
    }
}

types::MappingStepCommand MappingAlgorithmImpl_323084962_212223036::nextStep(const types::DroneState& state,
                                                         const types::LidarScanResult* latest_scan) {
    (void)latest_scan; // The output map already reflects prior scans.
    ensureInitialized();

    // Scanning phase: emit the ring of scans one command at a time.
    if (phase_ == Phase::Scan) {
        if (scan_index_ < static_cast<std::size_t>(kNumScans)) {
            types::MappingStepCommand cmd;
            cmd.scan_orientation = scanDir(scan_index_);
            cmd.status = types::AlgorithmStatus::Working;
            ++scan_index_;
            return cmd;
        }
        phase_ = Phase::Plan;
    }

    if (phase_ == Phase::Plan) {
        const geom::VoxelIndex start = geom::worldToVoxel(state.position, cfg_);
        const std::vector<geom::VoxelIndex> path = planPath(start);
        if (path.empty()) {
            phase_ = Phase::Done;
        } else {
            buildMoveQueue(state, path);
            phase_ = Phase::Move;
        }
    }

    if (phase_ == Phase::Move) {
        if (move_queue_.empty()) {
            // Arrived at the frontier; rescan from the new location.
            phase_ = Phase::Scan;
            scan_index_ = 0;
            return nextStep(state, latest_scan);
        }
        types::MappingStepCommand cmd;
        cmd.movement = move_queue_.front();
        move_queue_.pop_front();
        cmd.status = types::AlgorithmStatus::Working;
        return cmd;
    }

    // Phase::Done
    types::MappingStepCommand cmd;
    cmd.status = anyUnmappedRemains() ? types::AlgorithmStatus::FinishedWithUnmappableVoxels
                                      : types::AlgorithmStatus::Finished;
    return cmd;
}

} // namespace algorithm_323084962_212223036

// Loaded when dlopen() is called on this plugin: constructing the global
// registration object hands the algorithm factory to the simulator's registrar.
// The macro pastes its argument into a variable name, so it only accepts an
// unqualified identifier - hence the using-declaration.
using algorithm_323084962_212223036::MappingAlgorithmImpl_323084962_212223036;
REGISTER_MAPPING_ALGORITHM(MappingAlgorithmImpl_323084962_212223036);
