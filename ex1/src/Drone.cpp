#include "drone/Drone.h"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>

namespace drone {

namespace {

// Returns the smallest signed difference (in degrees) from `from` to `to`,
// in the range (-180, 180].
double signed_delta_deg(double from, double to) {
    double d = std::fmod(to - from, 360.0);
    if (d > 180.0)  d -= 360.0;
    if (d <= -180.0) d += 360.0;
    return d;
}

} // namespace

Drone::Drone(BuildingMap& map, const DroneConfig& cfg)
    : map_(map), cfg_(cfg) {}

Cell Drone::current_cell() const {
    return map_.grid().cell_at(last_position_);
}

units::Angle Drone::scan_dir_yaw_offset(ScanDir dir) const {
    // Offsets are *absolute* yaws here; we relate them to current yaw in
    // build_scan_command by subtracting yaw_known_.
    switch (dir) {
        case ScanDir::PlusX:  return 0   * units::deg;
        case ScanDir::PlusY:  return 90  * units::deg;
        case ScanDir::MinusX: return 180 * units::deg;
        case ScanDir::MinusY: return 270 * units::deg;
        case ScanDir::PlusZ:
        case ScanDir::MinusZ: return 0   * units::deg;
        default:              return 0   * units::deg;
    }
}

units::Angle Drone::scan_dir_pitch_offset(ScanDir dir) const {
    switch (dir) {
        case ScanDir::PlusZ:  return  90 * units::deg;
        case ScanDir::MinusZ: return -90 * units::deg;
        default:              return   0 * units::deg;
    }
}

DroneCommand Drone::build_scan_command() {
    DroneCommand cmd;
    cmd.kind = DroneCommand::Kind::Scan;
    const auto dir = static_cast<ScanDir>(scan_index_);
    // The lidar's scan() takes a yaw OFFSET relative to current yaw, so
    // we subtract our known yaw from the absolute target.
    const double abs_yaw_target = scan_dir_yaw_offset(dir).numerical_value_in(units::deg);
    const double offset = signed_delta_deg(yaw_known_.numerical_value_in(units::deg), abs_yaw_target);
    cmd.scan_xy    = offset * units::deg;
    cmd.scan_pitch = scan_dir_pitch_offset(dir);
    return cmd;
}

void Drone::apply_full_scan_to_map(const LidarFrame& frame) {
    // Back-project every ray in the lidar frame to a world-space position.
    // For each ray we walk the cells along the line from the drone's
    // position to the hit point, marking them empty. If the ray hit an
    // obstacle we mark the target cell occupied.
    const auto& grid = map_.grid();
    const double cs = grid.cell_size().numerical_value_in(units::cm);
    const Cell here = current_cell();

    // Mark the drone's own cell as empty.
    if (map_.get_cell(here) == voxel::kUnmapped)
        map_.set_cell(here, voxel::kEmpty);

    if (frame.side <= 0) return;

    const double half_fov = units::to_rad(frame.fov) / 2.0;
    const double yaw_rad  = units::to_rad(units::normalized(yaw_known_ + frame.yaw_offset));
    const double pitch_base = units::to_rad(frame.pitch_offset);

    // Build the same local axes used by LidarMock.
    const double fwd_x = std::cos(pitch_base) * std::cos(yaw_rad);
    const double fwd_y = std::cos(pitch_base) * std::sin(yaw_rad);
    const double fwd_z = std::sin(pitch_base);

    const double right_x = -std::sin(yaw_rad);
    const double right_y =  std::cos(yaw_rad);

    const double up_x = -std::sin(pitch_base) * std::cos(yaw_rad);
    const double up_y = -std::sin(pitch_base) * std::sin(yaw_rad);
    const double up_z =  std::cos(pitch_base);

    const double ox = last_position_.x.numerical_value_in(units::cm);
    const double oy = last_position_.y.numerical_value_in(units::cm);
    const double oz = last_position_.z.numerical_value_in(units::cm);

    const int side   = frame.side;
    const int center = side / 2;

    for (int row = 0; row < side; ++row) {
        for (int col = 0; col < side; ++col) {
            const double dist = frame.at(row, col);
            if (dist < -1.5) continue; // -2: too close, skip

            const double v_frac = (row - center) / static_cast<double>(center);
            const double h_frac = (col - center) / static_cast<double>(center);
            const double a_h = h_frac * half_fov;
            const double a_v = v_frac * half_fov;

            double dx = fwd_x + right_x * std::tan(a_h) + up_x * std::tan(a_v);
            double dy = fwd_y + right_y * std::tan(a_h) + up_y * std::tan(a_v);
            double dz = fwd_z + /* right_z=0 */ up_z * std::tan(a_v);
            const double len = std::sqrt(dx*dx + dy*dy + dz*dz);
            dx /= len; dy /= len; dz /= len;

            bool has_hit;
            double ray_dist;
            if (dist < 0.0) {
                // -1: nothing in range. Use max_range.
                ray_dist = cfg_.lidar_max_range.numerical_value_in(units::cm);
                has_hit = false;
            } else {
                ray_dist = dist;
                has_hit = true;
            }

            // Walk along the ray in cell-sized steps and mark each cell.
            const int steps = static_cast<int>(std::ceil(ray_dist / cs));
            Cell prev_c = here;
            for (int s = 1; s <= steps; ++s) {
                const double t = std::min(static_cast<double>(s) * cs, ray_dist);
                const Cell c = grid.cell_at(Position{
                    (ox + dx * t) * units::cm,
                    (oy + dy * t) * units::cm,
                    (oz + dz * t) * units::cm});
                if (c == prev_c) continue;
                prev_c = c;
                if (!grid.in_bounds(c)) break;

                if (s < steps || !has_hit) {
                    // Intermediate cell or end of a no-hit ray: empty.
                    if (map_.get_cell(c) == voxel::kUnmapped)
                        map_.set_cell(c, voxel::kEmpty);
                }
            }

            // If the ray hit something, mark the cell at the hit point.
            if (has_hit) {
                const Cell hit = grid.cell_at(Position{
                    (ox + dx * ray_dist) * units::cm,
                    (oy + dy * ray_dist) * units::cm,
                    (oz + dz * ray_dist) * units::cm});
                if (grid.in_bounds(hit)) {
                    if (map_.get_cell(hit) == voxel::kUnmapped)
                        map_.set_cell(hit, voxel::kOccupied);
                }
            }
        }
    }
}

std::vector<Cell> Drone::bfs_to_frontier(Cell start) {
    // BFS through known-empty cells. Goal: a known-empty cell that has
    // at least one kUnmapped 6-neighbor (the "frontier").
    const auto& grid = map_.grid();
    std::queue<Cell> q;
    std::unordered_map<Cell, Cell, CellHash> parent;
    q.push(start);
    parent[start] = start;

    const int dx[6] = {1, -1, 0, 0, 0, 0};
    const int dy[6] = {0, 0, 1, -1, 0, 0};
    const int dz[6] = {0, 0, 0, 0, 1, -1};

    auto is_traversable = [&](Cell c) {
        return grid.in_bounds(c) && map_.get_cell(c) == voxel::kEmpty;
    };
    auto has_unknown_neighbor = [&](Cell c) {
        for (int i = 0; i < 6; ++i) {
            const Cell n{c.x + dx[i], c.y + dy[i], c.z + dz[i]};
            if (grid.in_bounds(n) && map_.get_cell(n) == voxel::kUnmapped)
                return true;
        }
        return false;
    };

    Cell goal{};
    bool found = false;
    while (!q.empty()) {
        Cell c = q.front(); q.pop();
        if (c.x != start.x || c.y != start.y || c.z != start.z) {
            // Don't accept the start as a goal: we already scanned from
            // here, so its frontier (if any) is unreachable in practice.
            if (has_unknown_neighbor(c)) { goal = c; found = true; break; }
        }
        for (int i = 0; i < 6; ++i) {
            const Cell n{c.x + dx[i], c.y + dy[i], c.z + dz[i]};
            if (parent.find(n) != parent.end()) continue;
            if (!is_traversable(n)) continue;
            parent[n] = c;
            q.push(n);
        }
    }

    std::vector<Cell> path;
    if (!found) return path;
    Cell cur = goal;
    while (!(cur.x == start.x && cur.y == start.y && cur.z == start.z)) {
        path.push_back(cur);
        cur = parent[cur];
    }
    std::reverse(path.begin(), path.end());
    return path;
}

void Drone::enqueue_move_to_neighbor(Cell from, Cell to) {
    const int dx = to.x - from.x;
    const int dy = to.y - from.y;
    const int dz = to.z - from.z;
    const double cs = map_.grid().cell_size().numerical_value_in(units::cm);

    if (dz != 0 && dx == 0 && dy == 0) {
        DroneCommand c;
        c.kind = DroneCommand::Kind::Elevate;
        c.length = static_cast<double>(dz) * cs * units::cm;
        pending_moves_.push_back(c);
        // Optimistic position update so subsequent planning steps in the
        // same path use the right reference.
        last_position_.z = last_position_.z + static_cast<double>(dz) * cs * units::cm;
        return;
    }

    // Horizontal step: rotate to face the direction, then advance.
    double target_yaw_deg = 0.0;
    if      (dx ==  1 && dy ==  0) target_yaw_deg =   0.0;
    else if (dx ==  0 && dy ==  1) target_yaw_deg =  90.0;
    else if (dx == -1 && dy ==  0) target_yaw_deg = 180.0;
    else if (dx ==  0 && dy == -1) target_yaw_deg = 270.0;

    double delta = signed_delta_deg(yaw_known_.numerical_value_in(units::deg), target_yaw_deg);
    const double max_rot = cfg_.max_rotate_per_cmd.numerical_value_in(units::deg);
    while (std::abs(delta) > 1e-6) {
        const double chunk = std::min(std::abs(delta), max_rot);
        DroneCommand c;
        c.kind = DroneCommand::Kind::Rotate;
        c.rot_dir = (delta > 0.0) ? RotateDirection::Right : RotateDirection::Left;
        c.angle = chunk * units::deg;
        pending_moves_.push_back(c);
        delta -= std::copysign(chunk, delta);
    }
    yaw_known_ = units::normalized(target_yaw_deg * units::deg);

    DroneCommand adv;
    adv.kind = DroneCommand::Kind::Advance;
    adv.length = cs * units::cm;
    pending_moves_.push_back(adv);

    // Optimistic position update.
    last_position_.x = last_position_.x + static_cast<double>(dx) * cs * units::cm;
    last_position_.y = last_position_.y + static_cast<double>(dy) * cs * units::cm;
}

void Drone::plan_next_target() {
    const Cell here = current_cell();
    auto path = bfs_to_frontier(here);
    if (path.empty()) {
        state_ = State::Done;
        return;
    }
    Cell prev = here;
    for (const Cell& step : path) {
        enqueue_move_to_neighbor(prev, step);
        prev = step;
    }
    state_ = State::Moving;
}

void Drone::start_scanning_phase() {
    scan_index_ = 0;
    state_ = State::Scanning;
}

DroneCommand Drone::next_command() {
    switch (state_) {
        case State::NeedLocation: {
            DroneCommand c;
            c.kind = DroneCommand::Kind::GetLocation;
            return c;
        }
        case State::Scanning:
            return build_scan_command();
        case State::Planning: {
            // Planning is synchronous: do it here, then either move or finish.
            plan_next_target();
            if (state_ == State::Done) {
                DroneCommand c;
                c.kind = DroneCommand::Kind::Finished;
                return c;
            }
            return next_command();
        }
        case State::Moving: {
            if (pending_moves_.empty()) {
                start_scanning_phase();
                return next_command();
            }
            DroneCommand c = pending_moves_.front();
            pending_moves_.pop_front();
            return c;
        }
        case State::Done:
        default: {
            DroneCommand c;
            c.kind = DroneCommand::Kind::Finished;
            return c;
        }
    }
}

void Drone::on_location(Position p, units::Angle yaw) {
    last_position_ = p;
    yaw_known_     = units::normalized(yaw);
    if (state_ == State::NeedLocation) {
        // Mark the current cell as known empty so the BFS frontier
        // can grow from here.
        const Cell here = current_cell();
        if (map_.get_cell(here) == voxel::kUnmapped)
            map_.set_cell(here, voxel::kEmpty);
        start_scanning_phase();
    }
}

void Drone::on_scan(const LidarFrame& frame) {
    if (state_ != State::Scanning) return;
    last_scan_ = frame;
    apply_full_scan_to_map(frame);
    ++scan_index_;
    if (scan_index_ >= static_cast<int>(ScanDir::Count)) {
        state_ = State::Planning;
    }
}

void Drone::on_move_result(MoveResult result) {
    if (result == MoveResult::Collision) {
        // Drop the rest of this path and re-sync position/yaw before
        // attempting to plan again.
        pending_moves_.clear();
        ++planning_failures_;
        if (planning_failures_ > 100) {
            state_ = State::Done;
            return;
        }
        state_ = State::NeedLocation;
        return;
    }
    // For Ok / Clamped we trust our optimistic update made at queue time.
    (void)result;
}

} // namespace drone
