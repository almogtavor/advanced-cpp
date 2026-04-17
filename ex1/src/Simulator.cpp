#include "simulator/Simulator.h"
#include "util/Logger.h"

namespace drone {

namespace {

// Builds a BuildingMap that shares the truth's coordinate frame so that
// scoring is straightforward. Cells outside the mission polygon or height
// range are pre-marked as kOutOfBounds.
BuildingMap build_known_map(const BuildingTruth& truth,
                            const MissionConfig& mission) {
    const auto& g = truth.grid();
    return BuildingMap(mission, g.cell_size(), g.origin(),
                       g.nx(), g.ny(), g.nz());
}

} // namespace

Simulator::Simulator(BuildingTruth truth,
                     DroneConfig   drone_cfg,
                     MissionConfig mission)
    : mission_(std::move(mission)),
      world_{std::move(truth), std::move(drone_cfg), {}, 0 * units::deg, false},
      known_map_(build_known_map(world_.truth, mission_)),
      pos_mock_(world_),
      lidar_mock_(world_),
      move_mock_(world_),
      drone_(known_map_, world_.drone_config) {
    world_.position = mission_.start;
    world_.yaw      = 0 * units::deg;
    LOG_INFO("Simulator initialized. Grid: " +
             std::to_string(world_.truth.grid().nx()) + "x" +
             std::to_string(world_.truth.grid().ny()) + "x" +
             std::to_string(world_.truth.grid().nz()) +
             " cell_size=" + std::to_string(world_.truth.grid().cell_size().numerical_value_in(units::cm)) + "cm");
    LOG_INFO("Start position: (" +
             std::to_string(world_.position.x.numerical_value_in(units::cm)) + ", " +
             std::to_string(world_.position.y.numerical_value_in(units::cm)) + ", " +
             std::to_string(world_.position.z.numerical_value_in(units::cm)) + ")");
}

SimulationReport Simulator::run(int max_commands) {
    int command_count = 0;
    int safety = 0;
    while (safety++ < max_commands) {
        const DroneCommand cmd = drone_.next_command();
        ++command_count;
        switch (cmd.kind) {
            case DroneCommand::Kind::Finished: {
                SimulationReport report = score_against_truth();
                report.command_count  = command_count;
                report.drone_collided = world_.collided;
                return report;
            }
            case DroneCommand::Kind::GetLocation:
                drone_.on_location(pos_mock_.get_position(), pos_mock_.get_yaw());
                break;
            case DroneCommand::Kind::Scan: {
                const LidarFrame f = lidar_mock_.scan(cmd.scan_xy, cmd.scan_pitch);
                drone_.on_scan(f);
                break;
            }
            case DroneCommand::Kind::Rotate: {
                const auto r = move_mock_.rotate(cmd.rot_dir, cmd.angle);
                drone_.on_move_result(r);
                break;
            }
            case DroneCommand::Kind::Advance: {
                const auto r = move_mock_.advance(cmd.length);
                drone_.on_move_result(r);
                if (r == MoveResult::Collision) {
                    LOG_WARNING("Drone collision at command #" +
                                std::to_string(command_count) +
                                " pos=(" +
                                std::to_string(world_.position.x.numerical_value_in(units::cm)) + "," +
                                std::to_string(world_.position.y.numerical_value_in(units::cm)) + "," +
                                std::to_string(world_.position.z.numerical_value_in(units::cm)) + ")");
                }
                break;
            }
            case DroneCommand::Kind::Elevate: {
                const auto r = move_mock_.elevate(cmd.length);
                drone_.on_move_result(r);
                break;
            }
        }
    }
    // Hit the safety cap. Score what we have.
    LOG_WARNING("Safety cap reached at " + std::to_string(max_commands) + " commands");
    SimulationReport report = score_against_truth();
    report.command_count  = command_count;
    report.drone_collided = world_.collided;
    return report;
}

SimulationReport Simulator::score_against_truth() const {
    SimulationReport r{};
    const auto& kg = known_map_.grid();
    const auto& tg = world_.truth.grid();
    for (int z = 0; z < kg.nz(); ++z) {
        for (int y = 0; y < kg.ny(); ++y) {
            for (int x = 0; x < kg.nx(); ++x) {
                const Cell c{x, y, z};
                const int8_t k = kg.get(c);
                if (k == voxel::kOutOfBounds) continue;
                ++r.total_in_bounds_cells;
                const int8_t t = tg.get(c);
                if (k == voxel::kUnmapped) {
                    ++r.unmapped_cells;
                    continue;
                }
                // Treat truth's out-of-bounds (e.g., truth grid smaller)
                // as kEmpty for comparison purposes.
                const int8_t t_eff = (t == voxel::kOutOfBounds) ? voxel::kEmpty : t;
                if (k == t_eff) ++r.correct_cells;
                else            ++r.incorrect_cells;
            }
        }
    }
    if (r.total_in_bounds_cells > 0) {
        r.score = 100.0 * static_cast<double>(r.correct_cells) /
                  static_cast<double>(r.total_in_bounds_cells);
    } else {
        r.score = 0.0;
    }
    return r;
}

} // namespace drone
