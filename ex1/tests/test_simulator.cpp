// Included by test_main.cpp
#include "simulator/Simulator.h"

using namespace drone;
using namespace units;

namespace {

BuildingTruth make_room(int nx, int ny) {
    VoxelGrid g(10 * cm, Position{}, nx, ny, 1, voxel::kEmpty);
    for (int x = 0; x < nx; ++x) {
        g.set(Cell{x, 0, 0},      voxel::kOccupied);
        g.set(Cell{x, ny - 1, 0}, voxel::kOccupied);
    }
    for (int y = 0; y < ny; ++y) {
        g.set(Cell{0, y, 0},      voxel::kOccupied);
        g.set(Cell{nx - 1, y, 0}, voxel::kOccupied);
    }
    return BuildingTruth(std::move(g));
}

MissionConfig make_mission(int nx_cm, int ny_cm) {
    MissionConfig mission;
    mission.min_x = 0 * cm;
    mission.max_x = static_cast<double>(nx_cm) * cm;
    mission.min_y = 0 * cm;
    mission.max_y = static_cast<double>(ny_cm) * cm;
    mission.height_min = 0  * cm;
    mission.height_max = 10 * cm;
    mission.start = Position{15 * cm, 15 * cm, 5 * cm};
    return mission;
}

DroneConfig make_drone_cfg() {
    DroneConfig cfg;
    cfg.lidar_z_min =   1 * cm;
    cfg.lidar_z_max = 200 * cm;
    cfg.lidar_d     =   2 * cm;
    cfg.lidar_fovc  =   4;
    cfg.max_rotate_per_cmd  = 180 * deg;
    cfg.max_advance_per_cmd = 100 * cm;
    cfg.max_elevate_per_cmd = 100 * cm;
    // Small-room tests below use rooms only a few cells wide, where the
    // default 30 cm sphere wouldn't physically fit alongside walls. Use
    // a sub-cell drone (clearance = 0) so these tests focus on the
    // algorithm rather than passage geometry. Clearance behavior is
    // covered separately in test_drone / test_movement_mock.
    cfg.min_passage_width  = 5 * cm;
    cfg.min_passage_length = 5 * cm;
    cfg.min_passage_height = 5 * cm;
    return cfg;
}

} // namespace

TEST(simulator_small_room_finishes_with_decent_score) {
    BuildingTruth truth = make_room(7, 7);
    MissionConfig mission = make_mission(70, 70);
    DroneConfig cfg = make_drone_cfg();

    Simulator sim(std::move(truth), cfg, mission);
    auto report = sim.run(50000);

    CHECK(report.command_count > 0);
    CHECK(!report.drone_collided);
    CHECK(report.score >= 80.0);
}

TEST(simulator_room_with_inaccessible_pocket_does_not_collide) {
    BuildingTruth truth = make_room(9, 9);
    auto& g = truth.grid();
    for (int x = 4; x < 7; ++x) {
        g.set(Cell{x, 4, 0}, voxel::kOccupied);
        g.set(Cell{x, 6, 0}, voxel::kOccupied);
    }
    for (int y = 4; y < 7; ++y) {
        g.set(Cell{4, y, 0}, voxel::kOccupied);
        g.set(Cell{6, y, 0}, voxel::kOccupied);
    }

    MissionConfig mission = make_mission(90, 90);
    Simulator sim(std::move(truth), make_drone_cfg(), mission);
    auto report = sim.run(50000);

    CHECK(!report.drone_collided);
    CHECK(report.score > 50.0);
}
