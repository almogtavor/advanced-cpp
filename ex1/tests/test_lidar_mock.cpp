// Included by test_main.cpp
#include "mocks/LidarMock.h"
#include "mocks/MockWorld.h"

using namespace drone;
using namespace units;

namespace {
MockWorld build_world_with_corridor() {
    // 11x3 floor with walls on the long sides. Drone in middle facing +X.
    VoxelGrid g(10 * cm, Position{}, 11, 3, 1, voxel::kEmpty);
    for (int x = 0; x < 11; ++x) {
        g.set(Cell{x, 0, 0}, voxel::kOccupied);
        g.set(Cell{x, 2, 0}, voxel::kOccupied);
    }
    g.set(Cell{10, 1, 0}, voxel::kOccupied); // end wall

    MockWorld world;
    world.truth = BuildingTruth(std::move(g));
    world.drone_config.lidar_z_min =  1 * cm;
    world.drone_config.lidar_z_max = 200 * cm;
    world.drone_config.lidar_d      =  2 * cm;
    world.drone_config.lidar_fovc   =  4;
    world.position = Position{15 * cm, 15 * cm, 5 * cm}; // cell (1,1,0)
    world.yaw = 0 * deg;
    return world;
}
} // namespace

TEST(lidar_mock_central_beam_hits_end_wall) {
    MockWorld world = build_world_with_corridor();
    LidarMock lidar(world);
    LidarFrame f = lidar.scan(0 * deg, 0 * deg);
    // Frame must include at least the central beam.
    CHECK(!f.beams.empty());
    const LidarBeam& center = f.beams.front();
    CHECK_EQ(center.circle, 0);
    // The end wall is at cell x=10 (range 100..110 cm); drone at x=15.
    // Central ray hit distance lives in the (70, 100) window given the
    // half-cell step used by the mock.
    CHECK(center.distance_cm > 70.0);
    CHECK(center.distance_cm < 100.0);
}

TEST(lidar_mock_total_beam_count_matches_spec) {
    MockWorld world = build_world_with_corridor();
    LidarMock lidar(world);
    LidarFrame f = lidar.scan(0 * deg, 0 * deg);
    // FOVC=4 -> 1 + 4 + 16 + 64 = 85 beams.
    CHECK_EQ(static_cast<int>(f.beams.size()), 85);
}

TEST(lidar_mock_open_direction_returns_no_hit) {
    // Empty world: no walls anywhere within the lidar's range.
    VoxelGrid g(10 * cm, Position{}, 60, 60, 1, voxel::kEmpty);
    MockWorld world;
    world.truth = BuildingTruth(std::move(g));
    world.drone_config.lidar_z_min =  1 * cm;
    world.drone_config.lidar_z_max = 100 * cm;
    world.drone_config.lidar_d      =  2 * cm;
    world.drone_config.lidar_fovc   =  4;
    world.position = Position{300 * cm, 300 * cm, 5 * cm};
    world.yaw = 0 * deg;

    LidarMock lidar(world);
    LidarFrame f = lidar.scan(0 * deg, 0 * deg);
    CHECK(!f.beams.empty());
    // Central beam should be reported as "no hit" (distance_cm == -1).
    CHECK_NEAR(f.beams.front().distance_cm, -1.0, 1e-9);
}
