// Included by test_main.cpp
#include "mocks/LidarMock.h"
#include "mocks/MockWorld.h"

using namespace drone;
using namespace units;

namespace {
MockWorld build_world_with_corridor() {
    // 11x3 floor at one Z layer with walls on the long sides. Drone in
    // the middle of the corridor facing +X.
    VoxelGrid g(10 * cm, Position{}, 11, 3, 1, voxel::kEmpty);
    for (int x = 0; x < 11; ++x) {
        g.set(Cell{x, 0, 0}, voxel::kOccupied);
        g.set(Cell{x, 2, 0}, voxel::kOccupied);
    }
    // End wall
    g.set(Cell{10, 1, 0}, voxel::kOccupied);

    MockWorld world;
    world.truth = BuildingTruth(std::move(g));
    world.drone_config.lidar_fov         = 60 * deg;
    world.drone_config.lidar_min_range   =  1 * cm;
    world.drone_config.lidar_max_range   = 200 * cm;
    world.drone_config.lidar_res_dist_a  = 50 * cm;
    world.drone_config.lidar_res_side_a  =  5 * cm;
    world.drone_config.lidar_res_dist_b  = 200 * cm;
    world.drone_config.lidar_res_side_b  = 20 * cm;
    world.position = Position{15 * cm, 15 * cm, 5 * cm}; // cell (1,1,0)
    world.yaw = 0 * deg;
    return world;
}
} // namespace

TEST(lidar_mock_central_ray_hits_end_wall) {
    MockWorld world = build_world_with_corridor();
    LidarMock lidar(world);
    LidarFrame f = lidar.scan(0 * deg, 0 * deg);
    CHECK(f.side > 0);
    const int c = f.side / 2;
    const double d = f.at(c, c);
    // Drone is at x=15 cm; end wall first occupied cell starts at x=100.
    // The central ray should detect a hit somewhere between 80 and 95 cm
    // (we step the world in 5 cm increments).
    CHECK(d > 0.0);
    CHECK(d > 70.0);
    CHECK(d < 100.0);
}

TEST(lidar_mock_open_direction_returns_minus_one) {
    // Empty world: no walls anywhere within the lidar's range.
    VoxelGrid g(10 * cm, Position{}, 60, 60, 1, voxel::kEmpty);
    MockWorld world;
    world.truth = BuildingTruth(std::move(g));
    world.drone_config.lidar_fov         = 60 * deg;
    world.drone_config.lidar_min_range   =  1 * cm;
    world.drone_config.lidar_max_range   = 100 * cm;
    world.drone_config.lidar_res_dist_a  = 50 * cm;
    world.drone_config.lidar_res_side_a  =  5 * cm;
    world.drone_config.lidar_res_dist_b  = 200 * cm;
    world.drone_config.lidar_res_side_b  = 20 * cm;
    world.position = Position{300 * cm, 300 * cm, 5 * cm};
    world.yaw = 0 * deg;

    LidarMock lidar(world);
    LidarFrame f = lidar.scan(0 * deg, 0 * deg);
    const int c = f.side / 2;
    CHECK_NEAR(f.at(c, c), -1.0, 1e-9);
}
