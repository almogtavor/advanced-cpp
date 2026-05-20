// Included by test_main.cpp
#include "mocks/MockWorld.h"
#include "mocks/MovementMock.h"

using namespace drone;
using namespace units;

namespace {
MockWorld build_open_world() {
    VoxelGrid g(10 * cm, Position{}, 10, 10, 2, voxel::kEmpty);
    MockWorld world;
    world.truth = BuildingTruth(std::move(g));
    world.drone_config.max_rotate_per_cmd  = 180 * deg;
    world.drone_config.max_advance_per_cmd = 100 * cm;
    world.drone_config.max_elevate_per_cmd = 100 * cm;
    world.position = Position{15 * cm, 15 * cm, 5 * cm};
    world.yaw = 0 * deg;
    return world;
}
} // namespace

TEST(movement_mock_advance_moves_in_yaw_direction) {
    MockWorld world = build_open_world();
    MovementMock m(world);
    auto r = m.advance(20 * cm);
    CHECK(r == MoveResult::Ok);
    CHECK_NEAR(world.position.x.numerical_value_in(cm), 35.0, 1e-6);
    CHECK_NEAR(world.position.y.numerical_value_in(cm), 15.0, 1e-6);
}

TEST(movement_mock_rotate_changes_yaw) {
    MockWorld world = build_open_world();
    MovementMock m(world);
    m.rotate(RotateDirection::Right, 90 * deg);
    CHECK_NEAR(world.yaw.numerical_value_in(deg), 90.0, 1e-6);
    m.rotate(RotateDirection::Left, 30 * deg);
    CHECK_NEAR(world.yaw.numerical_value_in(deg), 60.0, 1e-6);
}

TEST(movement_mock_advance_blocked_by_wall) {
    MockWorld world = build_open_world();
    // Place a wall directly in front of the drone.
    world.truth.grid().set(Cell{2, 1, 0}, voxel::kOccupied);
    MovementMock m(world);
    auto r = m.advance(50 * cm);
    CHECK(r == MoveResult::Collision);
    CHECK(world.collided);
    // Position should be unchanged from start (drone gets stopped before
    // entering the occupied cell).
    CHECK_NEAR(world.position.x.numerical_value_in(cm), 15.0, 1e-6);
}

TEST(movement_mock_advance_clamps_to_max) {
    MockWorld world = build_open_world();
    world.drone_config.max_advance_per_cmd = 30 * cm;
    MovementMock m(world);
    auto r = m.advance(80 * cm);
    CHECK(r == MoveResult::Clamped);
    CHECK_NEAR(world.position.x.numerical_value_in(cm), 45.0, 1e-6);
}

TEST(movement_mock_sphere_clips_neighbor) {
    // Center cell is empty but a wall sits one cell diagonally ahead.
    // With clearance_cells=1 (sphere occupies a 3x3 footprint), advancing
    // into a cell whose Chebyshev-1 neighborhood touches the wall must
    // report Collision even though the center voxel itself is free.
    MockWorld world = build_open_world();
    world.clearance_cells = 1;
    // Wall at (2,2,0); drone starts at cell (1,1,0)=15,15,5. Advancing
    // along +X would move toward cell (2,1,0), whose Chebyshev-1 hood
    // includes (2,2,0).
    world.truth.grid().set(Cell{2, 2, 0}, voxel::kOccupied);
    MovementMock m(world);
    auto r = m.advance(15 * cm);
    CHECK(r == MoveResult::Collision);
    CHECK(world.collided);
}

TEST(movement_mock_zero_clearance_preserves_old_behavior) {
    // With clearance_cells=0 the sphere is sub-cell; only the center cell
    // is checked, so a diagonal-wall scenario does NOT trigger collision.
    MockWorld world = build_open_world();
    world.clearance_cells = 0;
    world.truth.grid().set(Cell{2, 2, 0}, voxel::kOccupied);
    MovementMock m(world);
    auto r = m.advance(15 * cm);
    CHECK(r == MoveResult::Ok);
}
