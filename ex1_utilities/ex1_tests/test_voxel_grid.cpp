// Included by test_main.cpp
#include "world/VoxelGrid.h"

using namespace drone;
using namespace units;

TEST(voxel_grid_basic_set_get) {
    VoxelGrid g(10 * cm,
                Position{0 * cm, 0 * cm, 0 * cm},
                4, 4, 2,
                voxel::kEmpty);
    CHECK_EQ(g.nx(), 4);
    CHECK_EQ(g.ny(), 4);
    CHECK_EQ(g.nz(), 2);
    CHECK_EQ(static_cast<int>(g.get(Cell{0, 0, 0})), static_cast<int>(voxel::kEmpty));
    g.set(Cell{1, 2, 1}, voxel::kOccupied);
    CHECK_EQ(static_cast<int>(g.get(Cell{1, 2, 1})), static_cast<int>(voxel::kOccupied));
}

TEST(voxel_grid_out_of_bounds_returns_oob) {
    VoxelGrid g(10 * cm, Position{}, 2, 2, 2, voxel::kEmpty);
    CHECK_EQ(static_cast<int>(g.get(Cell{-1, 0, 0})), static_cast<int>(voxel::kOutOfBounds));
    CHECK_EQ(static_cast<int>(g.get(Cell{2, 0, 0})),  static_cast<int>(voxel::kOutOfBounds));
    CHECK_EQ(static_cast<int>(g.get(Cell{0, 0, 5})),  static_cast<int>(voxel::kOutOfBounds));
}

TEST(voxel_grid_continuous_to_cell) {
    VoxelGrid g(10 * cm,
                Position{0 * cm, 0 * cm, 0 * cm},
                10, 10, 10,
                voxel::kEmpty);
    Cell c = g.cell_at(Position{25 * cm, 5 * cm, 95 * cm});
    CHECK_EQ(c.x, 2);
    CHECK_EQ(c.y, 0);
    CHECK_EQ(c.z, 9);
    Position center = g.center_of(Cell{2, 0, 9});
    CHECK_NEAR(center.x.in_cm(), 25.0, 1e-9);
    CHECK_NEAR(center.y.in_cm(),  5.0, 1e-9);
    CHECK_NEAR(center.z.in_cm(), 95.0, 1e-9);
}

TEST(voxel_grid_origin_offset) {
    VoxelGrid g(10 * cm,
                Position{100 * cm, 100 * cm, 0 * cm},
                4, 4, 1,
                voxel::kEmpty);
    Cell c = g.cell_at(Position{135 * cm, 105 * cm, 0 * cm});
    CHECK_EQ(c.x, 3);
    CHECK_EQ(c.y, 0);
    CHECK_EQ(c.z, 0);
}
