// Included by test_main.cpp
#include "world/BuildingMap.h"

using namespace drone;
using namespace units;

TEST(building_map_rectangle_marks_outside_oob) {
    MissionConfig mission;
    mission.min_x = 0  * cm;
    mission.max_x = 30 * cm;
    mission.min_y = 0  * cm;
    mission.max_y = 30 * cm;
    mission.height_min = 0  * cm;
    mission.height_max = 10 * cm;

    BuildingMap m(mission, 10 * cm,
                  Position{-10 * cm, -10 * cm, 0 * cm},
                  5, 5, 1);

    // Cell {0,0,0} center is at (-5,-5,5) — outside the rectangle.
    CHECK_EQ(static_cast<int>(m.get_cell(Cell{0, 0, 0})),
             static_cast<int>(voxel::kOutOfBounds));
    // Cell {2,2,0} center is at (15,15,5) — inside the rectangle.
    CHECK_EQ(static_cast<int>(m.get_cell(Cell{2, 2, 0})),
             static_cast<int>(voxel::kUnmapped));
}

TEST(building_map_height_filter_marks_oob) {
    MissionConfig mission;
    mission.min_x = 0   * cm;
    mission.max_x = 100 * cm;
    mission.min_y = 0   * cm;
    mission.max_y = 100 * cm;
    mission.height_min = 10 * cm;
    mission.height_max = 30 * cm;

    BuildingMap m(mission, 10 * cm, Position{0*cm,0*cm,0*cm}, 10, 10, 5);

    CHECK_EQ(static_cast<int>(m.get_cell(Cell{5, 5, 0})),
             static_cast<int>(voxel::kOutOfBounds));
    CHECK_EQ(static_cast<int>(m.get_cell(Cell{5, 5, 4})),
             static_cast<int>(voxel::kOutOfBounds));
    CHECK_EQ(static_cast<int>(m.get_cell(Cell{5, 5, 1})),
             static_cast<int>(voxel::kUnmapped));
    CHECK_EQ(static_cast<int>(m.get_cell(Cell{5, 5, 2})),
             static_cast<int>(voxel::kUnmapped));
}
