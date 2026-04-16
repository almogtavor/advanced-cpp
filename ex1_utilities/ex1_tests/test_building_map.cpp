// Included by test_main.cpp
#include "world/BuildingMap.h"

using namespace drone;
using namespace units;

TEST(building_map_polygon_marks_outside_oob) {
    MissionConfig mission;
    // 30x30 cm square boundary at origin.
    mission.boundary_polygon = {
        {0 * cm,  0 * cm},
        {30 * cm, 0 * cm},
        {30 * cm, 30 * cm},
        {0 * cm,  30 * cm},
    };
    mission.height_min = 0  * cm;
    mission.height_max = 10 * cm;

    BuildingMap m(mission, 10 * cm,
                  Position{-10 * cm, -10 * cm, 0 * cm},
                  5, 5, 1);

    // Cell at far corner is outside the polygon.
    CHECK_EQ(static_cast<int>(m.get_cell(Cell{0, 0, 0})),
             static_cast<int>(voxel::kOutOfBounds));
    // Cell whose center is inside [0..30] x [0..30] should be unmapped.
    CHECK_EQ(static_cast<int>(m.get_cell(Cell{2, 2, 0})),
             static_cast<int>(voxel::kUnmapped));
}

TEST(building_map_height_filter_marks_oob) {
    MissionConfig mission;
    mission.boundary_polygon = {
        {0 * cm,   0 * cm},
        {100 * cm, 0 * cm},
        {100 * cm, 100 * cm},
        {0 * cm,   100 * cm},
    };
    mission.height_min = 10 * cm;
    mission.height_max = 30 * cm;

    BuildingMap m(mission, 10 * cm, Position{0*cm,0*cm,0*cm}, 10, 10, 5);

    // Layer 0 (centers at z=5) is below height_min.
    CHECK_EQ(static_cast<int>(m.get_cell(Cell{5, 5, 0})),
             static_cast<int>(voxel::kOutOfBounds));
    // Layer 4 (centers at z=45) is above height_max.
    CHECK_EQ(static_cast<int>(m.get_cell(Cell{5, 5, 4})),
             static_cast<int>(voxel::kOutOfBounds));
    // Layer 1 (z=15) and 2 (z=25) are in range.
    CHECK_EQ(static_cast<int>(m.get_cell(Cell{5, 5, 1})),
             static_cast<int>(voxel::kUnmapped));
    CHECK_EQ(static_cast<int>(m.get_cell(Cell{5, 5, 2})),
             static_cast<int>(voxel::kUnmapped));
}
