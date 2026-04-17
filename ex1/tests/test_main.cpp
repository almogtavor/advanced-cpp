// Test runner. All test files are #included here so the framework
// stays single-translation-unit and the build is dead simple.
#include "test_framework.h"

#include "test_units.cpp"
#include "test_voxel_grid.cpp"
#include "test_building_map.cpp"
#include "test_config_parser.cpp"
#include "test_map_io.cpp"
#include "test_lidar_mock.cpp"
#include "test_movement_mock.cpp"
#include "test_drone.cpp"
#include "test_simulator.cpp"

int main() {
    return tf::run_all();
}
