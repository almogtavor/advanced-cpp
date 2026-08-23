// ConfigLoader tests: YAML parsing of every config type, defaults, unit
// conversions (diameter->radius), missing-key errors, and composition loading.

#include "support/TestSupport.h"

#include <drone_mapper/ConfigLoader.h>

#include <gtest/gtest.h>

using namespace dmtest;
namespace cfg = drone_mapper::config;

TEST(ConfigLoader, DroneDiameterBecomesRadius) {
    const auto drone = cfg::parseDroneConfig(R"(
drone_config:
  dimensions_cm: 30
  max_rotate_deg: 45
  max_advance_cm: 50
  max_elevate_cm: 40
)");
    EXPECT_DOUBLE_EQ(g::lcm(drone.radius), 15.0);
    EXPECT_DOUBLE_EQ(g::hdeg(drone.max_rotate), 45.0);
    EXPECT_DOUBLE_EQ(g::lcm(drone.max_advance), 50.0);
    EXPECT_DOUBLE_EQ(g::lcm(drone.max_elevate), 40.0);
}

TEST(ConfigLoader, LidarConfigParsed) {
    const auto lidar = cfg::parseLidarConfig(R"(
lidar_config:
  z_min_cm: 20
  z_max_cm: 120
  d_cm: 2.5
  fov_circles: 5
)");
    EXPECT_DOUBLE_EQ(g::lcm(lidar.z_min), 20.0);
    EXPECT_DOUBLE_EQ(g::lcm(lidar.z_max), 120.0);
    EXPECT_DOUBLE_EQ(g::lcm(lidar.d), 2.5);
    EXPECT_EQ(lidar.fov_circles, 5u);
}

TEST(ConfigLoader, MissionConfigParsed) {
    const auto mission = cfg::parseMissionConfig(R"(
mission_config:
  max_steps: 2400
  boundaries:
    x_boundary: { min_cm: -500, max_cm: -30 }
    y_boundary: { min_cm: 30, max_cm: 400 }
    height_boundary: { min_cm: -30, max_cm: 300 }
  gps_resolution_cm: 10
  output_mapping_resolution_factor: 2
)");
    EXPECT_EQ(mission.max_steps, 2400u);
    EXPECT_DOUBLE_EQ(g::lcm(mission.gps_resolution), 10.0);
    EXPECT_DOUBLE_EQ(mission.output_mapping_resolution_factor, 2.0);
    EXPECT_DOUBLE_EQ(g::xcm(mission.mission_bounds.min_x), -500.0);
    EXPECT_DOUBLE_EQ(g::xcm(mission.mission_bounds.max_x), -30.0);
    EXPECT_DOUBLE_EQ(g::ycm(mission.mission_bounds.max_y), 400.0);
    EXPECT_DOUBLE_EQ(g::zcm(mission.mission_bounds.min_height), -30.0);
}

TEST(ConfigLoader, MissionResolutionFactorDefaultsToOne) {
    const auto mission = cfg::parseMissionConfig(R"(
mission_config:
  max_steps: 100
  boundaries:
    x_boundary: { min_cm: 0, max_cm: 100 }
    y_boundary: { min_cm: 0, max_cm: 100 }
    height_boundary: { min_cm: 0, max_cm: 100 }
  gps_resolution_cm: 10
)");
    EXPECT_DOUBLE_EQ(mission.output_mapping_resolution_factor, 1.0);
}

TEST(ConfigLoader, SimulationConfigParsed) {
    const auto sim = cfg::parseSimulationConfig(R"(
simulation_config:
  map_filename: "maps/office.npy"
  map_resolution_cm: 10
  initial_drone_position: { x_cm: 250, y_cm: 200, height_cm: 150 }
  initial_angle_deg: 90
  map_axes_offset: { x_offset: 1000, y_offset: 1000, height_offset: 1500 }
)");
    EXPECT_EQ(sim.map_filename.string(), "maps/office.npy");
    EXPECT_DOUBLE_EQ(g::lcm(sim.map_resolution), 10.0);
    EXPECT_DOUBLE_EQ(g::xcm(sim.initial_drone_position.x), 250.0);
    EXPECT_DOUBLE_EQ(g::zcm(sim.initial_drone_position.z), 150.0);
    EXPECT_DOUBLE_EQ(g::hdeg(sim.initial_angle), 90.0);
    EXPECT_DOUBLE_EQ(g::xcm(sim.map_offset.x), 1000.0);
    EXPECT_DOUBLE_EQ(g::zcm(sim.map_offset.z), 1500.0);
}

TEST(ConfigLoader, SimulationOffsetDefaultsToZero) {
    const auto sim = cfg::parseSimulationConfig(R"(
simulation_config:
  map_filename: "m.npy"
  map_resolution_cm: 10
  initial_drone_position: { x_cm: 0, y_cm: 0, height_cm: 0 }
  initial_angle_deg: 0
)");
    EXPECT_DOUBLE_EQ(g::xcm(sim.map_offset.x), 0.0);
    EXPECT_DOUBLE_EQ(g::ycm(sim.map_offset.y), 0.0);
    EXPECT_DOUBLE_EQ(g::zcm(sim.map_offset.z), 0.0);
}

TEST(ConfigLoader, MissingDroneKeyThrows) {
    EXPECT_THROW((void)cfg::parseDroneConfig("drone_config: { max_rotate_deg: 45 }"),
                 std::runtime_error);
}

TEST(ConfigLoader, MissingLidarKeyThrows) {
    EXPECT_THROW((void)cfg::parseLidarConfig("lidar_config: { z_min_cm: 20 }"), std::runtime_error);
}

TEST(ConfigLoader, MissingMissionBoundaryThrows) {
    EXPECT_THROW((void)cfg::parseMissionConfig(R"(
mission_config:
  max_steps: 100
  gps_resolution_cm: 10
)"),
                 std::runtime_error);
}

TEST(ConfigLoader, WrongTopLevelKeyThrows) {
    EXPECT_THROW((void)cfg::parseDroneConfig("lidar_config: {}"), std::runtime_error);
}

TEST(ConfigLoader, LoadsCompositionFromFile) {
    const auto composition = cfg::loadComposition(dataFile("integration/composition.yaml"));
    ASSERT_EQ(composition.simulation_mission_groups.size(), 1u);
    EXPECT_EQ(std::get<1>(composition.simulation_mission_groups.front()).size(), 1u);
    EXPECT_EQ(composition.drones.size(), 1u);
    EXPECT_EQ(composition.lidars.size(), 1u);
}

TEST(ConfigLoader, CompositionResolvesMapPathToAbsolute) {
    const auto composition = cfg::loadComposition(dataFile("integration/composition.yaml"));
    const auto& sim = std::get<0>(composition.simulation_mission_groups.front());
    EXPECT_TRUE(sim.map_filename.is_absolute());
    EXPECT_TRUE(std::filesystem::exists(sim.map_filename));
}

TEST(ConfigLoader, CompositionCartesianDimensionsCorrect) {
    const auto composition = cfg::loadComposition(dataFile("integration/composition.yaml"));
    // 1 sim group * 1 mission * 1 drone * 1 lidar = 1 run in this fixture.
    const auto& drone = composition.drones.front();
    EXPECT_DOUBLE_EQ(g::lcm(drone.radius), 4.0); // dimensions_cm 8 -> radius 4
}

TEST(ConfigLoader, MissingFileThrows) {
    EXPECT_THROW((void)cfg::loadComposition(dataFile("does_not_exist.yaml")), std::exception);
}
