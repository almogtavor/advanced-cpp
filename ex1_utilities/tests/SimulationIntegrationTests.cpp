#include "TestFramework.hpp"
#include "TestUtils.hpp"
#include "drone_mapper/Simulator.hpp"

DM_TEST(RunSimulationTinyCanonicalMapSucceeds) {
  testutil::TempDir dir{"sim_tiny"};
  testutil::write_input_bundle(
      dir,
      testutil::valid_drone_config_text(),
      testutil::mission_config_text(dm::MissionConfig{0, 0, 0, 0, 0, 0, 0, 0}),
      testutil::map_input_text(1, 1, 1, dm::Position{0, 0, 0}));

  const auto result = dm::run_simulation(dir.path());

  DM_ASSERT_TRUE(result.success);
  DM_ASSERT_TRUE(dir.exists("map_output.txt"));
}

DM_TEST(RunSimulationMediumRoomWithObstacleCompletesSuccessfully) {
  testutil::TempDir dir{"sim_room"};
  testutil::write_input_bundle(
      dir,
      testutil::valid_drone_config_text(),
      testutil::mission_config_text(dm::MissionConfig{0, 0, 3, 3, 0, 0, 0, 0}),
      testutil::map_input_text(4, 4, 1, dm::Position{0, 0, 0}, {{2, 1, 0}, {2, 2, 0}}));

  const auto result = dm::run_simulation(dir.path());

  DM_ASSERT_TRUE(result.success);
  DM_ASSERT_TRUE(result.score >= 0.0);
  DM_ASSERT_TRUE(result.score <= 100.0);
}

DM_TEST(RunSimulationRepeatedRunsAreDeterministic) {
  testutil::TempDir dir{"sim_repeat"};
  testutil::write_input_bundle(
      dir,
      testutil::valid_drone_config_text(),
      testutil::mission_config_text(dm::MissionConfig{0, 0, 3, 0, 0, 0, 0, 0}),
      testutil::map_input_text(4, 1, 1, dm::Position{0, 0, 0}, {{3, 0, 0}}));

  const auto first = dm::run_simulation(dir.path());
  const auto first_output = dir.read("map_output.txt");
  const auto second = dm::run_simulation(dir.path());
  const auto second_output = dir.read("map_output.txt");

  DM_ASSERT_TRUE(first.success);
  DM_ASSERT_TRUE(second.success);
  DM_ASSERT_EQ(first.score, second.score);
  DM_ASSERT_EQ(first_output, second_output);
}

DM_TEST(RunSimulationWithNarrowCorridorAndDeadEndTerminatesCleanly) {
  testutil::TempDir dir{"sim_corridor"};
  testutil::write_input_bundle(
      dir,
      testutil::valid_drone_config_text(),
      testutil::mission_config_text(dm::MissionConfig{0, 0, 4, 2, 0, 0, 0, 0}),
      testutil::map_input_text(
          5,
          3,
          1,
          dm::Position{0, 1, 0},
          {{1, 0, 0}, {1, 2, 0}, {3, 0, 0}, {3, 2, 0}, {4, 1, 0}}));

  const auto result = dm::run_simulation(dir.path());

  DM_ASSERT_TRUE(result.success);
  DM_ASSERT_TRUE(result.score >= 0.0);
}

DM_TEST(RunSimulationHandlesBoundaryStartPosition) {
  testutil::TempDir dir{"sim_boundary"};
  testutil::write_input_bundle(
      dir,
      testutil::valid_drone_config_text(),
      testutil::mission_config_text(dm::MissionConfig{0, 0, 2, 2, 0, 0, 0, 0}),
      testutil::map_input_text(3, 3, 1, dm::Position{0, 0, 0}, {{1, 1, 0}}));

  const auto result = dm::run_simulation(dir.path());

  DM_ASSERT_TRUE(result.success);
}

DM_TEST(RunSimulationWithRecoveredConfigStillProducesOutput) {
  testutil::TempDir dir{"sim_recovered"};
  testutil::write_input_bundle(
      dir,
      "min_pass_width_cm=50\n"
      "min_pass_length_cm=60\n"
      "min_pass_height_cm=70\n"
      "lidar_fov_deg=720\n"
      "lidar_min_range_cm=200\n"
      "lidar_max_range_cm=100\n"
      "resolution_near_distance_cm=50\n"
      "resolution_near_cell_cm=10\n"
      "resolution_far_distance_cm=300\n"
      "resolution_far_cell_cm=20\n"
      "max_rotate_deg=360\n"
      "max_advance_cm=100\n",
      "boundary_min_x=0\n"
      "boundary_min_y=0\n"
      "boundary_max_x=1\n"
      "boundary_max_y=1\n"
      "boundary_min_z=0\n"
      "boundary_max_z=0\n"
      "xy_decimals=0\n"
      "z_decimals=0\n",
      testutil::map_input_text(2, 2, 1, dm::Position{0, 0, 0}, {{1, 1, 0}}));

  const auto result = dm::run_simulation(dir.path());

  DM_ASSERT_TRUE(result.success);
  DM_ASSERT_TRUE(dir.exists("input_errors.txt"));
  DM_ASSERT_TRUE(dir.exists("map_output.txt"));
}
