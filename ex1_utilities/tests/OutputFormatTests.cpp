#include "TestFramework.hpp"
#include "TestUtils.hpp"
#include "drone_mapper/FileParsers.hpp"
#include "drone_mapper/Simulator.hpp"

DM_TEST(WriteWorldMapProducesExpectedFormatAndOrdering) {
  testutil::TempDir dir{"output_direct"};
  const dm::WorldDescription world{2, 2, 1, dm::Position{1, 0, 0}, {}};
  const std::vector<dm::CellState> dense{
      dm::CellState::Empty,
      dm::CellState::Occupied,
      dm::CellState::Unmapped,
      dm::CellState::OutOfBounds};

  dm::write_world_map(dir.file("map_output.txt"), world, dense);

  const auto lines = testutil::split_lines(dir.read("map_output.txt"));
  DM_ASSERT_EQ(lines.size(), 7u);
  DM_ASSERT_EQ(lines[0], "size=2,2,1");
  DM_ASSERT_EQ(lines[1], "start=1,0,0");
  DM_ASSERT_EQ(lines[2], "# x,y,z,state");
  DM_ASSERT_EQ(lines[3], "0,0,0,0");
  DM_ASSERT_EQ(lines[4], "1,0,0,1");
  DM_ASSERT_EQ(lines[5], "0,1,0,-1");
  DM_ASSERT_EQ(lines[6], "1,1,0,-2");
}

DM_TEST(RunSimulationOverwritesExistingMapOutputFile) {
  testutil::TempDir dir{"output_overwrite"};
  testutil::write_input_bundle(
      dir,
      testutil::valid_drone_config_text(),
      testutil::mission_config_text(dm::MissionConfig{0, 0, 0, 0, 0, 0, 0, 0}),
      testutil::map_input_text(1, 1, 1, dm::Position{0, 0, 0}));
  dir.write("map_output.txt", "stale data");

  const auto result = dm::run_simulation(dir.path());

  DM_ASSERT_TRUE(result.success);
  const auto content = dir.read("map_output.txt");
  testutil::expect_contains(content, "size=1,1,1");
  testutil::expect_not_contains(content, "stale data");
}

DM_TEST(RunSimulationOutputReflectsMappedOccupiedAndEmptyCells) {
  testutil::TempDir dir{"output_states"};
  testutil::write_input_bundle(
      dir,
      testutil::valid_drone_config_text(),
      testutil::mission_config_text(dm::MissionConfig{0, 0, 1, 0, 0, 0, 0, 0}),
      testutil::map_input_text(2, 1, 1, dm::Position{0, 0, 0}, {{1, 0, 0}}));

  const auto result = dm::run_simulation(dir.path());

  DM_ASSERT_TRUE(result.success);
  const auto lines = testutil::split_lines(dir.read("map_output.txt"));
  DM_ASSERT_EQ(lines[3], "0,0,0,0");
  DM_ASSERT_EQ(lines[4], "1,0,0,1");
}

DM_TEST(RunSimulationOutputIsStableAcrossRepeatedIdenticalRuns) {
  testutil::TempDir dir{"output_stable"};
  testutil::write_input_bundle(
      dir,
      testutil::valid_drone_config_text(),
      testutil::mission_config_text(dm::MissionConfig{0, 0, 2, 0, 0, 0, 0, 0}),
      testutil::map_input_text(3, 1, 1, dm::Position{0, 0, 0}, {{2, 0, 0}}));

  const auto first = dm::run_simulation(dir.path());
  const auto first_output = dir.read("map_output.txt");
  const auto second = dm::run_simulation(dir.path());
  const auto second_output = dir.read("map_output.txt");

  DM_ASSERT_TRUE(first.success);
  DM_ASSERT_TRUE(second.success);
  DM_ASSERT_EQ(first_output, second_output);
}
