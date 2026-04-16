#include "TestFramework.hpp"
#include "TestUtils.hpp"
#include "drone_mapper/Simulator.hpp"

DM_TEST(RunSimulationPerfectMatchProducesHundredScore) {
  testutil::TempDir dir{"score_perfect"};
  testutil::write_input_bundle(
      dir,
      testutil::valid_drone_config_text(),
      testutil::mission_config_text(dm::MissionConfig{0, 0, 0, 0, 0, 0, 0, 0}),
      testutil::map_input_text(1, 1, 1, dm::Position{0, 0, 0}));

  const auto result = dm::run_simulation(dir.path());

  DM_ASSERT_TRUE(result.success);
  DM_ASSERT_EQ(result.score, 100.0);
  DM_ASSERT_EQ(testutil::parse_score_message(result.message), 100.0);
}

DM_TEST(RunSimulationUnreachableRegionProducesExpectedIntermediateScore) {
  testutil::TempDir dir{"score_partial"};
  testutil::write_input_bundle(
      dir,
      testutil::valid_drone_config_text(),
      testutil::mission_config_text(dm::MissionConfig{0, 0, 4, 4, 0, 0, 0, 0}),
      testutil::map_input_text(
          5,
          5,
          1,
          dm::Position{0, 0, 0},
          {{1, 1, 0}, {2, 1, 0}, {3, 1, 0}, {1, 2, 0}, {3, 2, 0}, {1, 3, 0}, {2, 3, 0}, {3, 3, 0}}));

  const auto result = dm::run_simulation(dir.path());

  DM_ASSERT_TRUE(result.success);
  DM_ASSERT_EQ(result.score, 96.0);
}

DM_TEST(RunSimulationInconsistentOccupiedStartCanProduceZeroScoreWithoutCrashing) {
  testutil::TempDir dir{"score_zero"};
  testutil::write_input_bundle(
      dir,
      testutil::valid_drone_config_text(),
      testutil::mission_config_text(dm::MissionConfig{0, 0, 0, 0, 0, 0, 0, 0}),
      testutil::map_input_text(1, 1, 1, dm::Position{0, 0, 0}, {{0, 0, 0}}));

  const auto result = dm::run_simulation(dir.path());

  DM_ASSERT_TRUE(result.success);
  DM_ASSERT_EQ(result.score, 0.0);
}

DM_TEST(RunSimulationScoreIsAlwaysBoundedAndMessageIsFormatted) {
  testutil::TempDir dir{"score_bounds"};
  testutil::write_input_bundle(
      dir,
      testutil::valid_drone_config_text(),
      testutil::mission_config_text(dm::MissionConfig{0, 0, 2, 2, 0, 0, 0, 0}),
      testutil::map_input_text(3, 3, 1, dm::Position{0, 0, 0}, {{1, 1, 0}}));

  const auto result = dm::run_simulation(dir.path());

  DM_ASSERT_TRUE(result.success);
  DM_ASSERT_TRUE(result.score >= 0.0);
  DM_ASSERT_TRUE(result.score <= 100.0);
  testutil::expect_contains(result.message, "Mapping score:");
}
