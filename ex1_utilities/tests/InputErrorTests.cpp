#include "TestFramework.hpp"
#include "TestUtils.hpp"
#include "drone_mapper/Config.hpp"
#include "drone_mapper/Simulator.hpp"

DM_TEST(WriteInputErrorsSkipsFileWhenNoErrorsExist) {
  testutil::TempDir dir{"errors_none"};

  dm::write_input_errors(dir.path(), {});

  DM_ASSERT_TRUE(!dir.exists("input_errors.txt"));
}

DM_TEST(WriteInputErrorsWritesEveryRecoveredMessage) {
  testutil::TempDir dir{"errors_file"};

  dm::write_input_errors(dir.path(), {"first issue", "second issue"});

  DM_ASSERT_TRUE(dir.exists("input_errors.txt"));
  const auto content = dir.read("input_errors.txt");
  testutil::expect_contains(content, "first issue");
  testutil::expect_contains(content, "second issue");
}

DM_TEST(RunSimulationWithRecoverableInputErrorsSucceedsAndWritesInputErrors) {
  testutil::TempDir dir{"errors_recoverable"};
  testutil::write_input_bundle(
      dir,
      "max_advance_cm=100\n"
      "max_elevate_cm=100\n"
      "max_rotate_deg=360\n"
      "bad line\n",
      "boundary_min_x=0\n"
      "boundary_min_y=0\n"
      "boundary_max_x=1\n"
      "boundary_max_y=1\n"
      "boundary_min_z=0\n"
      "boundary_max_z=0\n"
      "xy_decimals=\n"
      "z_decimals=0\n",
      "size=2,2,1\n"
      "start=0,0,0\n"
      "oops\n"
      "1,1,0\n");

  const auto result = dm::run_simulation(dir.path());

  DM_ASSERT_TRUE(result.success);
  DM_ASSERT_TRUE(dir.exists("input_errors.txt"));
  DM_ASSERT_TRUE(dir.exists("map_output.txt"));
  const auto errors = dir.read("input_errors.txt");
  testutil::expect_contains(errors, "Ignoring malformed line");
  testutil::expect_contains(errors, "Missing key 'min_pass_width_cm'");
  testutil::expect_contains(errors, "xy_decimals");
  testutil::expect_contains(errors, "Ignoring malformed occupied cell line");
}

DM_TEST(RunSimulationWithCleanInputDoesNotWriteInputErrors) {
  testutil::TempDir dir{"errors_clean"};
  testutil::write_input_bundle(
      dir,
      testutil::valid_drone_config_text(),
      testutil::mission_config_text(dm::MissionConfig{0, 0, 1, 1, 0, 0, 0, 0}),
      testutil::map_input_text(2, 2, 1, dm::Position{0, 0, 0}, {{1, 1, 0}}));

  const auto result = dm::run_simulation(dir.path());

  DM_ASSERT_TRUE(result.success);
  DM_ASSERT_TRUE(!dir.exists("input_errors.txt"));
}

DM_TEST(RunSimulationMissingInputFileFailsGracefullyWithoutOutputs) {
  testutil::TempDir dir{"errors_missing"};
  dir.write("drone_config.txt", testutil::valid_drone_config_text());
  dir.write("mission_config.txt", testutil::mission_config_text());

  const auto result = dm::run_simulation(dir.path());

  DM_ASSERT_TRUE(!result.success);
  testutil::expect_contains(result.message, "Cannot open file");
  DM_ASSERT_TRUE(!dir.exists("map_output.txt"));
  DM_ASSERT_TRUE(!dir.exists("input_errors.txt"));
}

DM_TEST(RunSimulationMalformedMapFailsGracefullyWithoutCreatingMisleadingOutput) {
  testutil::TempDir dir{"errors_bad_map"};
  testutil::write_input_bundle(
      dir,
      testutil::valid_drone_config_text(),
      testutil::mission_config_text(),
      "size=a,b,c\n"
      "start=0,0,0\n");

  const auto result = dm::run_simulation(dir.path());

  DM_ASSERT_TRUE(!result.success);
  testutil::expect_contains(result.message, "Invalid size line");
  DM_ASSERT_TRUE(!dir.exists("map_output.txt"));
}
