#include "TestFramework.hpp"
#include "TestUtils.hpp"
#include "drone_mapper/Cli.hpp"

#include <sstream>

DM_TEST(ResolveInputDirectoryUsesCurrentPathWhenNoArgumentIsProvided) {
  char program[] = "drone_mapper";
  char* argv[] = {program};

  const auto resolved = dm::resolve_input_directory(1, argv, "C:/tmp/current");

  DM_ASSERT_EQ(resolved.string(), "C:/tmp/current");
}

DM_TEST(ResolveInputDirectoryUsesExplicitArgumentWhenProvided) {
  char program[] = "drone_mapper";
  char directory[] = "C:/tmp/input";
  char* argv[] = {program, directory};

  const auto resolved = dm::resolve_input_directory(2, argv, "C:/tmp/current");

  DM_ASSERT_EQ(resolved.string(), "C:/tmp/input");
}

DM_TEST(RunCliPrintsSuccessMessageAndReturnsZero) {
  testutil::TempDir dir{"cli_success"};
  testutil::write_input_bundle(
      dir,
      testutil::valid_drone_config_text(),
      testutil::mission_config_text(dm::MissionConfig{0, 0, 0, 0, 0, 0, 0, 0}),
      testutil::map_input_text(1, 1, 1, dm::Position{0, 0, 0}));
  std::ostringstream out;
  std::string directory = dir.path().string();
  char program[] = "drone_mapper";
  char* argv[] = {program, directory.data()};

  const auto code = dm::run_cli(2, argv, "unused", out);

  DM_ASSERT_EQ(code, 0);
  testutil::expect_contains(out.str(), "Mapping score:");
}

DM_TEST(RunCliPrintsFailureMessageAndReturnsOne) {
  testutil::TempDir dir{"cli_failure"};
  std::ostringstream out;
  std::string directory = dir.path().string();
  char program[] = "drone_mapper";
  char* argv[] = {program, directory.data()};

  const auto code = dm::run_cli(2, argv, "unused", out);

  DM_ASSERT_EQ(code, 1);
  testutil::expect_contains(out.str(), "Simulation failed:");
}
