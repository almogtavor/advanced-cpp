#include "TestFramework.hpp"
#include "TestUtils.hpp"
#include "drone_mapper/FileParsers.hpp"

DM_TEST(ParseMissionConfigValidMissionLoadsAllFields) {
  testutil::TempDir dir{"mission_valid"};
  dir.write("mission_config.txt",
            "boundary_min_x=-2\n"
            "boundary_min_y=-3\n"
            "boundary_max_x=10\n"
            "boundary_max_y=11\n"
            "boundary_min_z=1\n"
            "boundary_max_z=4\n"
            "xy_decimals=2\n"
            "z_decimals=3\n");

  dm::ErrorList errors;
  const auto mission = dm::parse_mission_config(dir.file("mission_config.txt"), errors);

  DM_ASSERT_TRUE(errors.empty());
  DM_ASSERT_EQ(mission.boundary_min_x, -2);
  DM_ASSERT_EQ(mission.boundary_min_y, -3);
  DM_ASSERT_EQ(mission.boundary_max_x, 10);
  DM_ASSERT_EQ(mission.boundary_max_y, 11);
  DM_ASSERT_EQ(mission.boundary_min_z, 1);
  DM_ASSERT_EQ(mission.boundary_max_z, 4);
  DM_ASSERT_EQ(mission.xy_decimals, 2);
  DM_ASSERT_EQ(mission.z_decimals, 3);
}

DM_TEST(ParseMissionConfigEmptyFileUsesDefaultsAndReportsMissingKeys) {
  testutil::TempDir dir{"mission_empty"};
  dir.write("mission_config.txt", "");

  dm::ErrorList errors;
  const auto mission = dm::parse_mission_config(dir.file("mission_config.txt"), errors);

  DM_ASSERT_EQ(errors.size(), 8u);
  DM_ASSERT_EQ(mission.boundary_min_x, 0);
  DM_ASSERT_EQ(mission.boundary_max_x, 0);
  DM_ASSERT_EQ(mission.boundary_min_z, 0);
  DM_ASSERT_EQ(mission.xy_decimals, 0);
  testutil::expect_contains(errors.front(), "boundary_min_x");
  testutil::expect_contains(errors.back(), "z_decimals");
}

DM_TEST(ParseMissionConfigInvalidValuesFallbackAndRecordErrors) {
  testutil::TempDir dir{"mission_invalid"};
  dir.write("mission_config.txt",
            "boundary_min_x=left\n"
            "boundary_min_y=\n"
            "boundary_max_x=5\n"
            "boundary_max_y=up\n"
            "boundary_min_z=0\n"
            "boundary_max_z=down\n"
            "xy_decimals=\n"
            "z_decimals=1\n");

  dm::ErrorList errors;
  const auto mission = dm::parse_mission_config(dir.file("mission_config.txt"), errors);

  DM_ASSERT_EQ(errors.size(), 5u);
  DM_ASSERT_EQ(mission.boundary_min_x, 0);
  DM_ASSERT_EQ(mission.boundary_min_y, 0);
  DM_ASSERT_EQ(mission.boundary_max_x, 5);
  DM_ASSERT_EQ(mission.boundary_max_y, 0);
  DM_ASSERT_EQ(mission.boundary_max_z, 0);
  DM_ASSERT_EQ(mission.xy_decimals, 0);
  DM_ASSERT_EQ(mission.z_decimals, 1);
}

DM_TEST(ParseMissionConfigWhitespaceDuplicatesUnknownAndMalformedLinesBehavePredictably) {
  testutil::TempDir dir{"mission_noise"};
  dir.write("mission_config.txt",
            " boundary_min_x = 0 \n"
            "boundary_min_y=0\n"
            "boundary_max_x=2\n"
            "boundary_max_y=2\n"
            "boundary_min_z=0\n"
            "boundary_max_z=0\n"
            "xy_decimals=0\n"
            "xy_decimals=4\n"
            "z_decimals=1\n"
            "unused=42\n"
            "not a pair\n");

  dm::ErrorList errors;
  const auto mission = dm::parse_mission_config(dir.file("mission_config.txt"), errors);

  DM_ASSERT_EQ(mission.xy_decimals, 4);
  DM_ASSERT_EQ(mission.z_decimals, 1);
  DM_ASSERT_EQ(errors.size(), 1u);
  testutil::expect_contains(errors[0], "Ignoring malformed line");
}

DM_TEST(ParseMissionConfigNumericButOddRangesAreAcceptedAsProvided) {
  testutil::TempDir dir{"mission_ranges"};
  dir.write("mission_config.txt",
            "boundary_min_x=5\n"
            "boundary_min_y=4\n"
            "boundary_max_x=4\n"
            "boundary_max_y=3\n"
            "boundary_min_z=2\n"
            "boundary_max_z=1\n"
            "xy_decimals=-1\n"
            "z_decimals=99\n");

  dm::ErrorList errors;
  const auto mission = dm::parse_mission_config(dir.file("mission_config.txt"), errors);

  DM_ASSERT_TRUE(errors.empty());
  DM_ASSERT_EQ(mission.boundary_min_x, 5);
  DM_ASSERT_EQ(mission.boundary_max_x, 4);
  DM_ASSERT_EQ(mission.boundary_min_z, 2);
  DM_ASSERT_EQ(mission.boundary_max_z, 1);
  DM_ASSERT_EQ(mission.xy_decimals, -1);
  DM_ASSERT_EQ(mission.z_decimals, 99);
}
