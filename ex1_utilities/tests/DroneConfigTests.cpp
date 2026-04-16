#include "TestFramework.hpp"
#include "TestUtils.hpp"
#include "drone_mapper/FileParsers.hpp"

DM_TEST(ParseDroneConfigValidConfigLoadsAllFields) {
  testutil::TempDir dir{"drone_valid"};
  dir.write("drone_config.txt",
            "min_pass_width_cm=55.5\n"
            "min_pass_length_cm=65.5\n"
            "min_pass_height_cm=75.5\n"
            "lidar_fov_deg=120\n"
            "lidar_min_range_cm=12.5\n"
            "lidar_max_range_cm=650.5\n"
            "resolution_near_distance_cm=33.3\n"
            "resolution_near_cell_cm=11.1\n"
            "resolution_far_distance_cm=444.4\n"
            "resolution_far_cell_cm=22.2\n"
            "max_rotate_deg=270\n"
            "max_advance_cm=130\n"
            "max_elevate_cm=140\n");

  dm::ErrorList errors;
  const auto config = dm::parse_drone_config(dir.file("drone_config.txt"), errors);

  DM_ASSERT_TRUE(errors.empty());
  DM_ASSERT_EQ(config.min_pass_width.as_double(), 55.5);
  DM_ASSERT_EQ(config.min_pass_length.as_double(), 65.5);
  DM_ASSERT_EQ(config.min_pass_height.as_double(), 75.5);
  DM_ASSERT_EQ(config.lidar_fov.as_double(), 120.0);
  DM_ASSERT_EQ(config.lidar_min_range.as_double(), 12.5);
  DM_ASSERT_EQ(config.lidar_max_range.as_double(), 650.5);
  DM_ASSERT_EQ(config.resolution_near_distance.as_double(), 33.3);
  DM_ASSERT_EQ(config.resolution_near_cell.as_double(), 11.1);
  DM_ASSERT_EQ(config.resolution_far_distance.as_double(), 444.4);
  DM_ASSERT_EQ(config.resolution_far_cell.as_double(), 22.2);
  DM_ASSERT_EQ(config.max_rotate.as_double(), 270.0);
  DM_ASSERT_EQ(config.max_advance.as_double(), 130.0);
  DM_ASSERT_EQ(config.max_elevate.as_double(), 140.0);
}

DM_TEST(ParseDroneConfigEmptyFileUsesDefaultsAndReportsAllMissingKeys) {
  testutil::TempDir dir{"drone_empty"};
  dir.write("drone_config.txt", "");

  dm::ErrorList errors;
  const auto config = dm::parse_drone_config(dir.file("drone_config.txt"), errors);

  DM_ASSERT_EQ(errors.size(), 13u);
  DM_ASSERT_EQ(config.min_pass_width.as_double(), 50.0);
  DM_ASSERT_EQ(config.lidar_fov.as_double(), 90.0);
  DM_ASSERT_EQ(config.max_advance.as_double(), 100.0);
  testutil::expect_contains(errors.front(), "Missing key");
  testutil::expect_contains(errors.back(), "max_elevate_cm");
}

DM_TEST(ParseDroneConfigInvalidAndEmptyValuesFallbackIndividually) {
  testutil::TempDir dir{"drone_invalid"};
  dir.write("drone_config.txt",
            "min_pass_width_cm=\n"
            "min_pass_length_cm=abc\n"
            "min_pass_height_cm=70\n"
            "lidar_fov_deg=oops\n"
            "lidar_min_range_cm=1\n"
            "lidar_max_range_cm=500\n"
            "resolution_near_distance_cm=\n"
            "resolution_near_cell_cm=10\n"
            "resolution_far_distance_cm=300\n"
            "resolution_far_cell_cm=20\n"
            "max_rotate_deg=not-a-number\n"
            "max_advance_cm=100\n"
            "max_elevate_cm=100\n");

  dm::ErrorList errors;
  const auto config = dm::parse_drone_config(dir.file("drone_config.txt"), errors);

  DM_ASSERT_EQ(config.min_pass_width.as_double(), 50.0);
  DM_ASSERT_EQ(config.min_pass_length.as_double(), 50.0);
  DM_ASSERT_EQ(config.min_pass_height.as_double(), 70.0);
  DM_ASSERT_EQ(config.lidar_fov.as_double(), 90.0);
  DM_ASSERT_EQ(config.resolution_near_distance.as_double(), 50.0);
  DM_ASSERT_EQ(config.max_rotate.as_double(), 90.0);
  DM_ASSERT_EQ(errors.size(), 5u);
  testutil::expect_contains(errors[0], "min_pass_width_cm");
  testutil::expect_contains(errors[1], "min_pass_length_cm");
  testutil::expect_contains(errors[2], "lidar_fov_deg");
  testutil::expect_contains(errors[3], "resolution_near_distance_cm");
  testutil::expect_contains(errors[4], "max_rotate_deg");
}

DM_TEST(ParseDroneConfigWhitespaceRepeatedKeysUnknownAndMalformedLinesBehavePredictably) {
  testutil::TempDir dir{"drone_noise"};
  dir.write("drone_config.txt",
            "  min_pass_width_cm = 50  \n"
            "min_pass_length_cm=60\n"
            "min_pass_height_cm=70\n"
            "lidar_fov_deg=90\n"
            "lidar_min_range_cm=1\n"
            "lidar_max_range_cm=500\n"
            "resolution_near_distance_cm=50\n"
            "resolution_near_cell_cm=10\n"
            "resolution_far_distance_cm=300\n"
            "resolution_far_cell_cm=20\n"
            "max_rotate_deg=180\n"
            "max_advance_cm=100\n"
            "max_advance_cm=125\n"
            "max_elevate_cm=100\n"
            "unknown_key=999\n"
            "not a key value line\n");

  dm::ErrorList errors;
  const auto config = dm::parse_drone_config(dir.file("drone_config.txt"), errors);

  DM_ASSERT_EQ(config.max_advance.as_double(), 125.0);
  DM_ASSERT_EQ(errors.size(), 1u);
  testutil::expect_contains(errors[0], "Ignoring malformed line");
  testutil::expect_not_contains(errors[0], "unknown_key");
}

DM_TEST(ParseDroneConfigNegativeZeroAndLargeNumbersAreAcceptedAsProvided) {
  testutil::TempDir dir{"drone_extremes"};
  dir.write("drone_config.txt",
            "min_pass_width_cm=-1\n"
            "min_pass_length_cm=0\n"
            "min_pass_height_cm=999999\n"
            "lidar_fov_deg=725\n"
            "lidar_min_range_cm=900\n"
            "lidar_max_range_cm=100\n"
            "resolution_near_distance_cm=0.001\n"
            "resolution_near_cell_cm=-0.001\n"
            "resolution_far_distance_cm=1000000\n"
            "resolution_far_cell_cm=0\n"
            "max_rotate_deg=-360\n"
            "max_advance_cm=0\n"
            "max_elevate_cm=1000000\n");

  dm::ErrorList errors;
  const auto config = dm::parse_drone_config(dir.file("drone_config.txt"), errors);

  DM_ASSERT_TRUE(errors.empty());
  DM_ASSERT_EQ(config.min_pass_width.as_double(), -1.0);
  DM_ASSERT_EQ(config.min_pass_length.as_double(), 0.0);
  DM_ASSERT_EQ(config.min_pass_height.as_double(), 999999.0);
  DM_ASSERT_EQ(config.lidar_fov.as_double(), 725.0);
  DM_ASSERT_EQ(config.lidar_min_range.as_double(), 900.0);
  DM_ASSERT_EQ(config.lidar_max_range.as_double(), 100.0);
  DM_ASSERT_EQ(config.resolution_near_distance.as_double(), 0.001);
  DM_ASSERT_EQ(config.resolution_near_cell.as_double(), -0.001);
  DM_ASSERT_EQ(config.max_rotate.as_double(), -360.0);
  DM_ASSERT_EQ(config.max_advance.as_double(), 0.0);
}
