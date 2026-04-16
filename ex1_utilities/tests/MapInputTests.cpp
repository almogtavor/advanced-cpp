#include "TestFramework.hpp"
#include "TestUtils.hpp"
#include "drone_mapper/FileParsers.hpp"

DM_TEST(ParseWorldMapValidSmallMapLoadsExpectedCells) {
  testutil::TempDir dir{"map_small"};
  dir.write("map_input.txt",
            "size=3,2,1\n"
            "start=1,0,0\n"
            "0,0,0\n"
            "2,1,0\n");

  dm::ErrorList errors;
  const auto world = dm::parse_world_map(dir.file("map_input.txt"), errors);

  DM_ASSERT_TRUE(errors.empty());
  DM_ASSERT_EQ(world.size_x, 3);
  DM_ASSERT_EQ(world.size_y, 2);
  DM_ASSERT_EQ(world.size_z, 1);
  DM_ASSERT_EQ(world.start.x, 1);
  DM_ASSERT_EQ(world.occupied_cells.size(), 2u);
  DM_ASSERT_EQ(world.occupied_cells[1].x, 2);
}

DM_TEST(ParseWorldMapAllowsCommentsWhitespaceAndOutOfOrderHeaders) {
  testutil::TempDir dir{"map_order"};
  dir.write("map_input.txt",
            "  start=0,1,0  # chosen start\n"
            "\n"
            "1,0,0\n"
            " size=4,3,2 \n"
            "2,2,1 # occupied\n");

  dm::ErrorList errors;
  const auto world = dm::parse_world_map(dir.file("map_input.txt"), errors);

  DM_ASSERT_TRUE(errors.empty());
  DM_ASSERT_EQ(world.size_x, 4);
  DM_ASSERT_EQ(world.size_y, 3);
  DM_ASSERT_EQ(world.size_z, 2);
  DM_ASSERT_EQ(world.start.y, 1);
  DM_ASSERT_EQ(world.occupied_cells.size(), 2u);
  DM_ASSERT_EQ(world.occupied_cells[0].x, 1);
}

DM_TEST(ParseWorldMapEmptyFileFailsBecauseSizeIsMissing) {
  testutil::TempDir dir{"map_empty"};
  dir.write("map_input.txt", "");

  dm::ErrorList errors;
  const auto message = testutil::expect_throws([&] { (void)dm::parse_world_map(dir.file("map_input.txt"), errors); });

  testutil::expect_contains(message, "Map size must be positive");
}

DM_TEST(ParseWorldMapInvalidSizeLineFailsImmediately) {
  testutil::TempDir dir{"map_bad_size"};
  dir.write("map_input.txt",
            "size=3,two,1\n"
            "start=0,0,0\n");

  dm::ErrorList errors;
  const auto message = testutil::expect_throws([&] { (void)dm::parse_world_map(dir.file("map_input.txt"), errors); });

  testutil::expect_contains(message, "Invalid size line");
}

DM_TEST(ParseWorldMapInvalidStartLineFailsImmediately) {
  testutil::TempDir dir{"map_bad_start"};
  dir.write("map_input.txt",
            "size=3,3,1\n"
            "start=0,zero,0\n");

  dm::ErrorList errors;
  const auto message = testutil::expect_throws([&] { (void)dm::parse_world_map(dir.file("map_input.txt"), errors); });

  testutil::expect_contains(message, "Invalid start line");
}

DM_TEST(ParseWorldMapMalformedOccupiedCellsAreRecoveredAndReported) {
  testutil::TempDir dir{"map_recover"};
  dir.write("map_input.txt",
            "size=2,2,1\n"
            "start=0,0,0\n"
            "1,1,0\n"
            "oops\n"
            "0,1\n");

  dm::ErrorList errors;
  const auto world = dm::parse_world_map(dir.file("map_input.txt"), errors);

  DM_ASSERT_EQ(world.occupied_cells.size(), 1u);
  DM_ASSERT_EQ(errors.size(), 2u);
  testutil::expect_contains(errors[0], "Ignoring malformed occupied cell line");
}

DM_TEST(ParseWorldMapMissingStartDefaultsToOriginWithoutError) {
  testutil::TempDir dir{"map_no_start"};
  dir.write("map_input.txt",
            "size=1,1,1\n");

  dm::ErrorList errors;
  const auto world = dm::parse_world_map(dir.file("map_input.txt"), errors);

  DM_ASSERT_TRUE(errors.empty());
  DM_ASSERT_EQ(world.start.x, 0);
  DM_ASSERT_EQ(world.start.y, 0);
  DM_ASSERT_EQ(world.start.z, 0);
}

DM_TEST(ParseWorldMapPreservesDuplicateAndNegativeCoordinatesAsWritten) {
  testutil::TempDir dir{"map_duplicates"};
  dir.write("map_input.txt",
            "size=3,3,1\n"
            "start=0,0,0\n"
            "-1,0,0\n"
            "2,2,0\n"
            "2,2,0\n");

  dm::ErrorList errors;
  const auto world = dm::parse_world_map(dir.file("map_input.txt"), errors);

  DM_ASSERT_TRUE(errors.empty());
  DM_ASSERT_EQ(world.occupied_cells.size(), 3u);
  DM_ASSERT_EQ(world.occupied_cells[0].x, -1);
  DM_ASSERT_EQ(world.occupied_cells[2].x, 2);
}
