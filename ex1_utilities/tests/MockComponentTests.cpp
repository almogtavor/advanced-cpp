#include "TestFramework.hpp"
#include "drone_mapper/Mocks.hpp"

namespace {

dm::DroneCapabilities test_capabilities() {
  dm::DroneCapabilities capabilities;
  capabilities.max_rotate = dm::AngleDeg{720.0};
  capabilities.max_advance = dm::DistanceCm{100.0};
  capabilities.max_elevate = dm::DistanceCm{100.0};
  capabilities.lidar_min_range = dm::DistanceCm{1.0};
  capabilities.lidar_max_range = dm::DistanceCm{500.0};
  return capabilities;
}

dm::MockWorld empty_world(int x = 4, int y = 4, int z = 3) {
  return dm::MockWorld{dm::WorldDescription{x, y, z, dm::Position{0, 0, 0}, {}}};
}

}  // namespace

DM_TEST(MockWorldReportsBoundsOccupancyAndEmptyCells) {
  dm::MockWorld world{dm::WorldDescription{3, 3, 2, dm::Position{0, 0, 0}, {dm::Position{1, 1, 0}}}};

  DM_ASSERT_TRUE(world.in_bounds(dm::Position{2, 2, 1}));
  DM_ASSERT_TRUE(!world.in_bounds(dm::Position{3, 0, 0}));
  DM_ASSERT_TRUE(world.is_occupied(dm::Position{1, 1, 0}));
  DM_ASSERT_TRUE(world.is_empty(dm::Position{0, 0, 0}));
  DM_ASSERT_TRUE(!world.is_empty(dm::Position{1, 1, 0}));
}

DM_TEST(MockPositionSensorReturnsCurrentSharedPosition) {
  dm::Position position{2, 1, 0};
  dm::MockPositionSensor sensor{&position};

  DM_ASSERT_EQ(sensor.current_position().x, 2);
  position.y = 3;
  DM_ASSERT_EQ(sensor.current_position().y, 3);
}

DM_TEST(MockMovementDriverAdvanceAndElevateClampToMaximumCellStep) {
  auto world = empty_world();
  dm::Position position{1, 1, 1};
  dm::AngleDeg heading{0.0};
  auto capabilities = test_capabilities();
  dm::MockMovementDriver driver{&position, &heading, &world, capabilities};

  DM_ASSERT_TRUE(driver.advance(dm::DistanceCm{250.0}));
  DM_ASSERT_EQ(position.x, 2);
  DM_ASSERT_TRUE(driver.elevate(dm::DistanceCm{150.0}));
  DM_ASSERT_EQ(position.z, 2);
}

DM_TEST(MockMovementDriverNegativeAdvanceMovesBackwardAlongCurrentHeading) {
  auto world = empty_world();
  dm::Position position{1, 1, 0};
  dm::AngleDeg heading{0.0};
  dm::MockMovementDriver driver{&position, &heading, &world, test_capabilities()};

  DM_ASSERT_TRUE(driver.advance(dm::DistanceCm{-100.0}));
  DM_ASSERT_EQ(position.x, 0);
}

DM_TEST(MockMovementDriverRotationSemanticsAreAbsoluteAndNormalized) {
  auto world = empty_world();
  dm::Position position{1, 1, 0};
  dm::AngleDeg heading{0.0};
  dm::MockMovementDriver driver{&position, &heading, &world, test_capabilities()};

  driver.rotate_left(dm::AngleDeg{450.0});
  DM_ASSERT_EQ(heading.as_double(), 90.0);
  driver.rotate_left(dm::AngleDeg{180.0});
  DM_ASSERT_EQ(heading.as_double(), 180.0);
  driver.rotate_right(dm::AngleDeg{90.0});
  DM_ASSERT_EQ(heading.as_double(), 270.0);
  driver.rotate_right(dm::AngleDeg{-90.0});
  DM_ASSERT_EQ(heading.as_double(), 90.0);
}

DM_TEST(MockMovementDriverRejectsUnsupportedHeadingBlockedMovesAndOutOfBoundsMoves) {
  dm::MockWorld world{dm::WorldDescription{3, 3, 1, dm::Position{0, 0, 0}, {dm::Position{2, 1, 0}}}};
  dm::Position position{1, 1, 0};
  dm::AngleDeg heading{45.0};
  dm::MockMovementDriver driver{&position, &heading, &world, test_capabilities()};

  DM_ASSERT_TRUE(!driver.advance(dm::DistanceCm{100.0}));
  heading = dm::AngleDeg{0.0};
  DM_ASSERT_TRUE(!driver.advance(dm::DistanceCm{100.0}));
  position = dm::Position{0, 0, 0};
  DM_ASSERT_TRUE(!driver.advance(dm::DistanceCm{-100.0}));
}

DM_TEST(MockLidarDetectsVisibleObstacleAtExactDistance) {
  dm::MockWorld world{dm::WorldDescription{5, 1, 1, dm::Position{0, 0, 0}, {dm::Position{3, 0, 0}}}};
  dm::Position position{0, 0, 0};
  dm::MockLidarSensor lidar{&position, &world, test_capabilities()};

  const auto hit = lidar.scan(dm::Direction3D{1, 0, 0});

  DM_ASSERT_EQ(hit.distance_cells, 3);
}

DM_TEST(MockLidarReturnsMinusTwoWhenObstacleIsCloserThanMinimumRange) {
  auto capabilities = test_capabilities();
  capabilities.lidar_min_range = dm::DistanceCm{250.0};
  dm::MockWorld world{dm::WorldDescription{3, 1, 1, dm::Position{0, 0, 0}, {dm::Position{1, 0, 0}}}};
  dm::Position position{0, 0, 0};
  dm::MockLidarSensor lidar{&position, &world, capabilities};

  const auto hit = lidar.scan(dm::Direction3D{1, 0, 0});

  DM_ASSERT_EQ(hit.distance_cells, -2);
}

DM_TEST(MockLidarReturnsMinusOneWhenNoObstacleIsFoundWithinRange) {
  auto capabilities = test_capabilities();
  capabilities.lidar_max_range = dm::DistanceCm{200.0};
  dm::MockWorld world{dm::WorldDescription{5, 1, 1, dm::Position{0, 0, 0}, {dm::Position{4, 0, 0}}}};
  dm::Position position{0, 0, 0};
  dm::MockLidarSensor lidar{&position, &world, capabilities};

  const auto hit = lidar.scan(dm::Direction3D{1, 0, 0});

  DM_ASSERT_EQ(hit.distance_cells, -1);
}

DM_TEST(MockLidarSupportsVerticalScansAndStopsAtNearestObstacle) {
  dm::MockWorld world{dm::WorldDescription{1, 1, 4, dm::Position{0, 0, 0}, {dm::Position{0, 0, 1}, dm::Position{0, 0, 3}}}};
  dm::Position position{0, 0, 0};
  dm::MockLidarSensor lidar{&position, &world, test_capabilities()};

  const auto hit = lidar.scan(dm::Direction3D{0, 0, 1});

  DM_ASSERT_EQ(hit.distance_cells, 1);
}
