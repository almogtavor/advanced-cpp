// SimulationRun component tests for the mock GPS and mock DroneMovement, as
// required by the assignment ("this component test should also test your mock
// implementations for the GPS and the DroneMovement").

#include "support/TestSupport.h"

#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockMovement.h>

#include <gtest/gtest.h>

using namespace dmtest;

TEST(SimulationRun, GpsReturnsConstructedPose) {
    dm::MockGPS gps{pos(10, 20, 30), orient(90, 5), g::plen(10)};
    EXPECT_DOUBLE_EQ(g::xcm(gps.position().x), 10.0);
    EXPECT_DOUBLE_EQ(g::ycm(gps.position().y), 20.0);
    EXPECT_DOUBLE_EQ(g::zcm(gps.position().z), 30.0);
    EXPECT_DOUBLE_EQ(g::hdeg(gps.heading().horizontal), 90.0);
    EXPECT_DOUBLE_EQ(g::adeg(gps.heading().altitude), 5.0);
}

TEST(SimulationRun, GpsResolutionGetter) {
    dm::MockGPS gps{pos(0, 0, 0), orient(0), g::plen(7)};
    EXPECT_DOUBLE_EQ(g::lcm(gps.resolution()), 7.0);
}

TEST(SimulationRun, GpsSettersUpdatePose) {
    dm::MockGPS gps{pos(0, 0, 0), orient(0), g::plen(10)};
    gps.setPosition(pos(1, 2, 3));
    gps.setHeading(orient(45, 10));
    EXPECT_DOUBLE_EQ(g::xcm(gps.position().x), 1.0);
    EXPECT_DOUBLE_EQ(g::hdeg(gps.heading().horizontal), 45.0);
    EXPECT_DOUBLE_EQ(g::adeg(gps.heading().altitude), 10.0);
}

TEST(SimulationRun, MovementRotateRightIncreasesHeading) {
    dm::MockGPS gps{pos(0, 0, 0), orient(0), g::plen(10)};
    dm::MockMovement movement{gps};
    ASSERT_TRUE(movement.rotate(t::RotationDirection::Right, g::hang(30)));
    EXPECT_DOUBLE_EQ(g::hdeg(gps.heading().horizontal), 30.0);
}

TEST(SimulationRun, MovementRotateLeftDecreasesHeading) {
    dm::MockGPS gps{pos(0, 0, 0), orient(90), g::plen(10)};
    dm::MockMovement movement{gps};
    ASSERT_TRUE(movement.rotate(t::RotationDirection::Left, g::hang(30)));
    EXPECT_DOUBLE_EQ(g::hdeg(gps.heading().horizontal), 60.0);
}

TEST(SimulationRun, MovementAdvanceEastMovesPlusX) {
    dm::MockGPS gps{pos(0, 0, 0), orient(0), g::plen(10)}; // 0deg = east (+x)
    dm::MockMovement movement{gps};
    ASSERT_TRUE(movement.advance(g::plen(25)));
    EXPECT_NEAR(g::xcm(gps.position().x), 25.0, 1e-9);
    EXPECT_NEAR(g::ycm(gps.position().y), 0.0, 1e-9);
    EXPECT_NEAR(g::zcm(gps.position().z), 0.0, 1e-9);
}

TEST(SimulationRun, MovementAdvanceSouthMovesPlusY) {
    dm::MockGPS gps{pos(0, 0, 0), orient(90), g::plen(10)}; // 90deg = south (+y)
    dm::MockMovement movement{gps};
    ASSERT_TRUE(movement.advance(g::plen(25)));
    EXPECT_NEAR(g::xcm(gps.position().x), 0.0, 1e-6);
    EXPECT_NEAR(g::ycm(gps.position().y), 25.0, 1e-9);
}

TEST(SimulationRun, MovementAdvanceWestMovesMinusX) {
    dm::MockGPS gps{pos(100, 100, 0), orient(180), g::plen(10)};
    dm::MockMovement movement{gps};
    ASSERT_TRUE(movement.advance(g::plen(40)));
    EXPECT_NEAR(g::xcm(gps.position().x), 60.0, 1e-6);
    EXPECT_NEAR(g::ycm(gps.position().y), 100.0, 1e-6);
}

TEST(SimulationRun, MovementElevateChangesHeightOnly) {
    dm::MockGPS gps{pos(10, 20, 30), orient(0), g::plen(10)};
    dm::MockMovement movement{gps};
    ASSERT_TRUE(movement.elevate(g::plen(15)));
    EXPECT_DOUBLE_EQ(g::zcm(gps.position().z), 45.0);
    EXPECT_DOUBLE_EQ(g::xcm(gps.position().x), 10.0);
    EXPECT_DOUBLE_EQ(g::ycm(gps.position().y), 20.0);
}

TEST(SimulationRun, MovementElevateAcceptsNegative) {
    dm::MockGPS gps{pos(0, 0, 50), orient(0), g::plen(10)};
    dm::MockMovement movement{gps};
    ASSERT_TRUE(movement.elevate(g::plen(-20)));
    EXPECT_DOUBLE_EQ(g::zcm(gps.position().z), 30.0);
}

TEST(SimulationRun, MovementRotateThenAdvanceComposes) {
    dm::MockGPS gps{pos(0, 0, 0), orient(0), g::plen(10)};
    dm::MockMovement movement{gps};
    movement.rotate(t::RotationDirection::Right, g::hang(90)); // now facing +y
    movement.advance(g::plen(10));
    EXPECT_NEAR(g::xcm(gps.position().x), 0.0, 1e-6);
    EXPECT_NEAR(g::ycm(gps.position().y), 10.0, 1e-6);
}

TEST(SimulationRun, MovementAdvancesAccumulate) {
    dm::MockGPS gps{pos(0, 0, 0), orient(0), g::plen(10)};
    dm::MockMovement movement{gps};
    movement.advance(g::plen(10));
    movement.advance(g::plen(10));
    movement.advance(g::plen(5));
    EXPECT_NEAR(g::xcm(gps.position().x), 25.0, 1e-6);
}
