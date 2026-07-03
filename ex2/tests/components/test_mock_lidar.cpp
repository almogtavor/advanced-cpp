// MockLidar component tests: beam ray-marching against a hidden truth map,
// z_min/z_max handling, misses, heading rotation, and FOV beam counts.

#include "support/Mocks.h"
#include "support/TestSupport.h"

#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>

#include <gtest/gtest.h>

#include <limits>
#include <memory>

using namespace dmtest;

namespace {

t::LidarConfigData lidarCfg(double z_min = 20, double z_max = 120, double d = 2.5,
                            std::size_t fov = 1) {
    return t::LidarConfigData{g::plen(z_min), g::plen(z_max), g::plen(d), fov};
}

// Hidden map: 20 x 20 x 5 voxels at res 10 (extent 200 x 200 x 50), all empty.
std::unique_ptr<dm::Map3DImpl> emptyHidden() {
    return makeMap(mapConfig(bounds(0, 200, 0, 200, 0, 50), pos(0, 0, 0), 10),
                   t::VoxelOccupancy::Empty);
}

bool isMiss(dm::PhysicalLength d) { return g::lcm(d) > 1e6; }

} // namespace

TEST(MockLidar, CenterBeamMeasuresDistanceToObstacle) {
    auto map = emptyHidden();
    map->set(pos(55, 25, 25), t::VoxelOccupancy::Occupied); // voxel index x=5
    dm::MockGPS gps{pos(5, 25, 25), orient(0), g::plen(10)};
    dm::MockLidar lidar{lidarCfg(), *map, gps};

    const auto scan = lidar.scan(orient(0, 0));
    ASSERT_FALSE(scan.empty());
    EXPECT_NEAR(g::lcm(scan.front().distance), 45.0, 1.5);
}

TEST(MockLidar, DetectsObstacleNearMaxRange) {
    // Catches "beam only reaches 2/3 of z_max" style bugs.
    auto map = emptyHidden();
    map->set(pos(125, 25, 25), t::VoxelOccupancy::Occupied); // ~115cm away
    dm::MockGPS gps{pos(5, 25, 25), orient(0), g::plen(10)};
    dm::MockLidar lidar{lidarCfg(20, 120), *map, gps};

    const auto scan = lidar.scan(orient(0, 0));
    ASSERT_FALSE(scan.empty());
    EXPECT_FALSE(isMiss(scan.front().distance));
    EXPECT_NEAR(g::lcm(scan.front().distance), 115.0, 2.0);
}

TEST(MockLidar, MissReturnsSentinelDistance) {
    auto map = emptyHidden(); // nothing occupied
    dm::MockGPS gps{pos(5, 25, 25), orient(0), g::plen(10)};
    dm::MockLidar lidar{lidarCfg(), *map, gps};

    const auto scan = lidar.scan(orient(0, 0));
    ASSERT_FALSE(scan.empty());
    EXPECT_TRUE(isMiss(scan.front().distance));
}

TEST(MockLidar, ObstacleCloserThanZMinReturnsZero) {
    auto map = emptyHidden();
    map->set(pos(25, 25, 25), t::VoxelOccupancy::Occupied); // ~15cm, below z_min 20
    dm::MockGPS gps{pos(5, 25, 25), orient(0), g::plen(10)};
    dm::MockLidar lidar{lidarCfg(20, 120), *map, gps};

    const auto scan = lidar.scan(orient(0, 0));
    ASSERT_FALSE(scan.empty());
    EXPECT_DOUBLE_EQ(g::lcm(scan.front().distance), 0.0);
}

TEST(MockLidar, HeadingRotatesBeamDirection) {
    auto map = emptyHidden();
    map->set(pos(25, 55, 25), t::VoxelOccupancy::Occupied); // along +y
    dm::MockGPS gps{pos(25, 5, 25), orient(90), g::plen(10)}; // facing +y (south)
    dm::MockLidar lidar{lidarCfg(), *map, gps};

    const auto scan = lidar.scan(orient(0, 0));
    ASSERT_FALSE(scan.empty());
    EXPECT_NEAR(g::lcm(scan.front().distance), 45.0, 2.0);
}

TEST(MockLidar, ScanOrientationSteersBeamRelativeToHeading) {
    auto map = emptyHidden();
    map->set(pos(25, 55, 25), t::VoxelOccupancy::Occupied); // along +y
    dm::MockGPS gps{pos(25, 5, 25), orient(0), g::plen(10)}; // facing +x
    dm::MockLidar lidar{lidarCfg(), *map, gps};

    // Steer the scan 90 degrees to look along +y even though heading is +x.
    const auto scan = lidar.scan(orient(90, 0));
    ASSERT_FALSE(scan.empty());
    EXPECT_NEAR(g::lcm(scan.front().distance), 45.0, 2.0);
}

TEST(MockLidar, EmptyMapCenterBeamMisses) {
    auto map = emptyHidden();
    dm::MockGPS gps{pos(100, 25, 25), orient(180), g::plen(10)};
    dm::MockLidar lidar{lidarCfg(), *map, gps};
    const auto scan = lidar.scan(orient(0, 0));
    ASSERT_FALSE(scan.empty());
    EXPECT_TRUE(isMiss(scan.front().distance));
}

TEST(MockLidar, CenterHitCarriesScanOrientationAngle) {
    auto map = emptyHidden();
    dm::MockGPS gps{pos(5, 25, 25), orient(0), g::plen(10)};
    dm::MockLidar lidar{lidarCfg(), *map, gps};
    const auto scan = lidar.scan(orient(30, 0));
    ASSERT_FALSE(scan.empty());
    EXPECT_NEAR(g::hdeg(scan.front().angle.horizontal), 30.0, 1e-6);
}

TEST(MockLidar, ZeroFovCirclesYieldsNoBeams) {
    auto map = emptyHidden();
    dm::MockGPS gps{pos(5, 25, 25), orient(0), g::plen(10)};
    dm::MockLidar lidar{lidarCfg(20, 120, 2.5, 0), *map, gps};
    EXPECT_TRUE(lidar.scan(orient(0, 0)).empty());
}

TEST(MockLidar, OneFovCircleYieldsSingleBeam) {
    auto map = emptyHidden();
    dm::MockGPS gps{pos(5, 25, 25), orient(0), g::plen(10)};
    dm::MockLidar lidar{lidarCfg(20, 120, 2.5, 1), *map, gps};
    EXPECT_EQ(lidar.scan(orient(0, 0)).size(), 1u);
}

TEST(MockLidar, TwoFovCirclesYieldFiveBeams) {
    auto map = emptyHidden();
    dm::MockGPS gps{pos(5, 25, 25), orient(0), g::plen(10)};
    dm::MockLidar lidar{lidarCfg(20, 120, 2.5, 2), *map, gps};
    EXPECT_EQ(lidar.scan(orient(0, 0)).size(), 1u + 4u);
}

TEST(MockLidar, ThreeFovCirclesYield21Beams) {
    auto map = emptyHidden();
    dm::MockGPS gps{pos(5, 25, 25), orient(0), g::plen(10)};
    dm::MockLidar lidar{lidarCfg(20, 120, 2.5, 3), *map, gps};
    EXPECT_EQ(lidar.scan(orient(0, 0)).size(), 1u + 4u + 16u);
}

TEST(MockLidar, ConfigGetterReturnsConfig) {
    auto map = emptyHidden();
    dm::MockGPS gps{pos(5, 25, 25), orient(0), g::plen(10)};
    dm::MockLidar lidar{lidarCfg(15, 95, 3.0, 4), *map, gps};
    const auto cfg = lidar.config();
    EXPECT_DOUBLE_EQ(g::lcm(cfg.z_min), 15.0);
    EXPECT_DOUBLE_EQ(g::lcm(cfg.z_max), 95.0);
    EXPECT_DOUBLE_EQ(g::lcm(cfg.d), 3.0);
    EXPECT_EQ(cfg.fov_circles, 4u);
}

TEST(MockLidar, BeamStopsAtNearestObstacle) {
    auto map = emptyHidden();
    map->set(pos(55, 25, 25), t::VoxelOccupancy::Occupied);  // nearer
    map->set(pos(155, 25, 25), t::VoxelOccupancy::Occupied); // farther
    dm::MockGPS gps{pos(5, 25, 25), orient(0), g::plen(10)};
    dm::MockLidar lidar{lidarCfg(), *map, gps};
    const auto scan = lidar.scan(orient(0, 0));
    ASSERT_FALSE(scan.empty());
    EXPECT_NEAR(g::lcm(scan.front().distance), 45.0, 1.5); // stops at the near one
}

TEST(MockLidar, ObstacleBeyondMaxRangeIsMissed) {
    auto map = emptyHidden();
    map->set(pos(195, 25, 25), t::VoxelOccupancy::Occupied); // ~190cm, beyond z_max 120
    dm::MockGPS gps{pos(5, 25, 25), orient(0), g::plen(10)};
    dm::MockLidar lidar{lidarCfg(20, 120), *map, gps};
    const auto scan = lidar.scan(orient(0, 0));
    ASSERT_FALSE(scan.empty());
    EXPECT_TRUE(isMiss(scan.front().distance));
}

TEST(MockLidar, MovingGpsChangesMeasuredDistance) {
    auto map = emptyHidden();
    map->set(pos(105, 25, 25), t::VoxelOccupancy::Occupied);
    dm::MockGPS gps{pos(5, 25, 25), orient(0), g::plen(10)};
    dm::MockLidar lidar{lidarCfg(), *map, gps};
    const double before = g::lcm(lidar.scan(orient(0, 0)).front().distance);
    gps.setPosition(pos(55, 25, 25)); // move 50cm closer
    const double after = g::lcm(lidar.scan(orient(0, 0)).front().distance);
    EXPECT_NEAR(before - after, 50.0, 2.0);
}
