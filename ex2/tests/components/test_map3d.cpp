// Map3D component tests: NPY loading/normalisation, world<->voxel geometry,
// bounds, mutation, and round-trip save/load.

#include "support/TestSupport.h"

#include <gtest/gtest.h>

#include <memory>

using namespace dmtest;

namespace {

std::unique_ptr<dm::Map3DImpl> loadMap(const std::string& name, double res = 10.0,
                                       dm::Position3D offset = pos(0, 0, 0)) {
    auto array = dm::loadHiddenMapArray(dataFile(name));
    const auto& shape = array->Shape();
    const auto b = bounds(g::xcm(offset.x), g::xcm(offset.x) + shape[0] * res,
                          g::ycm(offset.y), g::ycm(offset.y) + shape[1] * res,
                          g::zcm(offset.z), g::zcm(offset.z) + shape[2] * res);
    return std::make_unique<dm::Map3DImpl>(array, mapConfig(b, offset, res));
}

} // namespace

TEST(Map3D, LoadsSingleOccupiedVoxelAtItsCenter) {
    auto map = loadMap("single_center5.npy"); // occupied at (2,2,2), res 10
    EXPECT_EQ(map->atVoxel(pos(25, 25, 25)), t::VoxelOccupancy::Occupied);
}

TEST(Map3D, EmptyCellsReadAsEmpty) {
    auto map = loadMap("single_center5.npy");
    EXPECT_EQ(map->atVoxel(pos(5, 5, 5)), t::VoxelOccupancy::Empty);
    EXPECT_EQ(map->atVoxel(pos(45, 45, 45)), t::VoxelOccupancy::Empty);
}

TEST(Map3D, AxisOrderIsXYZ) {
    auto map = loadMap("voxel_x3y1z4.npy"); // occupied at x=3,y=1,z=4
    EXPECT_EQ(map->atVoxel(pos(35, 15, 45)), t::VoxelOccupancy::Occupied);
    // The transposed position must NOT be occupied.
    EXPECT_NE(map->atVoxel(pos(45, 15, 35)), t::VoxelOccupancy::Occupied);
}

TEST(Map3D, OutsideArrayReturnsOutOfBounds) {
    auto map = loadMap("single_center5.npy"); // extent 0..50
    EXPECT_EQ(map->atVoxel(pos(-5, 25, 25)), t::VoxelOccupancy::OutOfBounds);
    EXPECT_EQ(map->atVoxel(pos(55, 25, 25)), t::VoxelOccupancy::OutOfBounds);
    EXPECT_EQ(map->atVoxel(pos(25, 25, 500)), t::VoxelOccupancy::OutOfBounds);
}

TEST(Map3D, ResolutionScalesVoxelSize) {
    auto map = loadMap("single_center5.npy", 20.0); // res 20 => voxel (2,2,2) center 50
    EXPECT_EQ(map->atVoxel(pos(50, 50, 50)), t::VoxelOccupancy::Occupied);
    EXPECT_NE(map->atVoxel(pos(25, 25, 25)), t::VoxelOccupancy::Occupied);
}

TEST(Map3D, OffsetShiftsWorldCoordinates) {
    auto map = loadMap("single_center5.npy", 10.0, pos(100, 200, 300));
    // Voxel (2,2,2) center now at offset + 25.
    EXPECT_EQ(map->atVoxel(pos(125, 225, 325)), t::VoxelOccupancy::Occupied);
    EXPECT_NE(map->atVoxel(pos(25, 25, 25)), t::VoxelOccupancy::Occupied);
}

TEST(Map3D, NegativeOffsetHandled) {
    auto map = loadMap("single_center5.npy", 10.0, pos(-50, -50, -50));
    EXPECT_EQ(map->atVoxel(pos(-25, -25, -25)), t::VoxelOccupancy::Occupied);
}

TEST(Map3D, NormalisesNonBinaryBlockIds) {
    auto map = loadMap("blockids5.npy"); // uint8 values 18 and 45 -> Occupied
    EXPECT_EQ(map->atVoxel(pos(15, 15, 15)), t::VoxelOccupancy::Occupied);
    EXPECT_EQ(map->atVoxel(pos(25, 25, 25)), t::VoxelOccupancy::Occupied);
    EXPECT_EQ(map->atVoxel(pos(5, 5, 5)), t::VoxelOccupancy::Empty);
}

TEST(Map3D, IsInBoundsRespectsBoundaries) {
    const auto cfg = mapConfig(bounds(0, 100, 0, 100, 0, 100), pos(0, 0, 0), 10);
    auto map = makeMap(cfg, t::VoxelOccupancy::Unmapped);
    EXPECT_TRUE(map->isInBounds(pos(50, 50, 50)));
    EXPECT_TRUE(map->isInBounds(pos(0, 0, 0)));      // min corner is inclusive
    EXPECT_FALSE(map->isInBounds(pos(100, 50, 50))); // max is exclusive
    EXPECT_FALSE(map->isInBounds(pos(-1, 50, 50)));
    EXPECT_FALSE(map->isInBounds(pos(50, 50, 120)));
}

TEST(Map3D, SetThenAtVoxelRoundTrips) {
    const auto cfg = mapConfig(bounds(0, 50, 0, 50, 0, 50), pos(0, 0, 0), 10);
    auto map = makeMap(cfg, t::VoxelOccupancy::Unmapped);
    map->set(pos(25, 25, 25), t::VoxelOccupancy::Occupied);
    map->set(pos(5, 5, 5), t::VoxelOccupancy::Empty);
    map->set(pos(45, 45, 45), t::VoxelOccupancy::PotentiallyOccupied);
    EXPECT_EQ(map->atVoxel(pos(25, 25, 25)), t::VoxelOccupancy::Occupied);
    EXPECT_EQ(map->atVoxel(pos(5, 5, 5)), t::VoxelOccupancy::Empty);
    EXPECT_EQ(map->atVoxel(pos(45, 45, 45)), t::VoxelOccupancy::PotentiallyOccupied);
}

TEST(Map3D, SetOutOfRangeIsIgnored) {
    const auto cfg = mapConfig(bounds(0, 50, 0, 50, 0, 50), pos(0, 0, 0), 10);
    auto map = makeMap(cfg, t::VoxelOccupancy::Unmapped);
    map->set(pos(500, 500, 500), t::VoxelOccupancy::Occupied); // no-op, no crash
    EXPECT_EQ(map->atVoxel(pos(25, 25, 25)), t::VoxelOccupancy::Unmapped);
}

TEST(Map3D, NewMapStartsUnmapped) {
    const auto cfg = mapConfig(bounds(0, 30, 0, 30, 0, 30), pos(0, 0, 0), 10);
    auto map = makeMap(cfg, t::VoxelOccupancy::Unmapped);
    EXPECT_EQ(countValue(*map, t::VoxelOccupancy::Unmapped), 27); // 3x3x3
}

TEST(Map3D, GetMapConfigReturnsConstructionConfig) {
    const auto cfg = mapConfig(bounds(0, 100, 10, 60, -20, 40), pos(0, 10, -20), 5);
    auto map = makeMap(cfg, t::VoxelOccupancy::Unmapped);
    const auto got = map->getMapConfig();
    EXPECT_DOUBLE_EQ(g::lcm(got.resolution), 5.0);
    EXPECT_DOUBLE_EQ(g::xcm(got.boundaries.max_x), 100.0);
    EXPECT_DOUBLE_EQ(g::zcm(got.offset.z), -20.0);
}

TEST(Map3D, SaveThenReloadPreservesOccupancy) {
    const auto cfg = mapConfig(bounds(0, 40, 0, 40, 0, 40), pos(0, 0, 0), 10);
    auto map = makeMap(cfg, t::VoxelOccupancy::Empty);
    map->set(pos(15, 25, 35), t::VoxelOccupancy::Occupied);

    const auto path = std::filesystem::temp_directory_path() / "dm_map3d_roundtrip.npy";
    map->save(path);
    ASSERT_TRUE(std::filesystem::exists(path));

    auto array = std::make_shared<NpyArray>();
    ASSERT_EQ(array->LoadNPY(path.string()), nullptr);
    ASSERT_EQ(array->Shape().size(), 3u);
    EXPECT_EQ(array->Shape()[0], 4u);
    // Occupied byte at (1,2,3): flat = (1*4 + 2)*4 + 3 = 27
    EXPECT_EQ(array->Data<std::int8_t>()[27],
              static_cast<std::int8_t>(t::VoxelOccupancy::Occupied));
    std::filesystem::remove(path);
}

TEST(Map3D, DefaultConfigMapReadsUnmapped) {
    // A map with zero resolution (default MapConfig) cannot index; reads Unmapped.
    auto array = dm::makeOccupancyGrid(mapConfig(bounds(0, 10, 0, 10, 0, 10), pos(0, 0, 0), 10),
                                       t::VoxelOccupancy::Empty);
    dm::Map3DImpl map{array, t::MapConfig{}};
    EXPECT_EQ(map.atVoxel(pos(5, 5, 5)), t::VoxelOccupancy::Unmapped);
}

TEST(Map3D, ConstructWithNullThrows) {
    EXPECT_THROW(dm::Map3DImpl(std::shared_ptr<NpyArray>{}), std::invalid_argument);
}

TEST(Map3D, EachAxisIndependentlyIndexed) {
    // Distinct extents catch flat-index bugs that swap ny/nz.
    auto array = dm::loadHiddenMapArray(dataFile("voxel_x3y1z4.npy"));
    ASSERT_EQ(array->Shape()[0], 6u);
    auto map = loadMap("voxel_x3y1z4.npy");
    // Neighbours of the occupied voxel are empty.
    EXPECT_EQ(map->atVoxel(pos(25, 15, 45)), t::VoxelOccupancy::Empty); // x-1
    EXPECT_EQ(map->atVoxel(pos(35, 5, 45)), t::VoxelOccupancy::Empty);  // y-1
    EXPECT_EQ(map->atVoxel(pos(35, 15, 35)), t::VoxelOccupancy::Empty); // z-1
}

TEST(Map3D, SaveCreatesParentDirectories) {
    const auto cfg = mapConfig(bounds(0, 20, 0, 20, 0, 20), pos(0, 0, 0), 10);
    auto map = makeMap(cfg, t::VoxelOccupancy::Empty);
    const auto dir = std::filesystem::temp_directory_path() / "dm_map3d_newdir" / "sub";
    std::filesystem::remove_all(std::filesystem::temp_directory_path() / "dm_map3d_newdir");
    const auto path = dir / "m.npy";
    map->save(path);
    EXPECT_TRUE(std::filesystem::exists(path));
    std::filesystem::remove_all(std::filesystem::temp_directory_path() / "dm_map3d_newdir");
}
