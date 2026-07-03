// MapsComparison tests: the agreement-ratio scoring contract required by the
// assignment (identical=100, distinct~0, in-between reasonable).

#include "support/TestSupport.h"

#include <drone_mapper/MapsComparison.h>

#include <gtest/gtest.h>

using namespace dmtest;

namespace {

t::MapConfig cube(double res = 10) {
    return mapConfig(bounds(0, 40, 0, 40, 0, 40), pos(0, 0, 0), res); // 4x4x4
}

double score(const dm::IMap3D& origin, dm::IMap3D& target) {
    return dm::MapsComparison::compare(origin, std::vector<dm::IMap3D*>{&target}).front();
}

} // namespace

TEST(MapsComparison, IdenticalEmptyMapsScore100) {
    auto a = makeMap(cube(), t::VoxelOccupancy::Empty);
    auto b = makeMap(cube(), t::VoxelOccupancy::Empty);
    EXPECT_DOUBLE_EQ(score(*a, *b), 100.0);
}

TEST(MapsComparison, IdenticalOccupiedMapsScore100) {
    auto a = makeMap(cube(), t::VoxelOccupancy::Empty);
    auto b = makeMap(cube(), t::VoxelOccupancy::Empty);
    a->set(pos(5, 5, 5), t::VoxelOccupancy::Occupied);
    b->set(pos(5, 5, 5), t::VoxelOccupancy::Occupied);
    EXPECT_DOUBLE_EQ(score(*a, *b), 100.0);
}

TEST(MapsComparison, CompletelyOppositeMapsScore0) {
    auto a = makeMap(cube(), t::VoxelOccupancy::Occupied);
    auto b = makeMap(cube(), t::VoxelOccupancy::Empty);
    EXPECT_DOUBLE_EQ(score(*a, *b), 0.0);
}

TEST(MapsComparison, HalfMatchingScores50) {
    // 64 voxels; make exactly half of the target disagree.
    auto a = makeMap(cube(), t::VoxelOccupancy::Occupied);
    auto b = makeMap(cube(), t::VoxelOccupancy::Occupied);
    // Flip x index 0 and 1 planes (32 voxels) to Empty in the target.
    for (double x : {5.0, 15.0}) {
        for (double y = 5; y < 40; y += 10) {
            for (double z = 5; z < 40; z += 10) {
                b->set(pos(x, y, z), t::VoxelOccupancy::Empty);
            }
        }
    }
    EXPECT_DOUBLE_EQ(score(*a, *b), 50.0);
}

TEST(MapsComparison, SimilarMapsScoreHighButNot100) {
    auto a = makeMap(cube(), t::VoxelOccupancy::Occupied);
    auto b = makeMap(cube(), t::VoxelOccupancy::Occupied);
    b->set(pos(5, 5, 5), t::VoxelOccupancy::Empty); // single mismatch out of 64
    const double s = score(*a, *b);
    EXPECT_GT(s, 95.0);
    EXPECT_LT(s, 100.0);
}

TEST(MapsComparison, PotentiallyOccupiedCountsAsOccupied) {
    auto a = makeMap(cube(), t::VoxelOccupancy::Empty);
    auto b = makeMap(cube(), t::VoxelOccupancy::Empty);
    a->set(pos(5, 5, 5), t::VoxelOccupancy::Occupied);
    b->set(pos(5, 5, 5), t::VoxelOccupancy::PotentiallyOccupied);
    EXPECT_DOUBLE_EQ(score(*a, *b), 100.0);
}

TEST(MapsComparison, UnmappedTargetMismatchesOccupiedOrigin) {
    auto a = makeMap(cube(), t::VoxelOccupancy::Empty);
    auto b = makeMap(cube(), t::VoxelOccupancy::Unmapped);
    a->set(pos(5, 5, 5), t::VoxelOccupancy::Occupied);
    // Origin has 1 occupied; target treats all as not-occupied. 63/64 agree.
    const double s = score(*a, *b);
    EXPECT_GT(s, 98.0);
    EXPECT_LT(s, 100.0);
}

TEST(MapsComparison, ReturnsOneScorePerTarget) {
    auto origin = makeMap(cube(), t::VoxelOccupancy::Empty);
    auto t1 = makeMap(cube(), t::VoxelOccupancy::Empty);
    auto t2 = makeMap(cube(), t::VoxelOccupancy::Occupied);
    const auto scores = dm::MapsComparison::compare(
        *origin, std::vector<dm::IMap3D*>{t1.get(), t2.get()});
    ASSERT_EQ(scores.size(), 2u);
    EXPECT_DOUBLE_EQ(scores[0], 100.0);
    EXPECT_DOUBLE_EQ(scores[1], 0.0);
}

TEST(MapsComparison, NullTargetScoresMinusOne) {
    auto origin = makeMap(cube(), t::VoxelOccupancy::Empty);
    const auto scores = dm::MapsComparison::compare(*origin, std::vector<dm::IMap3D*>{nullptr});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], -1.0);
}

TEST(MapsComparison, ScoreIsBoundedZeroToHundred) {
    auto a = makeMap(cube(), t::VoxelOccupancy::Occupied);
    auto b = makeMap(cube(), t::VoxelOccupancy::Empty);
    const double s = score(*a, *b);
    EXPECT_GE(s, 0.0);
    EXPECT_LE(s, 100.0);
}

TEST(MapsComparison, QuarterOccupiedMatchScoresProportionally) {
    // Origin: x index 0 plane occupied (16/64). Target identical -> 100.
    auto a = makeMap(cube(), t::VoxelOccupancy::Empty);
    auto b = makeMap(cube(), t::VoxelOccupancy::Empty);
    for (double y = 5; y < 40; y += 10)
        for (double z = 5; z < 40; z += 10)
            a->set(pos(5, y, z), t::VoxelOccupancy::Occupied);
    // target maps only half the plane correctly (8 of 16), rest empty.
    int flips = 0;
    for (double y = 5; y < 40 && flips < 8; y += 10)
        for (double z = 5; z < 40 && flips < 8; z += 10, ++flips)
            b->set(pos(5, y, z), t::VoxelOccupancy::Occupied);
    // 8 occupied match + 48 empty match = 56/64.
    EXPECT_NEAR(score(*a, *b), 100.0 * 56.0 / 64.0, 1e-6);
}

TEST(MapsComparison, EmptyTargetVectorReturnsEmpty) {
    auto origin = makeMap(cube(), t::VoxelOccupancy::Empty);
    EXPECT_TRUE(dm::MapsComparison::compare(*origin, std::vector<dm::IMap3D*>{}).empty());
}

TEST(MapsComparison, SymmetricForIdenticalGeometry) {
    auto a = makeMap(cube(), t::VoxelOccupancy::Empty);
    auto b = makeMap(cube(), t::VoxelOccupancy::Empty);
    a->set(pos(15, 15, 15), t::VoxelOccupancy::Occupied);
    b->set(pos(25, 25, 25), t::VoxelOccupancy::Occupied);
    EXPECT_DOUBLE_EQ(score(*a, *b), score(*b, *a));
}

TEST(MapsComparison, MoreOverlapScoresHigher) {
    auto origin = makeMap(cube(), t::VoxelOccupancy::Empty);
    for (double x = 5; x < 40; x += 10)
        origin->set(pos(x, 5, 5), t::VoxelOccupancy::Occupied); // 4 occupied

    auto good = makeMap(cube(), t::VoxelOccupancy::Empty);
    good->set(pos(5, 5, 5), t::VoxelOccupancy::Occupied);
    good->set(pos(15, 5, 5), t::VoxelOccupancy::Occupied);

    auto bad = makeMap(cube(), t::VoxelOccupancy::Empty);
    bad->set(pos(35, 35, 35), t::VoxelOccupancy::Occupied); // no overlap

    EXPECT_GT(score(*origin, *good), score(*origin, *bad));
}

TEST(MapsComparison, ScoreUsesOriginResolutionGrid) {
    // A finer origin still yields 100 when the target agrees everywhere.
    auto a = makeMap(mapConfig(bounds(0, 20, 0, 20, 0, 20), pos(0, 0, 0), 5),
                     t::VoxelOccupancy::Empty);
    auto b = makeMap(mapConfig(bounds(0, 20, 0, 20, 0, 20), pos(0, 0, 0), 5),
                     t::VoxelOccupancy::Empty);
    EXPECT_DOUBLE_EQ(score(*a, *b), 100.0);
}

TEST(MapsComparison, DistinctPatternsScoreLow) {
    auto a = makeMap(cube(), t::VoxelOccupancy::Empty);
    auto b = makeMap(cube(), t::VoxelOccupancy::Empty);
    // Fill checkerboard-ish opposite halves.
    for (double x = 5; x < 40; x += 10)
        for (double y = 5; y < 40; y += 10)
            for (double z = 5; z < 40; z += 10) {
                const bool occ = static_cast<int>(x / 10) < 2;
                a->set(pos(x, y, z), occ ? t::VoxelOccupancy::Occupied : t::VoxelOccupancy::Empty);
                b->set(pos(x, y, z), occ ? t::VoxelOccupancy::Empty : t::VoxelOccupancy::Occupied);
            }
    EXPECT_LT(score(*a, *b), 5.0);
}

TEST(MapsComparison, OutOfBoundsTargetSampleCountsAsFree) {
    // Origin larger than the target; target samples outside its array read as
    // OutOfBounds, which is "not occupied" and matches the empty origin.
    auto origin = makeMap(mapConfig(bounds(0, 60, 0, 60, 0, 60), pos(0, 0, 0), 10),
                          t::VoxelOccupancy::Empty);
    auto target = makeMap(mapConfig(bounds(0, 20, 0, 20, 0, 20), pos(0, 0, 0), 10),
                          t::VoxelOccupancy::Empty);
    EXPECT_DOUBLE_EQ(score(*origin, *target), 100.0);
}
