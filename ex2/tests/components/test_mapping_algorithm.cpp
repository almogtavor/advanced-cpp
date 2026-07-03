// MappingAlgorithm tests: the frontier-exploration state machine driven purely
// through the read-only output map, in isolation from lidar/drone control.

#include "support/TestSupport.h"

#include <drone_mapper/MappingAlgorithmImpl.h>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

using namespace dmtest;

namespace {

t::DroneConfigData drone(double radius, double rot, double adv, double ele) {
    return t::DroneConfigData{g::plen(radius), g::hang(rot), g::plen(adv), g::plen(ele)};
}

t::DroneState st(dm::Position3D p, double heading) {
    return t::DroneState{p, orient(heading), 0};
}

// Calls nextStep n times, returning the commands.
std::vector<t::MappingStepCommand> steps(dm::MappingAlgorithmImpl& algo,
                                         const t::DroneState& state, int n) {
    std::vector<t::MappingStepCommand> out;
    for (int i = 0; i < n; ++i) {
        out.push_back(algo.nextStep(state, nullptr));
    }
    return out;
}

} // namespace

TEST(MappingAlgorithm, FirstCommandIsScanNotMovement) {
    auto out = makeMap(mapConfig(bounds(0, 50, 0, 10, 0, 10), pos(0, 0, 0), 10),
                       t::VoxelOccupancy::Unmapped);
    dm::MappingAlgorithmImpl algo{{}, {}, drone(4, 90, 10, 20), *out};
    const auto cmd = algo.nextStep(st(pos(5, 5, 5), 0), nullptr);
    EXPECT_TRUE(cmd.scan_orientation.has_value());
    EXPECT_FALSE(cmd.movement.has_value());
    EXPECT_EQ(cmd.status, t::AlgorithmStatus::Working);
}

TEST(MappingAlgorithm, ScanPhaseEmitsSixScans) {
    auto out = makeMap(mapConfig(bounds(0, 50, 0, 10, 0, 10), pos(0, 0, 0), 10),
                       t::VoxelOccupancy::Unmapped);
    dm::MappingAlgorithmImpl algo{{}, {}, drone(4, 90, 10, 20), *out};
    const auto cmds = steps(algo, st(pos(5, 5, 5), 0), 6);
    for (const auto& c : cmds) {
        EXPECT_TRUE(c.scan_orientation.has_value());
        EXPECT_FALSE(c.movement.has_value());
    }
}

TEST(MappingAlgorithm, ScanRingCoversUpAndDown) {
    auto out = makeMap(mapConfig(bounds(0, 50, 0, 10, 0, 10), pos(0, 0, 0), 10),
                       t::VoxelOccupancy::Unmapped);
    dm::MappingAlgorithmImpl algo{{}, {}, drone(4, 90, 10, 20), *out};
    bool up = false, down = false;
    for (const auto& c : steps(algo, st(pos(5, 5, 5), 0), 6)) {
        if (!c.scan_orientation) continue;
        const double alt = g::adeg(c.scan_orientation->altitude);
        if (alt > 45) up = true;
        if (alt < -45) down = true;
    }
    EXPECT_TRUE(up);
    EXPECT_TRUE(down);
}

TEST(MappingAlgorithm, PlansAdvanceToFrontier) {
    auto out = makeMap(mapConfig(bounds(0, 50, 0, 10, 0, 10), pos(0, 0, 0), 10),
                       t::VoxelOccupancy::Unmapped);
    out->set(pos(5, 5, 5), t::VoxelOccupancy::Empty);  // voxel 0 (start)
    out->set(pos(15, 5, 5), t::VoxelOccupancy::Empty); // voxel 1 (frontier: voxel 2 unmapped)
    dm::MappingAlgorithmImpl algo{{}, {}, drone(4, 90, 10, 20), *out};

    const auto cmds = steps(algo, st(pos(5, 5, 5), 0), 7);
    const auto& move = cmds.back();
    ASSERT_TRUE(move.movement.has_value());
    EXPECT_EQ(move.movement->type, t::MovementCommandType::Advance);
    EXPECT_NEAR(g::lcm(move.movement->distance), 10.0, 1e-6);
}

TEST(MappingAlgorithm, RotatesToFaceFrontier) {
    auto out = makeMap(mapConfig(bounds(0, 50, 0, 10, 0, 10), pos(0, 0, 0), 10),
                       t::VoxelOccupancy::Unmapped);
    out->set(pos(5, 5, 5), t::VoxelOccupancy::Empty);
    out->set(pos(15, 5, 5), t::VoxelOccupancy::Empty);
    dm::MappingAlgorithmImpl algo{{}, {}, drone(4, 90, 10, 20), *out};

    // Drone faces west (180); frontier is east -> expect two 90-degree turns.
    const auto cmds = steps(algo, st(pos(5, 5, 5), 180), 9);
    ASSERT_TRUE(cmds[6].movement.has_value());
    ASSERT_TRUE(cmds[7].movement.has_value());
    EXPECT_EQ(cmds[6].movement->type, t::MovementCommandType::Rotate);
    EXPECT_EQ(cmds[7].movement->type, t::MovementCommandType::Rotate);
    EXPECT_NEAR(g::hdeg(cmds[6].movement->angle), 90.0, 1e-6);
    ASSERT_TRUE(cmds[8].movement.has_value());
    EXPECT_EQ(cmds[8].movement->type, t::MovementCommandType::Advance);
}

TEST(MappingAlgorithm, ChunksAdvanceByMaxAdvance) {
    auto out = makeMap(mapConfig(bounds(0, 50, 0, 10, 0, 10), pos(0, 0, 0), 10),
                       t::VoxelOccupancy::Unmapped);
    out->set(pos(5, 5, 5), t::VoxelOccupancy::Empty);
    out->set(pos(15, 5, 5), t::VoxelOccupancy::Empty);
    dm::MappingAlgorithmImpl algo{{}, {}, drone(4, 90, 4, 20), *out}; // max_advance 4

    const auto cmds = steps(algo, st(pos(5, 5, 5), 0), 9);
    double total = 0;
    for (int i = 6; i < 9; ++i) {
        ASSERT_TRUE(cmds[i].movement.has_value());
        EXPECT_EQ(cmds[i].movement->type, t::MovementCommandType::Advance);
        EXPECT_LE(g::lcm(cmds[i].movement->distance), 4.0 + 1e-6);
        total += g::lcm(cmds[i].movement->distance);
    }
    EXPECT_NEAR(total, 10.0, 1e-6);
}

TEST(MappingAlgorithm, EmitsElevateForVerticalFrontier) {
    auto out = makeMap(mapConfig(bounds(0, 10, 0, 10, 0, 30), pos(0, 0, 0), 10),
                       t::VoxelOccupancy::Unmapped);
    out->set(pos(5, 5, 5), t::VoxelOccupancy::Empty);  // z 0
    out->set(pos(5, 5, 15), t::VoxelOccupancy::Empty); // z 1 (frontier: z 2 unmapped)
    dm::MappingAlgorithmImpl algo{{}, {}, drone(4, 90, 10, 20), *out};

    const auto cmds = steps(algo, st(pos(5, 5, 5), 0), 7);
    ASSERT_TRUE(cmds.back().movement.has_value());
    EXPECT_EQ(cmds.back().movement->type, t::MovementCommandType::Elevate);
    EXPECT_NEAR(g::lcm(cmds.back().movement->distance), 10.0, 1e-6);
}

TEST(MappingAlgorithm, FinishedWhenFullyMapped) {
    auto out = makeMap(mapConfig(bounds(0, 30, 0, 10, 0, 10), pos(0, 0, 0), 10),
                       t::VoxelOccupancy::Empty); // no unmapped voxels
    dm::MappingAlgorithmImpl algo{{}, {}, drone(4, 90, 10, 20), *out};
    const auto cmds = steps(algo, st(pos(5, 5, 5), 0), 7);
    EXPECT_EQ(cmds.back().status, t::AlgorithmStatus::Finished);
    EXPECT_FALSE(cmds.back().movement.has_value());
}

TEST(MappingAlgorithm, FinishedWithUnmappableWhenFrontierUnreachable) {
    auto out = makeMap(mapConfig(bounds(0, 30, 0, 10, 0, 10), pos(0, 0, 0), 10),
                       t::VoxelOccupancy::Unmapped);
    out->set(pos(5, 5, 5), t::VoxelOccupancy::Empty);     // voxel 0 (start)
    out->set(pos(15, 5, 5), t::VoxelOccupancy::Occupied); // voxel 1 blocks the way
    // voxel 2 remains Unmapped but is unreachable.
    dm::MappingAlgorithmImpl algo{{}, {}, drone(4, 90, 10, 20), *out};
    const auto cmds = steps(algo, st(pos(5, 5, 5), 0), 7);
    EXPECT_EQ(cmds.back().status, t::AlgorithmStatus::FinishedWithUnmappableVoxels);
}

TEST(MappingAlgorithm, NeverPathsThroughOccupiedVoxel) {
    auto out = makeMap(mapConfig(bounds(0, 30, 0, 10, 0, 10), pos(0, 0, 0), 10),
                       t::VoxelOccupancy::Unmapped);
    out->set(pos(5, 5, 5), t::VoxelOccupancy::Empty);
    out->set(pos(15, 5, 5), t::VoxelOccupancy::Occupied);
    dm::MappingAlgorithmImpl algo{{}, {}, drone(4, 90, 10, 20), *out};
    // Run several steps: no Advance may ever be issued into the wall.
    for (const auto& c : steps(algo, st(pos(5, 5, 5), 0), 12)) {
        if (c.movement && c.movement->type == t::MovementCommandType::Advance) {
            FAIL() << "Algorithm tried to advance into an occupied voxel.";
        }
    }
}

TEST(MappingAlgorithm, RespectsAllMovementLimits) {
    auto out = makeMap(mapConfig(bounds(0, 40, 0, 10, 0, 10), pos(0, 0, 0), 10),
                       t::VoxelOccupancy::Unmapped);
    out->set(pos(5, 5, 5), t::VoxelOccupancy::Empty);
    out->set(pos(15, 5, 5), t::VoxelOccupancy::Empty);
    const auto d = drone(4, 45, 3, 7);
    dm::MappingAlgorithmImpl algo{{}, {}, d, *out};
    for (const auto& c : steps(algo, st(pos(5, 5, 5), 200), 20)) {
        if (!c.movement) continue;
        switch (c.movement->type) {
        case t::MovementCommandType::Rotate:
            EXPECT_LE(g::hdeg(c.movement->angle), 45.0 + 1e-6);
            break;
        case t::MovementCommandType::Advance:
            EXPECT_LE(g::lcm(c.movement->distance), 3.0 + 1e-6);
            break;
        case t::MovementCommandType::Elevate:
            EXPECT_LE(std::abs(g::lcm(c.movement->distance)), 7.0 + 1e-6);
            break;
        case t::MovementCommandType::Hover:
            break;
        }
    }
}

TEST(MappingAlgorithm, RescansAfterMovementQueueEmpties) {
    auto out = makeMap(mapConfig(bounds(0, 50, 0, 10, 0, 10), pos(0, 0, 0), 10),
                       t::VoxelOccupancy::Unmapped);
    out->set(pos(5, 5, 5), t::VoxelOccupancy::Empty);
    out->set(pos(15, 5, 5), t::VoxelOccupancy::Empty);
    dm::MappingAlgorithmImpl algo{{}, {}, drone(4, 90, 10, 20), *out};

    // 6 scans + 1 advance (single-command queue). The 8th step should scan
    // again from the assumed new location.
    steps(algo, st(pos(5, 5, 5), 0), 7);
    const auto after = algo.nextStep(st(pos(15, 5, 5), 0), nullptr);
    EXPECT_TRUE(after.scan_orientation.has_value());
}

TEST(MappingAlgorithm, StaysWorkingWhileFrontierRemains) {
    auto out = makeMap(mapConfig(bounds(0, 50, 0, 10, 0, 10), pos(0, 0, 0), 10),
                       t::VoxelOccupancy::Unmapped);
    out->set(pos(5, 5, 5), t::VoxelOccupancy::Empty);
    out->set(pos(15, 5, 5), t::VoxelOccupancy::Empty);
    dm::MappingAlgorithmImpl algo{{}, {}, drone(4, 90, 10, 20), *out};
    for (const auto& c : steps(algo, st(pos(5, 5, 5), 0), 7)) {
        EXPECT_NE(c.status, t::AlgorithmStatus::Finished);
    }
}

TEST(MappingAlgorithm, EmptyMissionBoundsFinishImmediately) {
    // Output map with a single voxel and no unmapped neighbours -> Finished.
    auto out = makeMap(mapConfig(bounds(0, 10, 0, 10, 0, 10), pos(0, 0, 0), 10),
                       t::VoxelOccupancy::Empty);
    dm::MappingAlgorithmImpl algo{{}, {}, drone(4, 90, 10, 20), *out};
    const auto cmds = steps(algo, st(pos(5, 5, 5), 0), 7);
    EXPECT_EQ(cmds.back().status, t::AlgorithmStatus::Finished);
}

TEST(MappingAlgorithm, ClearanceAvoidsCellsAdjacentToObstacle) {
    // Drone radius >= resolution forces one-voxel clearance from obstacles.
    auto out = makeMap(mapConfig(bounds(0, 50, 0, 10, 0, 10), pos(0, 0, 0), 10),
                       t::VoxelOccupancy::Unmapped);
    out->set(pos(5, 5, 5), t::VoxelOccupancy::Empty);
    out->set(pos(15, 5, 5), t::VoxelOccupancy::Empty);
    out->set(pos(25, 5, 5), t::VoxelOccupancy::Occupied); // adjacent to voxel 1
    dm::MappingAlgorithmImpl algo{{}, {}, drone(12, 90, 10, 20), *out}; // radius 12 -> clearance 1
    // Voxel 1 is within clearance of the obstacle, so it is not traversable and
    // the drone cannot advance into it.
    for (const auto& c : steps(algo, st(pos(5, 5, 5), 0), 10)) {
        if (c.movement && c.movement->type == t::MovementCommandType::Advance) {
            FAIL() << "Advanced despite required obstacle clearance.";
        }
    }
}

TEST(MappingAlgorithm, ForwardScanEmittedFirst) {
    auto out = makeMap(mapConfig(bounds(0, 30, 0, 10, 0, 10), pos(0, 0, 0), 10),
                       t::VoxelOccupancy::Unmapped);
    dm::MappingAlgorithmImpl algo{{}, {}, drone(4, 90, 10, 20), *out};
    const auto cmd = algo.nextStep(st(pos(5, 5, 5), 0), nullptr);
    ASSERT_TRUE(cmd.scan_orientation.has_value());
    EXPECT_NEAR(g::hdeg(cmd.scan_orientation->horizontal), 0.0, 1e-6);
    EXPECT_NEAR(g::adeg(cmd.scan_orientation->altitude), 0.0, 1e-6);
}
