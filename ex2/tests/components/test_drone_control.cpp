// DroneControl tests: the per-step orchestration of algorithm -> movement ->
// scan -> map, in isolation using gmock doubles for every collaborator.

#include "support/Mocks.h"
#include "support/TestSupport.h"

#include <drone_mapper/DroneControlImpl.h>

#include <gtest/gtest.h>

#include <memory>

using namespace dmtest;
using ::testing::_;
using ::testing::InSequence;
using ::testing::IsNull;
using ::testing::NiceMock;
using ::testing::NotNull;
using ::testing::Return;

namespace {

t::LidarConfigData lidarCfg() {
    return t::LidarConfigData{g::plen(20), g::plen(120), g::plen(2.5), 1};
}

t::MappingStepCommand moveCmd(t::MovementCommandType type, double value) {
    t::MappingStepCommand cmd;
    t::MovementCommand m;
    m.type = type;
    if (type == t::MovementCommandType::Rotate) {
        m.angle = g::hang(value);
        m.rotation = t::RotationDirection::Right;
    } else {
        m.distance = g::plen(value);
    }
    cmd.movement = m;
    cmd.status = t::AlgorithmStatus::Working;
    return cmd;
}

t::MappingStepCommand finishedCmd() {
    t::MappingStepCommand cmd;
    cmd.status = t::AlgorithmStatus::Finished;
    return cmd;
}

t::MappingStepCommand scanCmd() {
    t::MappingStepCommand cmd;
    cmd.scan_orientation = orient(0, 0);
    cmd.status = t::AlgorithmStatus::Working;
    return cmd;
}

} // namespace

class DroneControl : public ::testing::Test {
protected:
    t::DroneConfigData drone_{g::plen(4), g::hang(90), g::plen(30), g::plen(20)};
    t::MissionConfigData mission_{};
    NiceMock<MockGps> gps_;
    NiceMock<MockMovement> movement_;
    NiceMock<MockLidar> lidar_;
    std::unique_ptr<dm::Map3DImpl> map_ =
        makeMap(mapConfig(bounds(0, 200, 0, 200, 0, 200), pos(0, 0, 0), 10),
                t::VoxelOccupancy::Unmapped);
    std::unique_ptr<NiceMock<MockMappingAlgorithm>> algo_;

    void SetUp() override {
        algo_ = std::make_unique<NiceMock<MockMappingAlgorithm>>(
            mission_, t::LidarConfigData{}, drone_, *map_);
        ON_CALL(gps_, position()).WillByDefault(Return(pos(55, 55, 55)));
        ON_CALL(gps_, heading()).WillByDefault(Return(orient(0)));
        ON_CALL(lidar_, config()).WillByDefault(Return(lidarCfg()));
    }

    dm::DroneControlImpl make() {
        return dm::DroneControlImpl(drone_, mission_, lidar_, gps_, movement_, *map_, *algo_);
    }
};

TEST_F(DroneControl, AlgorithmFinishedYieldsCompleted) {
    EXPECT_CALL(*algo_, nextStep(_, _)).WillOnce(Return(finishedCmd()));
    auto dc = make();
    const auto r = dc.step();
    EXPECT_EQ(r.status, t::DroneStepStatus::Completed);
}

TEST_F(DroneControl, AdvanceWithinLimitCallsMovement) {
    EXPECT_CALL(*algo_, nextStep(_, _)).WillOnce(Return(moveCmd(t::MovementCommandType::Advance, 10)));
    EXPECT_CALL(movement_, advance(_)).WillOnce(Return(t::MovementResult{true, {}}));
    auto dc = make();
    EXPECT_EQ(dc.step().status, t::DroneStepStatus::Continue);
}

TEST_F(DroneControl, AdvanceExceedingLimitErrorsWithoutMoving) {
    EXPECT_CALL(*algo_, nextStep(_, _)).WillOnce(Return(moveCmd(t::MovementCommandType::Advance, 50)));
    EXPECT_CALL(movement_, advance(_)).Times(0);
    auto dc = make();
    EXPECT_EQ(dc.step().status, t::DroneStepStatus::Error);
}

TEST_F(DroneControl, RotateExceedingLimitErrors) {
    EXPECT_CALL(*algo_, nextStep(_, _)).WillOnce(Return(moveCmd(t::MovementCommandType::Rotate, 120)));
    EXPECT_CALL(movement_, rotate(_, _)).Times(0);
    auto dc = make();
    EXPECT_EQ(dc.step().status, t::DroneStepStatus::Error);
}

TEST_F(DroneControl, ElevateExceedingLimitErrors) {
    EXPECT_CALL(*algo_, nextStep(_, _)).WillOnce(Return(moveCmd(t::MovementCommandType::Elevate, 40)));
    EXPECT_CALL(movement_, elevate(_)).Times(0);
    auto dc = make();
    EXPECT_EQ(dc.step().status, t::DroneStepStatus::Error);
}

TEST_F(DroneControl, RotateWithinLimitCallsMovement) {
    EXPECT_CALL(*algo_, nextStep(_, _)).WillOnce(Return(moveCmd(t::MovementCommandType::Rotate, 45)));
    EXPECT_CALL(movement_, rotate(_, _)).WillOnce(Return(t::MovementResult{true, {}}));
    auto dc = make();
    EXPECT_EQ(dc.step().status, t::DroneStepStatus::Continue);
}

TEST_F(DroneControl, ElevateWithinLimitAllowsNegative) {
    EXPECT_CALL(*algo_, nextStep(_, _)).WillOnce(Return(moveCmd(t::MovementCommandType::Elevate, -15)));
    EXPECT_CALL(movement_, elevate(_)).WillOnce(Return(t::MovementResult{true, {}}));
    auto dc = make();
    EXPECT_EQ(dc.step().status, t::DroneStepStatus::Continue);
}

TEST_F(DroneControl, MovementFailureReturnsError) {
    EXPECT_CALL(*algo_, nextStep(_, _)).WillOnce(Return(moveCmd(t::MovementCommandType::Advance, 10)));
    EXPECT_CALL(movement_, advance(_)).WillOnce(Return(t::MovementResult{false, "blocked"}));
    auto dc = make();
    EXPECT_EQ(dc.step().status, t::DroneStepStatus::Error);
}

TEST_F(DroneControl, HoverDoesNoMovement) {
    EXPECT_CALL(*algo_, nextStep(_, _)).WillOnce(Return(moveCmd(t::MovementCommandType::Hover, 0)));
    EXPECT_CALL(movement_, advance(_)).Times(0);
    EXPECT_CALL(movement_, rotate(_, _)).Times(0);
    EXPECT_CALL(movement_, elevate(_)).Times(0);
    auto dc = make();
    EXPECT_EQ(dc.step().status, t::DroneStepStatus::Continue);
}

TEST_F(DroneControl, ScanAppliesToOutputMap) {
    t::LidarScanResult scan{t::LidarHit{g::plen(45), orient(0, 0)}};
    EXPECT_CALL(*algo_, nextStep(_, _)).WillOnce(Return(scanCmd()));
    EXPECT_CALL(lidar_, scan(_)).WillOnce(Return(scan));
    auto dc = make();
    EXPECT_EQ(dc.step().status, t::DroneStepStatus::Continue);
    // Hit at origin(55) + 45 along +x -> occupied voxel at x=100.
    EXPECT_EQ(map_->atVoxel(pos(100, 55, 55)), t::VoxelOccupancy::Occupied);
}

TEST_F(DroneControl, MovementHappensBeforeScan) {
    t::MappingStepCommand cmd = scanCmd();
    t::MovementCommand m;
    m.type = t::MovementCommandType::Advance;
    m.distance = g::plen(10);
    cmd.movement = m;

    EXPECT_CALL(*algo_, nextStep(_, _)).WillOnce(Return(cmd));
    InSequence seq;
    EXPECT_CALL(movement_, advance(_)).WillOnce(Return(t::MovementResult{true, {}}));
    EXPECT_CALL(lidar_, scan(_)).WillOnce(Return(t::LidarScanResult{}));
    auto dc = make();
    EXPECT_EQ(dc.step().status, t::DroneStepStatus::Continue);
}

TEST_F(DroneControl, FirstStepPassesNullScanThenStoredScan) {
    t::LidarScanResult scan{t::LidarHit{g::plen(45), orient(0, 0)}};
    EXPECT_CALL(*algo_, nextStep(_, IsNull())).WillOnce(Return(scanCmd()));
    EXPECT_CALL(*algo_, nextStep(_, NotNull())).WillOnce(Return(finishedCmd()));
    ON_CALL(lidar_, scan(_)).WillByDefault(Return(scan));
    auto dc = make();
    EXPECT_EQ(dc.step().status, t::DroneStepStatus::Continue); // stores scan
    EXPECT_EQ(dc.step().status, t::DroneStepStatus::Completed);
}

TEST_F(DroneControl, StateReflectsGpsAndStepIndex) {
    ON_CALL(gps_, position()).WillByDefault(Return(pos(10, 20, 30)));
    ON_CALL(gps_, heading()).WillByDefault(Return(orient(45)));
    EXPECT_CALL(*algo_, nextStep(_, _))
        .WillRepeatedly(Return(moveCmd(t::MovementCommandType::Hover, 0)));
    auto dc = make();
    (void)dc.step();
    (void)dc.step();
    const auto s = dc.state();
    EXPECT_EQ(s.step_index, 2u);
    EXPECT_DOUBLE_EQ(g::xcm(s.position.x), 10.0);
    EXPECT_DOUBLE_EQ(g::hdeg(s.heading.horizontal), 45.0);
}

TEST_F(DroneControl, StepIndexDoesNotAdvanceOnCompletion) {
    EXPECT_CALL(*algo_, nextStep(_, _)).WillOnce(Return(finishedCmd()));
    auto dc = make();
    (void)dc.step();
    EXPECT_EQ(dc.state().step_index, 0u);
}

TEST_F(DroneControl, RotateForwardsDirectionToMovement) {
    t::MappingStepCommand cmd;
    t::MovementCommand m;
    m.type = t::MovementCommandType::Rotate;
    m.angle = g::hang(30);
    m.rotation = t::RotationDirection::Left;
    cmd.movement = m;
    EXPECT_CALL(*algo_, nextStep(_, _)).WillOnce(Return(cmd));
    EXPECT_CALL(movement_, rotate(t::RotationDirection::Left, _))
        .WillOnce(Return(t::MovementResult{true, {}}));
    auto dc = make();
    EXPECT_EQ(dc.step().status, t::DroneStepStatus::Continue);
}

TEST_F(DroneControl, FinishedWithUnmappableAlsoCompletes) {
    t::MappingStepCommand cmd;
    cmd.status = t::AlgorithmStatus::FinishedWithUnmappableVoxels;
    EXPECT_CALL(*algo_, nextStep(_, _)).WillOnce(Return(cmd));
    auto dc = make();
    EXPECT_EQ(dc.step().status, t::DroneStepStatus::Completed);
}
