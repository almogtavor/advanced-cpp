// MissionControl tests: the step loop, boundary validation, collision
// detection against the hidden map, status determination, and output saving.

#include "support/Mocks.h"
#include "support/TestSupport.h"

#include <drone_mapper/ErrorLog.h>
#include <drone_mapper/MissionControlImpl.h>

#include <gtest/gtest.h>

#include <stdexcept>

using namespace dmtest;
using ::testing::_;
using ::testing::NiceMock;
using ::testing::Return;
using ::testing::Throw;

namespace {
t::DroneStepResult contRes() { return {t::DroneStepStatus::Continue, {}}; }
t::DroneStepResult doneRes() { return {t::DroneStepStatus::Completed, {}}; }
t::DroneStepResult errRes() { return {t::DroneStepStatus::Error, "boom"}; }
} // namespace

class MissionControl : public ::testing::Test {
protected:
    t::MissionConfigData mission_{};
    t::DroneConfigData drone_{g::plen(4), g::hang(90), g::plen(30), g::plen(20)};
    NiceMock<MockMap> hidden_;
    NiceMock<MockMutableMap> output_;
    NiceMock<MockDroneControl> dc_;
    std::filesystem::path file_ = std::filesystem::temp_directory_path() / "dm_mc_out.npy";

    void SetUp() override {
        mission_.mission_bounds = bounds(0, 100, 0, 100, 0, 100);
        mission_.max_steps = 10;
        dm::globalErrorLog().open(std::filesystem::temp_directory_path() / "dm_mc_log.txt");
        ON_CALL(hidden_, atVoxel(_)).WillByDefault(Return(t::VoxelOccupancy::Empty));
        ON_CALL(dc_, state()).WillByDefault(Return(t::DroneState{pos(50, 50, 50), orient(0), 0}));
    }

    dm::MissionControlImpl make() {
        return dm::MissionControlImpl(mission_, drone_, hidden_, output_, dc_, file_);
    }
};

TEST_F(MissionControl, InvalidBoundsErrorsImmediately) {
    mission_.mission_bounds = bounds(100, 0, 0, 100, 0, 100); // inverted x
    EXPECT_CALL(dc_, step()).Times(0);
    EXPECT_CALL(output_, save(_)).Times(0);
    auto mc = make();
    const auto r = mc.runMission();
    EXPECT_EQ(r.status, t::MissionRunStatus::Error);
    ASSERT_FALSE(r.errors.empty());
    EXPECT_EQ(r.errors.front().code, "MISSION_BOUNDARY_INVALID");
}

TEST_F(MissionControl, CompletesWhenDroneCompletes) {
    EXPECT_CALL(dc_, step())
        .WillOnce(Return(contRes()))
        .WillOnce(Return(contRes()))
        .WillOnce(Return(doneRes()));
    EXPECT_CALL(output_, save(_)).Times(1);
    auto mc = make();
    const auto r = mc.runMission();
    EXPECT_EQ(r.status, t::MissionRunStatus::Completed);
    EXPECT_EQ(r.steps, 3u);
    EXPECT_TRUE(r.errors.empty());
}

TEST_F(MissionControl, MaxStepsWhenNeverCompletes) {
    mission_.max_steps = 5;
    EXPECT_CALL(dc_, step()).Times(5).WillRepeatedly(Return(contRes()));
    auto mc = make();
    const auto r = mc.runMission();
    EXPECT_EQ(r.status, t::MissionRunStatus::MaxSteps);
    EXPECT_EQ(r.steps, 5u);
}

TEST_F(MissionControl, CollisionWithHiddenMapErrors) {
    ON_CALL(hidden_, atVoxel(_)).WillByDefault(Return(t::VoxelOccupancy::Occupied));
    EXPECT_CALL(dc_, step()).WillOnce(Return(contRes()));
    auto mc = make();
    const auto r = mc.runMission();
    EXPECT_EQ(r.status, t::MissionRunStatus::Error);
    ASSERT_FALSE(r.errors.empty());
    EXPECT_EQ(r.errors.front().code, "DRONE_HITS_OBSTACLE");
}

TEST_F(MissionControl, DroneStepErrorPropagates) {
    EXPECT_CALL(dc_, step()).WillOnce(Return(errRes()));
    auto mc = make();
    const auto r = mc.runMission();
    EXPECT_EQ(r.status, t::MissionRunStatus::Error);
    ASSERT_FALSE(r.errors.empty());
    EXPECT_EQ(r.errors.front().code, "DRONE_STEP_ERROR");
}

TEST_F(MissionControl, SavesOutputMapOnSuccess) {
    EXPECT_CALL(dc_, step()).WillOnce(Return(doneRes()));
    EXPECT_CALL(output_, save(file_)).Times(1);
    auto mc = make();
    (void)mc.runMission();
}

TEST_F(MissionControl, SaveFailureBecomesError) {
    EXPECT_CALL(dc_, step()).WillOnce(Return(doneRes()));
    EXPECT_CALL(output_, save(_)).WillOnce(Throw(std::runtime_error("disk full")));
    auto mc = make();
    const auto r = mc.runMission();
    EXPECT_EQ(r.status, t::MissionRunStatus::Error);
    ASSERT_FALSE(r.errors.empty());
    EXPECT_EQ(r.errors.back().code, "OUTPUT_MAP_SAVE_ERROR");
}

TEST_F(MissionControl, ZeroMaxStepsReturnsMaxStepsWithZeroSteps) {
    mission_.max_steps = 0;
    EXPECT_CALL(dc_, step()).Times(0);
    EXPECT_CALL(output_, save(_)).Times(1);
    auto mc = make();
    const auto r = mc.runMission();
    EXPECT_EQ(r.status, t::MissionRunStatus::MaxSteps);
    EXPECT_EQ(r.steps, 0u);
}

TEST_F(MissionControl, CollisionIsLoggedImmediately) {
    ON_CALL(hidden_, atVoxel(_)).WillByDefault(Return(t::VoxelOccupancy::Occupied));
    EXPECT_CALL(dc_, step()).WillOnce(Return(contRes()));
    const std::size_t before = dm::globalErrorLog().count();
    auto mc = make();
    (void)mc.runMission();
    EXPECT_GT(dm::globalErrorLog().count(), before);
}

TEST_F(MissionControl, StopsSteppingAfterCompletion) {
    EXPECT_CALL(dc_, step()).WillOnce(Return(doneRes())); // exactly once
    auto mc = make();
    const auto r = mc.runMission();
    EXPECT_EQ(r.steps, 1u);
}

TEST_F(MissionControl, InvertedHeightBoundsAlsoInvalid) {
    mission_.mission_bounds = bounds(0, 100, 0, 100, 50, 50); // zero-height
    auto mc = make();
    const auto r = mc.runMission();
    EXPECT_EQ(r.status, t::MissionRunStatus::Error);
    EXPECT_EQ(r.errors.front().code, "MISSION_BOUNDARY_INVALID");
}

TEST_F(MissionControl, CollisionCheckedEveryStep) {
    // Drone completes on the 2nd step; collision (state) checked each step.
    EXPECT_CALL(dc_, step()).WillOnce(Return(contRes())).WillOnce(Return(doneRes()));
    EXPECT_CALL(dc_, state()).Times(::testing::AtLeast(2))
        .WillRepeatedly(Return(t::DroneState{pos(50, 50, 50), orient(0), 0}));
    auto mc = make();
    EXPECT_EQ(mc.runMission().status, t::MissionRunStatus::Completed);
}

TEST_F(MissionControl, SuccessfulRunLeavesLogUntouched) {
    EXPECT_CALL(dc_, step()).WillOnce(Return(doneRes()));
    const std::size_t before = dm::globalErrorLog().count();
    auto mc = make();
    (void)mc.runMission();
    EXPECT_EQ(dm::globalErrorLog().count(), before);
}
