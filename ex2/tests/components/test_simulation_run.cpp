// SimulationRun tests for SimulationRunImpl::run(): running the mission,
// scoring the output against the hidden map, and assembling SimulationResult.

#include "support/Mocks.h"
#include "support/TestSupport.h"

#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/SimulationRunImpl.h>

#include <gtest/gtest.h>

#include <functional>
#include <memory>

using namespace dmtest;
using ::testing::NiceMock;
using ::testing::Return;

namespace {

t::MissionRunResult missionResult(t::MissionRunStatus status, std::size_t steps = 1) {
    t::MissionRunResult r;
    r.status = status;
    r.steps = steps;
    if (status == t::MissionRunStatus::Error) {
        r.errors.push_back(t::ErrorRef{"X", "err"});
    }
    return r;
}

// Builds a fully-wired SimulationRunImpl around a mocked mission control and
// runs it. The output map may be mutated before the run to shape the score.
t::SimulationResult runHarness(t::MissionConfigData mission, const t::MissionRunResult& mr,
                               const std::function<void(dm::Map3DImpl&)>& mutate = {}) {
    const auto cfg = mapConfig(bounds(0, 40, 0, 40, 0, 40), pos(0, 0, 0), 10);
    auto hidden = makeMap(cfg, t::VoxelOccupancy::Empty);
    auto output = makeMap(cfg, t::VoxelOccupancy::Empty);
    if (mutate) {
        mutate(*output);
    }
    const t::LidarConfigData lcfg{g::plen(20), g::plen(120), g::plen(2.5), 1};
    const t::DroneConfigData drone{g::plen(4), g::hang(90), g::plen(30), g::plen(20)};

    auto gps = std::make_unique<dm::MockGPS>(pos(5, 5, 5), orient(0), g::plen(10));
    auto movement = std::make_unique<dm::MockMovement>(*gps);
    auto lidar = std::make_unique<dm::MockLidar>(lcfg, *hidden, *gps);
    auto algo = std::make_unique<dm::MappingAlgorithmImpl>(mission, lcfg, drone, *output);
    auto dc = std::make_unique<NiceMock<MockDroneControl>>();
    auto mc = std::make_unique<NiceMock<MockMissionControl>>();
    EXPECT_CALL(*mc, runMission()).WillOnce(Return(mr));

    t::SimulationConfigData sim;
    sim.map_filename = "scene.npy";
    sim.map_resolution = g::plen(10);

    dm::SimulationRunImpl run(std::move(hidden), std::move(output), std::move(gps),
                              std::move(movement), std::move(lidar), std::move(algo),
                              std::move(dc), std::move(mc), sim, mission,
                              "out/output_map_0000.npy");
    return run.run();
}

t::MissionConfigData mission(double factor = 1.0) {
    t::MissionConfigData m;
    m.max_steps = 100;
    m.gps_resolution = g::plen(10);
    m.output_mapping_resolution_factor = factor;
    m.mission_bounds = bounds(0, 40, 0, 40, 0, 40);
    return m;
}

} // namespace

TEST(SimulationRun, CompletedRunScoresByComparison) {
    const auto result = runHarness(mission(), missionResult(t::MissionRunStatus::Completed));
    EXPECT_DOUBLE_EQ(result.mission_score, 100.0); // identical empty maps
}

TEST(SimulationRun, ErroredRunScoresMinusOne) {
    const auto result = runHarness(mission(), missionResult(t::MissionRunStatus::Error));
    EXPECT_DOUBLE_EQ(result.mission_score, -1.0);
}

TEST(SimulationRun, PartialScoreWhenOutputDiffers) {
    const auto result = runHarness(mission(), missionResult(t::MissionRunStatus::Completed),
                                   [](dm::Map3DImpl& out) {
                                       out.set(pos(5, 5, 5), t::VoxelOccupancy::Occupied);
                                   });
    EXPECT_GT(result.mission_score, 0.0);
    EXPECT_LT(result.mission_score, 100.0);
}

TEST(SimulationRun, ResultCarriesConfigsAndOutputFile) {
    const auto result = runHarness(mission(), missionResult(t::MissionRunStatus::Completed));
    EXPECT_EQ(result.simulation_config.map_filename.string(), "scene.npy");
    EXPECT_EQ(result.output_map_file.string(), "out/output_map_0000.npy");
    EXPECT_EQ(result.mission_config.max_steps, 100u);
}

TEST(SimulationRun, MissionResultsRecorded) {
    const auto result = runHarness(mission(), missionResult(t::MissionRunStatus::MaxSteps, 42));
    ASSERT_EQ(result.mission_results.size(), 1u);
    EXPECT_EQ(result.mission_results.front().status, t::MissionRunStatus::MaxSteps);
    EXPECT_EQ(result.mission_results.front().steps, 42u);
}

TEST(SimulationRun, ResolutionAcceptedForIntegerFactor) {
    const auto result = runHarness(mission(2.0), missionResult(t::MissionRunStatus::Completed));
    EXPECT_EQ(result.resolution_request_status, t::ResolutionRequestStatus::Accepted);
}

TEST(SimulationRun, ResolutionTooSmallForSubUnitFactor) {
    const auto result = runHarness(mission(0.5), missionResult(t::MissionRunStatus::Completed));
    EXPECT_EQ(result.resolution_request_status, t::ResolutionRequestStatus::IgnoredTooSmall);
}

TEST(SimulationRun, ResolutionIgnoredForNonIntegerFactor) {
    const auto result = runHarness(mission(1.5), missionResult(t::MissionRunStatus::Completed));
    EXPECT_EQ(result.resolution_request_status, t::ResolutionRequestStatus::Ignored);
}

TEST(SimulationRun, OutputMapConfigReported) {
    const auto result = runHarness(mission(), missionResult(t::MissionRunStatus::Completed));
    EXPECT_DOUBLE_EQ(g::lcm(result.output_map_config.resolution), 10.0);
}

TEST(SimulationRun, ConstructorRejectsNullDependency) {
    const auto cfg = mapConfig(bounds(0, 40, 0, 40, 0, 40), pos(0, 0, 0), 10);
    auto hidden = makeMap(cfg, t::VoxelOccupancy::Empty);
    auto output = makeMap(cfg, t::VoxelOccupancy::Empty);
    EXPECT_THROW(dm::SimulationRunImpl(std::move(hidden), std::move(output), nullptr, nullptr,
                                       nullptr, nullptr, nullptr, nullptr, {}, {}, "x.npy"),
                 std::invalid_argument);
}

TEST(SimulationRun, MaxStepsRunIsStillScored) {
    const auto result = runHarness(mission(), missionResult(t::MissionRunStatus::MaxSteps));
    EXPECT_GE(result.mission_score, 0.0); // not an error, so a real score
}
