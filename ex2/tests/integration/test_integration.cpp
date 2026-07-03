// Integration tests exercising the whole flow end to end: the real
// SimulationManager + factory with the real mapping algorithm, and a second
// pass wiring the real components around a mock algorithm.

#include "support/Mocks.h"
#include "support/TestSupport.h"

#include <drone_mapper/ConfigLoader.h>
#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MapBuilder.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>

#include <gtest/gtest.h>

#include <memory>

using namespace dmtest;
using ::testing::_;
using ::testing::NiceMock;
using ::testing::Return;

namespace {

std::filesystem::path tmpOut(const std::string& name) {
    const auto p = std::filesystem::temp_directory_path() / ("dm_it_" + name);
    std::filesystem::remove_all(p);
    return p;
}

dm::types::SimulationManagerReport runComposition(const std::filesystem::path& composition,
                                                  const std::filesystem::path& out) {
    const auto data = dm::config::loadComposition(composition);
    auto factory = std::make_unique<dm::SimulationRunFactoryImpl>();
    dm::SimulationManager manager{std::move(factory)};
    return manager.run(data, out);
}

} // namespace

TEST(Integration, RealAlgorithmMapsOpenScene) {
    const auto out = tmpOut("open");
    const auto report = runComposition(dataFile("integration/composition.yaml"), out);
    ASSERT_EQ(report.runs.size(), 1u);
    const auto& run = report.runs.front();
    EXPECT_GE(run.mission_score, 0.0);
    EXPECT_LE(run.mission_score, 100.0);
    ASSERT_FALSE(run.mission_results.empty());
    EXPECT_NE(run.mission_results.front().status, dm::types::MissionRunStatus::Error);
}

TEST(Integration, ProducesOutputArtifacts) {
    const auto out = tmpOut("artifacts");
    (void)runComposition(dataFile("integration/composition.yaml"), out);
    EXPECT_TRUE(std::filesystem::exists(out / "simulation_output.yaml"));
    EXPECT_TRUE(std::filesystem::exists(out / "output_results"));
    EXPECT_TRUE(std::filesystem::exists(out / "output_results" / "output_map_0000.npy"));
    EXPECT_TRUE(std::filesystem::exists(out / "output_results" / "error_log.txt"));
}

TEST(Integration, OutputMapIsReloadableNpy) {
    const auto out = tmpOut("reload");
    (void)runComposition(dataFile("integration/composition.yaml"), out);
    auto array = std::make_shared<NpyArray>();
    ASSERT_EQ(array->LoadNPY((out / "output_results" / "output_map_0000.npy").string()), nullptr);
    EXPECT_EQ(array->Shape().size(), 3u);
}

TEST(Integration, RealAlgorithmMapsFreeSpaceAsEmpty) {
    const auto out = tmpOut("freespace");
    (void)runComposition(dataFile("integration/composition.yaml"), out);
    // Reload the output map and confirm the drone marked some voxels empty.
    auto array = std::make_shared<NpyArray>();
    ASSERT_EQ(array->LoadNPY((out / "output_results" / "output_map_0000.npy").string()), nullptr);
    long empties = 0;
    const std::int8_t* data = array->Data<std::int8_t>();
    for (std::size_t i = 0; i < array->NumValue(); ++i) {
        if (data[i] == static_cast<std::int8_t>(dm::types::VoxelOccupancy::Empty)) {
            ++empties;
        }
    }
    EXPECT_GT(empties, 0);
}

TEST(Integration, DeterministicScoreAcrossRuns) {
    const auto a = runComposition(dataFile("integration/composition.yaml"), tmpOut("det_a"));
    const auto b = runComposition(dataFile("integration/composition.yaml"), tmpOut("det_b"));
    ASSERT_EQ(a.runs.size(), 1u);
    ASSERT_EQ(b.runs.size(), 1u);
    EXPECT_DOUBLE_EQ(a.runs.front().mission_score, b.runs.front().mission_score);
}

TEST(Integration, ContinuesPastUnreadableMap) {
    // One good scenario and one whose map file does not exist.
    dm::types::SimulationConfigData good;
    good.map_filename = dataFile("integration/open_scene.npy");
    good.map_resolution = g::plen(10);
    good.initial_drone_position = pos(60, 60, 30);
    good.initial_angle = g::hang(0);

    dm::types::SimulationConfigData bad = good;
    bad.map_filename = dataFile("integration/nonexistent_map.npy");

    dm::types::MissionConfigData mission;
    mission.max_steps = 2000;
    mission.gps_resolution = g::plen(10);
    mission.output_mapping_resolution_factor = 1;
    mission.mission_bounds = bounds(0, 120, 0, 120, 20, 50);

    dm::types::SimulationCompositionData comp;
    comp.simulation_mission_groups.emplace_back(good, std::vector{mission});
    comp.simulation_mission_groups.emplace_back(bad, std::vector{mission});
    comp.drones.push_back(dm::types::DroneConfigData{g::plen(4), g::hang(90), g::plen(30), g::plen(20)});
    comp.lidars.push_back(dm::types::LidarConfigData{g::plen(20), g::plen(90), g::plen(2.5), 4});

    auto factory = std::make_unique<dm::SimulationRunFactoryImpl>();
    dm::SimulationManager manager{std::move(factory)};
    const auto report = manager.run(comp, tmpOut("continue"));

    ASSERT_EQ(report.runs.size(), 2u);
    EXPECT_GE(report.runs[0].mission_score, 0.0); // good scenario scored
    EXPECT_DOUBLE_EQ(report.runs[1].mission_score, -1.0); // bad scenario errored
}

TEST(Integration, MockAlgorithmDrivesFullStack) {
    // Wire the real components (maps, gps, movement, lidar, drone/mission
    // control) around a scripted mock algorithm.
    dm::types::SimulationConfigData sim;
    sim.map_filename = dataFile("integration/open_scene.npy");
    sim.map_resolution = g::plen(10);
    sim.initial_drone_position = pos(60, 60, 30);

    auto hidden_array = dm::loadHiddenMapArray(sim.map_filename);
    const auto hidden_cfg = dm::makeHiddenMapConfig(sim, *hidden_array);
    auto hidden = std::make_unique<dm::Map3DImpl>(hidden_array, hidden_cfg);

    dm::types::MissionConfigData mission;
    mission.max_steps = 50;
    mission.gps_resolution = g::plen(10);
    mission.output_mapping_resolution_factor = 1;
    mission.mission_bounds = bounds(0, 120, 0, 120, 20, 50);
    const auto out_cfg = dm::makeOutputMapConfig(mission);
    auto output = std::make_unique<dm::Map3DImpl>(
        dm::makeOccupancyGrid(out_cfg, dm::types::VoxelOccupancy::Unmapped), out_cfg);

    const dm::types::DroneConfigData drone{g::plen(4), g::hang(90), g::plen(30), g::plen(20)};
    const dm::types::LidarConfigData lidar{g::plen(20), g::plen(90), g::plen(2.5), 4};

    dm::MockGPS gps{sim.initial_drone_position, orient(0), g::plen(10)};
    dm::MockMovement movement{gps};
    dm::MockLidar mock_lidar{lidar, *hidden, gps};
    NiceMock<MockMappingAlgorithm> algo{mission, lidar, drone, *output};

    // Scan straight down (map the floor), then finish.
    dm::types::MappingStepCommand scan_down;
    scan_down.scan_orientation = orient(0, -90);
    scan_down.status = dm::types::AlgorithmStatus::Working;
    dm::types::MappingStepCommand finished;
    finished.status = dm::types::AlgorithmStatus::Finished;
    EXPECT_CALL(algo, nextStep(_, _))
        .WillOnce(Return(scan_down))
        .WillRepeatedly(Return(finished));

    dm::DroneControlImpl drone_control{drone, mission, mock_lidar, gps, movement, *output, algo};
    const auto out_file = tmpOut("mockalgo") / "map.npy";
    dm::MissionControlImpl mission_control{mission, drone, *hidden, *output, drone_control, out_file};

    const auto result = mission_control.runMission();
    EXPECT_EQ(result.status, dm::types::MissionRunStatus::Completed);
    // The single downward scan should have mapped voxels beneath the drone
    // (empty free space, an occupied hit, or an uncertain near-hit).
    EXPECT_GT(countValue(*output, dm::types::VoxelOccupancy::Occupied) +
                  countValue(*output, dm::types::VoxelOccupancy::Empty) +
                  countValue(*output, dm::types::VoxelOccupancy::PotentiallyOccupied),
              0);
    EXPECT_TRUE(std::filesystem::exists(out_file));
}

TEST(Integration, WallSceneRunsWithinBudget) {
    // Build a composition pointing at the wall scene and run it.
    dm::types::SimulationConfigData sim;
    sim.map_filename = dataFile("integration/wall_scene.npy");
    sim.map_resolution = g::plen(10);
    sim.initial_drone_position = pos(30, 60, 30);
    sim.initial_angle = g::hang(0);

    dm::types::MissionConfigData mission;
    mission.max_steps = 3000;
    mission.gps_resolution = g::plen(10);
    mission.output_mapping_resolution_factor = 1;
    mission.mission_bounds = bounds(0, 120, 0, 120, 20, 50);

    dm::types::SimulationCompositionData comp;
    comp.simulation_mission_groups.emplace_back(sim, std::vector{mission});
    comp.drones.push_back(dm::types::DroneConfigData{g::plen(4), g::hang(90), g::plen(30), g::plen(20)});
    comp.lidars.push_back(dm::types::LidarConfigData{g::plen(20), g::plen(90), g::plen(2.5), 4});

    auto factory = std::make_unique<dm::SimulationRunFactoryImpl>();
    dm::SimulationManager manager{std::move(factory)};
    const auto report = manager.run(comp, tmpOut("wall"));
    ASSERT_EQ(report.runs.size(), 1u);
    EXPECT_GE(report.runs.front().mission_score, 0.0);
}

TEST(Integration, ResolutionFactorReflectedInReport) {
    dm::types::SimulationConfigData sim;
    sim.map_filename = dataFile("integration/open_scene.npy");
    sim.map_resolution = g::plen(10);
    sim.initial_drone_position = pos(60, 60, 30);

    dm::types::MissionConfigData mission;
    mission.max_steps = 1500;
    mission.gps_resolution = g::plen(10);
    mission.output_mapping_resolution_factor = 2; // coarser output
    mission.mission_bounds = bounds(0, 120, 0, 120, 20, 50);

    dm::types::SimulationCompositionData comp;
    comp.simulation_mission_groups.emplace_back(sim, std::vector{mission});
    comp.drones.push_back(dm::types::DroneConfigData{g::plen(4), g::hang(90), g::plen(30), g::plen(20)});
    comp.lidars.push_back(dm::types::LidarConfigData{g::plen(20), g::plen(90), g::plen(2.5), 4});

    auto factory = std::make_unique<dm::SimulationRunFactoryImpl>();
    dm::SimulationManager manager{std::move(factory)};
    const auto report = manager.run(comp, tmpOut("resfactor"));
    ASSERT_EQ(report.runs.size(), 1u);
    EXPECT_EQ(report.runs.front().resolution_request_status,
              dm::types::ResolutionRequestStatus::Accepted);
    EXPECT_DOUBLE_EQ(g::lcm(report.runs.front().output_map_config.resolution), 20.0);
}

TEST(Integration, EmptyReportWhenNoScenarios) {
    auto factory = std::make_unique<dm::SimulationRunFactoryImpl>();
    dm::SimulationManager manager{std::move(factory)};
    const auto report = manager.run(dm::types::SimulationCompositionData{}, tmpOut("noscen"));
    EXPECT_TRUE(report.runs.empty());
    EXPECT_EQ(report.metric, "output_map_accuracy");
}
