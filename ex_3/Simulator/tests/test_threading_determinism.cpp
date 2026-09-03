// The sweep runs every (plugin x simulation x mission x drone x lidar)
// combination, optionally across worker threads. Threading must not change a
// single result: this test runs the identical work list serially and in
// parallel and requires the outcomes to match exactly.
//
// A race in the work list, in the pre-sized results vector, or in the
// output-map naming would surface here as a mismatch or a missing file.

#include <Simulator/ConfigLoader.h>
#include <Simulator/Sweep.h>

#include <Common/IMappingAlgorithm.h>
#include <Common/IMissionControl.h>
#include <Common/MissionControlFactory.h>

#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;
using namespace common;

// A deterministic stand-in for a real algorithm: it sweeps a fixed number of
// scans, then reports Finished. No randomness, no clock, no shared state - so
// any run-to-run difference must come from the simulator, which is the point.
class FakeAlgorithm final : public IMappingAlgorithm {
public:
    using IMappingAlgorithm::IMappingAlgorithm;

    [[nodiscard]] types::MappingStepCommand nextStep(
        const types::DroneState& state, const types::LidarScanResult*) override {
        types::MappingStepCommand command;
        if (state.step_index >= kScans) {
            command.status = types::AlgorithmStatus::Finished;
            return command;
        }
        // Scan in a fixed pattern around the start pose.
        command.scan_orientation = Orientation{
            static_cast<double>(state.step_index) * 60.0 * horizontal_angle[deg],
            0.0 * altitude_angle[deg]};
        return command;
    }

private:
    static constexpr std::size_t kScans = 6;
};

// Drives the drone through the algorithm and saves the output map, exactly as a
// real mission control does, but with no error handling to vary the outcome.
class FakeMissionControl final : public IMissionControl {
public:
    explicit FakeMissionControl(MissionControlDependencies dependencies)
        : deps_(dependencies), output_map_file_(dependencies.output_map_file) {}

    [[nodiscard]] types::MissionRunResult runMission() override {
        types::MissionRunResult result;
        result.status = types::MissionRunStatus::Completed;

        for (std::size_t step = 0; step < deps_.mission_config.max_steps; ++step) {
            const types::DroneState state{deps_.gps.position(), deps_.gps.heading(), step};
            const types::MappingStepCommand command =
                deps_.mapping_algorithm.nextStep(state, nullptr);
            result.steps = step + 1;
            if (command.status != types::AlgorithmStatus::Working) {
                break;
            }
            if (command.scan_orientation.has_value()) {
                const types::LidarScanResult scan = deps_.lidar.scan(*command.scan_orientation);
                for (const types::LidarHit& hit : scan) {
                    (void)hit; // the map write below is what matters for scoring
                }
                deps_.output_map.set(deps_.gps.position(), types::VoxelOccupancy::Empty);
            }
        }

        deps_.output_map.save(output_map_file_);
        return result;
    }

private:
    MissionControlDependencies deps_;
    fs::path output_map_file_;
};

[[nodiscard]] simulator::SweepPlugin makePlugin(const std::string& name) {
    simulator::SweepPlugin plugin;
    plugin.name = name;
    plugin.algorithm = [](MappingAlgorithmDependencies d) -> std::unique_ptr<IMappingAlgorithm> {
        return std::make_unique<FakeAlgorithm>(std::move(d));
    };
    plugin.mission_control = [](MissionControlDependencies d) -> std::unique_ptr<IMissionControl> {
        return std::make_unique<FakeMissionControl>(std::move(d));
    };
    return plugin;
}

// Two missions x two drones x two lidars = 8 runs per plugin, 16 in total, so
// the worker threads genuinely interleave.
[[nodiscard]] simulator::types::SimulationCompositionData makeComposition() {
    const fs::path inputs = EX3_INPUTS_DIR;

    simulator::types::SimulationConfigData sim =
        simulator::config::loadSimulationConfig(inputs / "simulation" / "small_simulation_room.yaml");
    sim.map_filename = inputs / "map" / "scenario_small.npy";

    std::vector<common::types::MissionConfigData> missions{
        simulator::config::loadMissionConfig(inputs / "mission" / "small_mission_room.yaml"),
        simulator::config::loadMissionConfig(inputs / "mission" / "small_mission_out.yaml"),
    };

    simulator::types::SimulationCompositionData composition;
    composition.composition_file = inputs / "sim_compose.yaml";
    composition.simulation_mission_groups.emplace_back(sim, missions);
    composition.drone_configs.push_back(
        simulator::config::loadDroneConfig(inputs / "drone" / "drone_small.yaml"));
    composition.drone_configs.push_back(
        simulator::config::loadDroneConfig(inputs / "drone" / "drone_large.yaml"));
    composition.lidar_configs.push_back(
        simulator::config::loadLidarConfig(inputs / "lidar" / "lidar_short.yaml"));
    composition.lidar_configs.push_back(
        simulator::config::loadLidarConfig(inputs / "lidar" / "lidar_long.yaml"));
    return composition;
}

[[nodiscard]] simulator::SweepResult runWith(int num_threads, const fs::path& dir) {
    fs::create_directories(dir);
    std::vector<simulator::SweepPlugin> plugins{makePlugin("PluginA.so"), makePlugin("PluginB.so")};
    simulator::Sweep sweep(simulator::SweepMode::Competition, std::move(plugins), makeComposition(),
                           dir, num_threads, /*verbose=*/false);
    return sweep.run();
}

} // namespace

TEST(ThreadingDeterminism, ParallelSweepMatchesSerialSweep) {
    const fs::path root = fs::temp_directory_path() / "ex3_determinism";
    fs::remove_all(root);

    const simulator::SweepResult serial = runWith(1, root / "serial");
    const simulator::SweepResult parallel = runWith(8, root / "parallel");

    ASSERT_TRUE(serial.errors.empty()) << "serial sweep reported plugin errors";
    ASSERT_TRUE(parallel.errors.empty()) << "parallel sweep reported plugin errors";
    ASSERT_EQ(serial.totals.size(), parallel.totals.size());
    ASSERT_EQ(serial.totals.size(), 2u) << "both plugins should have produced results";

    for (std::size_t i = 0; i < serial.totals.size(); ++i) {
        EXPECT_EQ(serial.totals[i].name, parallel.totals[i].name);
        EXPECT_DOUBLE_EQ(serial.totals[i].total_score, parallel.totals[i].total_score)
            << "score differs for " << serial.totals[i].name;
        EXPECT_EQ(serial.totals[i].total_steps, parallel.totals[i].total_steps)
            << "step count differs for " << serial.totals[i].name;
    }

    // Both plugins run identical work, so a correct sweep scores them equally.
    // If the output-map filenames collided across threads, one run would
    // overwrite another's map and the scores would diverge.
    EXPECT_DOUBLE_EQ(parallel.totals[0].total_score, parallel.totals[1].total_score);

    // 2 plugins x 1 simulation x 2 missions x 2 drones x 2 lidars = 16 maps.
    const auto countMaps = [](const fs::path& dir) {
        std::size_t n = 0;
        for (const auto& entry : fs::directory_iterator(dir)) {
            if (entry.path().extension() == ".npy") {
                ++n;
            }
        }
        return n;
    };
    EXPECT_EQ(countMaps(root / "serial"), 16u);
    EXPECT_EQ(countMaps(root / "parallel"), 16u)
        << "a filename collision would leave fewer maps than runs";

    fs::remove_all(root);
}
