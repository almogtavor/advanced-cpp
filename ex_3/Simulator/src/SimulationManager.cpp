#include <Simulator/SimulationManager.h>

#include <UserCommon/ErrorLog.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <ctime>
#include <exception>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

namespace simulator {

using namespace common;
using namespace user_common_323084962_212223036;

namespace {

[[nodiscard]] std::string utcNow() {
    using clock = std::chrono::system_clock;
    const std::time_t t = clock::to_time_t(clock::now());
    std::tm tm_utc{};
#if defined(_WIN32)
    gmtime_s(&tm_utc, &t);
#else
    gmtime_r(&t, &tm_utc);
#endif
    char buffer[32];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &tm_utc);
    return buffer;
}

// One unit of work: a fully identified single mission run.
struct Task {
    std::size_t group = 0; // index into simulation_mission_groups
    std::size_t mission = 0;
    std::size_t drone = 0;
    std::size_t lidar = 0;
};

} // namespace

SimulationManager::SimulationManager(RunFactoryMaker make_run_factory, int num_threads)
    : make_run_factory_(std::move(make_run_factory)), num_threads_(num_threads) {
    if (!make_run_factory_) {
        throw std::invalid_argument("SimulationManager requires a run factory maker.");
    }
}

types::SimulationManagerReport SimulationManager::run(
    const types::SimulationCompositionData& composition,
    const std::filesystem::path& output_path) {

    // Build the whole work list up front. Every combination is known in
    // advance, so the results vector is pre-sized and each task writes only to
    // its own index - no locking on results.
    std::vector<Task> tasks;
    for (std::size_t g = 0; g < composition.simulation_mission_groups.size(); ++g) {
        const auto& missions = std::get<1>(composition.simulation_mission_groups[g]);
        for (std::size_t m = 0; m < missions.size(); ++m) {
            for (std::size_t d = 0; d < composition.drone_configs.size(); ++d) {
                for (std::size_t l = 0; l < composition.lidar_configs.size(); ++l) {
                    tasks.push_back(Task{g, m, d, l});
                }
            }
        }
    }

    std::vector<types::SimulationResult> results(tasks.size());

    // A plugin call may throw; the simulator itself must survive it.
    const auto runTask = [&](std::size_t i) {
        const Task& t = tasks[i];
        const auto& [simulation, missions] = composition.simulation_mission_groups[t.group];
        const common::types::MissionConfigData& mission = missions[t.mission];
        const common::types::DroneConfigData& drone = composition.drone_configs[t.drone];
        const common::types::LidarConfigData& lidar = composition.lidar_configs[t.lidar];

        std::ostringstream label;
        label << "sim" << t.group << "_mission" << t.mission << "_drone" << t.drone << "_lidar"
              << t.lidar;

        try {
            const std::unique_ptr<ISimulationRunFactory> factory = make_run_factory_(label.str());
            const std::unique_ptr<ISimulationRun> run =
                factory->create(simulation, mission, drone, lidar, output_path);
            results[i] = run->run();
        } catch (const std::exception& ex) {
            globalErrorLog().log("SIMULATION_RUN_ERROR", ex.what());
            results[i].simulation_config = simulation;
            results[i].mission_config = mission;
            results[i].mission_score = -1.0;
            common::types::MissionRunResult mission_result;
            mission_result.status = common::types::MissionRunStatus::Error;
            mission_result.errors.push_back(
                common::types::ErrorRef{"SIMULATION_RUN_ERROR", ex.what()});
            results[i].mission_results.push_back(std::move(mission_result));
        } catch (...) {
            globalErrorLog().log("SIMULATION_RUN_ERROR", "unknown error");
            results[i].mission_score = -1.0;
            common::types::MissionRunResult mission_result;
            mission_result.status = common::types::MissionRunStatus::Error;
            results[i].mission_results.push_back(std::move(mission_result));
        }
    };

    // num_threads absent or 1 -> main thread only. >= 2 -> that many workers in
    // addition to main, so the total is never exactly 2. Never more workers
    // than there is work to do.
    const std::size_t requested = num_threads_ >= 2 ? static_cast<std::size_t>(num_threads_) : 0;
    const std::size_t workers = std::min(requested, tasks.size());

    if (workers == 0) {
        for (std::size_t i = 0; i < tasks.size(); ++i) {
            runTask(i);
        }
    } else {
        std::atomic<std::size_t> next{0};
        std::vector<std::thread> pool;
        pool.reserve(workers);
        for (std::size_t w = 0; w < workers; ++w) {
            pool.emplace_back([&] {
                for (std::size_t i = next++; i < tasks.size(); i = next++) {
                    runTask(i);
                }
            });
        }
        for (std::thread& t : pool) {
            t.join();
        }
    }

    return types::SimulationManagerReport{
        composition.composition_file,
        utcNow(),
        "output_map_accuracy",
        std::tuple<double, double>{0.0, 100.0},
        -1,
        std::move(results),
    };
}

} // namespace simulator
