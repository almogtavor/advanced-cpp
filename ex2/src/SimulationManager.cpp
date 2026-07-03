#include <drone_mapper/SimulationManager.h>

#include <drone_mapper/ErrorLog.h>
#include <drone_mapper/MapBuilder.h>
#include <drone_mapper/ReportWriter.h>

#include <chrono>
#include <ctime>
#include <exception>
#include <stdexcept>
#include <utility>

namespace drone_mapper {

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

// Builds a fully-errored result for a scenario that could not run at all
// (e.g. its map file failed to load), so the report still lists it with -1.
[[nodiscard]] types::SimulationResult makeErrorResult(const types::SimulationConfigData& sim,
                                                      const types::MissionConfigData& mission,
                                                      const std::string& code,
                                                      const std::string& message) {
    types::SimulationResult result;
    result.simulation_config = sim;
    result.mission_config = mission;
    result.resolution_request_status = decideResolution(mission).status;
    result.mission_results.push_back(types::MissionRunResult{
        types::MissionRunStatus::Error, 0, {types::ErrorRef{code, message}}});
    result.mission_score = -1.0;
    return result;
}

} // namespace

SimulationManager::SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory)
    : run_factory_(std::move(run_factory)) {
    if (!run_factory_) {
        throw std::invalid_argument("SimulationManager requires a run factory.");
    }
}

types::SimulationManagerReport SimulationManager::run(const types::SimulationCompositionData& composition,
                                                      const std::filesystem::path& output_path) {
    const std::filesystem::path results_dir = output_path / "output_results";
    globalErrorLog().open(results_dir / "error_log.txt");

    std::vector<types::SimulationResult> runs;
    for (const auto& [simulation, missions] : composition.simulation_mission_groups) {
        for (const types::MissionConfigData& mission : missions) {
            for (const types::DroneConfigData& drone : composition.drones) {
                for (const types::LidarConfigData& lidar : composition.lidars) {
                    try {
                        std::unique_ptr<ISimulationRun> run =
                            run_factory_->create(simulation, mission, drone, lidar, output_path);
                        runs.push_back(run->run());
                    } catch (const std::exception& ex) {
                        globalErrorLog().log("SIMULATION_RUN_ERROR", ex.what());
                        runs.push_back(makeErrorResult(simulation, mission, "SIMULATION_RUN_ERROR", ex.what()));
                    }
                }
            }
        }
    }

    types::SimulationManagerReport report{
        utcNow(),
        "output_map_accuracy",
        std::tuple<double, double>{0.0, 100.0},
        -1,
        std::move(runs),
    };

    try {
        ReportWriter::write(report, output_path / "simulation_output.yaml");
    } catch (const std::exception& ex) {
        globalErrorLog().log("REPORT_WRITE_ERROR", ex.what());
    }

    return report;
}

} // namespace drone_mapper
