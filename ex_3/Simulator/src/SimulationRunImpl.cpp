#include <Simulator/SimulationRunImpl.h>
#include <Simulator/SimulationTypes.h>

#include <Simulator/MapBuilder.h>
#include <Simulator/MapsComparison.h>

#include <stdexcept>
#include <utility>
#include <vector>

namespace simulator {

using namespace common;

SimulationRunImpl::SimulationRunImpl(std::unique_ptr<const IMap3D> hidden_map,
                                     std::unique_ptr<IMutableMap3D> output_map,
                                     std::unique_ptr<IGPS> gps,
                                     std::unique_ptr<IDroneMovement> movement,
                                     std::unique_ptr<ILidar> lidar,
                                     std::unique_ptr<IMappingAlgorithm> mapping_algorithm,
                                     std::unique_ptr<IMissionControl> mission_control,
                                     types::SimulationConfigData simulation_config,
                                     common::types::MissionConfigData mission_config,
                                     std::filesystem::path output_map_file)
    : hidden_map_(std::move(hidden_map)),
      output_map_(std::move(output_map)),
      gps_(std::move(gps)),
      movement_(std::move(movement)),
      lidar_(std::move(lidar)),
      mapping_algorithm_(std::move(mapping_algorithm)),
      mission_control_(std::move(mission_control)),
      simulation_config_(std::move(simulation_config)),
      mission_config_(std::move(mission_config)),
      output_map_file_(std::move(output_map_file)) {
    if (!hidden_map_ ||
        !output_map_ ||
        !gps_ ||
        !movement_ ||
        !lidar_ ||
        !mapping_algorithm_ ||
        !mission_control_) {
        throw std::invalid_argument("SimulationRunImpl requires injected dependencies.");
    }
}

types::SimulationResult SimulationRunImpl::run() {
    const common::types::MissionRunResult mission_result = mission_control_->runMission();

    types::SimulationResult result;
    result.simulation_config = simulation_config_;
    result.mission_config = mission_config_;
    result.mission_results.push_back(mission_result);
    result.output_map_file = output_map_file_;
    result.output_map_config = output_map_->getMapConfig();

    // Resolution request status. Start from the factor-based decision, then
    // downgrade to Ignored if the actual output resolution ended up coarser
    // than requested (e.g. the run clamped it up to the map resolution because
    // it does not support mapping finer than the source map). This keeps the
    // reported status consistent with the actual output resolution.
    const ResolutionDecision decision = decideResolution(mission_config_);
    result.resolution_request_status = decision.status;
    if (result.output_map_config.resolution > decision.resolution &&
        decision.status == types::ResolutionRequestStatus::Accepted) {
        result.resolution_request_status = types::ResolutionRequestStatus::Ignored;
    }

    if (mission_result.status == common::types::MissionRunStatus::Error) {
        // A failed run is scored -1 (the report's error score).
        result.mission_score = -1.0;
    } else {
        const std::vector<double> scores =
            MapsComparison::compare(*hidden_map_, std::vector<IMap3D*>{output_map_.get()});
        result.mission_score = scores.empty() ? -1.0 : scores.front();
    }
    return result;
}

} // namespace simulator
