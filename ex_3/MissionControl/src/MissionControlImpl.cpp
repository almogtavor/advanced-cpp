#include <MissionControl/MissionControlImpl.h>

#include <Common/MissionControlRegistration.h>
#include <UserCommon/ErrorCodes.h>
#include <UserCommon/ErrorLog.h>
#include <UserCommon/MapGeometry.h>

#include <utility>

namespace mission_control_323084962_212223036 {

using namespace common;
using namespace user_common_323084962_212223036;

namespace {

// True when the mission bounds are degenerate (empty or inverted) on any axis.
[[nodiscard]] bool boundsInvalid(const types::MappingBounds& b) {
    return geom::xcm(b.min_x) >= geom::xcm(b.max_x) ||
           geom::ycm(b.min_y) >= geom::ycm(b.max_y) ||
           geom::zcm(b.min_height) >= geom::zcm(b.max_height);
}

} // namespace

MissionControlImpl_323084962_212223036::MissionControlImpl_323084962_212223036(
    MissionControlDependencies dependencies)
    : mission_(dependencies.mission_config),
      output_map_(dependencies.output_map),
      output_map_file_(std::move(dependencies.output_map_file)),
      verbose_(dependencies.verbose),
      drone_control_(dependencies.drone_config,
                     dependencies.mission_config,
                     dependencies.lidar,
                     dependencies.gps,
                     dependencies.movement,
                     dependencies.output_map,
                     dependencies.mapping_algorithm) {}

types::MissionRunResult MissionControlImpl_323084962_212223036::runMission() {
    types::MissionRunResult result;

    if (boundsInvalid(mission_.mission_bounds)) {
        const types::ErrorRef err{kMissionBoundaryInvalidCode,
                                  "Mission boundaries are empty or inverted."};
        globalErrorLog().log(err.code, err.message);
        result.status = types::MissionRunStatus::Error;
        result.errors.push_back(err);
        return result;
    }

    result.status = types::MissionRunStatus::MaxSteps;
    for (std::size_t step = 0; step < mission_.max_steps; ++step) {
        const types::DroneStepResult step_result = drone_control_.step();
        result.steps = step + 1;

        if (step_result.status == types::DroneStepStatus::Completed) {
            result.status = types::MissionRunStatus::Completed;
            break;
        }
        if (step_result.status == types::DroneStepStatus::Error) {
            // A refused movement carrying the shared collision message means
            // the drone drove its centre into an obstacle; anything else is a
            // generic step failure.
            const bool collision = step_result.message == kDroneHitsObstacleMessage;
            const types::ErrorRef err{
                collision ? kDroneHitsObstacleCode : kDroneStepErrorCode,
                step_result.message};
            globalErrorLog().log(err.code, err.message);
            result.status = types::MissionRunStatus::Error;
            result.errors.push_back(err);
            break;
        }
    }

    try {
        output_map_.save(output_map_file_);
    } catch (const std::exception& ex) {
        const types::ErrorRef err{kOutputMapSaveErrorCode, ex.what()};
        globalErrorLog().log(err.code, err.message);
        result.status = types::MissionRunStatus::Error;
        result.errors.push_back(err);
    }

    if (verbose_) {
        globalErrorLog().log("MISSION_VERBOSE",
                             "Mission finished after " + std::to_string(result.steps) +
                                 " steps; output map written to " + output_map_file_.string());
    }

    return result;
}

} // namespace mission_control_323084962_212223036

// Loaded when dlopen() is called on this plugin: constructing the global
// registration object hands the mission-control factory to the simulator's
// registrar.
using mission_control_323084962_212223036::MissionControlImpl_323084962_212223036;
REGISTER_MISSION_CONTROL(MissionControlImpl_323084962_212223036);
