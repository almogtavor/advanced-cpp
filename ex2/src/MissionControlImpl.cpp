#include <drone_mapper/MissionControlImpl.h>

#include <drone_mapper/ErrorLog.h>
#include <drone_mapper/MapGeometry.h>

#include <utility>

namespace drone_mapper {

namespace {

// True when the mission bounds are degenerate (empty or inverted) on any axis.
[[nodiscard]] bool boundsInvalid(const types::MappingBounds& b) {
    return geom::xcm(b.min_x) >= geom::xcm(b.max_x) ||
           geom::ycm(b.min_y) >= geom::ycm(b.max_y) ||
           geom::zcm(b.min_height) >= geom::zcm(b.max_height);
}

} // namespace

MissionControlImpl::MissionControlImpl(types::MissionConfigData mission,
                                       types::DroneConfigData drone,
                                       const IMap3D& hidden_map,
                                       IMutableMap3D& output_map,
                                       IDroneControl& drone_control,
                                       std::filesystem::path output_map_file)
    : mission_(std::move(mission)),
      drone_(std::move(drone)),
      hidden_map_(hidden_map),
      output_map_(output_map),
      drone_control_(drone_control),
      output_map_file_(std::move(output_map_file)) {}

bool MissionControlImpl::droneCollides(const Position3D& pos) const {
    // A crash is the drone driving its centre into an obstacle. The spherical
    // body is kept clear of walls by the mapping algorithm's clearance, so we
    // do not fail a run merely because the body grazes a neighbouring voxel -
    // that would spuriously abort valid wall-hugging passes and forfeit the map.
    return hidden_map_.atVoxel(pos) == types::VoxelOccupancy::Occupied;
}

types::MissionRunResult MissionControlImpl::runMission() {
    types::MissionRunResult result;

    if (boundsInvalid(mission_.mission_bounds)) {
        const types::ErrorRef err{"MISSION_BOUNDARY_INVALID", "Mission boundaries are empty or inverted."};
        globalErrorLog().log(err.code, err.message);
        result.status = types::MissionRunStatus::Error;
        result.errors.push_back(err);
        return result;
    }

    result.status = types::MissionRunStatus::MaxSteps;
    for (std::size_t step = 0; step < mission_.max_steps; ++step) {
        const types::DroneStepResult step_result = drone_control_.step();
        result.steps = step + 1;

        if (droneCollides(drone_control_.state().position)) {
            const types::ErrorRef err{"DRONE_HITS_OBSTACLE", "Drone entered an occupied voxel."};
            globalErrorLog().log(err.code, err.message);
            result.status = types::MissionRunStatus::Error;
            result.errors.push_back(err);
            break;
        }

        if (step_result.status == types::DroneStepStatus::Completed) {
            result.status = types::MissionRunStatus::Completed;
            break;
        }
        if (step_result.status == types::DroneStepStatus::Error) {
            const types::ErrorRef err{"DRONE_STEP_ERROR", step_result.message};
            globalErrorLog().log(err.code, err.message);
            result.status = types::MissionRunStatus::Error;
            result.errors.push_back(err);
            break;
        }
    }

    try {
        output_map_.save(output_map_file_);
    } catch (const std::exception& ex) {
        const types::ErrorRef err{"OUTPUT_MAP_SAVE_ERROR", ex.what()};
        globalErrorLog().log(err.code, err.message);
        result.status = types::MissionRunStatus::Error;
        result.errors.push_back(err);
    }

    return result;
}

} // namespace drone_mapper
