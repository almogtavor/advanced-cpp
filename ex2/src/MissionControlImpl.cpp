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
    if (hidden_map_.atVoxel(pos) == types::VoxelOccupancy::Occupied) {
        return true;
    }
    // Treat the drone as a sphere: probe the surface along each axis so the
    // body cannot clip a wall even when its centre is still in free space.
    const double r = geom::lcm(drone_.radius);
    if (r <= 0.0) {
        return false;
    }
    const Position3D probes[6] = {
        Position3D{geom::xlen(geom::xcm(pos.x) + r), pos.y, pos.z},
        Position3D{geom::xlen(geom::xcm(pos.x) - r), pos.y, pos.z},
        Position3D{pos.x, geom::ylen(geom::ycm(pos.y) + r), pos.z},
        Position3D{pos.x, geom::ylen(geom::ycm(pos.y) - r), pos.z},
        Position3D{pos.x, pos.y, geom::zlen(geom::zcm(pos.z) + r)},
        Position3D{pos.x, pos.y, geom::zlen(geom::zcm(pos.z) - r)},
    };
    for (const Position3D& p : probes) {
        if (hidden_map_.atVoxel(p) == types::VoxelOccupancy::Occupied) {
            return true;
        }
    }
    return false;
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
