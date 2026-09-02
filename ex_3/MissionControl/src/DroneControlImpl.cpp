#include <MissionControl/DroneControlImpl.h>

#include <UserCommon/MapGeometry.h>
#include <UserCommon/ScanResultToVoxels.h>

#include <cmath>
#include <utility>

namespace mission_control_323084962_212223036 {

using namespace common;
using namespace user_common_323084962_212223036;

namespace {
constexpr double kLimitEps = 1e-6;
} // namespace

DroneControlImpl::DroneControlImpl(types::DroneConfigData drone,
                                   types::MissionConfigData mission,
                                   ILidar& lidar,
                                   IGPS& gps,
                                   IDroneMovement& movement,
                                   IMutableMap3D& output_map,
                                   IMappingAlgorithm& mapping_algorithm)
    : drone_(std::move(drone)),
      mission_(std::move(mission)),
      lidar_(lidar),
      gps_(gps),
      movement_(movement),
      output_map_(output_map),
      mapping_algorithm_(mapping_algorithm) {}

types::DroneStepResult DroneControlImpl::step() {
    (void)mission_; // Step budgeting lives in MissionControl; kept for parity.

    const types::DroneState current{gps_.position(), gps_.heading(), step_index_};
    const types::LidarScanResult* scan_ptr = has_scan_ ? &last_scan_ : nullptr;
    const types::MappingStepCommand command = mapping_algorithm_.nextStep(current, scan_ptr);

    if (command.status == types::AlgorithmStatus::Finished ||
        command.status == types::AlgorithmStatus::FinishedWithUnmappableVoxels) {
        return types::DroneStepResult{types::DroneStepStatus::Completed, "Mapping finished."};
    }

    // Execute the movement first (if any), enforcing the drone's limits.
    if (command.movement.has_value()) {
        const types::MovementCommand& m = *command.movement;
        types::MovementResult result{true, {}};
        switch (m.type) {
        case types::MovementCommandType::Rotate:
            if (geom::hdeg(m.angle) > geom::hdeg(drone_.max_rotate) + kLimitEps) {
                return types::DroneStepResult{types::DroneStepStatus::Error,
                                              "Rotation exceeds drone max_rotate."};
            }
            result = movement_.rotate(m.rotation, m.angle);
            break;
        case types::MovementCommandType::Advance:
            if (geom::lcm(m.distance) > geom::lcm(drone_.max_advance) + kLimitEps) {
                return types::DroneStepResult{types::DroneStepStatus::Error,
                                              "Advance exceeds drone max_advance."};
            }
            result = movement_.advance(m.distance);
            break;
        case types::MovementCommandType::Elevate:
            if (std::abs(geom::lcm(m.distance)) > geom::lcm(drone_.max_elevate) + kLimitEps) {
                return types::DroneStepResult{types::DroneStepStatus::Error,
                                              "Elevation exceeds drone max_elevate."};
            }
            result = movement_.elevate(m.distance);
            break;
        case types::MovementCommandType::Hover:
            break;
        }
        if (!result) {
            return types::DroneStepResult{types::DroneStepStatus::Error, result.message};
        }
    }

    // Then scan (from the updated pose) and fold the scan into the output map.
    // scan_orientation is optional, so the mapping algorithm can choose to skip LiDAR scanning on some steps.
    if (command.scan_orientation.has_value()) {
        last_scan_ = lidar_.scan(*command.scan_orientation);
        has_scan_ = true;
        ScanResultToVoxels::applyToMap(output_map_, gps_.position(), gps_.heading(),
                                       last_scan_, lidar_.config());
    }

    ++step_index_;
    return types::DroneStepResult{types::DroneStepStatus::Continue, {}};
}

types::DroneState DroneControlImpl::state() const {
    return types::DroneState{gps_.position(), gps_.heading(), step_index_};
}

} // namespace mission_control_323084962_212223036
