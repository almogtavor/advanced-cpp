#pragma once

// Error codes and messages shared between the MissionControl and the
// Simulator. The Simulator's mock movement refuses a step that would drive the
// drone's centre into an obstacle; MissionControl recognises that refusal by
// this message so the run is reported under the specific code rather than the
// generic step-error one.

namespace user_common_323084962_212223036 {

inline constexpr const char* kDroneHitsObstacleCode = "DRONE_HITS_OBSTACLE";
inline constexpr const char* kDroneHitsObstacleMessage = "Drone entered an occupied voxel.";
inline constexpr const char* kDroneStepErrorCode = "DRONE_STEP_ERROR";
inline constexpr const char* kMissionBoundaryInvalidCode = "MISSION_BOUNDARY_INVALID";
inline constexpr const char* kOutputMapSaveErrorCode = "OUTPUT_MAP_SAVE_ERROR";

} // namespace user_common_323084962_212223036
