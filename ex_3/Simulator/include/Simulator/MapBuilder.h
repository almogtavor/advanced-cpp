#pragma once

// Helpers that build in-memory voxel maps and their MapConfig geometry.
// Kept separate from Map3DImpl so both the run factory and the tests can
// construct fully-configured hidden and output maps the same way.

#include <TinyNPY.h>
#include <Simulator/SimulationTypes.h>

#include <Common/Types.h>

#include <filesystem>
#include <memory>

namespace simulator {

using namespace common;

struct ResolutionDecision {
    PhysicalLength resolution{};
    types::ResolutionRequestStatus status = types::ResolutionRequestStatus::Accepted;
};

// Turns the mission's requested output_mapping_resolution_factor into an
// actual output resolution plus a request status:
//   factor < 1            -> IgnoredTooSmall, fall back to gps_resolution
//   factor non-integer    -> Ignored,        fall back to gps_resolution
//   factor integer >= 1   -> Accepted,        gps_resolution * factor
[[nodiscard]] ResolutionDecision decideResolution(const common::types::MissionConfigData& mission);

// Output map geometry: mission bounds, offset at the bounds minimum corner,
// and the decided output resolution.
[[nodiscard]] common::types::MapConfig makeOutputMapConfig(const common::types::MissionConfigData& mission);

// Hidden map geometry: simulation offset/resolution, bounds spanning the
// loaded .npy extent.
[[nodiscard]] common::types::MapConfig makeHiddenMapConfig(const types::SimulationConfigData& sim,
                                                   const NpyArray& loaded);

// Loads a hidden map .npy, normalising every voxel to Empty(0)/Occupied(1).
// Throws std::runtime_error if the file cannot be read.
[[nodiscard]] std::shared_ptr<NpyArray> loadHiddenMapArray(const std::filesystem::path& path);

// Allocates an int8 occupancy grid sized to the config, filled with `fill`.
[[nodiscard]] std::shared_ptr<NpyArray> makeOccupancyGrid(const common::types::MapConfig& config,
                                                          common::types::VoxelOccupancy fill);

} // namespace simulator
