#pragma once

#include <Common/IMissionControl.h>
#include <Common/MissionControlFactory.h>
#include <MissionControl/DroneControlImpl.h>
#include <UserCommon/ErrorLog.h>

#include <filesystem>

namespace mission_control_323084962_212223036 {

using namespace common;

// Runs a single drone mission.
//
// Changed from assignment 2: the mission is now built from
// MissionControlDependencies bundle& creates its own DroneControlImpl from
// the sensors it is handed, instead of receiving a ready-made IDroneControl.
// It also no longer sees the hidden truth map so collisions are detected on the
// simulator side and surface as a failed movement (see UserCommon/ErrorCodes.h).
class MissionControlImpl_323084962_212223036 final : public IMissionControl {
public:
    explicit MissionControlImpl_323084962_212223036(MissionControlDependencies dependencies);

    [[nodiscard]] types::MissionRunResult runMission() override;

private:
    types::MissionConfigData mission_;
    IMutableMap3D& output_map_;
    std::filesystem::path output_map_file_;
    bool verbose_ = false;
    // Per-mission log, so concurrent missions never share a stream. With
    // -verbose it is opened next to the output map; otherwise it only counts
    // (errors also travel back in MissionRunResult, which the reports render).
    user_common_323084962_212223036::ErrorLog log_;
    DroneControlImpl drone_control_;
};

} // namespace mission_control_323084962_212223036
