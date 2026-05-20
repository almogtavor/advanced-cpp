#pragma once

#include <deque>
#include <optional>
#include <vector>

#include "config/DroneConfig.h"
#include "driver/IMovementDriver.h"
#include "sensors/ILidarSensor.h"
#include "sensors/LidarFrame.h"
#include "world/BuildingMap.h"

namespace drone {

// Command produced by the drone for the simulator to execute. The drone
// only produces these descriptions; it does not call the sensors/driver
// directly. This matches the assignment's "main loop" pattern where the
// simulator asks the drone for a command and then dispatches it.
struct DroneCommand {
    enum class Kind {
        GetLocation,
        Scan,
        Rotate,
        Advance,
        Elevate,
        Finished,
    };

    Kind kind{Kind::Finished};
    units::Angle  angle{};
    units::Length length{};
    units::Angle  scan_xy{};
    units::Angle  scan_pitch{};
    RotateDirection rot_dir{RotateDirection::Right};
};

// The drone's autonomous mapping algorithm. The drone owns the
// BuildingMap representing its accumulated knowledge. It is given a
// reference to the drone capabilities and uses the simulator-provided
// sensor results to update its state.
//
// Algorithm: a frontier-based BFS exploration on a voxel grid that
// matches the building's cell size. At each waypoint the drone takes
// six cardinal scans (forward/back/left/right/up/down), updates the
// known map, then plans a path through known-empty cells to the
// nearest cell adjacent to an unknown voxel.
class Drone {
public:
    Drone(BuildingMap& map, const DroneConfig& cfg);

    // Produces the next command for the simulator to execute.
    DroneCommand next_command();

    // Result-feeding entry points called by the simulator after each
    // command is dispatched.
    void on_location(Position p, units::Angle yaw);
    void on_scan(const LidarFrame& frame);
    void on_move_result(MoveResult result);

    // Read-only access for tests.
    const BuildingMap& map() const { return map_; }
    bool finished() const { return state_ == State::Done; }

private:
    enum class State {
        NeedLocation,        // need to query position before doing anything else
        Scanning,            // working through the 6 cardinal scans
        Planning,            // computing the next frontier and path
        Moving,              // executing a queued sequence of move commands
        Done,
    };

    enum class ScanDir {
        PlusX = 0, PlusY, MinusX, MinusY, PlusZ, MinusZ, Count
    };

    void start_scanning_phase();
    void plan_next_target();
    void enqueue_move_to_neighbor(Cell from, Cell to);

    DroneCommand build_scan_command();
    DroneCommand build_next_move_command();

    void apply_full_scan_to_map(const LidarFrame& frame);
    std::vector<Cell> bfs_to_frontier(Cell start);

    Cell current_cell() const;
    units::Angle direction_to_yaw(ScanDir dir) const;
    units::Angle scan_dir_yaw_offset(ScanDir dir) const;
    units::Angle scan_dir_pitch_offset(ScanDir dir) const;

    BuildingMap& map_;
    const DroneConfig& cfg_;

    State state_{State::NeedLocation};

    Position last_position_{};
    units::Angle yaw_known_{0 * units::deg};

    int scan_index_{0};                        // 0..5 over ScanDir
    std::optional<LidarFrame> last_scan_{};

    // The path through known-empty cells the drone is currently following.
    std::deque<Cell> path_{};
    std::deque<DroneCommand> pending_moves_{};

    // Chebyshev-distance buffer (in voxels) around each waypoint that
    // must be free of occupied cells for the spherical body to fit.
    // Derived from min(min_passage_*) / 2 and the map cell size in the
    // constructor. 0 means the drone is smaller than one cell.
    int clearance_cells_{0};
};

} // namespace drone
