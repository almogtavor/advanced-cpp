// Included by test_main.cpp
#include "drone/Drone.h"

using namespace drone;
using namespace units;

namespace {
MissionConfig small_mission() {
    MissionConfig mission;
    mission.min_x = 0 * cm; mission.max_x = 100 * cm;
    mission.min_y = 0 * cm; mission.max_y = 100 * cm;
    mission.height_min = 0  * cm;
    mission.height_max = 10 * cm;
    return mission;
}
} // namespace

TEST(drone_initial_command_is_get_location) {
    BuildingMap m(small_mission(), 10 * cm, Position{}, 10, 10, 1);
    DroneConfig cfg;
    Drone d(m, cfg);
    auto cmd = d.next_command();
    CHECK(cmd.kind == DroneCommand::Kind::GetLocation);
}

TEST(drone_after_location_emits_scan_commands) {
    BuildingMap m(small_mission(), 10 * cm, Position{}, 10, 10, 1);
    DroneConfig cfg;
    Drone d(m, cfg);
    (void)d.next_command(); // GetLocation
    d.on_location(Position{15 * cm, 15 * cm, 5 * cm}, 0 * deg);
    auto cmd = d.next_command();
    CHECK(cmd.kind == DroneCommand::Kind::Scan);
}

TEST(drone_respects_clearance_for_sphere_body) {
    // With default DroneConfig (min_passage=30/30/50, cell_size=10cm) the
    // drone has a 15 cm radius -> 1-cell Chebyshev clearance. A cell that
    // is empty itself but has an occupied neighbor inside that buffer
    // must not be picked as a BFS waypoint.
    //
    // We verify this indirectly: paint a tight corridor where the only
    // empty cells touch occupied neighbors, then check the drone reports
    // Finished without attempting any Advance/Elevate, since no safe
    // frontier path exists from the start cell.
    BuildingMap m(small_mission(), 10 * cm, Position{}, 10, 10, 1);
    DroneConfig cfg; // defaults: clearance == 1
    Drone d(m, cfg);

    // Manually populate the known map: only the 3x3 patch around the
    // start cell is empty; everything else is marked occupied. With
    // clearance==1, no other cell has a full clear neighborhood, so the
    // BFS cannot expand from the start cell.
    for (int x = 0; x < 10; ++x) {
        for (int y = 0; y < 10; ++y) {
            const Cell c{x, y, 0};
            if (x >= 1 && x <= 3 && y >= 1 && y <= 3) {
                m.set_cell(c, voxel::kEmpty);
            } else {
                m.set_cell(c, voxel::kOccupied);
            }
        }
    }

    (void)d.next_command(); // GetLocation
    // Place the drone at (2,2,0), the only cell with a fully-empty
    // Chebyshev-1 neighborhood in the patch above.
    d.on_location(Position{25 * cm, 25 * cm, 5 * cm}, 0 * deg);

    // Walk through scans + planning. Cap iterations defensively.
    for (int i = 0; i < 200; ++i) {
        auto cmd = d.next_command();
        if (cmd.kind == DroneCommand::Kind::Finished) {
            CHECK(true);
            return;
        }
        if (cmd.kind == DroneCommand::Kind::Scan) {
            LidarFrame f;
            f.yaw_offset = cmd.scan_xy;
            f.pitch_offset = cmd.scan_pitch;
            d.on_scan(f);
        }
        // We should never see Advance/Elevate here: every reachable
        // frontier is too close to an occupied cell.
        CHECK(cmd.kind != DroneCommand::Kind::Advance);
        CHECK(cmd.kind != DroneCommand::Kind::Elevate);
    }
    CHECK(false); // safety: should have finished long before this point
}
