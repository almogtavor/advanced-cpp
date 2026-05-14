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
