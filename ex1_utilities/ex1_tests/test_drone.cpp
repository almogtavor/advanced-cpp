// Included by test_main.cpp
#include "drone/Drone.h"

using namespace drone;
using namespace units;

TEST(drone_initial_command_is_get_location) {
    MissionConfig mission;
    mission.boundary_polygon = {
        {0 * cm,   0 * cm},
        {100 * cm, 0 * cm},
        {100 * cm, 100 * cm},
        {0 * cm,   100 * cm},
    };
    mission.height_min = 0  * cm;
    mission.height_max = 10 * cm;
    BuildingMap m(mission, 10 * cm, Position{}, 10, 10, 1);

    DroneConfig cfg;
    Drone d(m, cfg);
    auto cmd = d.next_command();
    CHECK(cmd.kind == DroneCommand::Kind::GetLocation);
}

TEST(drone_after_location_emits_scan_commands) {
    MissionConfig mission;
    mission.boundary_polygon = {
        {0 * cm,   0 * cm},
        {100 * cm, 0 * cm},
        {100 * cm, 100 * cm},
        {0 * cm,   100 * cm},
    };
    mission.height_min = 0  * cm;
    mission.height_max = 10 * cm;
    BuildingMap m(mission, 10 * cm, Position{}, 10, 10, 1);

    DroneConfig cfg;
    Drone d(m, cfg);
    (void)d.next_command(); // GetLocation
    d.on_location(Position{15 * cm, 15 * cm, 5 * cm}, 0 * deg);
    auto cmd = d.next_command();
    CHECK(cmd.kind == DroneCommand::Kind::Scan);
}
