// Included by test_main.cpp
#include <cstdio>
#include <fstream>
#include <unistd.h>

#include "io/ConfigParser.h"

using namespace drone;

namespace {
std::string write_temp(const std::string& contents) {
    static int counter = 0;
    const std::string path = "/tmp/drone_mapper_test_" +
                             std::to_string(::getpid()) + "_" +
                             std::to_string(counter++) + ".txt";
    std::ofstream out(path);
    out << contents;
    return path;
}
} // namespace

TEST(config_parser_drone_basic) {
    const std::string p = write_temp(R"(
# drone capabilities
min_passage_width 50
min_passage_length 60
min_passage_height 100
lidar_z_min 10
lidar_z_max 400
lidar_d 2.5
lidar_fovc 5
max_rotate_per_cmd 90
max_advance_per_cmd 50
max_elevate_per_cmd 50
)");
    DroneConfig cfg;
    auto r = ConfigParser::load_drone_config(p, cfg);
    CHECK(r.ok);
    CHECK_NEAR(cfg.min_passage_width.numerical_value_in(units::cm), 50.0, 1e-9);
    CHECK_NEAR(cfg.lidar_z_min.numerical_value_in(units::cm), 10.0, 1e-9);
    CHECK_NEAR(cfg.lidar_z_max.numerical_value_in(units::cm), 400.0, 1e-9);
    CHECK_NEAR(cfg.lidar_d.numerical_value_in(units::cm), 2.5, 1e-9);
    CHECK_EQ(cfg.lidar_fovc, 5);
    CHECK_NEAR(cfg.max_advance_per_cmd.numerical_value_in(units::cm), 50.0, 1e-9);
    std::remove(p.c_str());
}

TEST(config_parser_drone_unknown_key_recovers) {
    const std::string p = write_temp(R"(
lidar_z_min 5
SOMETHING_BAD 42
lidar_z_max 200
)");
    DroneConfig cfg;
    auto r = ConfigParser::load_drone_config(p, cfg);
    CHECK(r.ok);
    CHECK(!r.errors.empty());      // an error was recorded
    CHECK_NEAR(cfg.lidar_z_min.numerical_value_in(units::cm), 5.0, 1e-9);
    CHECK_NEAR(cfg.lidar_z_max.numerical_value_in(units::cm), 200.0, 1e-9);
    std::remove(p.c_str());
}

TEST(config_parser_mission_rectangle) {
    const std::string p = write_temp(R"(
start 50 50 10
min_x 0
max_x 100
min_y 0
max_y 100
height_min 0
height_max 100
xy_resolution 1
height_resolution 1
)");
    MissionConfig m;
    auto r = ConfigParser::load_mission_config(p, m);
    CHECK(r.ok);
    CHECK_NEAR(m.min_x.numerical_value_in(units::cm), 0.0, 1e-9);
    CHECK_NEAR(m.max_x.numerical_value_in(units::cm), 100.0, 1e-9);
    CHECK_NEAR(m.start.x.numerical_value_in(units::cm), 50.0, 1e-9);
    CHECK_NEAR(m.height_max.numerical_value_in(units::cm), 100.0, 1e-9);
    CHECK_EQ(m.xy_resolution_decimals, 1);
    CHECK_EQ(m.height_resolution_decimals, 1);
    // Default yaw when not specified.
    CHECK_NEAR(m.start_yaw.numerical_value_in(units::deg), 0.0, 1e-9);
    std::remove(p.c_str());
}

TEST(config_parser_mission_start_with_angle) {
    const std::string p = write_temp(R"(
start 25 25 5 90
min_x 0
max_x 100
min_y 0
max_y 100
height_min 0
height_max 100
xy_resolution 1
height_resolution 1
)");
    MissionConfig m;
    auto r = ConfigParser::load_mission_config(p, m);
    CHECK(r.ok);
    CHECK_NEAR(m.start.x.numerical_value_in(units::cm), 25.0, 1e-9);
    CHECK_NEAR(m.start_yaw.numerical_value_in(units::deg), 90.0, 1e-9);
    std::remove(p.c_str());
}

TEST(config_parser_missing_file_returns_not_ok) {
    DroneConfig cfg;
    auto r = ConfigParser::load_drone_config("/tmp/__definitely_missing__.txt", cfg);
    CHECK(!r.ok);
    CHECK(!r.errors.empty());
}
