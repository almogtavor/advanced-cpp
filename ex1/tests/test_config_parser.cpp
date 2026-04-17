// Included by test_main.cpp
#include <cstdio>
#include <fstream>
#include <unistd.h>

#include "io/ConfigParser.h"

using namespace drone;

namespace {
std::string write_temp(const std::string& contents) {
    // Use a stable name in the system temp directory.
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
lidar_fov 90
lidar_min_range 10
lidar_max_range 400
lidar_res_dist_a 100
lidar_res_side_a 5
lidar_res_dist_b 400
lidar_res_side_b 20
max_rotate_per_cmd 90
max_advance_per_cmd 50
max_elevate_per_cmd 50
)");
    DroneConfig cfg;
    auto r = ConfigParser::load_drone_config(p, cfg);
    CHECK(r.ok);
    CHECK_NEAR(cfg.min_passage_width.numerical_value_in(units::cm), 50.0, 1e-9);
    CHECK_NEAR(cfg.lidar_fov.numerical_value_in(units::deg), 90.0, 1e-9);
    CHECK_NEAR(cfg.lidar_max_range.numerical_value_in(units::cm), 400.0, 1e-9);
    CHECK_NEAR(cfg.max_advance_per_cmd.numerical_value_in(units::cm), 50.0, 1e-9);
    std::remove(p.c_str());
}

TEST(config_parser_drone_unknown_key_recovers) {
    const std::string p = write_temp(R"(
lidar_fov 60
SOMETHING_BAD 42
lidar_max_range 200
)");
    DroneConfig cfg;
    auto r = ConfigParser::load_drone_config(p, cfg);
    CHECK(r.ok);
    CHECK(!r.errors.empty());      // an error was recorded
    CHECK_NEAR(cfg.lidar_fov.numerical_value_in(units::deg), 60.0, 1e-9);
    CHECK_NEAR(cfg.lidar_max_range.numerical_value_in(units::cm), 200.0, 1e-9);
    std::remove(p.c_str());
}

TEST(config_parser_mission_polygon) {
    const std::string p = write_temp(R"(
start 50 50 10
height_min 0
height_max 100
xy_decimal_places 0
height_decimal_places 0
polygon_vertex 0 0
polygon_vertex 100 0
polygon_vertex 100 100
polygon_vertex 0 100
)");
    MissionConfig m;
    auto r = ConfigParser::load_mission_config(p, m);
    CHECK(r.ok);
    CHECK_EQ(static_cast<int>(m.boundary_polygon.size()), 4);
    CHECK_NEAR(m.start.x.numerical_value_in(units::cm), 50.0, 1e-9);
    CHECK_NEAR(m.start.y.numerical_value_in(units::cm), 50.0, 1e-9);
    CHECK_NEAR(m.height_max.numerical_value_in(units::cm), 100.0, 1e-9);
    std::remove(p.c_str());
}

TEST(config_parser_missing_file_returns_not_ok) {
    DroneConfig cfg;
    auto r = ConfigParser::load_drone_config("/tmp/__definitely_missing__.txt", cfg);
    CHECK(!r.ok);
    CHECK(!r.errors.empty());
}
