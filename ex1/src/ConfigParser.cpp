#include "io/ConfigParser.h"

#include <cctype>
#include <fstream>
#include <sstream>
#include <string>

namespace drone {

namespace {

// Strip leading/trailing whitespace and remove anything after '#' (comment).
std::string strip(const std::string& in) {
    std::string s = in;
    const auto hash = s.find('#');
    if (hash != std::string::npos) s.erase(hash);
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) s.erase(s.begin());
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back())))  s.pop_back();
    return s;
}

bool parse_double(const std::string& tok, double& out) {
    try {
        std::size_t consumed = 0;
        out = std::stod(tok, &consumed);
        return consumed == tok.size();
    } catch (...) {
        return false;
    }
}

bool parse_int(const std::string& tok, int& out) {
    try {
        std::size_t consumed = 0;
        out = std::stoi(tok, &consumed);
        return consumed == tok.size();
    } catch (...) {
        return false;
    }
}

// Splits "key v1 v2 v3..." into key and the remaining tokens.
bool tokenize_line(const std::string& line, std::string& key,
                   std::vector<std::string>& values) {
    std::istringstream iss(line);
    if (!(iss >> key)) return false;
    std::string tok;
    while (iss >> tok) values.push_back(std::move(tok));
    return true;
}

void record_error(ParseResult& r, const std::string& msg) {
    r.errors.push_back(msg);
}

} // namespace

ParseResult ConfigParser::load_drone_config(const std::string& path,
                                            DroneConfig& out) {
    ParseResult result;
    std::ifstream in(path);
    if (!in) {
        result.ok = false;
        record_error(result, "drone_config: cannot open file: " + path);
        return result;
    }

    std::string raw;
    int line_no = 0;
    while (std::getline(in, raw)) {
        ++line_no;
        const std::string line = strip(raw);
        if (line.empty()) continue;

        std::string key;
        std::vector<std::string> values;
        if (!tokenize_line(line, key, values) || values.empty()) {
            record_error(result, "drone_config:" + std::to_string(line_no) +
                                  ": missing value for key '" + key + "'");
            continue;
        }

        double d = 0.0;
        if (!parse_double(values[0], d)) {
            record_error(result, "drone_config:" + std::to_string(line_no) +
                                  ": cannot parse number '" + values[0] + "'");
            continue;
        }

        // All length keys store cm in the file, all angle keys degrees.
        if      (key == "min_passage_width")    out.min_passage_width    = units::Length(d);
        else if (key == "min_passage_length")   out.min_passage_length   = units::Length(d);
        else if (key == "min_passage_height")   out.min_passage_height   = units::Length(d);
        else if (key == "lidar_fov")            out.lidar_fov            = units::Angle(d);
        else if (key == "lidar_min_range")      out.lidar_min_range      = units::Length(d);
        else if (key == "lidar_max_range")      out.lidar_max_range      = units::Length(d);
        else if (key == "lidar_res_dist_a")     out.lidar_res_dist_a     = units::Length(d);
        else if (key == "lidar_res_side_a")     out.lidar_res_side_a     = units::Length(d);
        else if (key == "lidar_res_dist_b")     out.lidar_res_dist_b     = units::Length(d);
        else if (key == "lidar_res_side_b")     out.lidar_res_side_b     = units::Length(d);
        else if (key == "max_rotate_per_cmd")   out.max_rotate_per_cmd   = units::Angle(d);
        else if (key == "max_advance_per_cmd")  out.max_advance_per_cmd  = units::Length(d);
        else if (key == "max_elevate_per_cmd")  out.max_elevate_per_cmd  = units::Length(d);
        else {
            record_error(result, "drone_config:" + std::to_string(line_no) +
                                  ": unknown key '" + key + "' (ignored)");
        }
    }
    return result;
}

ParseResult ConfigParser::load_mission_config(const std::string& path,
                                              MissionConfig& out) {
    ParseResult result;
    std::ifstream in(path);
    if (!in) {
        result.ok = false;
        record_error(result, "mission_config: cannot open file: " + path);
        return result;
    }

    std::string raw;
    int line_no = 0;
    while (std::getline(in, raw)) {
        ++line_no;
        const std::string line = strip(raw);
        if (line.empty()) continue;

        std::string key;
        std::vector<std::string> values;
        if (!tokenize_line(line, key, values)) continue;

        auto need_n = [&](std::size_t n) -> bool {
            if (values.size() < n) {
                record_error(result, "mission_config:" + std::to_string(line_no) +
                                      ": key '" + key + "' needs " +
                                      std::to_string(n) + " values");
                return false;
            }
            return true;
        };

        if (key == "start") {
            if (!need_n(3)) continue;
            double x, y, z;
            if (parse_double(values[0], x) && parse_double(values[1], y) &&
                parse_double(values[2], z)) {
                out.start = Position{units::Length(x), units::Length(y), units::Length(z)};
            } else {
                record_error(result, "mission_config:" + std::to_string(line_no) +
                                      ": bad start coords");
            }
        } else if (key == "height_min") {
            if (!need_n(1)) continue;
            double v; if (parse_double(values[0], v)) out.height_min = units::Length(v);
        } else if (key == "height_max") {
            if (!need_n(1)) continue;
            double v; if (parse_double(values[0], v)) out.height_max = units::Length(v);
        } else if (key == "xy_decimal_places") {
            if (!need_n(1)) continue;
            int v; if (parse_int(values[0], v)) out.xy_decimal_places = v;
        } else if (key == "height_decimal_places") {
            if (!need_n(1)) continue;
            int v; if (parse_int(values[0], v)) out.height_decimal_places = v;
        } else if (key == "polygon_vertex") {
            if (!need_n(2)) continue;
            double x, y;
            if (parse_double(values[0], x) && parse_double(values[1], y)) {
                out.boundary_polygon.emplace_back(units::Length(x), units::Length(y));
            } else {
                record_error(result, "mission_config:" + std::to_string(line_no) +
                                      ": bad polygon_vertex");
            }
        } else if (key == "recharge") {
            if (!need_n(3)) continue;
            double x, y, z;
            if (parse_double(values[0], x) && parse_double(values[1], y) &&
                parse_double(values[2], z)) {
                out.recharge_positions.push_back(
                    Position{units::Length(x), units::Length(y), units::Length(z)});
            }
        } else {
            record_error(result, "mission_config:" + std::to_string(line_no) +
                                  ": unknown key '" + key + "' (ignored)");
        }
    }

    if (out.boundary_polygon.size() < 3) {
        record_error(result,
            "mission_config: polygon has fewer than 3 vertices, boundary disabled");
    }
    if (out.height_max < out.height_min) {
        record_error(result,
            "mission_config: height_max < height_min, swapping");
        std::swap(out.height_min, out.height_max);
    }
    return result;
}

} // namespace drone
