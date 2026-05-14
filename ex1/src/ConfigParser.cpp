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
        if      (key == "min_passage_width")    out.min_passage_width    = d * units::cm;
        else if (key == "min_passage_length")   out.min_passage_length   = d * units::cm;
        else if (key == "min_passage_height")   out.min_passage_height   = d * units::cm;
        else if (key == "lidar_z_min")          out.lidar_z_min          = d * units::cm;
        else if (key == "lidar_z_max")          out.lidar_z_max          = d * units::cm;
        else if (key == "lidar_d")              out.lidar_d              = d * units::cm;
        else if (key == "lidar_fovc")           out.lidar_fovc           = static_cast<int>(d);
        else if (key == "max_rotate_per_cmd")   out.max_rotate_per_cmd   = d * units::deg;
        else if (key == "max_advance_per_cmd")  out.max_advance_per_cmd  = d * units::cm;
        else if (key == "max_elevate_per_cmd")  out.max_elevate_per_cmd  = d * units::cm;
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
                out.start = Position{x * units::cm, y * units::cm, z * units::cm};
            } else {
                record_error(result, "mission_config:" + std::to_string(line_no) +
                                      ": bad start coords");
            }
        } else if (key == "min_x") {
            if (!need_n(1)) continue;
            double v; if (parse_double(values[0], v)) out.min_x = v * units::cm;
        } else if (key == "max_x") {
            if (!need_n(1)) continue;
            double v; if (parse_double(values[0], v)) out.max_x = v * units::cm;
        } else if (key == "min_y") {
            if (!need_n(1)) continue;
            double v; if (parse_double(values[0], v)) out.min_y = v * units::cm;
        } else if (key == "max_y") {
            if (!need_n(1)) continue;
            double v; if (parse_double(values[0], v)) out.max_y = v * units::cm;
        } else if (key == "height_min") {
            if (!need_n(1)) continue;
            double v; if (parse_double(values[0], v)) out.height_min = v * units::cm;
        } else if (key == "height_max") {
            if (!need_n(1)) continue;
            double v; if (parse_double(values[0], v)) out.height_max = v * units::cm;
        } else if (key == "xy_resolution_cm") {
            if (!need_n(1)) continue;
            double v; if (parse_double(values[0], v)) out.xy_resolution = v * units::cm;
        } else if (key == "height_resolution_cm") {
            if (!need_n(1)) continue;
            double v; if (parse_double(values[0], v)) out.height_resolution = v * units::cm;
        } else if (key == "recharge") {
            if (!need_n(3)) continue;
            double x, y, z;
            if (parse_double(values[0], x) && parse_double(values[1], y) &&
                parse_double(values[2], z)) {
                out.recharge_positions.push_back(
                    Position{x * units::cm, y * units::cm, z * units::cm});
            }
        } else {
            record_error(result, "mission_config:" + std::to_string(line_no) +
                                  ": unknown key '" + key + "' (ignored)");
        }
    }

    if (out.max_x < out.min_x) {
        record_error(result,
            "mission_config: max_x < min_x, swapping");
        std::swap(out.min_x, out.max_x);
    }
    if (out.max_y < out.min_y) {
        record_error(result,
            "mission_config: max_y < min_y, swapping");
        std::swap(out.min_y, out.max_y);
    }
    if (out.height_max < out.height_min) {
        record_error(result,
            "mission_config: height_max < height_min, swapping");
        std::swap(out.height_min, out.height_max);
    }
    return result;
}

} // namespace drone
