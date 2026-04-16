#pragma once

#include "drone_mapper/Types.hpp"

#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace testutil {

namespace fs = std::filesystem;

class TempDir {
 public:
  explicit TempDir(std::string stem) {
    static int counter = 0;
    path_ = fs::temp_directory_path() / ("drone_mapper_" + stem + "_" + std::to_string(++counter));
    std::error_code ec;
    fs::remove_all(path_, ec);
    fs::create_directories(path_);
  }

  ~TempDir() {
    std::error_code ec;
    fs::remove_all(path_, ec);
  }

  [[nodiscard]] const fs::path& path() const { return path_; }
  [[nodiscard]] fs::path file(std::string_view name) const { return path_ / std::string{name}; }

  void write(std::string_view name, const std::string& content) const {
    std::ofstream out(file(name), std::ios::trunc);
    out << content;
  }

  [[nodiscard]] bool exists(std::string_view name) const { return fs::exists(file(name)); }

  [[nodiscard]] std::string read(std::string_view name) const {
    return read_file(file(name));
  }

  static std::string read_file(const fs::path& path) {
    std::ifstream in(path);
    std::stringstream buffer;
    buffer << in.rdbuf();
    return buffer.str();
  }

 private:
  fs::path path_;
};

inline void expect_contains(const std::string& haystack, const std::string& needle) {
  if (haystack.find(needle) == std::string::npos) {
    throw std::runtime_error("Expected substring not found: " + needle);
  }
}

inline void expect_not_contains(const std::string& haystack, const std::string& needle) {
  if (haystack.find(needle) != std::string::npos) {
    throw std::runtime_error("Unexpected substring found: " + needle);
  }
}

template <typename Fn>
std::string expect_throws(Fn&& fn) {
  try {
    fn();
  } catch (const std::exception& ex) {
    return ex.what();
  }
  throw std::runtime_error("Expected exception");
}

inline std::vector<std::string> split_lines(const std::string& text) {
  std::vector<std::string> lines;
  std::stringstream ss(text);
  std::string line;
  while (std::getline(ss, line)) {
    lines.push_back(line);
  }
  return lines;
}

inline double parse_score_message(const std::string& message) {
  const auto pos = message.rfind(' ');
  if (pos == std::string::npos) {
    throw std::runtime_error("Cannot parse score message: " + message);
  }
  return std::stod(message.substr(pos + 1));
}

inline std::string valid_drone_config_text() {
  return
      "min_pass_width_cm=50\n"
      "min_pass_length_cm=60\n"
      "min_pass_height_cm=70\n"
      "lidar_fov_deg=90\n"
      "lidar_min_range_cm=1\n"
      "lidar_max_range_cm=500\n"
      "resolution_near_distance_cm=50\n"
      "resolution_near_cell_cm=10\n"
      "resolution_far_distance_cm=300\n"
      "resolution_far_cell_cm=20\n"
      "max_rotate_deg=360\n"
      "max_advance_cm=100\n"
      "max_elevate_cm=100\n";
}

inline std::string mission_config_text(const dm::MissionConfig& mission = dm::MissionConfig{0, 0, 2, 2, 0, 0, 0, 0}) {
  std::ostringstream out;
  out << "boundary_min_x=" << mission.boundary_min_x << "\n"
      << "boundary_min_y=" << mission.boundary_min_y << "\n"
      << "boundary_max_x=" << mission.boundary_max_x << "\n"
      << "boundary_max_y=" << mission.boundary_max_y << "\n"
      << "boundary_min_z=" << mission.boundary_min_z << "\n"
      << "boundary_max_z=" << mission.boundary_max_z << "\n"
      << "xy_decimals=" << mission.xy_decimals << "\n"
      << "z_decimals=" << mission.z_decimals << "\n";
  return out.str();
}

inline std::string map_input_text(
    int size_x,
    int size_y,
    int size_z,
    dm::Position start,
    std::initializer_list<dm::Position> occupied = {}) {
  std::ostringstream out;
  out << "size=" << size_x << "," << size_y << "," << size_z << "\n";
  out << "start=" << start.x << "," << start.y << "," << start.z << "\n";
  for (const auto& cell : occupied) {
    out << cell.x << "," << cell.y << "," << cell.z << "\n";
  }
  return out.str();
}

inline void write_input_bundle(
    const TempDir& dir,
    const std::string& drone_text,
    const std::string& mission_text,
    const std::string& map_text) {
  dir.write("drone_config.txt", drone_text);
  dir.write("mission_config.txt", mission_text);
  dir.write("map_input.txt", map_text);
}

}  // namespace testutil
