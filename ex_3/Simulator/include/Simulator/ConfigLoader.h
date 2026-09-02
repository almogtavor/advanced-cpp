#pragma once

// YAML parsing for every configuration file described in the assignment.
// Parse functions accept raw YAML text (handy for tests); load functions read
// a file. All throw std::runtime_error with a descriptive message when a
// required key is missing or malformed. Relative map/config paths inside a
// composition are resolved against the composition file's directory.

#include <Common/Types.h>
#include <Simulator/SimulationTypes.h>

#include <filesystem>
#include <string>

namespace simulator::config {

using namespace common;

[[nodiscard]] common::types::DroneConfigData parseDroneConfig(const std::string& yaml_text);
[[nodiscard]] common::types::LidarConfigData parseLidarConfig(const std::string& yaml_text);
[[nodiscard]] common::types::MissionConfigData parseMissionConfig(const std::string& yaml_text);
[[nodiscard]] types::SimulationConfigData parseSimulationConfig(const std::string& yaml_text);

[[nodiscard]] common::types::DroneConfigData loadDroneConfig(const std::filesystem::path& file);
[[nodiscard]] common::types::LidarConfigData loadLidarConfig(const std::filesystem::path& file);
[[nodiscard]] common::types::MissionConfigData loadMissionConfig(const std::filesystem::path& file);
[[nodiscard]] types::SimulationConfigData loadSimulationConfig(const std::filesystem::path& file);

// Reads a simulation composition file and every file it references, resolving
// relative paths against the composition file's parent directory. The
// map_filename inside each simulation config is likewise made absolute.
[[nodiscard]] types::SimulationCompositionData loadComposition(const std::filesystem::path& file);

} // namespace simulator::config
