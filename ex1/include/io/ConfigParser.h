#pragma once

#include <string>
#include <vector>

#include "config/DroneConfig.h"
#include "config/MissionConfig.h"

namespace drone {

// Result of parsing a config file. Errors here are recoverable: if a key
// is missing or unparseable, the default from the struct is kept and an
// entry is appended to `errors`.
struct ParseResult {
    bool ok{true};                  // false only on hard parse failure (file missing)
    std::vector<std::string> errors;
};

class ConfigParser {
public:
    // Parses key/value config files. Format:
    //   # comments allowed
    //   key value [more values...]
    // Unknown keys are recorded as recoverable errors.
    static ParseResult load_drone_config(const std::string& path,
                                         DroneConfig& out);

    static ParseResult load_mission_config(const std::string& path,
                                           MissionConfig& out);
};

} // namespace drone
