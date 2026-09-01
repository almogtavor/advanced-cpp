#include "Simulator/CliConfig.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <system_error>

namespace simulator {

namespace fs = std::filesystem;

namespace {

const std::set<std::string> kKnownKeys = {
    "simulation",
    "mission_control_folder",
    "algorithm",
    "mission_control",
    "algorithms_folder",
    "num_threads"
};

bool canOpenFile(const std::string& path) {
    std::ifstream file(path);
    return file.good();
}

enum class FolderScan {
    Ok,
    NotTraversable,
    NoSharedLibraries
};

// Walks the folder without ever throwing: every filesystem call uses the
// error_code overload, so a permission-denied or vanished directory is
// reported rather than terminating the simulator.
FolderScan scanSharedLibraryFolder(const std::string& path) {
    std::error_code ec;

    fs::directory_iterator it(path, ec);

    if (ec) {
        return FolderScan::NotTraversable;
    }

    const fs::directory_iterator end;

    for (; it != end; it.increment(ec)) {
        if (ec) {
            return FolderScan::NotTraversable;
        }

        std::error_code entry_ec;

        if (it->is_regular_file(entry_ec) &&
            !entry_ec &&
            it->path().extension() == ".so") {
            return FolderScan::Ok;
        }
    }

    return FolderScan::NoSharedLibraries;
}

void validateFile(
    const std::string& value,
    const std::string& name,
    std::vector<std::string>& errors,
    bool require_shared_library = false) {

    if (value.empty()) {
        return;
    }

    std::error_code ec;

    if (!fs::exists(value, ec) || ec) {
        errors.push_back(
            name + " file does not exist: " + value
        );
        return;
    }

    if (!fs::is_regular_file(value, ec) || ec) {
        errors.push_back(
            name + " is not a regular file: " + value
        );
        return;
    }

    if (require_shared_library && fs::path(value).extension() != ".so") {
        errors.push_back(
            name + " is not a .so file: " + value
        );
        return;
    }

    if (!canOpenFile(value)) {
        errors.push_back(
            name + " cannot be opened: " + value
        );
    }
}

void validateSoFolder(
    const std::string& value,
    const std::string& name,
    std::vector<std::string>& errors) {

    if (value.empty()) {
        return;
    }

    std::error_code ec;

    if (!fs::exists(value, ec) || ec) {
        errors.push_back(
            name + " does not exist: " + value
        );
        return;
    }

    if (!fs::is_directory(value, ec) || ec) {
        errors.push_back(
            name + " is not a directory: " + value
        );
        return;
    }

    switch (scanSharedLibraryFolder(value)) {
        case FolderScan::Ok:
            break;

        case FolderScan::NotTraversable:
            errors.push_back(
                name + " cannot be traversed: " + value
            );
            break;

        case FolderScan::NoSharedLibraries:
            errors.push_back(
                name + " contains no .so files: " + value
            );
            break;
    }
}

} // namespace

CliConfig parseArguments(int argc, char* argv[]) {
    CliConfig config;

    std::set<std::string> seen_keys;
    bool mode_conflict = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        // Bare flags
        if (arg == "-comparative" || arg == "-competition") {
            const RunMode requested = (arg == "-comparative")
                ? RunMode::Comparative
                : RunMode::Competition;

            if (config.mode != RunMode::None && config.mode != requested) {
                config.errors.push_back(
                    "Cannot use both -comparative and -competition"
                );
                mode_conflict = true;
            }

            // On conflict the mode stays unset, so we do not go on to report
            // the "missing argument" errors of a mode the user may not want.
            config.mode = mode_conflict ? RunMode::None : requested;
            continue;
        }

        if (arg == "-verbose") {
            config.verbose = true;
            continue;
        }

        // key=value arguments
        std::size_t equal_pos = arg.find('=');

        if (equal_pos == std::string::npos) {
            config.errors.push_back(
                "Unsupported argument: " + arg
            );
            continue;
        }

        std::string key = arg.substr(0, equal_pos);
        std::string value = arg.substr(equal_pos + 1);

        if (kKnownKeys.count(key) == 0) {
            config.errors.push_back(
                "Unsupported argument: " + arg
            );
            continue;
        }

        if (!seen_keys.insert(key).second) {
            config.errors.push_back(
                "Duplicate argument: " + key
            );
            continue;
        }

        if (value.empty()) {
            config.errors.push_back(
                "Empty value for argument: " + key
            );
            continue;
        }

        if (key == "simulation") {
            config.simulation = value;
        }
        else if (key == "mission_control_folder") {
            config.mission_control_folder = value;
        }
        else if (key == "algorithm") {
            config.algorithm = value;
        }
        else if (key == "mission_control") {
            config.mission_control = value;
        }
        else if (key == "algorithms_folder") {
            config.algorithms_folder = value;
        }
        else if (key == "num_threads") {
            try {
                std::size_t consumed = 0;

                int number = std::stoi(value, &consumed);

                if (consumed != value.size() || number < 1) {
                    config.errors.push_back(
                        "Invalid num_threads value: " + value
                    );
                } else {
                    config.num_threads = number;
                }

            } catch (...) {
                config.errors.push_back(
                    "Invalid num_threads value: " + value
                );
            }
        }
    }

    // Mode validation. A conflict already reported its own error, so do not
    // also complain that the mode is missing.
    if (config.mode == RunMode::None && !mode_conflict) {
        config.errors.push_back(
            "Missing mode: use -comparative or -competition"
        );
    }

    // Common required argument
    if (config.simulation.empty()) {
        config.errors.push_back(
            "Missing argument: simulation"
        );
    }

    // Comparative requirements
    if (config.mode == RunMode::Comparative) {
        if (config.mission_control_folder.empty()) {
            config.errors.push_back(
                "Missing argument: mission_control_folder"
            );
        }

        if (config.algorithm.empty()) {
            config.errors.push_back(
                "Missing argument: algorithm"
            );
        }
    }

    // Competition requirements
    if (config.mode == RunMode::Competition) {
        if (config.mission_control.empty()) {
            config.errors.push_back(
                "Missing argument: mission_control"
            );
        }

        if (config.algorithms_folder.empty()) {
            config.errors.push_back(
                "Missing argument: algorithms_folder"
            );
        }
    }

    // Validate files/folders even if there were other errors.
    validateFile(
        config.simulation,
        "simulation",
        config.errors
    );

    if (config.mode == RunMode::Comparative) {
        validateFile(
            config.algorithm,
            "algorithm",
            config.errors,
            true
        );

        validateSoFolder(
            config.mission_control_folder,
            "mission_control_folder",
            config.errors
        );
    }

    if (config.mode == RunMode::Competition) {
        validateFile(
            config.mission_control,
            "mission_control",
            config.errors,
            true
        );

        validateSoFolder(
            config.algorithms_folder,
            "algorithms_folder",
            config.errors
        );
    }

    return config;
}

void printUsage(const std::string& program_name) {
    std::cerr
        << "Usage:\n\n"

        << "Comparative:\n"
        << program_name
        << " -comparative"
        << " simulation=<simulation.yaml>"
        << " mission_control_folder=<folder>"
        << " algorithm=<algorithm.so>"
        << " [num_threads=<num>]"
        << " [-verbose]\n\n"

        << "Competition:\n"
        << program_name
        << " -competition"
        << " simulation=<simulation.yaml>"
        << " mission_control=<mission_control.so>"
        << " algorithms_folder=<folder>"
        << " [num_threads=<num>]"
        << " [-verbose]\n";
}

} // namespace simulator
