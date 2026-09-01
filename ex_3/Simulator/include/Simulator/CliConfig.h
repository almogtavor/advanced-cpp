#pragma once

#include <string>
#include <vector>

namespace simulator {

enum class RunMode {
    None,
    Comparative,
    Competition
};

struct CliConfig {
    RunMode mode = RunMode::None;

    std::string simulation;
    std::string mission_control_folder;
    std::string algorithm;

    std::string mission_control;
    std::string algorithms_folder;

    // As parsed from the command line. 1 (or absent) means the simulation runs
    // on the main thread only; >= 2 means that many worker threads *in
    // addition to* main, so the total is never exactly 2.
    int num_threads = 1;
    bool verbose = false;

    std::vector<std::string> errors;

    bool valid() const {
        return errors.empty();
    }
};

CliConfig parseArguments(int argc, char* argv[]);

void printUsage(const std::string& program_name);

} // namespace simulator