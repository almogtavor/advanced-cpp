#include "Simulator/CliConfig.h"

#include <iostream>

int main(int argc, char* argv[]) {
    simulator::CliConfig config =
        simulator::parseArguments(argc, argv);

    if (!config.valid()) {
        std::cerr << "Errors:\n";

        for (const auto& error : config.errors) {
            std::cerr << "  - " << error << '\n';
        }

        std::cerr << '\n';

        simulator::printUsage(argv[0]);

        return 1;
    }

    if (config.mode == simulator::RunMode::Comparative) {
        std::cout << "Running comparative mode\n";
        std::cout << "Simulation: "
                  << config.simulation
                  << '\n';

        std::cout << "MissionControl folder: "
                  << config.mission_control_folder
                  << '\n';

        std::cout << "Algorithm: "
                  << config.algorithm
                  << '\n';
    }

    if (config.mode == simulator::RunMode::Competition) {
        std::cout << "Running competition mode\n";

        std::cout << "Simulation: "
                  << config.simulation
                  << '\n';

        std::cout << "MissionControl: "
                  << config.mission_control
                  << '\n';

        std::cout << "Algorithms folder: "
                  << config.algorithms_folder
                  << '\n';
    }

    std::cout << "Threads: "
              << config.num_threads
              << '\n';

    std::cout << "Verbose: "
              << (config.verbose ? "yes" : "no")
              << '\n';

    // Next phase:
    // load all required .so files here BEFORE starting threads.

    return 0;
}