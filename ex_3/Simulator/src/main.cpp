#include "Simulator/CliConfig.h"
#include "Simulator/ConfigLoader.h"
#include "Simulator/Registrar.h"
#include "Simulator/SharedLibrary.h"
#include "Simulator/SimulationManager.h"
#include "Simulator/SimulationRunFactoryImpl.h"

#include <UserCommon/ErrorLog.h>

#include <algorithm>
#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <system_error>
#include <vector>

namespace {

namespace fs = std::filesystem;

// Clears the registrar on every exit path. Declared *after* the loaded
// libraries so it is destroyed *before* them: the factories are std::functions
// whose code lives inside the .so files, so they must die before dlclose.
struct RegistrarGuard {
    ~RegistrarGuard() { simulator::Registrar::instance().clear(); }
};

// Loads one plugin and reports which factory index it contributed, so a
// failure can be attributed to the file that caused it.
bool loadPlugin(const std::string& path,
                std::vector<simulator::SharedLibrary>& libraries,
                std::string& error) {
    simulator::SharedLibrary library(path);
    if (!library.loaded()) {
        error = library.error();
        return false;
    }
    libraries.push_back(std::move(library));
    return true;
}

// First .so in a folder, in sorted order. The full comparative / competition
// sweeps will iterate all of them; this build runs the first one.
std::string firstSharedLibrary(const std::string& folder) {
    std::vector<std::string> found;
    std::error_code ec;
    for (fs::directory_iterator it(folder, ec), end; !ec && it != end; it.increment(ec)) {
        std::error_code entry_ec;
        if (it->is_regular_file(entry_ec) && !entry_ec && it->path().extension() == ".so") {
            found.push_back(it->path().string());
        }
    }
    std::sort(found.begin(), found.end());
    return found.empty() ? std::string{} : found.front();
}

} // namespace

int main(int argc, char* argv[]) {
    const simulator::CliConfig config = simulator::parseArguments(argc, argv);

    if (!config.valid()) {
        std::cerr << "Errors:\n";
        for (const auto& error : config.errors) {
            std::cerr << "  - " << error << '\n';
        }
        std::cerr << '\n';
        simulator::printUsage(argv[0]);
        return 1;
    }

    // Single-mission milestone: one algorithm against one mission control, run
    // on the main thread. In comparative mode the algorithm is given and the
    // mission control is the first .so in the folder; in competition mode it is
    // the other way round. Sweeping every plugin in the folder comes next.
    std::string algorithm_path;
    std::string mission_control_path;

    if (config.mode == simulator::RunMode::Comparative) {
        algorithm_path = config.algorithm;
        mission_control_path = firstSharedLibrary(config.mission_control_folder);
        if (mission_control_path.empty()) {
            std::cerr << "No .so found in " << config.mission_control_folder << '\n';
            return 1;
        }
    } else {
        mission_control_path = config.mission_control;
        algorithm_path = firstSharedLibrary(config.algorithms_folder);
        if (algorithm_path.empty()) {
            std::cerr << "No .so found in " << config.algorithms_folder << '\n';
            return 1;
        }
    }

    std::cout << "algorithm:       " << algorithm_path << '\n'
              << "mission control: " << mission_control_path << '\n';

    int exit_code = 0;
    {
        std::vector<simulator::SharedLibrary> libraries;
        RegistrarGuard guard; // destroyed before `libraries` - see above

        std::string error;
        if (!loadPlugin(algorithm_path, libraries, error)) {
            std::cerr << "Failed to load algorithm: " << error << '\n';
            return 1;
        }
        if (!loadPlugin(mission_control_path, libraries, error)) {
            std::cerr << "Failed to load mission control: " << error << '\n';
            return 1;
        }

        auto& registrar = simulator::Registrar::instance();
        if (registrar.mappingAlgorithms().empty()) {
            std::cerr << "No mapping algorithm registered by " << algorithm_path << '\n';
            return 1;
        }
        if (registrar.missionControls().empty()) {
            std::cerr << "No mission control registered by " << mission_control_path << '\n';
            return 1;
        }

        std::cout << "Loaded " << registrar.mappingAlgorithms().size() << " algorithm factory and "
                  << registrar.missionControls().size() << " mission control factory\n";

        try {
            const simulator::types::SimulationCompositionData composition =
                simulator::config::loadComposition(config.simulation);

            const fs::path output_path = fs::current_path();

            simulator::SimulationManager manager(
                std::make_unique<simulator::SimulationRunFactoryImpl>(
                    registrar.mappingAlgorithms().front(),
                    registrar.missionControls().front(),
                    config.verbose));

            const simulator::types::SimulationManagerReport report =
                manager.run(composition, output_path);

            std::cout << "Completed " << report.runs.size() << " run(s); report written to "
                      << (output_path / "simulation_output.yaml").string() << '\n';
        } catch (const std::exception& ex) {
            std::cerr << "Simulation failed: " << ex.what() << '\n';
            exit_code = 1;
        }
    } // guard clears the registrar, then the libraries dlclose

    return exit_code;
}
