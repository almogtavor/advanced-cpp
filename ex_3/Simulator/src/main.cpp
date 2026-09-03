#include "Simulator/CliConfig.h"
#include "Simulator/ConfigLoader.h"
#include "Simulator/Registrar.h"
#include "Simulator/ReportWriter.h"
#include "Simulator/SharedLibrary.h"
#include "Simulator/Sweep.h"

#include <UserCommon/ErrorLog.h>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <exception>
#include <filesystem>
#include <iostream>
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

[[nodiscard]] std::string utcNow() {
    using clock = std::chrono::system_clock;
    const std::time_t t = clock::to_time_t(clock::now());
    std::tm tm_utc{};
#if defined(_WIN32)
    gmtime_s(&tm_utc, &t);
#else
    gmtime_r(&t, &tm_utc);
#endif
    char buffer[32];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &tm_utc);
    return buffer;
}

// Every .so in a folder, in sorted order so runs are reproducible.
[[nodiscard]] std::vector<fs::path> sharedLibrariesIn(const std::string& folder) {
    std::vector<fs::path> found;
    std::error_code ec;
    for (fs::directory_iterator it(folder, ec), end; !ec && it != end; it.increment(ec)) {
        std::error_code entry_ec;
        if (it->is_regular_file(entry_ec) && !entry_ec && it->path().extension() == ".so") {
            found.push_back(it->path());
        }
    }
    std::sort(found.begin(), found.end());
    return found;
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

    const bool comparative = config.mode == simulator::RunMode::Comparative;

    // The fixed plugin is given by a file argument; the varying ones come from
    // the folder argument and are what the summary report ranks / groups.
    const std::string fixed_path = comparative ? config.algorithm : config.mission_control;
    const std::string varying_folder =
        comparative ? config.mission_control_folder : config.algorithms_folder;

    const std::filesystem::path results_dir = simulator::Sweep::makeResultsDir(
        varying_folder, comparative ? "comparative_results" : "competition");
    if (results_dir.empty()) {
        std::cerr << "Could not create the results directory under " << varying_folder << '\n';
        return 1;
    }

    int exit_code = 0;
    {
        std::vector<simulator::SharedLibrary> libraries;
        RegistrarGuard guard; // destroyed before `libraries` - see above

        auto& registrar = simulator::Registrar::instance();
        std::vector<std::string> load_errors;

        // The fixed plugin must load, otherwise there is nothing to run.
        // The scope is to ensure the library is destroyed after the registrar
        {
            simulator::SharedLibrary library(fixed_path);
            if (!library.loaded()) {
                std::cerr << "Failed to load " << fixed_path << ": " << library.error() << '\n';
                return 1;
            }
            libraries.push_back(std::move(library));
        }
        const bool fixed_ok = comparative ? !registrar.mappingAlgorithms().empty()
                                          : !registrar.missionControls().empty();
        if (!fixed_ok) {
            std::cerr << "No factory registered by " << fixed_path << '\n';
            return 1;
        }

        // Every varying plugin. One that fails to load is recorded and skipped
        // rather than aborting the whole sweep.
        std::vector<simulator::SweepPlugin> plugins;
        for (const fs::path& path : sharedLibrariesIn(varying_folder)) {
            const std::size_t before = comparative ? registrar.missionControls().size()
                                                   : registrar.mappingAlgorithms().size();
            simulator::SharedLibrary library(path.string());
            if (!library.loaded()) {
                load_errors.push_back(path.filename().string());
                std::cerr << "Skipping " << path.filename().string() << ": " << library.error()
                          << '\n';
                continue;
            }
            libraries.push_back(std::move(library));

            const std::size_t after = comparative ? registrar.missionControls().size()
                                                  : registrar.mappingAlgorithms().size();
            if (after == before) {
                // Loaded, but registered nothing usable.
                load_errors.push_back(path.filename().string());
                continue;
            }

            simulator::SweepPlugin plugin;
            plugin.name = path.filename().string();
            if (comparative) {
                plugin.algorithm = registrar.mappingAlgorithms().front();
                plugin.mission_control = registrar.missionControls()[before];
            } else {
                plugin.algorithm = registrar.mappingAlgorithms()[before];
                plugin.mission_control = registrar.missionControls().front();
            }
            plugins.push_back(std::move(plugin));
        }

        if (plugins.empty()) {
            std::cerr << "No usable plugin found in " << varying_folder << '\n';
            return 1;
        }

        std::cout << "Running " << plugins.size() << " plugin(s) with "
                  << (config.num_threads >= 2 ? config.num_threads : 1) << " thread(s)\n";

        try {
            const simulator::types::SimulationCompositionData composition =
                simulator::config::loadComposition(config.simulation);

            simulator::Sweep sweep(
                comparative ? simulator::SweepMode::Comparative : simulator::SweepMode::Competition,
                std::move(plugins), composition, results_dir, config.num_threads, config.verbose);

            simulator::SweepResult result = sweep.run();
            result.errors.insert(result.errors.begin(), load_errors.begin(), load_errors.end());

            std::vector<simulator::ReportWriter::PluginTotalsView> totals;
            totals.reserve(result.totals.size());
            for (const simulator::PluginTotals& t : result.totals) {
                totals.push_back({t.name, t.total_score, t.total_steps});
            }

            if (comparative) {
                simulator::ReportWriter::writeComparative(
                    results_dir / "comparative_report.yaml", composition.composition_file,
                    varying_folder, utcNow(), totals, result.errors);
            } else {
                simulator::ReportWriter::writeCompetitive(
                    results_dir / "competitive_report.yaml", composition.composition_file,
                    fs::path(fixed_path).filename().string(), utcNow(), totals, result.errors);
            }

            std::cout << "Results written to " << results_dir.string() << '\n';
        } catch (const std::exception& ex) {
            std::cerr << "Simulation failed: " << ex.what() << '\n';
            exit_code = 1;
        }
    } // guard clears the registrar, then the libraries dlclose

    return exit_code;
}
