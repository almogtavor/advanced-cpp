#include <Simulator/Sweep.h>

#include <Simulator/ReportWriter.h>
#include <Simulator/SimulationManager.h>
#include <Simulator/SimulationRunFactoryImpl.h>
#include <UserCommon/ErrorLog.h>

#include <algorithm>
#include <chrono>
#include <exception>
#include <memory>
#include <sstream>
#include <string>
#include <system_error>
#include <utility>

namespace simulator {

using namespace common;
using namespace user_common_323084962_212223036;

namespace {

// Strips directories and the .so suffix so a plugin name can go into a filename.
[[nodiscard]] std::string plainName(const std::string& name) {
    std::filesystem::path p(name);
    return p.stem().string();
}

} // namespace

std::filesystem::path Sweep::makeResultsDir(const std::filesystem::path& parent,
                                            const std::string& prefix) {
    // Seconds since the epoch: a new number per run, so repeated runs never
    // collide with an existing directory.
    const auto stamp = std::chrono::duration_cast<std::chrono::seconds>(
                           std::chrono::system_clock::now().time_since_epoch())
                           .count();

    std::error_code ec;
    for (int attempt = 0; attempt < 100; ++attempt) {
        std::ostringstream name;
        name << prefix << '_' << (stamp + attempt);
        const std::filesystem::path candidate = parent / name.str();
        if (std::filesystem::exists(candidate, ec)) {
            continue;
        }
        if (std::filesystem::create_directories(candidate, ec) && !ec) {
            return candidate;
        }
    }
    return {};
}

Sweep::Sweep(SweepMode mode,
             std::vector<SweepPlugin> plugins,
             types::SimulationCompositionData composition,
             std::filesystem::path results_dir,
             int num_threads,
             bool verbose)
    : mode_(mode),
      plugins_(std::move(plugins)),
      composition_(std::move(composition)),
      results_dir_(std::move(results_dir)),
      num_threads_(num_threads),
      verbose_(verbose) {}

SweepResult Sweep::run() {
    SweepResult sweep_result;
    sweep_result.results_dir = results_dir_;

    globalErrorLog().open(results_dir_ / "error_log.txt");

    for (const SweepPlugin& plugin : plugins_) {
        // Each run gets its own factory, labelled with the plugin and the run's
        // place in the composition, so output maps trace back to their mission
        // and nothing is shared between worker threads.
        const std::string prefix = plainName(plugin.name);
        SimulationManager::RunFactoryMaker maker =
            [&plugin, prefix, this](const std::string& run_label)
            -> std::unique_ptr<ISimulationRunFactory> {
            return std::make_unique<SimulationRunFactoryImpl>(
                plugin.algorithm, plugin.mission_control, prefix + "_" + run_label, verbose_);
        };

        types::SimulationManagerReport report;
        try {
            SimulationManager manager(std::move(maker), num_threads_);
            report = manager.run(composition_, results_dir_);
        } catch (const std::exception& ex) {
            globalErrorLog().log("PLUGIN_RUN_ERROR", plugin.name + ": " + ex.what());
            sweep_result.errors.push_back(plugin.name);
            continue;
        }

        // A plugin whose every run failed is reported as an error rather than
        // ranked against the ones that worked.
        PluginTotals totals;
        totals.name = plugin.name;
        bool any_success = false;
        for (const types::SimulationResult& result : report.runs) {
            const bool errored =
                std::any_of(result.mission_results.begin(), result.mission_results.end(),
                            [](const common::types::MissionRunResult& mission_result) {
                                return mission_result.status ==
                                       common::types::MissionRunStatus::Error;
                            });
            if (errored) {
                continue;
            }
            any_success = true;
            totals.total_score += result.mission_score;
            for (const common::types::MissionRunResult& mission_result : result.mission_results) {
                totals.total_steps += mission_result.steps;
            }
        }

        if (!any_success) {
            sweep_result.errors.push_back(plugin.name);
            continue;
        }

        // One assignment-2 style report per plugin, named after that plugin.
        try {
            ReportWriter::write(report,
                                results_dir_ / ("simulation_output_" + prefix + ".yaml"));
        } catch (const std::exception& ex) {
            globalErrorLog().log("REPORT_WRITE_ERROR", ex.what());
        }

        sweep_result.totals.push_back(std::move(totals));
    }

    if (mode_ == SweepMode::Competition) {
        // Score descending, then steps ascending.
        std::sort(sweep_result.totals.begin(), sweep_result.totals.end(),
                  [](const PluginTotals& a, const PluginTotals& b) {
                      if (a.total_score != b.total_score) {
                          return a.total_score > b.total_score;
                      }
                      return a.total_steps < b.total_steps;
                  });
    }
    // Comparative grouping is done by the report writer, which needs the raw
    // per-plugin totals to decide which plugins agreed.

    return sweep_result;
}

} // namespace simulator
