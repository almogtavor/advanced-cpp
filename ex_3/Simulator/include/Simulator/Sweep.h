#pragma once

// Runs one plugin sweep: every (simulation x mission x drone x lidar x plugin)
// combination, optionally across worker threads, then writes the per-plugin
// reports and the single comparative / competitive summary.

#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>
#include <Simulator/SimulationTypes.h>

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace simulator {

enum class SweepMode { Comparative, Competition };

// One loaded plugin taking part in the sweep. In comparative mode the varying
// plugin is the mission control; in competition mode it is the algorithm.
struct SweepPlugin {
    std::string name;                              // file name, e.g. "Algorithm_x.so"
    common::MappingAlgorithmFactory algorithm;     // fixed in comparative mode
    common::MissionControlFactory mission_control; // fixed in competition mode
};

// Per-plugin totals used by both summary formats.
struct PluginTotals {
    std::string name;
    double total_score = 0.0;
    std::size_t total_steps = 0;
};

struct SweepResult {
    std::vector<PluginTotals> totals;   // one per plugin that ran
    std::vector<std::string> errors;    // plugins that could not be loaded / run
    std::filesystem::path results_dir;
};

class Sweep {
public:
    Sweep(SweepMode mode,
          std::vector<SweepPlugin> plugins,
          types::SimulationCompositionData composition,
          std::filesystem::path results_dir,
          int num_threads,
          bool verbose);

    // Never throws: a failing plugin becomes an entry in SweepResult::errors.
    [[nodiscard]] SweepResult run();

    // Creates "<parent>/<prefix>_<time>". Returns an empty path on failure.
    [[nodiscard]] static std::filesystem::path makeResultsDir(
        const std::filesystem::path& parent, const std::string& prefix);

private:
    SweepMode mode_;
    std::vector<SweepPlugin> plugins_;
    types::SimulationCompositionData composition_;
    std::filesystem::path results_dir_;
    int num_threads_ = 1;
    bool verbose_ = false;
};

} // namespace simulator
