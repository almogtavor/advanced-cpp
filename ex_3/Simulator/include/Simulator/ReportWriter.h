#pragma once

// Serialises a SimulationManagerReport into the simulation_output.yaml file.
// The layout is a flat list of runs with a computed summary block, documented
// in README.md.

#include <Common/Types.h>

#include <cstddef>
#include <Simulator/SimulationTypes.h>

#include <filesystem>
#include <string>
#include <vector>

namespace simulator {

using namespace common;

class ReportWriter {
public:
    // Builds the YAML text for the report (also used directly by tests).
    [[nodiscard]] static std::string toYaml(const types::SimulationManagerReport& report);

    // Writes toYaml(report) to `file`, creating parent directories.
    static void write(const types::SimulationManagerReport& report,
                      const std::filesystem::path& file);

    // Assignment 3 summaries. `totals` is one entry per plugin that ran and
    // `errors` names the plugins that could not be loaded or run.
    struct PluginTotalsView {
        std::string name;
        double total_score = 0.0;
        std::size_t total_steps = 0;
    };

    // Groups plugins that produced identical results, sorted by group size
    // descending (most agreeing managers first).
    static void writeComparative(const std::filesystem::path& file,
                                 const std::filesystem::path& composition_file,
                                 const std::string& mission_control_folder,
                                 const std::string& generated_at_utc,
                                 const std::vector<PluginTotalsView>& totals,
                                 const std::vector<std::string>& errors);

    // Sorted by score descending, then steps ascending (done by the caller).
    static void writeCompetitive(const std::filesystem::path& file,
                                 const std::filesystem::path& composition_file,
                                 const std::string& mission_control,
                                 const std::string& generated_at_utc,
                                 const std::vector<PluginTotalsView>& totals,
                                 const std::vector<std::string>& errors);

    [[nodiscard]] static std::string statusString(common::types::MissionRunStatus status);
    [[nodiscard]] static std::string resolutionStatusString(types::ResolutionRequestStatus status);
};

} // namespace simulator
