#pragma once

// Serialises a SimulationManagerReport into the simulation_output.yaml file.
// The layout is a flat list of runs with a computed summary block, documented
// in README.md.

#include <Common/Types.h>
#include <Simulator/SimulationTypes.h>

#include <filesystem>
#include <string>

namespace simulator {

using namespace common;

class ReportWriter {
public:
    // Builds the YAML text for the report (also used directly by tests).
    [[nodiscard]] static std::string toYaml(const types::SimulationManagerReport& report);

    // Writes toYaml(report) to `file`, creating parent directories.
    static void write(const types::SimulationManagerReport& report,
                      const std::filesystem::path& file);

    [[nodiscard]] static std::string statusString(common::types::MissionRunStatus status);
    [[nodiscard]] static std::string resolutionStatusString(types::ResolutionRequestStatus status);
};

} // namespace simulator
