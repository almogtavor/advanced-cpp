#include <drone_mapper/ReportWriter.h>

#include <drone_mapper/MapGeometry.h>

#include <algorithm>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace drone_mapper {

std::string ReportWriter::statusString(types::MissionRunStatus status) {
    switch (status) {
    case types::MissionRunStatus::Completed: return "completed";
    case types::MissionRunStatus::MaxSteps: return "max_steps";
    case types::MissionRunStatus::Error: return "error";
    }
    return "unknown";
}

std::string ReportWriter::resolutionStatusString(types::ResolutionRequestStatus status) {
    switch (status) {
    case types::ResolutionRequestStatus::Accepted: return "ACCEPTED";
    case types::ResolutionRequestStatus::Ignored: return "IGNORED";
    case types::ResolutionRequestStatus::IgnoredTooSmall: return "IGNORED_TOO_SMALL";
    }
    return "UNKNOWN";
}

std::string ReportWriter::toYaml(const types::SimulationManagerReport& report) {
    std::ostringstream out;

    std::size_t scored = 0;
    std::size_t errored = 0;
    double sum = 0.0;
    double min_score = std::numeric_limits<double>::max();
    double max_score = std::numeric_limits<double>::lowest();
    for (const types::SimulationResult& run : report.runs) {
        if (run.mission_score < 0.0) {
            ++errored;
        } else {
            ++scored;
            sum += run.mission_score;
            min_score = std::min(min_score, run.mission_score);
            max_score = std::max(max_score, run.mission_score);
        }
    }
    const double average = scored > 0 ? sum / static_cast<double>(scored) : 0.0;
    if (scored == 0) {
        min_score = 0.0;
        max_score = 0.0;
    }

    out << "score_report:\n";
    out << "  generated_at_utc: \"" << report.generated_at_utc << "\"\n";
    out << "  metric: \"" << report.metric << "\"\n";
    out << "  score_range:\n";
    out << "    min: " << std::get<0>(report.score_range) << "\n";
    out << "    max: " << std::get<1>(report.score_range) << "\n";
    out << "  error_score: " << report.error_score << "\n";
    out << "  summary:\n";
    out << "    total_runs: " << report.runs.size() << "\n";
    out << "    scored_runs: " << scored << "\n";
    out << "    error_runs: " << errored << "\n";
    out << "    average_score: " << average << "\n";
    out << "    min_score: " << min_score << "\n";
    out << "    max_score: " << max_score << "\n";
    out << "  runs:\n";
    for (const types::SimulationResult& run : report.runs) {
        const types::MissionRunStatus status =
            run.mission_results.empty() ? types::MissionRunStatus::Error
                                        : run.mission_results.front().status;
        const std::size_t steps =
            run.mission_results.empty() ? 0 : run.mission_results.front().steps;

        out << "    - simulation_config: \"" << run.simulation_config.map_filename.string() << "\"\n";
        out << "      mission_max_steps: " << run.mission_config.max_steps << "\n";
        out << "      resolution_cm: " << geom::lcm(run.output_map_config.resolution) << "\n";
        out << "      resolution_request_status: " << resolutionStatusString(run.resolution_request_status) << "\n";
        out << "      status: \"" << statusString(status) << "\"\n";
        out << "      steps: " << steps << "\n";
        out << "      score: " << run.mission_score << "\n";
        out << "      output_map_file: \"" << run.output_map_file.string() << "\"\n";
        if (!run.mission_results.empty() && !run.mission_results.front().errors.empty()) {
            out << "      error_ref:\n";
            out << "        code: \"" << run.mission_results.front().errors.front().code << "\"\n";
        }
    }
    return out.str();
}

void ReportWriter::write(const types::SimulationManagerReport& report,
                         const std::filesystem::path& file) {
    if (file.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(file.parent_path(), ec);
    }
    std::ofstream out(file, std::ios::out | std::ios::trunc);
    if (!out) {
        throw std::runtime_error("Failed to write report file: " + file.string());
    }
    out << toYaml(report);
}

} // namespace drone_mapper
