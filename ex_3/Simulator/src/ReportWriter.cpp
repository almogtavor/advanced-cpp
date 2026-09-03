#include <Simulator/ReportWriter.h>
#include <Simulator/SimulationTypes.h>

#include <UserCommon/MapGeometry.h>

#include <algorithm>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <system_error>

#include <vector>

namespace simulator {

using namespace common;
using namespace user_common_323084962_212223036;

std::string ReportWriter::statusString(common::types::MissionRunStatus status) {
    switch (status) {
    case common::types::MissionRunStatus::Completed: return "completed";
    case common::types::MissionRunStatus::MaxSteps: return "max_steps";
    case common::types::MissionRunStatus::Error: return "error";
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
        const common::types::MissionRunStatus status =
            run.mission_results.empty() ? common::types::MissionRunStatus::Error
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


namespace {

// Two plugins "agree" when their totals match exactly - both the score and the
// step count, as in the assignment's example where 495/100 and 495/101 form
// separate groups.
[[nodiscard]] bool sameResults(const ReportWriter::PluginTotalsView& a,
                               const ReportWriter::PluginTotalsView& b) {
    return a.total_score == b.total_score && a.total_steps == b.total_steps;
}

void writeOrThrow(const std::filesystem::path& file, const std::string& text) {
    std::error_code ec;
    std::filesystem::create_directories(file.parent_path(), ec);
    std::ofstream out(file);
    if (!out) {
        throw std::runtime_error("Cannot open report file: " + file.string());
    }
    out << text;
}

} // namespace

void ReportWriter::writeComparative(const std::filesystem::path& file,
                                    const std::filesystem::path& composition_file,
                                    const std::string& mission_control_folder,
                                    const std::string& generated_at_utc,
                                    const std::vector<PluginTotalsView>& totals,
                                    const std::vector<std::string>& errors) {
    // Group plugins whose results are identical.
    std::vector<std::vector<PluginTotalsView>> groups;
    for (const PluginTotalsView& t : totals) {
        auto it = std::find_if(groups.begin(), groups.end(),
                               [&](const std::vector<PluginTotalsView>& g) {
                                   return sameResults(g.front(), t);
                               });
        if (it == groups.end()) {
            groups.push_back({t});
        } else {
            it->push_back(t);
        }
    }
    // Most agreeing managers first.
    std::stable_sort(groups.begin(), groups.end(),
                     [](const std::vector<PluginTotalsView>& a,
                        const std::vector<PluginTotalsView>& b) {
                         return a.size() > b.size();
                     });

    std::ostringstream out;
    out << "comparative_report:\n";
    out << "  composition_file: \"" << composition_file.string() << "\"\n";
    out << "  mission_control_folder: \"" << mission_control_folder << "\"\n";
    out << "  generated_at_utc: \"" << generated_at_utc << "\"\n";
    out << "\n  results_summary:\n";
    for (const std::vector<PluginTotalsView>& g : groups) {
        out << "    - same_results: [";
        for (std::size_t i = 0; i < g.size(); ++i) {
            out << (i ? ", " : "") << '"' << g[i].name << '"';
        }
        out << "]\n";
        out << "      total_score: " << g.front().total_score << "\n";
        out << "      total_steps: " << g.front().total_steps << "\n";
    }
    out << "\n  errors: [";
    for (std::size_t i = 0; i < errors.size(); ++i) {
        out << (i ? ", " : "") << '"' << errors[i] << '"';
    }
    out << "]\n";
    writeOrThrow(file, out.str());
}

void ReportWriter::writeCompetitive(const std::filesystem::path& file,
                                    const std::filesystem::path& composition_file,
                                    const std::string& mission_control,
                                    const std::string& generated_at_utc,
                                    const std::vector<PluginTotalsView>& totals,
                                    const std::vector<std::string>& errors) {
    std::ostringstream out;
    out << "competitive_report:\n";
    out << "  composition_file: \"" << composition_file.string() << "\"\n";
    out << "  mission_control: \"" << mission_control << "\"\n";
    out << "  generated_at_utc: \"" << generated_at_utc << "\"\n";
    out << "\n  results_summary:\n";
    for (const PluginTotalsView& t : totals) {
        out << "    - algorithm: \"" << t.name << "\"\n";
        out << "      total_score: " << t.total_score << "\n";
        out << "      total_steps: " << t.total_steps << "\n";
    }
    out << "\n  errors: [";
    for (std::size_t i = 0; i < errors.size(); ++i) {
        out << (i ? ", " : "") << '"' << errors[i] << '"';
    }
    out << "]\n";
    writeOrThrow(file, out.str());
}

} // namespace simulator
