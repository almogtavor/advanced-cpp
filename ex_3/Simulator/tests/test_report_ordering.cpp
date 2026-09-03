// The two assignment-3 summary formats have precise ordering rules:
//   comparative - group plugins with identical results, sort groups by the
//                 number of agreeing managers, descending.
//   competitive - sort by score descending, then by steps ascending.
//
// The numbers below are taken from the assignment's own examples, including the
// subtle case where 495/100 and 495/101 must NOT be grouped together: equal
// scores are not enough, the step counts must match too.

#include <Simulator/ReportWriter.h>

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;
using View = simulator::ReportWriter::PluginTotalsView;

[[nodiscard]] std::string readAll(const fs::path& file) {
    std::ifstream in(file);
    std::ostringstream buffer;
    buffer << in.rdbuf();
    return buffer.str();
}

// Position of `needle` in `haystack`, or npos. Used to assert relative order.
[[nodiscard]] std::size_t indexOf(const std::string& haystack, const std::string& needle) {
    return haystack.find(needle);
}

} // namespace

TEST(ReportOrdering, ComparativeGroupsIdenticalResultsAndSortsByGroupSize) {
    const fs::path file = fs::temp_directory_path() / "ex3_comparative_report.yaml";
    fs::remove(file);

    // Deliberately out of order, and manager4 shares a score with the big group
    // but differs by one step.
    const std::vector<View> totals{
        {"manager3.so", 502.0, 124},
        {"manager1.so", 495.0, 100},
        {"manager4.so", 495.0, 101},
        {"manager6.so", 502.0, 124},
        {"manager2.so", 495.0, 100},
        {"manager5.so", 495.0, 100},
    };
    const std::vector<std::string> errors{"manager7.so", "manager8.so"};

    simulator::ReportWriter::writeComparative(file, "simulation_compositions.yaml", "folder",
                                              "2026-05-30T23:31:10Z", totals, errors);
    const std::string yaml = readAll(file);

    // The three-member group must come first, then the two-member, then the
    // lone manager4 - strictly by group size, descending.
    const std::size_t group_of_three =
        indexOf(yaml, R"(same_results: ["manager1.so", "manager2.so", "manager5.so"])");
    const std::size_t group_of_two =
        indexOf(yaml, R"(same_results: ["manager3.so", "manager6.so"])");
    const std::size_t group_of_one = indexOf(yaml, R"(same_results: ["manager4.so"])");

    ASSERT_NE(group_of_three, std::string::npos) << "agreeing managers were not grouped:\n" << yaml;
    ASSERT_NE(group_of_two, std::string::npos) << yaml;
    ASSERT_NE(group_of_one, std::string::npos)
        << "manager4 shares a score with the top group but not its step count, "
           "so it must form its own group:\n"
        << yaml;

    EXPECT_LT(group_of_three, group_of_two) << "groups are not sorted by size descending";
    EXPECT_LT(group_of_two, group_of_one) << "groups are not sorted by size descending";

    EXPECT_NE(indexOf(yaml, "comparative_report:"), std::string::npos);
    EXPECT_NE(indexOf(yaml, R"(errors: ["manager7.so", "manager8.so"])"), std::string::npos)
        << "plugins that could not be loaded must appear in errors, not in the summary";
    EXPECT_EQ(indexOf(yaml, "manager7.so\", \"manager8.so\"]\n  results"), std::string::npos);

    fs::remove(file);
}

TEST(ReportOrdering, CompetitiveKeepsCallerOrderAndRendersEveryAlgorithm) {
    const fs::path file = fs::temp_directory_path() / "ex3_competitive_report.yaml";
    fs::remove(file);

    // Sweep sorts by score descending then steps ascending before writing; the
    // writer must preserve that order rather than re-sorting or reversing it.
    const std::vector<View> totals{
        {"algorithm1.so", 495.0, 100},
        {"algorithm3.so", 490.0, 97},
        {"algorithm4.so", 490.0, 113},
    };
    const std::vector<std::string> errors{"algorithm2.so", "algorithm5.so"};

    simulator::ReportWriter::writeCompetitive(file, "simulation_compositions.yaml",
                                              "mission_control_filename.so",
                                              "2026-05-30T23:31:10Z", totals, errors);
    const std::string yaml = readAll(file);

    const std::size_t first = indexOf(yaml, R"(algorithm: "algorithm1.so")");
    const std::size_t second = indexOf(yaml, R"(algorithm: "algorithm3.so")");
    const std::size_t third = indexOf(yaml, R"(algorithm: "algorithm4.so")");

    ASSERT_NE(first, std::string::npos) << yaml;
    ASSERT_NE(second, std::string::npos) << yaml;
    ASSERT_NE(third, std::string::npos) << yaml;

    EXPECT_LT(first, second) << "higher score must come first";
    EXPECT_LT(second, third) << "equal scores must be ordered by fewer steps first";

    EXPECT_NE(indexOf(yaml, "competitive_report:"), std::string::npos);
    EXPECT_NE(indexOf(yaml, R"(mission_control: "mission_control_filename.so")"),
              std::string::npos);
    EXPECT_NE(indexOf(yaml, R"(errors: ["algorithm2.so", "algorithm5.so"])"), std::string::npos);

    // Failed algorithms must not also appear as scored entries.
    EXPECT_EQ(indexOf(yaml, R"(algorithm: "algorithm2.so")"), std::string::npos);

    fs::remove(file);
}
