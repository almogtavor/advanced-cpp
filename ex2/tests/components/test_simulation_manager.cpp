// SimulationManager tests: cartesian expansion over the composition, error
// handling for scenarios that cannot run, report assembly, and output writing.

#include "support/Mocks.h"
#include "support/TestSupport.h"

#include <drone_mapper/SimulationManager.h>

#include <gtest/gtest.h>

#include <fstream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>

using namespace dmtest;
using ::testing::_;
using ::testing::Invoke;
using ::testing::NiceMock;
using ::testing::Return;
using ::testing::Throw;

namespace {

t::SimulationResult resultWithScore(double score) {
    t::SimulationResult r;
    r.mission_score = score;
    r.mission_results.push_back(t::MissionRunResult{t::MissionRunStatus::Completed, 5, {}});
    return r;
}

// Composition: 1 simulation with N missions, D drones, L lidars.
t::SimulationCompositionData composition(std::size_t missions, std::size_t drones,
                                         std::size_t lidars) {
    t::SimulationConfigData sim;
    sim.map_filename = "scene.npy";
    std::vector<t::MissionConfigData> ms;
    for (std::size_t i = 0; i < missions; ++i) {
        t::MissionConfigData m;
        m.mission_bounds = bounds(0, 40, 0, 40, 0, 40);
        m.gps_resolution = g::plen(10);
        m.output_mapping_resolution_factor = 1;
        ms.push_back(m);
    }
    t::SimulationCompositionData comp;
    comp.simulation_mission_groups.emplace_back(sim, ms);
    comp.drones.assign(drones, t::DroneConfigData{g::plen(4), g::hang(90), g::plen(30), g::plen(20)});
    comp.lidars.assign(lidars, t::LidarConfigData{g::plen(20), g::plen(120), g::plen(2.5), 1});
    return comp;
}

std::filesystem::path tmpOut(const std::string& name) {
    const auto p = std::filesystem::temp_directory_path() / ("dm_sm_" + name);
    std::filesystem::remove_all(p);
    return p;
}

} // namespace

TEST(SimulationManager, RunsCartesianProduct) {
    auto factory = std::make_unique<NiceMock<MockSimulationRunFactory>>();
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .Times(2 * 2 * 2)
        .WillRepeatedly(Invoke([](auto&&...) {
            auto run = std::make_unique<NiceMock<MockSimulationRun>>();
            ON_CALL(*run, run()).WillByDefault(Return(resultWithScore(90.0)));
            return run;
        }));
    dm::SimulationManager manager{std::move(factory)};
    const auto report = manager.run(composition(2, 2, 2), tmpOut("cartesian"));
    EXPECT_EQ(report.runs.size(), 8u);
}

TEST(SimulationManager, FactoryFailureBecomesErrorResult) {
    auto factory = std::make_unique<NiceMock<MockSimulationRunFactory>>();
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .WillRepeatedly(Throw(std::runtime_error("map missing")));
    dm::SimulationManager manager{std::move(factory)};
    const auto report = manager.run(composition(1, 1, 1), tmpOut("fail"));
    ASSERT_EQ(report.runs.size(), 1u);
    EXPECT_DOUBLE_EQ(report.runs.front().mission_score, -1.0);
}

TEST(SimulationManager, ReportMetadataPopulated) {
    auto factory = std::make_unique<NiceMock<MockSimulationRunFactory>>();
    ON_CALL(*factory, create(_, _, _, _, _)).WillByDefault(Invoke([](auto&&...) {
        auto run = std::make_unique<NiceMock<MockSimulationRun>>();
        ON_CALL(*run, run()).WillByDefault(Return(resultWithScore(50.0)));
        return run;
    }));
    dm::SimulationManager manager{std::move(factory)};
    const auto report = manager.run(composition(1, 1, 1), tmpOut("meta"));
    EXPECT_EQ(report.metric, "output_map_accuracy");
    EXPECT_DOUBLE_EQ(std::get<0>(report.score_range), 0.0);
    EXPECT_DOUBLE_EQ(std::get<1>(report.score_range), 100.0);
    EXPECT_EQ(report.error_score, -1);
    EXPECT_FALSE(report.generated_at_utc.empty());
}

TEST(SimulationManager, WritesSimulationOutputFile) {
    auto factory = std::make_unique<NiceMock<MockSimulationRunFactory>>();
    ON_CALL(*factory, create(_, _, _, _, _)).WillByDefault(Invoke([](auto&&...) {
        auto run = std::make_unique<NiceMock<MockSimulationRun>>();
        ON_CALL(*run, run()).WillByDefault(Return(resultWithScore(75.0)));
        return run;
    }));
    dm::SimulationManager manager{std::move(factory)};
    const auto out = tmpOut("outfile");
    (void)manager.run(composition(1, 1, 1), out);
    EXPECT_TRUE(std::filesystem::exists(out / "simulation_output.yaml"));
    EXPECT_TRUE(std::filesystem::exists(out / "output_results" / "error_log.txt"));
}

TEST(SimulationManager, NullFactoryThrows) {
    EXPECT_THROW(dm::SimulationManager{nullptr}, std::invalid_argument);
}

TEST(SimulationManager, EmptyCompositionProducesNoRuns) {
    auto factory = std::make_unique<NiceMock<MockSimulationRunFactory>>();
    EXPECT_CALL(*factory, create(_, _, _, _, _)).Times(0);
    dm::SimulationManager manager{std::move(factory)};
    const auto report = manager.run(t::SimulationCompositionData{}, tmpOut("empty"));
    EXPECT_TRUE(report.runs.empty());
}

TEST(SimulationManager, PreservesRunResults) {
    auto factory = std::make_unique<NiceMock<MockSimulationRunFactory>>();
    double scores[] = {30.0, 60.0};
    int idx = 0;
    ON_CALL(*factory, create(_, _, _, _, _)).WillByDefault(Invoke([&](auto&&...) {
        auto run = std::make_unique<NiceMock<MockSimulationRun>>();
        ON_CALL(*run, run()).WillByDefault(Return(resultWithScore(scores[idx++ % 2])));
        return run;
    }));
    dm::SimulationManager manager{std::move(factory)};
    const auto report = manager.run(composition(2, 1, 1), tmpOut("preserve"));
    ASSERT_EQ(report.runs.size(), 2u);
    EXPECT_DOUBLE_EQ(report.runs[0].mission_score, 30.0);
    EXPECT_DOUBLE_EQ(report.runs[1].mission_score, 60.0);
}

TEST(SimulationManager, ContinuesAfterOneScenarioFails) {
    auto factory = std::make_unique<NiceMock<MockSimulationRunFactory>>();
    int calls = 0;
    ON_CALL(*factory, create(_, _, _, _, _)).WillByDefault(Invoke([&](auto&&...) {
        if (calls++ == 0) {
            throw std::runtime_error("boom");
        }
        auto run = std::make_unique<NiceMock<MockSimulationRun>>();
        ON_CALL(*run, run()).WillByDefault(Return(resultWithScore(80.0)));
        return std::unique_ptr<dm::ISimulationRun>(std::move(run));
    }));
    dm::SimulationManager manager{std::move(factory)};
    const auto report = manager.run(composition(2, 1, 1), tmpOut("continue"));
    ASSERT_EQ(report.runs.size(), 2u);
    EXPECT_DOUBLE_EQ(report.runs[0].mission_score, -1.0);
    EXPECT_DOUBLE_EQ(report.runs[1].mission_score, 80.0);
}

TEST(SimulationManager, ReportFileContainsRunEntries) {
    auto factory = std::make_unique<NiceMock<MockSimulationRunFactory>>();
    ON_CALL(*factory, create(_, _, _, _, _)).WillByDefault(Invoke([](auto&&...) {
        auto run = std::make_unique<NiceMock<MockSimulationRun>>();
        ON_CALL(*run, run()).WillByDefault(Return(resultWithScore(42.0)));
        return run;
    }));
    dm::SimulationManager manager{std::move(factory)};
    const auto out = tmpOut("entries");
    (void)manager.run(composition(1, 1, 1), out);

    std::ifstream in(out / "simulation_output.yaml");
    const std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    EXPECT_NE(content.find("score_report"), std::string::npos);
    EXPECT_NE(content.find("runs:"), std::string::npos);
    EXPECT_NE(content.find("42"), std::string::npos);
}

TEST(SimulationManager, EachCombinationCreatesOneRun) {
    auto factory = std::make_unique<NiceMock<MockSimulationRunFactory>>();
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .Times(3) // 1 sim * 3 missions * 1 drone * 1 lidar
        .WillRepeatedly(Invoke([](auto&&...) {
            auto run = std::make_unique<NiceMock<MockSimulationRun>>();
            ON_CALL(*run, run()).WillByDefault(Return(resultWithScore(10.0)));
            return run;
        }));
    dm::SimulationManager manager{std::move(factory)};
    const auto report = manager.run(composition(3, 1, 1), tmpOut("combos"));
    EXPECT_EQ(report.runs.size(), 3u);
}

TEST(SimulationManager, SummaryAverageReflectsScores) {
    auto factory = std::make_unique<NiceMock<MockSimulationRunFactory>>();
    double scores[] = {30.0, 60.0};
    int idx = 0;
    ON_CALL(*factory, create(_, _, _, _, _)).WillByDefault(Invoke([&](auto&&...) {
        auto run = std::make_unique<NiceMock<MockSimulationRun>>();
        ON_CALL(*run, run()).WillByDefault(Return(resultWithScore(scores[idx++ % 2])));
        return run;
    }));
    dm::SimulationManager manager{std::move(factory)};
    const auto out = tmpOut("avg");
    (void)manager.run(composition(2, 1, 1), out);
    std::ifstream in(out / "simulation_output.yaml");
    const std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    EXPECT_NE(content.find("scored_runs: 2"), std::string::npos);
    EXPECT_NE(content.find("average_score: 45"), std::string::npos);
}

TEST(SimulationManager, AllErrorRunsReportedInSummary) {
    auto factory = std::make_unique<NiceMock<MockSimulationRunFactory>>();
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .WillRepeatedly(Throw(std::runtime_error("nope")));
    dm::SimulationManager manager{std::move(factory)};
    const auto out = tmpOut("allerr");
    const auto report = manager.run(composition(2, 1, 1), out);
    std::ifstream in(out / "simulation_output.yaml");
    const std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    EXPECT_NE(content.find("error_runs: 2"), std::string::npos);
    EXPECT_NE(content.find("scored_runs: 0"), std::string::npos);
}
