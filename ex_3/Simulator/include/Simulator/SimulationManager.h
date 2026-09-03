#pragma once

// Runs one plugin pair over a whole composition: every
// (simulation x mission x drone x lidar) combination, optionally across worker
// threads, returning the assignment-2 style report for that plugin.
//
// This is the implementation of the provided ISimulation interface. Sweep calls
// it once per plugin and turns the reports into the assignment-3 summary.

#include <Simulator/ISimulation.h>
#include <Simulator/ISimulationRunFactory.h>
#include <Simulator/SimulationTypes.h>

#include <functional>
#include <memory>
#include <string>

namespace simulator {

using namespace common;

class SimulationManager final : public ISimulation {
public:
    // Each run needs its own factory so the output map filename can identify
    // the run, and so no state is shared between worker threads. The maker is
    // handed a label of the form "sim0_mission1_drone0_lidar1".
    using RunFactoryMaker =
        std::function<std::unique_ptr<ISimulationRunFactory>(const std::string& run_label)>;

    SimulationManager(RunFactoryMaker make_run_factory, int num_threads);

    // Never throws: a run that fails becomes an errored entry in the report.
    [[nodiscard]] types::SimulationManagerReport run(
        const types::SimulationCompositionData& composition,
        const std::filesystem::path& output_path) override;

private:
    RunFactoryMaker make_run_factory_;
    int num_threads_ = 1;
};

} // namespace simulator
