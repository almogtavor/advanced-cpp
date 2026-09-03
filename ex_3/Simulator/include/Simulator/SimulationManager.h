#pragma once

#include <Simulator/ISimulation.h>
#include <Simulator/SimulationTypes.h>
#include <Simulator/ISimulationRunFactory.h>

#include <memory>

namespace simulator {

using namespace common;

class SimulationManager final : public ISimulation {
public:
    explicit SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory);

    // Changed: matches ISimulation's new SimulationManagerReport return type.
    [[nodiscard]] types::SimulationManagerReport run(const types::SimulationCompositionData& composition,
                                              const std::filesystem::path& output_path) override; // output - to save the output map for example

private:
    std::unique_ptr<ISimulationRunFactory> run_factory_;
};

} // namespace simulator
