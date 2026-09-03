#pragma once

#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>
#include <Simulator/ISimulationRunFactory.h>
#include <Simulator/SimulationTypes.h>

#include <cstddef>

namespace simulator {

using namespace common;

// Changed from assignment 2: the algorithm and the mission control are no
// longer constructed directly. They arrive as factories, registered by the
// plugin .so files when the simulator dlopen'd them, so this factory never
// names a concrete plugin class.
class SimulationRunFactoryImpl final : public ISimulationRunFactory {
public:
    SimulationRunFactoryImpl(MappingAlgorithmFactory algorithm_factory,
                             MissionControlFactory mission_control_factory,
                             bool verbose = false);

    [[nodiscard]] std::unique_ptr<ISimulationRun>
    create(const types::SimulationConfigData& simulation,
           const common::types::MissionConfigData& mission,
           const common::types::DroneConfigData& drone,
           const common::types::LidarConfigData& lidar,
           const std::filesystem::path& output_path) override;

private:
    MappingAlgorithmFactory algorithm_factory_;
    MissionControlFactory mission_control_factory_;
    bool verbose_ = false;
    // Sequence number used to give each run a unique output map filename.
    std::size_t next_index_ = 0;
};

} // namespace simulator
