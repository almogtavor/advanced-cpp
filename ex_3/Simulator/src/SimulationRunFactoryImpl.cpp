#include <Simulator/SimulationRunFactoryImpl.h>
#include <Simulator/SimulationTypes.h>

#include <Simulator/Map3DImpl.h>
#include <Simulator/MapBuilder.h>
#include <Simulator/MockGPS.h>
#include <Simulator/MockLidar.h>
#include <Simulator/MockMovement.h>
#include <Simulator/SimulationRunImpl.h>

#include <stdexcept>
#include <utility>
#include <memory>
#include <sstream>

namespace simulator {

using namespace common;

SimulationRunFactoryImpl::SimulationRunFactoryImpl(MappingAlgorithmFactory algorithm_factory,
                                                   MissionControlFactory mission_control_factory,
                                                   std::string run_label,
                                                   bool verbose)
    : algorithm_factory_(std::move(algorithm_factory)),
      mission_control_factory_(std::move(mission_control_factory)),
      run_label_(std::move(run_label)),
      verbose_(verbose) {
    if (!algorithm_factory_ || !mission_control_factory_) {
        throw std::invalid_argument("SimulationRunFactoryImpl requires both plugin factories.");
    }
}

std::unique_ptr<ISimulationRun>
SimulationRunFactoryImpl::create(const types::SimulationConfigData& simulation,
                                 const common::types::MissionConfigData& mission,
                                 const common::types::DroneConfigData& drone,
                                 const common::types::LidarConfigData& lidar,
                                 const std::filesystem::path& output_path) {
    // Hidden truth map from the .npy file (throws if the file cannot be read).
    std::shared_ptr<NpyArray> hidden_array = loadHiddenMapArray(simulation.map_filename);
    const common::types::MapConfig hidden_config = makeHiddenMapConfig(simulation, *hidden_array);
    auto hidden_map = std::make_unique<Map3DImpl>(hidden_array, hidden_config);

    // Output map sized to the mission bounds at the decided output resolution.
    common::types::MapConfig output_config = makeOutputMapConfig(mission);
    // Never map finer than the source map: a sub-map-resolution output cell can
    // sit inside a coarser occupied voxel that the finer output never fully
    // captured, so the drone could plan its centre into a wall. Clamping the
    // output resolution to at least the input map resolution keeps output cells
    // aligned to the hidden map and costs nothing in accuracy (the comparison
    // samples at the map resolution).
    if (output_config.resolution < simulation.map_resolution) {
        output_config.resolution = simulation.map_resolution;
    }
    std::shared_ptr<NpyArray> output_array =
        makeOccupancyGrid(output_config, common::types::VoxelOccupancy::Unmapped);
    auto output_map = std::make_unique<Map3DImpl>(output_array, output_config);

    auto gps = std::make_unique<MockGPS>(
        simulation.initial_drone_position,
        Orientation{simulation.initial_angle, 0.0 * altitude_angle[deg]},
        mission.gps_resolution);
    auto movement = std::make_unique<MockMovement>(*gps, *hidden_map);
    auto lidar_impl = std::make_unique<MockLidar>(lidar, *hidden_map, *gps);

    // Built through the plugin's registered factory rather than by name.
    std::unique_ptr<IMappingAlgorithm> mapping_algorithm =
        algorithm_factory_(MappingAlgorithmDependencies{mission, lidar, drone, *output_map});
    if (!mapping_algorithm) {
        throw std::runtime_error("Mapping algorithm factory returned no instance.");
    }

    const std::filesystem::path output_map_file =
        output_path / ("output_map_" + run_label_ + ".npy");

    std::unique_ptr<IMissionControl> mission_control =
        mission_control_factory_(MissionControlDependencies{mission,
                                                            drone,
                                                            *lidar_impl,
                                                            *gps,
                                                            *movement,
                                                            *output_map,
                                                            *mapping_algorithm,
                                                            output_map_file,
                                                            verbose_});
    if (!mission_control) {
        throw std::runtime_error("Mission control factory returned no instance.");
    }

    return std::make_unique<SimulationRunImpl>(
        std::move(hidden_map),
        std::move(output_map),
        std::move(gps),
        std::move(movement),
        std::move(lidar_impl),
        std::move(mapping_algorithm),
        std::move(mission_control),
        simulation,
        mission,
        output_map_file);
}

} // namespace simulator
