#include <drone_mapper/SimulationRunFactoryImpl.h>

#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MapBuilder.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/SimulationRunImpl.h>

#include <iomanip>
#include <memory>
#include <sstream>

namespace drone_mapper {

std::unique_ptr<ISimulationRun>
SimulationRunFactoryImpl::create(const types::SimulationConfigData& simulation,
                                 const types::MissionConfigData& mission,
                                 const types::DroneConfigData& drone,
                                 const types::LidarConfigData& lidar,
                                 const std::filesystem::path& output_path) {
    // Hidden truth map from the .npy file (throws if the file cannot be read).
    std::shared_ptr<NpyArray> hidden_array = loadHiddenMapArray(simulation.map_filename);
    const types::MapConfig hidden_config = makeHiddenMapConfig(simulation, *hidden_array);
    auto hidden_map = std::make_unique<Map3DImpl>(hidden_array, hidden_config);

    // Output map sized to the mission bounds at the decided output resolution.
    types::MapConfig output_config = makeOutputMapConfig(mission);
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
        makeOccupancyGrid(output_config, types::VoxelOccupancy::Unmapped);
    auto output_map = std::make_unique<Map3DImpl>(output_array, output_config);

    auto gps = std::make_unique<MockGPS>(
        simulation.initial_drone_position,
        Orientation{simulation.initial_angle, 0.0 * altitude_angle[deg]},
        mission.gps_resolution);
    auto movement = std::make_unique<MockMovement>(*gps);
    auto lidar_impl = std::make_unique<MockLidar>(lidar, *hidden_map, *gps);
    auto mapping_algorithm = std::make_unique<MappingAlgorithmImpl>(mission, lidar, drone, *output_map);

    auto drone_control = std::make_unique<DroneControlImpl>(
        drone, mission, *lidar_impl, *gps, *movement, *output_map, *mapping_algorithm);

    std::ostringstream name;
    name << "output_map_" << std::setfill('0') << std::setw(4) << next_index_++ << ".npy";
    const std::filesystem::path output_map_file = output_path / "output_results" / name.str();

    auto mission_control = std::make_unique<MissionControlImpl>(
        mission, drone, *hidden_map, *output_map, *drone_control, output_map_file);

    return std::make_unique<SimulationRunImpl>(
        std::move(hidden_map),
        std::move(output_map),
        std::move(gps),
        std::move(movement),
        std::move(lidar_impl),
        std::move(mapping_algorithm),
        std::move(drone_control),
        std::move(mission_control),
        simulation,
        mission,
        output_map_file);
}

} // namespace drone_mapper
