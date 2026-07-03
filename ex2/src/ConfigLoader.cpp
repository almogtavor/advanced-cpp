#include <drone_mapper/ConfigLoader.h>

#include <drone_mapper/MapGeometry.h>

#include <yaml-cpp/yaml.h>

#include <stdexcept>

namespace drone_mapper::config {

namespace {

// Fetches a required child node, throwing a clear error when it is absent.
[[nodiscard]] YAML::Node require(const YAML::Node& parent, const std::string& key,
                                 const std::string& context) {
    const YAML::Node child = parent[key];
    if (!child) {
        throw std::runtime_error("Config error: missing key '" + key + "' in " + context);
    }
    return child;
}

[[nodiscard]] double reqNum(const YAML::Node& parent, const std::string& key,
                            const std::string& context) {
    return require(parent, key, context).as<double>();
}

[[nodiscard]] types::MappingBounds parseBounds(const YAML::Node& node, const std::string& context) {
    const YAML::Node x = require(node, "x_boundary", context);
    const YAML::Node y = require(node, "y_boundary", context);
    const YAML::Node h = require(node, "height_boundary", context);
    return types::MappingBounds{
        geom::xlen(reqNum(x, "min_cm", context)), geom::xlen(reqNum(x, "max_cm", context)),
        geom::ylen(reqNum(y, "min_cm", context)), geom::ylen(reqNum(y, "max_cm", context)),
        geom::zlen(reqNum(h, "min_cm", context)), geom::zlen(reqNum(h, "max_cm", context)),
    };
}

[[nodiscard]] types::DroneConfigData parseDroneNode(const YAML::Node& root) {
    const YAML::Node node = require(root, "drone_config", "drone config");
    const double diameter = reqNum(node, "dimensions_cm", "drone_config");
    return types::DroneConfigData{
        geom::plen(diameter / 2.0), // dimensions_cm is a diameter; store radius
        geom::hang(reqNum(node, "max_rotate_deg", "drone_config")),
        geom::plen(reqNum(node, "max_advance_cm", "drone_config")),
        geom::plen(reqNum(node, "max_elevate_cm", "drone_config")),
    };
}

[[nodiscard]] types::LidarConfigData parseLidarNode(const YAML::Node& root) {
    const YAML::Node node = require(root, "lidar_config", "lidar config");
    return types::LidarConfigData{
        geom::plen(reqNum(node, "z_min_cm", "lidar_config")),
        geom::plen(reqNum(node, "z_max_cm", "lidar_config")),
        geom::plen(reqNum(node, "d_cm", "lidar_config")),
        require(node, "fov_circles", "lidar_config").as<std::size_t>(),
    };
}

[[nodiscard]] types::MissionConfigData parseMissionNode(const YAML::Node& root) {
    const YAML::Node node = require(root, "mission_config", "mission config");
    types::MissionConfigData mission;
    mission.max_steps = require(node, "max_steps", "mission_config").as<std::size_t>();
    mission.gps_resolution = geom::plen(reqNum(node, "gps_resolution_cm", "mission_config"));
    mission.mission_bounds = parseBounds(require(node, "boundaries", "mission_config"), "mission_config.boundaries");
    // Optional: defaults to 1 when absent.
    mission.output_mapping_resolution_factor =
        node["output_mapping_resolution_factor"] ? node["output_mapping_resolution_factor"].as<double>() : 1.0;
    return mission;
}

[[nodiscard]] types::SimulationConfigData parseSimulationNode(const YAML::Node& root) {
    const YAML::Node node = require(root, "simulation_config", "simulation config");
    types::SimulationConfigData sim;
    sim.map_filename = require(node, "map_filename", "simulation_config").as<std::string>();
    sim.map_resolution = geom::plen(reqNum(node, "map_resolution_cm", "simulation_config"));

    const YAML::Node pos = require(node, "initial_drone_position", "simulation_config");
    sim.initial_drone_position = Position3D{
        geom::xlen(reqNum(pos, "x_cm", "initial_drone_position")),
        geom::ylen(reqNum(pos, "y_cm", "initial_drone_position")),
        geom::zlen(reqNum(pos, "height_cm", "initial_drone_position")),
    };
    sim.initial_angle = geom::hang(reqNum(node, "initial_angle_deg", "simulation_config"));

    // Offset is optional; absent means the npy origin sits at world (0,0,0).
    if (node["map_axes_offset"]) {
        const YAML::Node off = node["map_axes_offset"];
        sim.map_offset = Position3D{
            geom::xlen(reqNum(off, "x_offset", "map_axes_offset")),
            geom::ylen(reqNum(off, "y_offset", "map_axes_offset")),
            geom::zlen(reqNum(off, "height_offset", "map_axes_offset")),
        };
    }
    return sim;
}

} // namespace

types::DroneConfigData parseDroneConfig(const std::string& yaml_text) {
    return parseDroneNode(YAML::Load(yaml_text));
}
types::LidarConfigData parseLidarConfig(const std::string& yaml_text) {
    return parseLidarNode(YAML::Load(yaml_text));
}
types::MissionConfigData parseMissionConfig(const std::string& yaml_text) {
    return parseMissionNode(YAML::Load(yaml_text));
}
types::SimulationConfigData parseSimulationConfig(const std::string& yaml_text) {
    return parseSimulationNode(YAML::Load(yaml_text));
}

types::DroneConfigData loadDroneConfig(const std::filesystem::path& file) {
    return parseDroneNode(YAML::LoadFile(file.string()));
}
types::LidarConfigData loadLidarConfig(const std::filesystem::path& file) {
    return parseLidarNode(YAML::LoadFile(file.string()));
}
types::MissionConfigData loadMissionConfig(const std::filesystem::path& file) {
    return parseMissionNode(YAML::LoadFile(file.string()));
}
types::SimulationConfigData loadSimulationConfig(const std::filesystem::path& file) {
    return parseSimulationNode(YAML::LoadFile(file.string()));
}

types::SimulationCompositionData loadComposition(const std::filesystem::path& file) {
    const YAML::Node root = YAML::LoadFile(file.string());
    const YAML::Node comp = require(root, "simulation_compositions", "composition");
    const std::filesystem::path base = std::filesystem::absolute(file).parent_path();

    auto resolve = [&](const std::string& rel) {
        std::filesystem::path p{rel};
        return p.is_absolute() ? p : (base / p);
    };

    types::SimulationCompositionData data;
    data.composition_file = file;

    for (const YAML::Node& entry : require(comp, "simulations", "simulation_compositions")) {
        types::SimulationConfigData sim =
            loadSimulationConfig(resolve(require(entry, "simulation_config", "simulations entry").as<std::string>()));
        // Make the map path absolute relative to the composition directory.
        sim.map_filename = resolve(sim.map_filename.string());

        std::vector<types::MissionConfigData> missions;
        for (const YAML::Node& m : require(entry, "mission_configs", "simulations entry")) {
            missions.push_back(loadMissionConfig(resolve(m.as<std::string>())));
        }
        data.simulation_mission_groups.emplace_back(std::move(sim), std::move(missions));
    }

    for (const YAML::Node& d : require(comp, "drone_configs", "simulation_compositions")) {
        data.drones.push_back(loadDroneConfig(resolve(d.as<std::string>())));
    }
    for (const YAML::Node& l : require(comp, "lidar_configs", "simulation_compositions")) {
        data.lidars.push_back(loadLidarConfig(resolve(l.as<std::string>())));
    }
    return data;
}

} // namespace drone_mapper::config
