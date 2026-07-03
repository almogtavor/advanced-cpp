#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MapBuilder.h>
#include <drone_mapper/MapGeometry.h>
#include <drone_mapper/MapsComparison.h>

#include <yaml-cpp/yaml.h>

#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace dm = drone_mapper;

namespace {

// Geometry a map is compared with. Missing pieces fall back to defaults so a
// bare "compare these two files" invocation works with no config.
struct MapGeometryOverride {
    double res_cm = 1.0;
    dm::Position3D offset{};
    std::optional<dm::types::MappingBounds> bounds{};
};

// Builds a Map3DImpl over an on-disk .npy using the given geometry; bounds
// default to the full loaded extent when not overridden.
[[nodiscard]] std::unique_ptr<dm::Map3DImpl> makeMap(const std::filesystem::path& path,
                                                     const MapGeometryOverride& geo) {
    std::shared_ptr<NpyArray> array = dm::loadHiddenMapArray(path);
    const NpyArray::shape_t& shape = array->Shape();
    const double nx = static_cast<double>(shape[0]);
    const double ny = static_cast<double>(shape[1]);
    const double nz = static_cast<double>(shape[2]);
    const double ox = dm::geom::xcm(geo.offset.x);
    const double oy = dm::geom::ycm(geo.offset.y);
    const double oz = dm::geom::zcm(geo.offset.z);

    dm::types::MapConfig cfg;
    cfg.resolution = dm::geom::plen(geo.res_cm);
    cfg.offset = geo.offset;
    cfg.boundaries = geo.bounds.value_or(dm::types::MappingBounds{
        dm::geom::xlen(ox), dm::geom::xlen(ox + nx * geo.res_cm),
        dm::geom::ylen(oy), dm::geom::ylen(oy + ny * geo.res_cm),
        dm::geom::zlen(oz), dm::geom::zlen(oz + nz * geo.res_cm),
    });
    return std::make_unique<dm::Map3DImpl>(array, cfg);
}

[[nodiscard]] MapGeometryOverride parseSection(const YAML::Node& section) {
    MapGeometryOverride geo;
    if (!section) {
        return geo;
    }
    if (section["map_res_cm"]) {
        geo.res_cm = section["map_res_cm"].as<double>();
    }
    if (const YAML::Node off = section["map_offset"]) {
        geo.offset = dm::Position3D{
            dm::geom::xlen(off["x_offset"] ? off["x_offset"].as<double>() : 0.0),
            dm::geom::ylen(off["y_offset"] ? off["y_offset"].as<double>() : 0.0),
            dm::geom::zlen(off["height_offset"] ? off["height_offset"].as<double>() : 0.0),
        };
    }
    if (const YAML::Node b = section["map_boundaries"]) {
        const YAML::Node x = b["x_boundary"];
        const YAML::Node y = b["y_boundary"];
        const YAML::Node h = b["height_boundary"];
        if (x && y && h) {
            geo.bounds = dm::types::MappingBounds{
                dm::geom::xlen(x["min_cm"].as<double>()), dm::geom::xlen(x["max_cm"].as<double>()),
                dm::geom::ylen(y["min_cm"].as<double>()), dm::geom::ylen(y["max_cm"].as<double>()),
                dm::geom::zlen(h["min_cm"].as<double>()), dm::geom::zlen(h["max_cm"].as<double>()),
            };
        }
    }
    return geo;
}

// Accepts either "comparison_config=<path>" or a bare "<path>".
[[nodiscard]] std::filesystem::path configPathArg(const std::string& arg) {
    const std::string prefix = "comparison_config=";
    if (arg.rfind(prefix, 0) == 0) {
        return arg.substr(prefix.size());
    }
    return arg;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 3 || argc > 4) {
        std::cout << "-1\n";
        std::cerr << "Usage: maps_comparison <origin_map> <target_map> [comparison_config=<path>]\n";
        return 1;
    }

    try {
        MapGeometryOverride origin_geo;
        MapGeometryOverride target_geo;
        if (argc == 4) {
            const YAML::Node root = YAML::LoadFile(configPathArg(argv[3]).string());
            const YAML::Node cfg = root["comparison_config"] ? root["comparison_config"] : root;
            origin_geo = parseSection(cfg["original"]);
            target_geo = parseSection(cfg["target"]);
        }

        const std::unique_ptr<dm::Map3DImpl> origin = makeMap(argv[1], origin_geo);
        const std::unique_ptr<dm::Map3DImpl> target = makeMap(argv[2], target_geo);

        const std::vector<double> scores =
            dm::MapsComparison::compare(*origin, std::vector<dm::IMap3D*>{target.get()});
        std::cout << (scores.empty() ? -1.0 : scores.front()) << '\n';
        return 0;
    } catch (const std::exception& ex) {
        std::cout << "-1\n";
        std::cerr << "maps_comparison error: " << ex.what() << '\n';
        return 1;
    }
}
