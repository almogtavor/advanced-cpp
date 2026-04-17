#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "config/DroneConfig.h"
#include "config/MissionConfig.h"
#include "io/ConfigParser.h"
#include "io/MapIO.h"
#include "simulator/Simulator.h"
#include "util/Logger.h"

namespace fs = std::filesystem;

namespace {

void write_input_errors(const fs::path& dir,
                        const std::vector<std::string>& errors) {
    if (errors.empty()) return;
    std::ofstream out(dir / "input_errors.txt");
    if (!out) return;
    out << "# Recovered input errors\n";
    for (const auto& e : errors) out << e << "\n";
}

int run(const fs::path& base_dir) {
    auto& logger = drone::Logger::instance();
    logger.set_file((base_dir / "drone_mapper.log").string());
    logger.set_level(drone::LogLevel::Debug);

    LOG_INFO("Drone Mapper starting, base_dir=" + base_dir.string());

    drone::DroneConfig drone_cfg;
    drone::MissionConfig mission;
    drone::BuildingTruth truth;

    std::vector<std::string> all_errors;

    LOG_INFO("Loading drone_config.txt");
    auto rd = drone::ConfigParser::load_drone_config(
        (base_dir / "drone_config.txt").string(), drone_cfg);
    for (auto& e : rd.errors) {
        LOG_WARNING("drone_config: " + e);
        all_errors.push_back(std::move(e));
    }
    if (!rd.ok) {
        LOG_ERROR("Cannot load drone_config.txt - aborting");
        std::cerr << "FATAL: cannot load drone_config.txt from "
                  << base_dir << "\n";
        return 1;
    }

    LOG_INFO("Loading mission_config.txt");
    auto rm = drone::ConfigParser::load_mission_config(
        (base_dir / "mission_config.txt").string(), mission);
    for (auto& e : rm.errors) {
        LOG_WARNING("mission_config: " + e);
        all_errors.push_back(std::move(e));
    }
    if (!rm.ok) {
        LOG_ERROR("Cannot load mission_config.txt - aborting");
        std::cerr << "FATAL: cannot load mission_config.txt from "
                  << base_dir << "\n";
        return 1;
    }

    LOG_INFO("Loading map_input.txt");
    auto rmap = drone::MapIO::load_truth(
        (base_dir / "map_input.txt").string(), truth);
    for (auto& e : rmap.errors) {
        LOG_WARNING("map_input: " + e);
        all_errors.push_back(std::move(e));
    }
    if (!rmap.ok) {
        LOG_ERROR("Cannot load map_input.txt - aborting");
        std::cerr << "FATAL: cannot load map_input.txt from "
                  << base_dir << "\n";
        return 1;
    }

    write_input_errors(base_dir, all_errors);

    LOG_INFO("Starting simulation");
    drone::Simulator sim(std::move(truth), drone_cfg, mission);
    const auto report = sim.run();

    auto save_res = drone::MapIO::save_map(
        (base_dir / "map_output.txt").string(), sim.known_map());
    if (!save_res.ok) {
        std::cerr << "ERROR: cannot write map_output.txt to "
                  << base_dir << "\n";
        return 1;
    }
    // Also write under the alternative name from the submission guidelines.
    drone::MapIO::save_map(
        (base_dir / "output_map.txt").string(), sim.known_map());

    LOG_INFO("Simulation complete, score=" + std::to_string(report.score) +
             " commands=" + std::to_string(report.command_count) +
             " collided=" + (report.drone_collided ? "yes" : "no"));

    std::cout << "Drone Mapper finished\n";
    std::cout << "  commands issued : " << report.command_count        << "\n";
    std::cout << "  in-bounds cells : " << report.total_in_bounds_cells << "\n";
    std::cout << "  correctly mapped: " << report.correct_cells        << "\n";
    std::cout << "  incorrectly classified: " << report.incorrect_cells << "\n";
    std::cout << "  unmapped (unreachable): " << report.unmapped_cells << "\n";
    std::cout << "  collisions      : " << (report.drone_collided ? "yes" : "no") << "\n";
    std::cout << "  score           : " << report.score << " / 100\n";
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    int rc = 0;
    try {
        fs::path dir = fs::current_path();
        if (argc >= 2) dir = fs::path(argv[1]);
        if (!fs::exists(dir) || !fs::is_directory(dir)) {
            std::cerr << "FATAL: input_output_files_path is not a directory: "
                      << dir << "\n";
            return 1;
        }
        rc = run(dir);
    } catch (const std::exception& e) {
        std::cerr << "FATAL: unhandled exception: " << e.what() << "\n";
        rc = 1;
    } catch (...) {
        std::cerr << "FATAL: unknown exception\n";
        rc = 1;
    }
    return rc;
}
