#include <drone_mapper/ConfigLoader.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>

#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>

// Usage: ./drone_mapper_simulation [<simulation.yaml>] [<output_path>]
//   - missing composition arg   -> "simulation.yaml" in the current directory
//   - relative path             -> resolved against the current directory
//   - absolute path             -> used as-is
int main(int argc, char** argv) {
    const std::filesystem::path composition_file =
        (argc >= 2) ? std::filesystem::path{argv[1]} : std::filesystem::path{"simulation.yaml"};
    const std::filesystem::path output_path =
        (argc >= 3) ? std::filesystem::path{argv[2]} : std::filesystem::current_path();

    try {
        const drone_mapper::types::SimulationCompositionData composition =
            drone_mapper::config::loadComposition(composition_file);

        auto run_factory = std::make_unique<drone_mapper::SimulationRunFactoryImpl>();
        drone_mapper::SimulationManager simulation{std::move(run_factory)};
        const drone_mapper::types::SimulationManagerReport report =
            simulation.run(composition, output_path);

        std::cout << "Completed " << report.runs.size() << " simulation run(s).\n"
                  << "Report written to " << (output_path / "simulation_output.yaml").string() << '\n';
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "Fatal error: " << ex.what() << '\n';
        return 1;
    }
}
