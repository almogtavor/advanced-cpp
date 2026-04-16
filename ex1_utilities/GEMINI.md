# Drone Mapper - Gemini Context

This project is a C++20 baseline implementation of a drone mapping simulator. It simulates a drone exploring a 3D grid world using mock sensors and a deterministic mapping algorithm.

## Project Overview

- **Language & Standards:** C++20, CMake 3.20+.
- **Architecture:** The project is divided into a library (`drone_mapper_lib`) and an executable (`drone_mapper`). It uses abstract interfaces to decouple the autonomous mapping algorithm from the simulator's hardware mocks.
- **World Model:** The world is discretized into a 1m (100cm) grid. Lidar scans and movements are cell-based.
- **Core Components:**
    - `Simulator`: Orchestrates input loading, world creation, algorithm execution, and scoring.
    - `DroneAlgorithm`: Implements a deterministic Depth-First Search (DFS) for autonomous exploration.
    - `Mocks`: Simulated hardware (Lidar, Position Sensor, Movement Driver) and the "hidden" ground-truth world.
    - `BuildingMap`: Represents the grid discovered and mapped by the drone.
    - `FileParsers`: Handles parsing of configuration and map files with recoverable error support.

## Building and Running

### Build Commands
```bash
cmake -S . -B build
cmake --build build
```

### Running the Simulator
```bash
# Usage: ./build/drone_mapper [input_directory]
# Example using provided samples:
./build/drone_mapper ./samples
```

### Running Tests
The project uses a custom minimal test framework.
```bash
ctest --test-dir build --output-on-failure
# Or run the test executable directly:
./build/drone_mapper_tests
```

## Development Conventions

- **Interfaces:** Always program against the interfaces defined in `include/drone_mapper/Interfaces.hpp` when modifying the mapping algorithm.
- **Types and Units:** Use the domain types in `Types.hpp` and unit wrappers in `Units.hpp`. Note that `Units.hpp` currently contains local strong wrappers (`DistanceCm`, `AngleDeg`) as a baseline.
- **Error Handling:** 
    - **Recoverable:** Log to `ErrorList`; these are written to `input_errors.txt` and do not stop execution.
    - **Unrecoverable:** Throw exceptions; these are caught by the simulator to return a failure result gracefully.
- **Testing:** Add new tests in `tests/` using the `DM_TEST` macro. Ensure `DM_ASSERT_TRUE` or `DM_ASSERT_EQ` are used for validations.
- **Grid Scaling:** Internal logic assumes 1 grid cell = 100cm.

## Key Files
- `LOGICAL_FLOW.md`: Detailed architectural documentation and execution flow.
- `include/drone_mapper/Interfaces.hpp`: Core abstractions for hardware and algorithms.
- `include/drone_mapper/Types.hpp`: Central domain model and data structures.
- `src/DroneAlgorithm.cpp`: The primary location for mapping logic.
- `samples/`: Contains example configuration and map files.
