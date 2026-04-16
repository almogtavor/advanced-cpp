# Drone Mapper – Logical Flow

## 1. Program entry
`src/main.cpp` is the executable entry point.

Flow:
1. Read the optional command-line argument.
2. Resolve the working directory:
   - if an argument was provided, use it as the input/output directory.
   - otherwise use the current working directory.
3. Call `dm::run_simulation(directory)`.
4. Print either:
   - success message with the mapping score, or
   - failure message with the unrecoverable error text.

## 2. Top-level simulation orchestration
`src/Simulator.cpp` owns the main orchestration.

`run_simulation()` performs these steps in order:
1. Load and parse all input files by calling `load_input_directory()`.
2. If parsing had recoverable problems, write them into `input_errors.txt`.
3. Build the hidden simulator world (`MockWorld`).
4. Initialize the mutable drone runtime state:
   - current `Position`
   - current `heading`
5. Build the public interfaces that the algorithm sees:
   - `MockPositionSensor`
   - `MockMovementDriver`
   - `MockLidarSensor`
   - `SparseBuildingMap`
6. Create the autonomous algorithm object: `SimpleDroneAlgorithm`.
7. Run the algorithm.
8. Serialize the map discovered by the algorithm.
9. Write the discovered map into `map_output.txt`.
10. Compare the discovered map against the hidden input world and compute a score.
11. Return a `SimulationResult`.

If any unrecoverable exception happens anywhere in this sequence, the function catches it and converts it into a failure result instead of crashing the program.

## 3. Input loading and validation
`src/Config.cpp` and `src/FileParsers.cpp` are responsible for input handling.

### 3.1 `load_input_directory()`
This function loads three required files from the chosen directory:
- `drone_config.txt`
- `mission_config.txt`
- `map_input.txt`

It returns a single `LoadedInput` struct that contains:
- parsed drone capabilities
- parsed mission boundaries
- parsed world description
- list of recovered parsing errors

### 3.2 `parse_key_value_file()`
Used for:
- `drone_config.txt`
- `mission_config.txt`

It:
1. opens the file
2. reads line by line
3. strips comments starting with `#`
4. trims whitespace
5. expects `key=value`
6. stores valid pairs in an unordered map
7. records malformed lines as recoverable errors

### 3.3 `parse_drone_config()`
Builds a typed `DroneCapabilities` object.

For each expected key:
- if the key exists and is valid, use it
- if missing or invalid, use a default value and append a recovery message to `ErrorList`

### 3.4 `parse_mission_config()`
Builds a typed `MissionConfig` object in the same style as the drone parser.

### 3.5 `parse_world_map()`
Parses the hidden environment file.

Expected format:
- one `size=x,y,z` line
- one `start=x,y,z` line
- then any number of occupied cell lines as `x,y,z`

This parser:
1. validates that size exists and is positive
2. validates that start is well-formed
3. collects occupied cells
4. treats malformed occupied-cell lines as recoverable errors
5. treats missing/invalid size or start as unrecoverable errors

## 4. Hidden world vs exposed map
There are two completely different map concepts in the program.

### 4.1 Hidden world: `MockWorld`
Defined in `include/drone_mapper/Mocks.hpp` and implemented in `src/Mocks.cpp`.

Purpose:
- represent the real building given by `map_input.txt`
- used only by simulator internals
- never exposed directly to the drone algorithm

Responsibilities:
- check whether a position is inside world bounds
- check whether a position is occupied
- check whether a position is empty

Implementation note:
occupied cells are stored in an `unordered_set<Position>` so collision and lidar queries stay fast.

### 4.2 Exposed discovered map: `SparseBuildingMap`
Defined in `include/drone_mapper/BuildingMap.hpp` and implemented in `src/BuildingMap.cpp`.

Purpose:
- represent only what the algorithm has discovered
- this is the only map the algorithm is allowed to write to and read from

Responsibilities:
- reject writes outside mission bounds
- return `CellState::OutOfBounds` for out-of-bounds reads
- store states in a dense flattened vector
- export the dense vector for scoring/output

## 5. Mock hardware layer
The simulator provides fake hardware components through interfaces.

### 5.1 `MockPositionSensor`
Reads the shared current `Position`.

### 5.2 `MockMovementDriver`
Applies movement commands to shared simulator state.

Behavior:
- clamps movement requests to configured max limits
- converts centimeters to grid-cell steps
- checks destination against the hidden world
- refuses illegal moves by returning `false`
- updates heading on rotation requests

Important simplification in this implementation:
- heading is effectively treated as one of the four cardinal horizontal directions
- movement is cell-based, not continuous

### 5.3 `MockLidarSensor`
Simulates a ray cast through the hidden world.

Behavior:
1. start from current drone position
2. move one grid cell at a time in the requested direction
3. stop if an occupied cell is reached
4. return:
   - positive distance in cells if a legal hit was found
   - `-2` if the hit is closer than the minimum lidar range
   - `-1` if nothing was found within the allowed range or the ray left the world

## 6. Autonomous algorithm
The autonomous behavior is implemented in `src/DroneAlgorithm.cpp` inside `SimpleDroneAlgorithm`.

### 6.1 Core idea
The algorithm is a deterministic depth-first search over the six cardinal neighbors:
- +X
- -X
- +Y
- -Y
- +Z
- -Z

This is not optimized. It is a baseline that explores reachable space safely and consistently.

### 6.2 `run()`
Main driver of the mission.

Steps:
1. create a `visited` set
2. define a recursive DFS lambda
3. start DFS from the current start position

### 6.3 DFS node logic
For each current cell:
1. mark it visited
2. mark it as `Empty` in the discovered map
3. call `scan_current_cell()`
4. iterate over all six neighbor directions
5. skip already visited neighbors
6. skip neighbors already known as `Occupied` or `OutOfBounds`
7. attempt physical move with `try_move_to(next)`
8. if move fails, mark neighbor as `Occupied`
9. if move succeeds:
   - recurse into that neighbor
   - after recursion, backtrack to the parent cell
10. if backtracking fails, throw an unrecoverable error

### 6.4 `scan_current_cell()`
This function converts lidar information into map updates.

For each cardinal direction:
- if lidar returns `-2`, mark the immediate adjacent cell as `Occupied`
- if lidar returns a positive hit distance:
  - mark intermediate cells as `Empty`
  - mark the final hit cell as `Occupied`
- if lidar returns `-1`, do nothing because no obstacle was confirmed within range

### 6.5 `try_move_to()`
This function translates a one-cell neighbor target into actual driver calls.

Cases:
- pure vertical neighbor → call `elevate()`
- horizontal neighbor in +X/-X/+Y/-Y →
  1. set the heading using rotation
  2. call `advance(100 cm)`

If the target is not an immediate neighbor, return `false`.

## 7. Writing the output file
After the algorithm finishes, `run_simulation()` calls:
- `SparseBuildingMap::serialize_dense()`
- `write_world_map()`

The output file format is:
- `size=x,y,z`
- `start=x,y,z`
- comment line `# x,y,z,state`
- then one line for every cell in dense order:
  `x,y,z,state`

State values are:
- `0` empty
- `1` occupied
- `-1` unmapped
- `-2` out of bounds

## 8. Scoring logic
Scoring is computed in `compute_score()` inside `src/Simulator.cpp`.

Method:
1. build an expected dense grid from the hidden input world
2. initialize every expected cell as `Empty`
3. overwrite occupied coordinates with `Occupied`
4. compare expected and actual dense vectors cell by cell
5. return `100 * correct / total`

This means the score rewards exact agreement with the hidden world on every cell inside the output dimensions.

## 9. Error handling philosophy
The project follows two levels of error handling.

### Recoverable errors
Examples:
- missing optional config key
- malformed occupied-cell line
- malformed key/value line

Handling:
- replace with default or skip bad line
- append explanation to `ErrorList`
- write all recovered issues into `input_errors.txt`

### Unrecoverable errors
Examples:
- file cannot be opened
- invalid or missing mandatory map size
- invalid start line
- DFS cannot backtrack after a successful descent

Handling:
- throw an exception
- catch it in `run_simulation()`
- return a failure result instead of crashing
- main prints the failure message and exits normally

## 10. Test flow
The project uses a very small custom test framework under `tests/`.

### `tests/TestFramework.hpp`
Provides:
- `DM_TEST(...)`
- `DM_ASSERT_TRUE(...)`
- `DM_ASSERT_EQ(...)`

### `tests/TestMain.cpp`
Provides:
- global test registry
- automatic registration support
- loop that runs all tests and prints `[PASS]` / `[FAIL]`

### Current tests
- `ParserTests.cpp`
  - verifies default recovery behavior
  - verifies world parsing of occupied cells
- `SimulationTests.cpp`
  - verifies that a minimal simulation run succeeds
  - verifies that `map_output.txt` is produced

## 11. Dependency graph
High-level dependency direction:

`main.cpp`
→ `Simulator`
→ `Config` + `FileParsers`
→ `Mocks` + `SparseBuildingMap`
→ `SimpleDroneAlgorithm`
→ `Interfaces`
→ `Types` + `Units`

This keeps the algorithm dependent on abstract interfaces rather than on simulator internals.

## 12. Practical reading order
If you want to understand the code quickly, read in this order:
1. `src/main.cpp`
2. `include/drone_mapper/Simulator.hpp`
3. `src/Simulator.cpp`
4. `include/drone_mapper/Types.hpp`
5. `include/drone_mapper/Interfaces.hpp`
6. `src/FileParsers.cpp`
7. `src/Mocks.cpp`
8. `src/DroneAlgorithm.cpp`
9. `src/BuildingMap.cpp`
10. `tests/`

That order follows the actual runtime flow of the program.
