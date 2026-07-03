Assignment 2 - Drone Mapper Simulation
======================================

A refactor of the ex1 drone mapper onto the exact ex2 API and file formats,
with full Component and Integration test suites (GTest + GMock).


BUILDING
--------
Dependencies are declared in vcpkg.json: mp-units, tinynpy, yaml-cpp, gtest.

  export VCPKG_ROOT=/path/to/vcpkg
  cmake --preset default          # or: cmake -S . -B build \
                                  #        -DCMAKE_TOOLCHAIN_FILE=$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake
  cmake --build --preset default  # or: cmake --build build

Targets:
  drone_mapper_simulation        - the simulator
  maps_comparison                - the standalone map comparison utility
  drone_mapper_simulation_test   - the full test suite


RUNNING THE SIMULATOR
---------------------
  ./drone_mapper_simulation [<simulation.yaml>] [<output_path>]

  - no composition argument -> reads "simulation.yaml" in the current directory
  - relative path           -> resolved against the current directory
  - absolute path           -> used as-is
  - no output path          -> current directory


RUNNING THE MAP COMPARISON UTILITY
----------------------------------
  ./maps_comparison <origin_map> <target_map> [comparison_config=<path>]

  Prints a single accuracy score in [0, 100] to stdout (100 = identical).
  On error prints -1 to stdout and a descriptive message to stderr. Without a
  comparison_config both maps are assumed to share offset, boundaries and
  resolution.


RUNNING THE TESTS
-----------------
  ./drone_mapper_simulation_test
  ./drone_mapper_simulation_test --gtest_filter=Integration.*
  ./drone_mapper_simulation_test --gtest_filter=SimulationManager.*
  ./drone_mapper_simulation_test --gtest_filter=SimulationRun.*      (also covers MockGPS + MockMovement)
  ./drone_mapper_simulation_test --gtest_filter=MissionControl.*
  ./drone_mapper_simulation_test --gtest_filter=DroneControl.*
  ./drone_mapper_simulation_test --gtest_filter=MappingAlgorithm.*
  ./drone_mapper_simulation_test --gtest_filter=MockLidar.*
  ./drone_mapper_simulation_test --gtest_filter=MapsComparison.*

Component tests live under tests/components/, integration tests under
tests/integration/, and shared helpers / GMock doubles under tests/support/.
Two extra suites, Map3D.* and ConfigLoader.*, cover the supporting map and
config-parsing code.


OUTPUT FORMAT
=============
The simulator writes, under the output path:

1) simulation_output.yaml
-------------------------
A single "score_report" mapping with a computed "summary" and a flat "runs"
list - one entry per simulation/mission/drone/lidar combination:

  score_report:
    generated_at_utc: "2026-07-03T09:25:31Z"
    metric: "output_map_accuracy"
    score_range: { min: 0, max: 100 }
    error_score: -1
    summary:
      total_runs: 8
      scored_runs: 7
      error_runs: 1
      average_score: 82.4
      min_score: 61.2
      max_score: 98.9
    runs:
      - simulation_config: "<hidden map file>"
        mission_max_steps: 2400
        resolution_cm: 20                       # actual output resolution
        resolution_request_status: ACCEPTED     # ACCEPTED | IGNORED | IGNORED_TOO_SMALL
        status: "completed"                     # completed | max_steps | error
        steps: 1231
        score: 93.5                             # -1 for an errored run
        output_map_file: "output_results/output_map_0000.npy"
        error_ref: { code: "DRONE_HITS_OBSTACLE" }   # only present on error

2) output_results/
------------------
  - output_map_XXXX.npy : one saved output map per run. int8 voxel codes:
        -3 potentially-occupied, -2 out-of-bounds, -1 unmapped,
         0 empty, 1 occupied.
        Named by run index in composition order.
  - error_log.txt       : every error, written and flushed the moment it
        occurs, as "<utc-timestamp> [CODE] message".


COORDINATE / MAP MODEL
----------------------
- Maps are .npy, C-order, axis order (X, Y, Z); flat = (x*ny + y)*nz + z.
- MapConfig bundles boundaries, offset and resolution. The npy voxel (0,0,0)
  corner maps to world "offset"; voxel_index = floor((world - offset)/resolution).
- Hidden maps are normalised on load to Empty(0)/Occupied(1); any non-zero input
  voxel (including Minecraft-style block ids) counts as occupied.
- The mapping algorithm keeps ceil(radius/resolution) voxels of clearance from
  obstacles so the spherical drone body never clips a wall.
