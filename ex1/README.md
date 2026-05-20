# Drone Mapper - Exercise 1

## Contributors

| Name | ID |
|------|----|
| Almog Tavor | 323084962 |
| Yonatan Kahana | 212223036 |

## Building

### With CMake + vcpkg (recommended, matches the course environment)

```bash
cmake --preset default      # or: cmake -B build -DCMAKE_TOOLCHAIN_FILE=$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake
cmake --build build
```

`vcpkg.json` declares `mp-units` as a manifest dependency, so vcpkg fetches
and builds it on first configure. This mirrors the teacher's
`example_mock_lidar` setup.

### With Make (mp-units must be installed on the host)

```bash
make                                     # if mp-units is in the default include path
make MP_UNITS_INC=$VCPKG_ROOT/installed/x64-linux/include   # explicit path
```

Requires gcc 11.4+ with `-std=c++20 -Wall -Wextra -Werror -pedantic`.

## Running

```
drone_mapper [<input_output_files_path>]
```

If no path is provided, the current working directory is used.

The program reads three input files from the given path and writes one output file:

| File | Direction | Description |
|------|-----------|-------------|
| `drone_config.txt` | input | Drone hardware capabilities |
| `mission_config.txt` | input | Mission boundaries and start position |
| `map_input.txt` | input | Building truth map (used only by mock sensors) |
| `map_output.txt` | output | Drone's reconstructed map |
| `input_errors.txt` | output (optional) | Created only when recoverable parse errors are found |

## Input file formats

All lengths are in centimeters. All angles are in degrees.

### drone_config.txt

Key-value pairs, one per line. Lines starting with `#` are comments.

```
# key                  value   description
min_passage_width      30      smallest width the drone will enter (cm)
min_passage_length     30      smallest length (cm)
min_passage_height     50      smallest height (cm)
lidar_z_min            5       Z-min: minimum measurable distance (cm)
lidar_z_max            200     Z-max: maximum range (cm)
lidar_d                2       D: spacing between consecutive beam circles at Z-min (cm)
lidar_fovc             4       FOVC: number of beam circles (1 -> central beam only)
max_rotate_per_cmd     180     max rotation per command (degrees)
max_advance_per_cmd    100     max horizontal move per command (cm)
max_elevate_per_cmd    100     max vertical move per command (cm)
```

Missing or invalid keys fall back to built-in defaults. Unknown keys are logged to `input_errors.txt`.

### mission_config.txt

Key-value pairs. The mapping boundary is the bounded rectangle
`[min_x, max_x] x [min_y, max_y]` combined with `[height_min, height_max]`.
The requested mapping resolution must match the map's `cell_size` (the
single supported resolution per the Ex1 clarification); a mismatch is a
fatal error.

```
start 25 25 5                 # initial position (x y z in cm)
start 25 25 5 90              # ...with optional XY-angle in degrees
                              #    (default 0 = east; 90 = south, 180 = west, 270 = north)
min_x 0                       # rectangle boundary in the XY plane (cm)
max_x 100
min_y 0
max_y 100
height_min 0                  # lower height limit (cm)
height_max 100                # upper height limit (cm)
xy_resolution 1               # decimal places after the dot in meter-based
                              # output; cell_size_cm = 100 / 10^N.
                              # N=1 -> 10 cm cells (matches the map's cell_size).
height_resolution 1
```

### map_input.txt / map_output.txt

Layered ASCII voxel grid:

```
cell_size 10                 # voxel side length (cm)
origin 0 0 0                 # world position of cell (0,0,0) corner
size 10 10 3                 # grid dimensions (nx ny nz cells)
layer 0                      # z-index of this layer
##########                   # ny rows of nx characters each
#........#
...
```

Character encoding:

| Char | Meaning | Value |
|------|---------|-------|
| `.`  | empty   |  0    |
| `#`  | occupied (wall/obstacle) | 1 |
| `?`  | not mapped | -1 |
| `_`  | outside mapping boundaries | -2 |

## Scoring formula

For every voxel inside the mission polygon and height range:

- Correctly classified (output matches truth): +1
- Incorrectly classified: +0
- Unmapped: +0

**Score = (correct / total_in_bounds) * 100**

A perfect score of 100 is possible only when every in-bounds voxel is reachable by the drone.

## Testing

```bash
make test          # builds and runs all unit/integration tests
```

The test suite uses a custom lightweight framework (`tests/test_framework.h`)
that requires no external libraries. Tests cover all major components: units,
voxel grid, building map, config parsing, map I/O, mock sensors/drivers, drone
logic, and full simulation integration.

Tests are an opt-in CMake bonus. To skip them entirely (and build only the
main binary), pass `-DBUILD_TESTS=OFF` to the CMake configure step.

## Visual simulation

```bash
python3 visualize.py inputs/set1/
```

Generates `visualization.html` in the given directory. Open in any browser to
see a side-by-side comparison of the truth map and the drone's output, with a
diff overlay and score computation. Requires Python 3 (standard library only,
no external packages).

## Strong types

All values and APIs use the mp-units library as required by the assignment.
The wrapper header `include/units/Units.h` provides `Length` and `Angle` type
aliases and brings `cm`, `m`, `deg` into the `units` namespace.
`5.0 * cm`, `90.0 * deg`, `1.0 * m` all produce strong quantity types.
The mp-units dependency is resolved through the course's vcpkg toolchain
(see `vcpkg.json`); the CMake build calls `find_package(mp-units CONFIG
REQUIRED)` and links `mp-units::mp-units`. gsl-lite is pulled in
transitively by mp-units.

## Algorithm overview

The drone uses a deterministic frontier-based BFS exploration on the voxel grid:

1. At each waypoint the drone takes six cardinal scans (+X, -X, +Y, -Y, +Z, -Z) using the lidar central ray to update its known map.
2. After scanning, a BFS through known-empty cells finds the nearest cell that has at least one unmapped neighbor (the "frontier").
3. The drone plans a path to that frontier and follows it step by step (rotate, then advance/elevate).
4. When no more frontier cells are reachable, the drone reports Finished.

Per the assignment, the drone is modeled as a perfect sphere whose effective
radius is `min(min_passage_width, min_passage_length, min_passage_height) / 2`.
Both the BFS frontier search and the `MovementMock` swept collision check
honor this radius (rounded down to whole cells, Chebyshev distance), so the
drone refuses to plan or execute moves through openings that are too
narrow for its body. If a collision is detected at runtime anyway, the
simulator terminates the run with a clear `FAILURE` notice as required by
the assignment (line 99).

## Sample input sets

| Set | Scenario | Expected score |
|-----|----------|---------------|
| `inputs/set1/` | Simple 10x10 empty square room | ~92% |
| `inputs/set2/` | 15x12 building with sealed inaccessible 3x3 pocket | ~95% |
| `inputs/set3/` | L-shaped building inside a 15x15 bounding rectangle | ~69% |

The set3 number is limited by the geometry: the bounding rectangle
includes the upper-right "notch" of the L, whose interior walls are
occluded from the drone's lidar by the L's inner wall. Those cells are
in-bounds per the mission configuration so they count against the score
even though no line-of-sight exists to them. A polygon-based mission
boundary (planned for ex2) would exclude that notch from scoring.

Each set contains an `original_output/` folder with the output from our run.
