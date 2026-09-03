# Assignment 3 - Drone Mapper

Almog Tavor (323084962), Yonatan Kahana (212223036)

A drone mapping simulator split into three separately-built projects that
communicate only through the course-provided `common/` interfaces. The
Algorithm and the MissionControl are shared libraries loaded at runtime by the
Simulator, so each part can be swapped for another team's implementation.

## Layout

```text
Algorithm/       -> Algorithm_323084962_212223036.so        (namespace algorithm_323084962_212223036)
MissionControl/  -> MissionControl_323084962_212223036.so   (namespace mission_control_323084962_212223036)
Simulator/       -> simulator_323084962_212223036           (namespace simulator)
common/          -> course-provided headers, used as-is
UserCommon/      -> our shared helpers                      (namespace user_common_323084962_212223036)
```

`UserCommon` holds `MapGeometry.h` (header-only unit/voxel conversions),
`ErrorCodes.h`, and `ErrorLog` / `ScanResultToVoxels`. The two `.cpp` files are
compiled into each project that needs them, since `UserCommon` has no makefile
of its own.

## Building

Dependencies are declared in `vcpkg.json`: `mp-units`, `yaml-cpp`, `tinynpy`,
`gtest`.

```sh
export VCPKG_ROOT=/path/to/vcpkg
cmake --preset debug          # or: --preset release
cmake --build --preset debug
```

The root `CMakeLists.txt` builds all three projects; each project also has its
own `CMakeLists.txt` and can be built on its own.

Plugins are forced to `PREFIX ""` / `SUFFIX ".so"` so the filenames match the
assignment on every platform. On macOS they additionally link with
`-undefined dynamic_lookup`, because the registration constructors they call
live in the simulator executable and are only resolved at `dlopen` time.

## Running

```sh
./simulator_323084962_212223036 -comparative \
    simulation=<composition.yaml> \
    mission_control_folder=<folder> \
    algorithm=<algorithm.so> \
    [num_threads=<n>] [-verbose]

./simulator_323084962_212223036 -competition \
    simulation=<composition.yaml> \
    mission_control=<mission_control.so> \
    algorithms_folder=<folder> \
    [num_threads=<n>] [-verbose]
```

Arguments may appear in any order. All problems are collected and reported
together - unknown arguments, missing arguments, duplicate arguments,
unreadable files, and folders that do not exist, cannot be traversed, or hold
no `.so` files - followed by a usage message.

## Registration and plugin lifetime

Each plugin `.cpp` ends with `REGISTER_MAPPING_ALGORITHM(...)` /
`REGISTER_MISSION_CONTROL(...)` at global scope. Loading the `.so` constructs
that global, whose constructor - implemented in the *Simulator* project, in
`MappingAlgorithmRegistration.cpp` and `MissionControlRegistration.cpp` - hands
the factory to `simulator::Registrar`. The Simulator therefore never names a
concrete plugin class.

Because the macro pastes its argument into a variable name, it only accepts an
unqualified identifier, so each plugin adds a `using` declaration before the
macro.

Shutdown order matters and is enforced structurally in `main`: `RegistrarGuard`
is declared *after* the `SharedLibrary` objects, so it is destroyed *before*
them. The registered factories are `std::function`s whose code lives inside the
`.so`, so they must be released before `dlclose`. Destroying them afterwards is
a segfault.

## Threading

`Sweep` runs each plugin through `SimulationManager` (our implementation of the
provided `ISimulation`). The manager builds that plugin's full work list -
every (simulation x mission x drone x lidar) combination - before any thread
starts, and pre-sizes the results vector so each task writes only its own index.
There is no locking on results; workers pull indices from a single
`std::atomic`.

* `num_threads` absent or `1` - the simulation runs on the main thread only.
* `num_threads >= 2` - that many workers *in addition to* main, so the total is
  never exactly 2.
* Never more workers than there are work items.

All `.so` files are loaded up front, before any thread starts, so the registrar
is never written concurrently and needs no lock. `ErrorLog` is guarded by a
mutex because the global instance is written from workers, and each
MissionControl instance keeps its own log so concurrent missions never share a
stream.

Measurements we've done:
| num_threads | real | user | Behaviour |
|---|---|---|---|
| absent | 1.03s | 0.98s | serial - main thread only |
| =1 | 1.07s | 0.97s | serial - main thread only |
| =2 | 0.50s | 0.96s | 2 workers + main (never a total of 2) |
| =8, only 2 items | 0.52s | 0.98s | capped at 2 - never more threads than work |

## Output

A results directory is created under the folder argument:
`comparative_results_<time>` or `competition_<time>`, where `<time>` is a
seconds-since-epoch stamp (retried if taken). Failure to create it is reported
to the screen rather than throwing. It contains:

* `output_map_<plugin>_sim<i>_mission<j>_drone<k>_lidar<l>.npy` - one per run,
  named so every map traces back to the mission that produced it.
* `error_log.txt` - simulator-side errors, written and flushed as they occur.
* `simulation_output_<plugin>.yaml` - one assignment-2 style report per plugin.
* `comparative_report.yaml` or `competitive_report.yaml` - the summary.

Comparative groups plugins whose totals are identical and sorts groups by size
descending. Competitive sorts by score descending, then steps ascending.
Plugins that fail to load, or register nothing, appear in `errors:` instead.

With `-verbose` each MissionControl also writes
`<output_map>_mission_control.log`.

Every plugin call is wrapped: a thrown exception becomes an error entry and the
run is scored `-1`, so the Simulator survives everything short of a plugin
crashing the process.

## Scoring

`output_map_accuracy`: the fraction of voxels in the hidden map's grid where
the output map agrees on occupied-vs-not, as a percentage in `[0, 100]`. Failed
runs score `-1`.

## Tests

Optional, and built only when GTest is available:

```sh
./build/debug/Simulator/simulator_tests
```

`ThreadingDeterminism` runs the same sweep single-threaded and multi-threaded
and asserts the results are byte-for-byte identical - a race in the work list,
the results vector, or the output-map naming would break it. `ReportOrdering`
checks the two summary formats against the grouping and sorting rules in the
assignment, including the case where equal scores but different step counts
must land in separate groups.

## Known issues

See `known_issues.xlsx` if submitted.

* Threading is per plugin rather than across the whole plugin matrix at once:
  `Sweep` calls `SimulationManager` once per plugin, and each call threads over
  that plugin's (simulation x mission x drone x lidar) combinations. This keeps
  `ISimulation` implemented as provided, at the cost of a barrier between
  plugins - with many plugins and few missions the last threads of one plugin
  can idle before the next begins.
* Without `-verbose`, MissionControl errors are returned in `MissionRunResult`
  and rendered into the per-plugin YAML, but are not additionally written to a
  MissionControl-side log file.
* The unload-on-demand bonus is not implemented; all `.so` files stay loaded
  for the duration of the sweep.
