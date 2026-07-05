ex2_tests_suite - portable end-to-end validator
===============================================

A cross-implementation validation suite for the ex2 drone mapper. Unlike C++
component tests (which are coupled to each person's internal helper headers),
this suite touches ONLY the assignment-standardised surface:

  - the two CLI binaries:  drone_mapper_simulation  and  maps_comparison
  - the mandated output names: simulation_output.yaml and output_results/
  - the .npy output-map format (and the fixed VoxelOccupancy codes)

Because it never includes any project header, it runs against ANYONE's compiled
binaries no matter how their code is structured. Share it with the group: if a
teammate's build passes this, their observable behaviour matches the spec.

It enforces ONLY behaviour that assignment2.md actually requires. It does NOT
assert things the spec leaves open - report-body layout ("you should decide how
to manage the results"), the error-log filename ("the format ... are yours"),
symmetry or specific in-between values of maps_comparison ("a reasonable
result"), determinism, exact output-map shape, or exit codes on bad input.


REQUIREMENTS
------------
  - python3 (standard library only - no numpy, no PyYAML)
  - the two ex2 binaries, OR an ex2 project plus VCPKG_ROOT so they can be built


USAGE
-----
  ./run.sh [<ex2_project_or_bin_dir>]

  <ex2_project_or_bin_dir> may be:
    - a directory already containing drone_mapper_simulation + maps_comparison
      (or a build/ subdir with them), or
    - an ex2 project directory to build (needs VCPKG_ROOT set).
    Default: ../ex2

Examples:
  ./run.sh ~/ex2/build                 # already-built binaries
  VCPKG_ROOT=~/vcpkg ./run.sh ~/ex2    # build then validate

Exit code is 0 iff every check passes. Runs in a few seconds.


WHAT IT CHECKS (78 checks, all spec-backed)
-------------------------------------------
maps_comparison  (spec: identical->100, distinct->~0, similar-><100, error->-1)
  - identical maps -> 100 (all 9 fixtures)
  - very distinct maps -> near 0 (both orderings)
  - very similar maps -> high but < 100 (1..4 voxel differences)
  - a very-similar pair scores above a very-distinct pair
  - every score is within [0, 100]
  - unreadable file(s) / missing arguments -> -1

drone_mapper_simulation
  basic          runs; writes simulation_output.yaml; creates output_results/;
                 exactly 1 output map that is a valid 3-D .npy using only legal
                 voxel codes and containing mapped voxels
  product        runs = missions x drones x lidars (2 x 2 -> 4 valid maps)
  multi-mission  2 missions -> 2 valid maps
  resolution     factor-1 vs factor-2 -> a different-size output map
  error          bad map file: continues to completion, report still written,
                 an error log is written under output_results
  mixed          good + bad: continues; the good run produces a valid map; the
                 bad run is logged
  max_steps      tiny budget: continues and still produces a valid map
  bad bounds     invalid mission bounds: does not crash; report still written
  CLI            no args -> ./simulation.yaml into cwd; relative composition
                 path resolves into the given output dir

Every output map is validated for: valid 3-D .npy, only legal VoxelOccupancy
codes (-3..1, which are fixed in Types.h), and (where the scenario should map
something) that it actually contains mapped voxels.


LAYOUT
------
  run.sh                   the runner
  tools/check_npy.py       stdlib .npy header + voxel-data validator
  fixtures/cmp/            6x6x6 maps for the comparison checks
  fixtures/scenarios/<n>/  self-contained compositions (each references only
                           bare filenames in its own folder):
                             basic product multimission resolution
                             error mixed maxsteps badbounds
