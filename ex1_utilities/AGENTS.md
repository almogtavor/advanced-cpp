# Repository Guidelines

## Project Structure & Module Organization
`include/drone_mapper/` contains public headers for the simulator, parsers, map model, mocks, and unit wrappers. `src/` holds the corresponding C++20 implementations plus `main.cpp`. `tests/` contains the lightweight in-repo test framework and integration-style parser/simulation tests. `samples/` provides example `drone_config.txt`, `mission_config.txt`, and `map_input.txt` inputs. `build/` is the generated CMake output directory and should stay disposable.

## Build, Test, and Development Commands
Configure once with `cmake -S . -B build` to generate the project files. Build with `cmake --build build`; this produces `drone_mapper` and, when testing is enabled, `drone_mapper_tests`. Run the full test suite with `ctest --test-dir build --output-on-failure`. For a manual simulation run, use `.\build\drone_mapper .\samples` on Windows or `./build/drone_mapper ./samples` on Unix-like shells.

## Coding Style & Naming Conventions
Match the existing C++20 style: 2-space indentation, braces on the same line, `#pragma once` in headers, and standard-library includes grouped after project headers. Keep public types in the `dm` namespace. Use `PascalCase` for types and test names (`SimulationProducesOutputFile`), `snake_case` for free functions and local variables (`parse_world_map`, `input_errors`), and `.hpp`/`.cpp` file pairs named after the module (`Simulator.hpp`, `Simulator.cpp`). No formatter or linter is configured here, so consistency with nearby files is the standard.

## Testing Guidelines
Tests use the custom harness in `tests/TestFramework.hpp`; add new cases with `DM_TEST(TestName)`. Prefer deterministic fixtures written to temporary directories, as the current tests do for parser and simulator flows. Add or update tests whenever parsing rules, movement logic, or output serialization changes. Run `ctest --test-dir build --output-on-failure` before submitting.

## Commit & Pull Request Guidelines
This workspace does not include `.git`, so no repository-specific commit history was available to inspect. Use short, imperative commit subjects such as `Add mission bounds validation` and keep them focused on one change. Pull requests should explain the affected module, summarize behavior changes, list verification commands, and attach sample input/output snippets when file formats or simulator output change.

## Configuration & Input Notes
The simulator expects three text inputs in a target directory: `drone_config.txt`, `mission_config.txt`, and `map_input.txt`. Preserve the current recovery behavior for malformed but non-fatal input and document any new keys or file format changes in `README.md`.
