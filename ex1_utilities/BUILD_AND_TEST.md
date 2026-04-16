# Build, Run, and Test Instructions

This project is built with CMake and targets C++20.

## Prerequisites

Install the following tools first:

- A C++20-capable compiler
  - Windows: Visual Studio 2022 Build Tools or Visual Studio 2022 with the C++ workload
  - Linux/macOS: `g++` or `clang++`
- CMake 3.20 or newer

On Windows, make sure `cmake` and `ctest` are available in your terminal `PATH`.

## Build the program

From the repository root:

```powershell
cmake -S . -B build
cmake --build build --config Debug
```

This creates the main executable:

- Windows with Visual Studio generator: `build\Debug\drone_mapper.exe`
- Linux/macOS: `build/drone_mapper`

## Run the program

The simulator expects these files inside the input directory:

- `drone_config.txt`
- `mission_config.txt`
- `map_input.txt`

Sample inputs are already provided in `samples/`.

### Windows PowerShell

```powershell
.\build\Debug\drone_mapper.exe .\samples
```

### Linux/macOS

```bash
./build/drone_mapper ./samples
```

## Run the tests

### Important note about GoogleTest

This repository does **not** currently use GoogleTest.
The tests are implemented with the local lightweight framework in `tests/TestFramework.hpp`,
and CMake builds a test executable named `drone_mapper_tests`.

Run the full test suite with CTest:

```powershell
ctest --test-dir build -C Debug --output-on-failure
```

You can also run the test executable directly:

### Windows PowerShell

```powershell
.\build\Debug\drone_mapper_tests.exe
```

### Linux/macOS

```bash
./build/drone_mapper_tests
```

## If you specifically need GoogleTest

GoogleTest is not configured in `CMakeLists.txt` yet. There is no `find_package(GTest)` or
linked `gtest` target in the current build.

If you want the tests to run with GoogleTest, the project must be updated to:

1. Add GoogleTest as a dependency.
2. Replace or wrap the custom `DM_TEST` framework.
3. Register the GoogleTest-based executable with CTest.

Until that change is made, the correct way to run tests in this repository is through
`drone_mapper_tests` and `ctest`.

## Troubleshooting

- If CTest says `Missing "-C <config>"?`, the build was generated with a multi-config generator
  such as Visual Studio. Use `cmake --build build --config Debug` and
  `ctest --test-dir build -C Debug --output-on-failure`.
- If `cmake` is not recognized, install CMake and reopen the terminal.
- If `ctest` is not recognized, it usually means CMake is not installed correctly or not on `PATH`.
- If the build fails on Windows, open a Developer PowerShell for Visual Studio so the compiler environment is loaded.
