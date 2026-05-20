cmake_minimum_required(VERSION 3.20)
project(drone_mapper LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Werror -pedantic)

# mp-units is resolved via the build environment (vcpkg manifest mode picks
# up vcpkg.json automatically when CMAKE_TOOLCHAIN_FILE points at vcpkg).
find_package(mp-units CONFIG REQUIRED)

set(LIB_SOURCES
    src/VoxelGrid.cpp
    src/BuildingMap.cpp
    src/ConfigParser.cpp
    src/MapIO.cpp
    src/LidarMock.cpp
    src/MovementMock.cpp
    src/Drone.cpp
    src/Simulator.cpp
)

add_library(drone_mapper_lib STATIC ${LIB_SOURCES})
target_include_directories(drone_mapper_lib PUBLIC include)
target_link_libraries(drone_mapper_lib PUBLIC mp-units::mp-units)

add_executable(drone_mapper src/main.cpp)
target_link_libraries(drone_mapper PRIVATE drone_mapper_lib)

# Tests are a bonus feature; the main binary builds and runs without them.
# Pass -DBUILD_TESTS=OFF to skip the test runner.
option(BUILD_TESTS "Build the run_tests executable (bonus)" ON)
if(BUILD_TESTS)
    add_executable(run_tests tests/test_main.cpp)
    target_include_directories(run_tests PRIVATE tests)
    target_link_libraries(run_tests PRIVATE drone_mapper_lib)

    enable_testing()
    add_test(NAME unit_tests COMMAND run_tests)
endif()
