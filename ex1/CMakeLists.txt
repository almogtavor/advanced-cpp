cmake_minimum_required(VERSION 3.16)
project(drone_mapper LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Werror -pedantic)

# --- mp-units (bundled in third_party/) ---
set(MP_UNITS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mp-units/src)
add_compile_definitions(gsl_FEATURE_GSL_COMPATIBILITY_MODE=1)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

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
target_include_directories(drone_mapper_lib PUBLIC
    include
    ${MP_UNITS_DIR}/core/include
    ${MP_UNITS_DIR}/systems/si/include
    ${MP_UNITS_DIR}/systems/isq/include
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/gsl-lite/include
)

add_executable(drone_mapper src/main.cpp)
target_link_libraries(drone_mapper PRIVATE drone_mapper_lib)

# --- Tests ---
add_executable(run_tests tests/test_main.cpp)
target_include_directories(run_tests PRIVATE tests)
target_link_libraries(run_tests PRIVATE drone_mapper_lib)

enable_testing()
add_test(NAME unit_tests COMMAND run_tests)
