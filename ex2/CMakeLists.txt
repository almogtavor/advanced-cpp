cmake_minimum_required(VERSION 3.15)
project(drone_mapper LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)


find_package(mp-units CONFIG REQUIRED)
find_package(tinynpy CONFIG REQUIRED)
find_package(yaml-cpp REQUIRED) # new!

add_library(drone_mapper
    src/DroneControlImpl.cpp
    src/MappingAlgorithmImpl.cpp
    src/MapsComparison.cpp
    src/MissionControlImpl.cpp
    src/Map3DImpl.cpp
    src/MapBuilder.cpp
    src/ConfigLoader.cpp
    src/ErrorLog.cpp
    src/ReportWriter.cpp
    src/MockGPS.cpp
    src/MockLidar.cpp
    src/MockMovement.cpp
    src/ScanResultToVoxels.cpp
    src/SimulationManager.cpp
    src/SimulationRunFactoryImpl.cpp
    src/SimulationRunImpl.cpp
)

target_compile_features(drone_mapper PUBLIC cxx_std_20)
target_compile_options(drone_mapper
    PRIVATE
        -Wall
        -Wextra
        -Werror
        -pedantic
)
target_include_directories(drone_mapper
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(drone_mapper
    PUBLIC
        mp-units::mp-units
        TinyNPY::TinyNPY
        yaml-cpp::yaml-cpp
)

add_executable(drone_mapper_simulation src/drone_mapper_simulation_main.cpp)
target_link_libraries(drone_mapper_simulation PRIVATE drone_mapper)
#Compile the maps_comparison util
add_executable(maps_comparison src/maps_comparison_main.cpp)
target_link_libraries(maps_comparison PRIVATE drone_mapper)

# ---- Tests (GTest + GMock) -----------------------------------------------
enable_testing()
find_package(GTest CONFIG REQUIRED)

file(GLOB_RECURSE DRONE_MAPPER_TEST_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/components/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/integration/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/support/*.cpp
)

add_executable(drone_mapper_simulation_test ${DRONE_MAPPER_TEST_SOURCES})
target_include_directories(drone_mapper_simulation_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_link_libraries(drone_mapper_simulation_test
    PRIVATE
        drone_mapper
        GTest::gtest
        GTest::gtest_main
        GTest::gmock)
# Point tests at the repo's test data directory.
target_compile_definitions(drone_mapper_simulation_test
    PRIVATE DRONE_MAPPER_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data")

add_test(NAME drone_mapper_simulation_test COMMAND drone_mapper_simulation_test)
