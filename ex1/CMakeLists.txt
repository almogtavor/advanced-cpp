cmake_minimum_required(VERSION 3.16)
project(drone_mapper LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Werror -pedantic)

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
target_include_directories(drone_mapper_lib PUBLIC include)

add_executable(drone_mapper src/main.cpp)
target_link_libraries(drone_mapper PRIVATE drone_mapper_lib)
