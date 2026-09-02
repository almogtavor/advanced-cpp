cmake_minimum_required(VERSION 3.15)
project(assignment_3 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(mp-units CONFIG REQUIRED)
find_package(tinynpy CONFIG REQUIRED)
find_package(yaml-cpp REQUIRED)

set(SUBMITTER_IDS "323084962_212223036")

function(drone_warnings target)
    target_compile_options(${target} PRIVATE -Wall -Wextra -Werror -pedantic)
endfunction()

# Shared settings for a dynamically loaded plugin: the assignment requires a
# ".so" name with no "lib" prefix on every platform, and the plugin references
# the registration symbols that live in the simulator executable.
function(drone_plugin target)
    set_target_properties(${target} PROPERTIES PREFIX "" SUFFIX ".so")
    if(APPLE)
        # macOS resolves undefined symbols at link time by default; the
        # registration constructors only exist in the host executable.
        target_link_options(${target} PRIVATE -undefined dynamic_lookup)
    endif()
    drone_warnings(${target})
endfunction()

# Our own code shared by more than one project. Header-only, so an
# INTERFACE library is enough - it carries the include path and nothing else.
# Mirrors the shape of common/CMakeLists.txt.
add_library(user_common INTERFACE)
add_library(user_common::user_common ALIAS user_common)

target_include_directories(user_common INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/UserCommon/include>
)

add_subdirectory(common)
add_subdirectory(Algorithm)
add_subdirectory(MissionControl)
add_subdirectory(Simulator)
