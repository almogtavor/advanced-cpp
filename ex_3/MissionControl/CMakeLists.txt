# The mission control plugin, loaded by the simulator via dlopen.
add_library(MissionControl_${SUBMITTER_IDS} SHARED
    src/MissionControlImpl.cpp
)

target_include_directories(MissionControl_${SUBMITTER_IDS}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/common_mission_control/include
)

target_link_libraries(MissionControl_${SUBMITTER_IDS}
    PRIVATE common::common
)

# PREFIX ""/SUFFIX ".so" + undefined-symbol handling for the registration hook.
drone_plugin(MissionControl_${SUBMITTER_IDS})
