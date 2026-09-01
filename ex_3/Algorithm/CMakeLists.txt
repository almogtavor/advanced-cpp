# The mapping algorithm plugin, loaded by the simulator via dlopen.
add_library(Algorithm_${SUBMITTER_IDS} SHARED
    src/MappingAlgorithmImpl.cpp
)

target_include_directories(Algorithm_${SUBMITTER_IDS}
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(Algorithm_${SUBMITTER_IDS}
    PRIVATE
        common::common
        user_common::user_common
)

# PREFIX ""/SUFFIX ".so" + undefined-symbol handling for the registration hook.
drone_plugin(Algorithm_${SUBMITTER_IDS})
