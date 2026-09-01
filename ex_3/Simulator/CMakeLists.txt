# The simulator executable. Loads Algorithm / MissionControl .so files at
# runtime, so it needs the dl library and must export its own symbols.
add_executable(simulator_${SUBMITTER_IDS}
    src/main.cpp
    src/Registrar.cpp
    src/MappingAlgorithmRegistration.cpp
    src/MissionControlRegistration.cpp
    src/SharedLibrary.cpp
    src/CliConfig.cpp
)

target_include_directories(simulator_${SUBMITTER_IDS}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/common_simulator/include
)

target_link_libraries(simulator_${SUBMITTER_IDS}
    PRIVATE
        common::common
        user_common::user_common
        ${CMAKE_DL_LIBS}
)

# The registration constructors inside each dlopen'd plugin resolve back into
# this executable, so its symbols have to be visible to the dynamic loader.
set_target_properties(simulator_${SUBMITTER_IDS} PROPERTIES ENABLE_EXPORTS ON)

drone_warnings(simulator_${SUBMITTER_IDS})
