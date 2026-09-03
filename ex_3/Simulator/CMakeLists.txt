# The simulator executable. Loads Algorithm / MissionControl .so files at
# runtime, so it needs the dl library and must export its own symbols.
# Everything except main(), so the tests can link the same objects the
# executable does instead of compiling them twice.
add_library(simulator_lib STATIC
    src/Registrar.cpp
    src/SharedLibrary.cpp
    src/CliConfig.cpp
    src/Map3DImpl.cpp
    src/MapBuilder.cpp
    src/ConfigLoader.cpp
    src/MockGPS.cpp
    src/MockLidar.cpp
    src/MockMovement.cpp
    src/SimulationRunImpl.cpp
    src/SimulationRunFactoryImpl.cpp
    src/SimulationManager.cpp
    src/ReportWriter.cpp
    src/MapsComparison.cpp
    src/Sweep.cpp
    ${CMAKE_SOURCE_DIR}/UserCommon/src/ErrorLog.cpp
    ${CMAKE_SOURCE_DIR}/UserCommon/src/ScanResultToVoxels.cpp
)

target_include_directories(simulator_lib
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/common_simulator/include
        ${CMAKE_SOURCE_DIR}/MissionControl/common_mission_control/include
)

target_link_libraries(simulator_lib
    PUBLIC
        common::common
        user_common::user_common
        TinyNPY::TinyNPY
        yaml-cpp::yaml-cpp
        ${CMAKE_DL_LIBS}
)

drone_warnings(simulator_lib)

# The registration constructors must be linked directly into the executable,
# not pulled from a static archive: nothing in the simulator calls them (the
# plugins do, at dlopen time), so the linker would discard the archive members
# and every .so would fail with "symbol not found in flat namespace".
add_executable(simulator_${SUBMITTER_IDS}
    src/main.cpp
    src/MappingAlgorithmRegistration.cpp
    src/MissionControlRegistration.cpp
)
target_link_libraries(simulator_${SUBMITTER_IDS} PRIVATE simulator_lib)

# The registration constructors inside each dlopen'd plugin resolve back into
# this executable, so its symbols have to be visible to the dynamic loader.
set_target_properties(simulator_${SUBMITTER_IDS} PROPERTIES ENABLE_EXPORTS ON)

drone_warnings(simulator_${SUBMITTER_IDS})

# ---- Tests (optional) ------------------------------------------------------
# Built only when GTest is available, so a grader without it still gets a clean
# build of the three required projects.
find_package(GTest CONFIG QUIET)
if(GTest_FOUND)
    add_executable(simulator_tests
        tests/test_threading_determinism.cpp
        tests/test_report_ordering.cpp
        tests/test_plugin_loading.cpp
        # Same reason as the executable: the plugin loading test dlopens real
        # .so files, whose registration constructors must resolve back into
        # this binary, so these cannot come from the static archive.
        src/MappingAlgorithmRegistration.cpp
        src/MissionControlRegistration.cpp
    )
    target_link_libraries(simulator_tests PRIVATE simulator_lib GTest::gtest_main)

    # Export symbols to dlopen'd plugins, exactly as the simulator does.
    set_target_properties(simulator_tests PROPERTIES ENABLE_EXPORTS ON)

    # The tests load the real plugins, so they must be built first.
    add_dependencies(simulator_tests
        Algorithm_${SUBMITTER_IDS}
        MissionControl_${SUBMITTER_IDS})

    target_compile_definitions(simulator_tests PRIVATE
        EX3_INPUTS_DIR="${CMAKE_SOURCE_DIR}/inputs"
        EX3_ALGORITHM_SO="$<TARGET_FILE:Algorithm_${SUBMITTER_IDS}>"
        EX3_MISSION_CONTROL_SO="$<TARGET_FILE:MissionControl_${SUBMITTER_IDS}>")
    drone_warnings(simulator_tests)

    enable_testing()
    add_test(NAME simulator_tests COMMAND simulator_tests)
else()
    message(STATUS "GTest not found - skipping simulator_tests")
endif()
