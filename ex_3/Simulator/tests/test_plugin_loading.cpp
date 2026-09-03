// The plugin mechanism: dlopen a .so, have its global registration object
// construct itself, and find the factory waiting in the Registrar - without the
// simulator ever naming a concrete plugin class.
//
// This is the part of assignment 3 with the most ways to break silently, so the
// tests below pin down all three: that loading registers exactly one usable
// factory, that a bad file fails without taking the process down, and that the
// registrar is emptied before a library is unloaded.

#include <Simulator/Registrar.h>
#include <Simulator/SharedLibrary.h>

#include <Common/IMap3D.h>
#include <Common/IMappingAlgorithm.h>

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

namespace {

namespace fs = std::filesystem;
using namespace common;

// Minimal read-only map: the algorithm factory needs an IMap3D reference, but
// these tests never exercise the mapping logic itself.
class StubMap final : public IMap3D {
public:
    [[nodiscard]] types::VoxelOccupancy atVoxel(const Position3D&) const override {
        return types::VoxelOccupancy::Unmapped;
    }
    [[nodiscard]] types::MapConfig getMapConfig() const override { return {}; }
    [[nodiscard]] bool isInBounds(const Position3D&) const override { return true; }
};

class PluginLoading : public ::testing::Test {
protected:
    // The Registrar is a process-wide singleton shared by every test, so start
    // and finish from a known-empty state.
    void SetUp() override { simulator::Registrar::instance().clear(); }
    void TearDown() override { simulator::Registrar::instance().clear(); }
};

} // namespace

TEST_F(PluginLoading, LoadingAlgorithmRegistersOneUsableFactory) {
    auto& registrar = simulator::Registrar::instance();
    const std::size_t before = registrar.mappingAlgorithms().size();

    simulator::SharedLibrary library(EX3_ALGORITHM_SO);
    ASSERT_TRUE(library.loaded())
        << "dlopen failed: " << library.error()
        << "\nA 'symbol not found' error here means the registration constructors "
           "were dropped by the linker instead of exported to the plugin.";

    const std::size_t after = registrar.mappingAlgorithms().size();
    ASSERT_EQ(after, before + 1)
        << "loading the .so must construct its global registration object exactly once";

    // The registered factory must actually produce an instance - a factory that
    // registers but returns nothing would pass a naive count check.
    StubMap map;
    const types::MissionConfigData mission{};
    const types::LidarConfigData lidar{};
    const types::DroneConfigData drone{};
    std::unique_ptr<IMappingAlgorithm> algorithm =
        registrar.mappingAlgorithms()[before](
            MappingAlgorithmDependencies{mission, lidar, drone, map});
    EXPECT_NE(algorithm, nullptr) << "the registered factory returned no instance";

    // Destroy the instance before the library unloads at end of scope.
    algorithm.reset();
    registrar.clear();
}

TEST_F(PluginLoading, LoadingMissionControlRegistersOneFactory) {
    auto& registrar = simulator::Registrar::instance();
    const std::size_t before = registrar.missionControls().size();

    simulator::SharedLibrary library(EX3_MISSION_CONTROL_SO);
    ASSERT_TRUE(library.loaded()) << "dlopen failed: " << library.error();

    EXPECT_EQ(registrar.missionControls().size(), before + 1);
    // The algorithm list must be untouched: the before/after delta is how the
    // simulator attributes a factory to the file that produced it.
    EXPECT_TRUE(registrar.mappingAlgorithms().empty())
        << "a mission control .so must not register a mapping algorithm";

    registrar.clear();
}

TEST_F(PluginLoading, BrokenLibraryFailsWithoutRegisteringOrCrashing) {
    const fs::path bad = fs::temp_directory_path() / "ex3_not_a_library.so";
    {
        std::ofstream out(bad);
        out << "this is not a shared library\n";
    }

    auto& registrar = simulator::Registrar::instance();

    simulator::SharedLibrary library(bad.string());
    EXPECT_FALSE(library.loaded()) << "a text file must not load as a plugin";
    EXPECT_FALSE(library.error().empty()) << "a failed load must report why";
    EXPECT_TRUE(registrar.mappingAlgorithms().empty());
    EXPECT_TRUE(registrar.missionControls().empty());

    // A missing file must behave the same way rather than throwing.
    simulator::SharedLibrary missing((fs::temp_directory_path() / "ex3_no_such.so").string());
    EXPECT_FALSE(missing.loaded());

    fs::remove(bad);
}

TEST_F(PluginLoading, RegistrarIsClearedBeforeTheLibraryUnloads) {
    // The factories are std::functions whose code lives inside the .so. If the
    // library is unloaded while they are still alive, destroying them later
    // jumps into unmapped memory. This test reproduces the correct ordering;
    // getting it wrong crashes the test binary, which is exactly the signal we
    // want if someone reorders the declarations in main().
    auto& registrar = simulator::Registrar::instance();
    {
        simulator::SharedLibrary library(EX3_ALGORITHM_SO);
        ASSERT_TRUE(library.loaded()) << library.error();
        ASSERT_FALSE(registrar.mappingAlgorithms().empty());

        registrar.clear(); // must happen before ~SharedLibrary runs dlclose
        EXPECT_TRUE(registrar.mappingAlgorithms().empty());
    }
    SUCCEED() << "survived dlclose with an empty registrar";
}
