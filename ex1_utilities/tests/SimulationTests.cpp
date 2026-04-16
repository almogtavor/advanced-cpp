#include "TestFramework.hpp"          // Custom lightweight test macros.
#include "drone_mapper/Simulator.hpp" // Top-level simulation API under test.

#include <filesystem>  // Temporary test directories are built with std::filesystem.
#include <fstream>     // Test input files are written with std::ofstream.

DM_TEST(SimulationProducesOutputFile) {                      // Verify that a simple simulation run completes and emits map_output.txt.
  namespace fs = std::filesystem;                            // Short alias to keep the test readable.
  const auto dir = fs::temp_directory_path() / "dm_sim_test";  // Create a dedicated temporary test directory path.
  fs::create_directories(dir);                               // Ensure the temporary directory exists.

  {
    std::ofstream out(dir / "drone_config.txt");            // Create the drone capability file.
    out << "min_pass_width_cm=50\n"                        // Write minimum pass width.
        << "min_pass_length_cm=50\n"                       // Write minimum pass length.
        << "min_pass_height_cm=50\n"                       // Write minimum pass height.
        << "lidar_fov_deg=90\n"                            // Write lidar field of view.
        << "lidar_min_range_cm=1\n"                        // Write lidar minimum range.
        << "lidar_max_range_cm=500\n"                      // Write lidar maximum range.
        << "resolution_near_distance_cm=50\n"              // Write near resolution sample distance.
        << "resolution_near_cell_cm=10\n"                  // Write near resolution sample cell size.
        << "resolution_far_distance_cm=300\n"              // Write far resolution sample distance.
        << "resolution_far_cell_cm=20\n"                   // Write far resolution sample cell size.
        << "max_rotate_deg=360\n"                          // Write a large enough rotation limit for the simplified driver.
        << "max_advance_cm=100\n"                          // Write one-cell horizontal movement.
        << "max_elevate_cm=100\n";                         // Write one-cell vertical movement.
  }

  {
    std::ofstream out(dir / "mission_config.txt");          // Create the mission config file.
    out << "boundary_min_x=0\n"                            // Write min X bound.
        << "boundary_min_y=0\n"                            // Write min Y bound.
        << "boundary_max_x=2\n"                            // Write max X bound.
        << "boundary_max_y=2\n"                            // Write max Y bound.
        << "boundary_min_z=0\n"                            // Write min Z bound.
        << "boundary_max_z=0\n"                            // Write max Z bound.
        << "xy_decimals=0\n"                               // Write XY precision.
        << "z_decimals=0\n";                               // Write Z precision.
  }

  {
    std::ofstream out(dir / "map_input.txt");               // Create the hidden world file.
    out << "size=3,3,1\n"                                  // Write world dimensions.
        << "start=0,0,0\n"                                 // Write the drone start position.
        << "1,1,0\n";                                      // Write a single occupied cell.
  }

  const auto result = dm::run_simulation(dir);               // Execute the simulator on the temporary directory.
  DM_ASSERT_TRUE(result.success);                            // The run must complete successfully.
  DM_ASSERT_TRUE(fs::exists(dir / "map_output.txt"));       // The output map file must be created.
}
