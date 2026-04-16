#include "TestFramework.hpp"            // Custom lightweight test macros.
#include "drone_mapper/FileParsers.hpp" // Parser functions under test.

#include <filesystem>  // Temporary test directories are built with std::filesystem.
#include <fstream>     // Test input files are written with std::ofstream.

DM_TEST(ParseDroneConfigRecoversMissingKeys) {                // Verify that missing config keys fall back to defaults and produce recovery messages.
  namespace fs = std::filesystem;                             // Short alias to keep the test readable.
  const auto dir = fs::temp_directory_path() / "dm_parser_test";  // Create a dedicated temporary test directory path.
  fs::create_directories(dir);                               // Ensure the temporary directory exists.
  {
    std::ofstream out(dir / "drone_config.txt");            // Create the drone config file.
    out << "max_advance_cm=150\n";                          // Write only one key so the parser must recover the rest.
  }

  dm::ErrorList errors;                                      // Collect recoverable parse warnings here.
  const auto config = dm::parse_drone_config(dir / "drone_config.txt", errors);  // Parse the file.
  DM_ASSERT_EQ(config.max_advance.as_double(), 150.0);       // The explicit value must be preserved.
  DM_ASSERT_TRUE(!errors.empty());                           // At least one missing-key recovery message must be produced.
}

DM_TEST(ParseWorldMapReadsOccupiedCells) {                   // Verify that occupied coordinates are parsed correctly.
  namespace fs = std::filesystem;                            // Short alias to keep the test readable.
  const auto dir = fs::temp_directory_path() / "dm_world_test";  // Create a dedicated temporary test directory path.
  fs::create_directories(dir);                               // Ensure the temporary directory exists.
  {
    std::ofstream out(dir / "map_input.txt");               // Create the world map file.
    out << "size=3,3,2\n";                                 // Write the world size header.
    out << "start=0,0,0\n";                                // Write the start coordinate header.
    out << "1,1,0\n";                                      // Write the first occupied cell.
    out << "2,2,1\n";                                      // Write the second occupied cell.
  }

  dm::ErrorList errors;                                      // Collect recoverable parse warnings here.
  const auto world = dm::parse_world_map(dir / "map_input.txt", errors);  // Parse the file.
  DM_ASSERT_EQ(world.size_x, 3);                             // The X dimension must match the header.
  DM_ASSERT_EQ(world.occupied_cells.size(), 2u);             // Both occupied cell lines must be parsed.
  DM_ASSERT_TRUE(errors.empty());                            // This well-formed file should not produce warnings.
}
