// Included by test_main.cpp
#include <cstdio>
#include <fstream>
#include <unistd.h>

#include "io/MapIO.h"

using namespace drone;
using namespace units;

namespace {
std::string write_temp_map(const std::string& contents) {
    static int counter = 0;
    const std::string path = "/tmp/drone_mapper_map_" +
                             std::to_string(::getpid()) + "_" +
                             std::to_string(counter++) + ".txt";
    std::ofstream out(path);
    out << contents;
    return path;
}
} // namespace

TEST(map_io_load_simple_truth) {
    const std::string p = write_temp_map(R"(
cell_size 10
origin 0 0 0
size 5 5 1
layer 0
#####
#...#
#...#
#...#
#####
)");
    BuildingTruth truth;
    auto r = MapIO::load_truth(p, truth);
    CHECK(r.ok);
    CHECK_EQ(truth.grid().nx(), 5);
    CHECK_EQ(truth.grid().ny(), 5);
    CHECK_EQ(truth.grid().nz(), 1);
    CHECK_EQ(static_cast<int>(truth.at(Cell{0, 0, 0})),
             static_cast<int>(voxel::kOccupied));
    CHECK_EQ(static_cast<int>(truth.at(Cell{2, 2, 0})),
             static_cast<int>(voxel::kEmpty));
    CHECK_EQ(static_cast<int>(truth.at(Cell{4, 4, 0})),
             static_cast<int>(voxel::kOccupied));
    std::remove(p.c_str());
}

TEST(map_io_round_trip_truth) {
    BuildingTruth t(VoxelGrid(10 * cm, Position{}, 3, 3, 1, voxel::kEmpty));
    t.grid().set(Cell{0, 0, 0}, voxel::kOccupied);
    t.grid().set(Cell{2, 2, 0}, voxel::kOccupied);

    const std::string path = "/tmp/drone_mapper_rt_" +
                             std::to_string(::getpid()) + ".txt";
    auto wr = MapIO::save_truth(path, t);
    CHECK(wr.ok);

    BuildingTruth loaded;
    auto rd = MapIO::load_truth(path, loaded);
    CHECK(rd.ok);
    CHECK_EQ(loaded.grid().nx(), 3);
    CHECK_EQ(static_cast<int>(loaded.at(Cell{0, 0, 0})),
             static_cast<int>(voxel::kOccupied));
    CHECK_EQ(static_cast<int>(loaded.at(Cell{2, 2, 0})),
             static_cast<int>(voxel::kOccupied));
    CHECK_EQ(static_cast<int>(loaded.at(Cell{1, 1, 0})),
             static_cast<int>(voxel::kEmpty));
    std::remove(path.c_str());
}

TEST(map_io_load_missing_file_fails_cleanly) {
    BuildingTruth t;
    auto r = MapIO::load_truth("/tmp/__missing_map_file__.txt", t);
    CHECK(!r.ok);
}
