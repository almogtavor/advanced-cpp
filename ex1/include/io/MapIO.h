#pragma once

#include <string>
#include <vector>

#include "io/ConfigParser.h"   // for ParseResult
#include "world/BuildingMap.h"
#include "world/BuildingTruth.h"

namespace drone {

// Map file format (text):
//
//   # comments allowed
//   cell_size <cm>
//   origin <x_cm> <y_cm> <z_cm>
//   size <nx> <ny> <nz>
//   layer <z_index>
//   <ny lines of nx characters each, where:
//      '.' = empty       (0)
//      '#' = occupied    (1)
//      '?' = not mapped  (-1, output only)
//      '_' = out of mapping bounds (-2, output only)>
//
// Layers may appear in any order. Missing layers are filled with '.'
// (kEmpty) on read and '?' (kUnmapped) on write of a drone map.
class MapIO {
public:
    static ParseResult load_truth(const std::string& path, BuildingTruth& out);
    static ParseResult save_map(const std::string& path, const BuildingMap& map);
    static ParseResult save_truth(const std::string& path, const BuildingTruth& truth);
};

} // namespace drone
