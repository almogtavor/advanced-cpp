#pragma once

#include "Simulator/SharedLibrary.h"

#include <cstddef>
#include <string>
#include <utility>

namespace simulator {

struct LoadedAlgorithmLibrary {
    std::string path;
    SharedLibrary library;
    std::size_t factory_index;

    LoadedAlgorithmLibrary(
        std::string path_,
        SharedLibrary library_,
        std::size_t factory_index_)
        : path(std::move(path_)),
          library(std::move(library_)),
          factory_index(factory_index_) {}
};

} // namespace simulator