#pragma once

#include "units/Units.h"

namespace drone {

// Continuous 3D position. Coordinates are strong-typed Lengths.
struct Position {
    units::Length x{};
    units::Length y{};
    units::Length z{};

    constexpr Position() = default;
    constexpr Position(units::Length x_, units::Length y_, units::Length z_)
        : x(x_), y(y_), z(z_) {}

    constexpr bool operator==(const Position& o) const {
        return x == o.x && y == o.y && z == o.z;
    }
};

// Integer voxel index used internally by the world grid.
struct Cell {
    int x{};
    int y{};
    int z{};

    constexpr bool operator==(const Cell& o) const {
        return x == o.x && y == o.y && z == o.z;
    }
};

struct CellHash {
    std::size_t operator()(const Cell& c) const noexcept {
        // Mix the three coordinates into a single 64-bit hash.
        std::size_t h = static_cast<std::size_t>(c.x) * 73856093u;
        h ^= static_cast<std::size_t>(c.y) * 19349663u;
        h ^= static_cast<std::size_t>(c.z) * 83492791u;
        return h;
    }
};

} // namespace drone
