#include "io/MapIO.h"

#include <cctype>
#include <fstream>
#include <sstream>
#include <string>

namespace drone {

namespace {

std::string strip(const std::string& in) {
    std::string s = in;
    const auto hash = s.find('#');
    if (hash != std::string::npos) s.erase(hash);
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) s.erase(s.begin());
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back())))  s.pop_back();
    return s;
}

bool parse_double(const std::string& tok, double& out) {
    try {
        std::size_t consumed = 0;
        out = std::stod(tok, &consumed);
        return consumed == tok.size();
    } catch (...) { return false; }
}
bool parse_int(const std::string& tok, int& out) {
    try {
        std::size_t consumed = 0;
        out = std::stoi(tok, &consumed);
        return consumed == tok.size();
    } catch (...) { return false; }
}

char encode(int8_t v) {
    switch (v) {
        case voxel::kEmpty:       return '.';
        case voxel::kOccupied:    return '#';
        case voxel::kUnmapped:    return '?';
        case voxel::kOutOfBounds: return '_';
        default:                  return '?';
    }
}

int8_t decode(char c) {
    switch (c) {
        case '.': return voxel::kEmpty;
        case '#': return voxel::kOccupied;
        case '?': return voxel::kUnmapped;
        case '_': return voxel::kOutOfBounds;
        default:  return voxel::kEmpty; // recovery: treat unknown as empty
    }
}

ParseResult write_grid(const std::string& path, const VoxelGrid& grid) {
    ParseResult result;
    std::ofstream out(path);
    if (!out) {
        result.ok = false;
        result.errors.push_back("map_output: cannot open file: " + path);
        return result;
    }
    out << "# Drone Mapper output map\n";
    out << "cell_size " << grid.cell_size().in_cm() << "\n";
    out << "origin "    << grid.origin().x.in_cm() << " "
                        << grid.origin().y.in_cm() << " "
                        << grid.origin().z.in_cm() << "\n";
    out << "size " << grid.nx() << " " << grid.ny() << " " << grid.nz() << "\n";

    for (int z = 0; z < grid.nz(); ++z) {
        out << "layer " << z << "\n";
        for (int y = 0; y < grid.ny(); ++y) {
            for (int x = 0; x < grid.nx(); ++x) {
                out << encode(grid.get(Cell{x, y, z}));
            }
            out << "\n";
        }
    }
    return result;
}

} // namespace

ParseResult MapIO::load_truth(const std::string& path, BuildingTruth& out) {
    ParseResult result;
    std::ifstream in(path);
    if (!in) {
        result.ok = false;
        result.errors.push_back("map_input: cannot open file: " + path);
        return result;
    }

    units::Length cell_size{10 * units::cm};
    Position origin{};
    int nx = 0, ny = 0, nz = 0;
    bool grid_initialized = false;

    int current_layer = -1;
    int rows_read = 0;
    std::vector<int8_t> data;

    auto ensure_grid = [&](ParseResult& r) -> bool {
        if (grid_initialized) return true;
        if (nx <= 0 || ny <= 0 || nz <= 0) {
            r.errors.push_back("map_input: layer block before size declaration");
            return false;
        }
        data.assign(static_cast<std::size_t>(nx) * ny * nz, voxel::kEmpty);
        grid_initialized = true;
        return true;
    };

    std::string raw;
    int line_no = 0;
    while (std::getline(in, raw)) {
        ++line_no;

        // Layer rows can legitimately start with '#' (the wall character),
        // so they bypass the comment-stripping done by strip().
        if (current_layer >= 0 && rows_read < ny) {
            // Trim CR for files written on Windows.
            std::string row = raw;
            if (!row.empty() && row.back() == '\r') row.pop_back();
            if (static_cast<int>(row.size()) < nx) {
                result.errors.push_back("map_input:" + std::to_string(line_no) +
                    ": layer row too short, padding with empty");
            }
            for (int x = 0; x < nx; ++x) {
                const char c = (x < static_cast<int>(row.size())) ? row[x] : '.';
                const int idx = (current_layer * ny + rows_read) * nx + x;
                data[static_cast<std::size_t>(idx)] = decode(c);
            }
            ++rows_read;
            if (rows_read == ny) current_layer = -1;
            continue;
        }

        const std::string line = strip(raw);
        if (line.empty()) continue;

        std::istringstream iss(line);
        std::string key;
        iss >> key;
        if (key == "cell_size") {
            std::string v; iss >> v;
            double d; if (parse_double(v, d)) cell_size = units::Length(d);
        } else if (key == "origin") {
            std::string a, b, c;
            iss >> a >> b >> c;
            double x, y, z;
            if (parse_double(a, x) && parse_double(b, y) && parse_double(c, z)) {
                origin = Position{units::Length(x), units::Length(y), units::Length(z)};
            }
        } else if (key == "size") {
            std::string a, b, c;
            iss >> a >> b >> c;
            int x, y, z;
            if (parse_int(a, x) && parse_int(b, y) && parse_int(c, z)) {
                nx = x; ny = y; nz = z;
            }
        } else if (key == "layer") {
            if (!ensure_grid(result)) return result;
            std::string v; iss >> v;
            int z;
            if (!parse_int(v, z) || z < 0 || z >= nz) {
                result.errors.push_back("map_input:" + std::to_string(line_no) +
                    ": invalid layer index, ignored");
                current_layer = -1;
                continue;
            }
            current_layer = z;
            rows_read = 0;
        } else {
            result.errors.push_back("map_input:" + std::to_string(line_no) +
                ": unknown key '" + key + "' (ignored)");
        }
    }

    if (!grid_initialized) {
        result.ok = false;
        result.errors.push_back("map_input: file contained no grid data");
        return result;
    }

    VoxelGrid grid(cell_size, origin, nx, ny, nz, voxel::kEmpty);
    for (int z = 0; z < nz; ++z)
        for (int y = 0; y < ny; ++y)
            for (int x = 0; x < nx; ++x) {
                const int idx = (z * ny + y) * nx + x;
                grid.set(Cell{x, y, z}, data[static_cast<std::size_t>(idx)]);
            }
    out = BuildingTruth(std::move(grid));
    return result;
}

ParseResult MapIO::save_map(const std::string& path, const BuildingMap& map) {
    return write_grid(path, map.grid());
}

ParseResult MapIO::save_truth(const std::string& path, const BuildingTruth& truth) {
    return write_grid(path, truth.grid());
}

} // namespace drone
