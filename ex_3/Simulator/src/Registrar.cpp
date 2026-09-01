#include "Simulator/Registrar.h"

#include <utility>

namespace simulator {

Registrar& Registrar::instance() {
    static Registrar registrar;
    return registrar;
}

void Registrar::add(common::MappingAlgorithmFactory factory) {
    mapping_algorithms_.push_back(std::move(factory));
}

void Registrar::add(common::MissionControlFactory factory) {
    mission_controls_.push_back(std::move(factory));
}

const std::vector<common::MappingAlgorithmFactory>&
Registrar::mappingAlgorithms() const {
    return mapping_algorithms_;
}

const std::vector<common::MissionControlFactory>&
Registrar::missionControls() const {
    return mission_controls_;
}

void Registrar::clear() {
    mapping_algorithms_.clear();
    mission_controls_.clear();
}

} // namespace simulator