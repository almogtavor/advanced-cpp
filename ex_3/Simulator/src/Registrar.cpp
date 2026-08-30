#include "Simulator/Registrar.h"

Registrar& Registrar::instance() {
    static Registrar registrar;
    return registrar;
}

void Registrar::add(MappingAlgorithmFactory factory) {
    mappingAlgorithms_.push_back(std::move(factory));
}

void Registrar::add(MissionControlFactory factory) {
    missionControls_.push_back(std::move(factory));
}

const std::vector<MappingAlgorithmFactory>& Registrar::mappingAlgorithms() const {
    return mappingAlgorithms_;
}

const std::vector<MissionControlFactory>& Registrar::missionControls() const {
    return missionControls_;
}