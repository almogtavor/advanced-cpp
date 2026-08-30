#include <utility>

#include "Simulator/Registrar.h"
#include "MappingAlgorithmRegistration.h"

common::MappingAlgorithmRegistration::MappingAlgorithmRegistration(
    MappingAlgorithmFactory factory) {
    Registrar::instance().add(std::move(factory));
}