#include <utility>

#include "Simulator/Registrar.h"
#include "MissionControlRegistration.h"

common::MissionControlRegistration::MissionControlRegistration(
    MissionControlFactory factory) {
    Registrar::instance().add(std::move(factory));
}