#include "common/MissionControlRegistration.h"
#include "Simulator/Registrar.h"

#include <utility>

namespace common {

MissionControlRegistration::MissionControlRegistration(
    MissionControlFactory factory) {

    simulator::Registrar::instance().registerMissionControl(
        std::move(factory));
}

} // namespace common