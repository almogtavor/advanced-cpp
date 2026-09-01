#pragma once

#include "common/MappingAlgorithmFactory.h"
#include "common/MissionControlFactory.h"

#include <cstddef>
#include <vector>

namespace simulator {

class Registrar {
public:
    static Registrar& instance();

    Registrar(const Registrar&) = delete;
    Registrar& operator=(const Registrar&) = delete;

    void registerMappingAlgorithm(common::MappingAlgorithmFactory factory);
    void registerMissionControl(common::MissionControlFactory factory);

    const std::vector<common::MappingAlgorithmFactory>& mappingAlgorithms() const;
    const std::vector<common::MissionControlFactory>& missionControls() const;

    void clear();

private:
    Registrar() = default;

    std::vector<common::MappingAlgorithmFactory> mapping_algorithms_;
    std::vector<common::MissionControlFactory> mission_controls_;
};

} // namespace simulator