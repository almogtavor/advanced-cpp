#pragma once

#include <vector>
#include "MappingAlgorithmFactory.h"
#include "MissionControlFactory.h"

namespace simulator {

class Registrar {
public:
    static Registrar& instance();

    void add(MappingAlgorithmFactory factory);
    void add(MissionControlFactory factory);

    const std::vector<MappingAlgorithmFactory>& mappingAlgorithms() const;
    const std::vector<MissionControlFactory>& missionControls() const;

private:
    Registrar() = default;

    std::vector<MappingAlgorithmFactory> mappingAlgorithms_;
    std::vector<MissionControlFactory> missionControls_;
};
} // namespace simulator