#include "Simulator/Registrar.h"
#include "Simulator/SharedLibrary.h"

#include <iostream>

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <algorithm.so>\n";
        return 1;
    }

    // To control exactly when the .so is unloaded, we scope the SharedLibrary instance.
    {
        simulator::SharedLibrary library(argv[1]);

        if (!library.loaded()) {
            std::cerr << library.error() << '\n';
            return 1;
        }

        auto& registrar = simulator::Registrar::instance();

        std::cout
            << registrar.mappingAlgorithms().size()
            << " algorithm factory registered\n";

        // Run algorithms here.
        // Destroy every algorithm / MissionControl instance first.

        // Destroy std::functions whose code lives in the .so.
        registrar.clear();

    } // NOW SharedLibrary::~SharedLibrary() calls dlclose safely.

    return 0;
}