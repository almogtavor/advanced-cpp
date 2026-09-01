#include "Simulator/LoadedAlgorithmLibrary.h"
#include "Simulator/Registrar.h"
#include "Simulator/SharedLibrary.h"

#include <iostream>
#include <string>
#include <utility>
#include <vector>

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: "
                  << argv[0]
                  << " <algorithm1.so> [algorithm2.so ...]\n";
        return 1;
    }

    auto& registrar = simulator::Registrar::instance();

    std::vector<simulator::LoadedAlgorithmLibrary> loaded_libraries;

    for (int i = 1; i < argc; ++i) {
        std::string path = argv[i];

        std::size_t before =
            registrar.mappingAlgorithms().size();

        simulator::SharedLibrary library(path);

        if (!library.loaded()) {
            std::cerr << "Failed to load "
                      << path
                      << ": "
                      << library.error()
                      << '\n';
            continue;
        }

        std::size_t after =
            registrar.mappingAlgorithms().size();

        std::size_t registered =
            after - before;

        if (registered != 1) {
            std::cerr << path
                      << " registered "
                      << registered
                      << " algorithm factories instead of 1\n";

            continue;
        }

        // Store the loaded library and its factory index for later use.
        loaded_libraries.emplace_back(
            std::move(path),
            std::move(library),
            before
        );

        std::cout << "Loaded "
                  << argv[i]
                  << ", factory index "
                  << before
                  << '\n';
    }

    std::cout << registrar.mappingAlgorithms().size()
              << " algorithm factory registered\n";

    // -----------------------------------------
    // Create and run algorithm instances here.
    //
    // Example idea:
    //
    // auto algorithm =
    //     registrar.mappingAlgorithms()[factory_index](dependencies);
    //
    // Make sure all algorithm instances are
    // destroyed before the cleanup below.
    // -----------------------------------------

    registrar.clear();

    loaded_libraries.clear();

    return 0;
}