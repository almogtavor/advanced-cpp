#include <iostream>
#include <string_view>


// The singleton in the Simulator holding a std::vector<MappingAlgorithmFactory>
class Simulator {
public:
    static Simulator& instance() {
        static Simulator sim;
        return sim;
    }

private:
    Simulator() = default;
};

// Walking skeleton: echo the command line so we can confirm the executable
// builds, links and runs. Argument parsing, plugin loading (dlopen) and the
// comparative / competition run modes still have to be implemented.
int main(int argc, char* argv[]) {
    std::cout << "simulator_323084962_212223036\n";
    std::cout << "argc = " << argc << '\n';
    for (int i = 0; i < argc; ++i) {
        std::cout << "  argv[" << i << "] = " << std::string_view{argv[i]} << '\n';
    }
    return 0;
}
