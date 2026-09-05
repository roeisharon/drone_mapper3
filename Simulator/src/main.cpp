#include "args/ArgsParser.h"
#include "modes/ModeRunner.h"

#include <exception>
#include <iostream>

int main(int argc, char* argv[]) {
    const auto args = simulator::parseArgs(argc, argv);
    if (!args) {
        return 1; // parseArgs already printed usage + errors
    }

    // The Simulator must not crash for any reason other than a plugin crash — this is the
    // last-resort boundary for everything else.
    try {
        return args->mode == simulator::RunMode::Comparative
                   ? simulator::runComparativeMode(*args)
                   : simulator::runCompetitionMode(*args);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    } catch (...) {
        std::cerr << "Error: unknown failure\n";
        return 1;
    }
}
