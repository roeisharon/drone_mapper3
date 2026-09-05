#pragma once

#include <cstddef>
#include <filesystem>
#include <iosfwd>
#include <optional>

namespace simulator {

enum class RunMode { Comparative, Competition };

// The validated command line. num_threads is normalized: missing, 0 or 1 all mean "run on the main
// thread only" (worker_threads() == 0); n >= 2 means n worker threads in addition to main.
struct ParsedArgs {
    RunMode mode = RunMode::Comparative;
    std::filesystem::path simulation_yaml;

    // Comparative mode
    std::filesystem::path mission_control_folder;
    std::filesystem::path algorithm_so;

    // Competition mode
    std::filesystem::path mission_control_so;
    std::filesystem::path algorithms_folder;

    std::size_t num_threads = 1;
    bool verbose = false;

    [[nodiscard]] std::size_t workerThreads() const noexcept {
        return num_threads >= 2 ? num_threads : 0;
    }
};

// Prints the usage block (both modes) to os.
void printUsage(std::ostream& os);

// Parses and validates argv. All arguments are order-independent. On any failure — unsupported
// arguments (all listed), missing arguments (all detailed), or a file/folder that does not exist
// or cannot be opened/traversed — prints the usage plus the error messages to stderr and returns
// nullopt (the program should then finish, per the spec).
std::optional<ParsedArgs> parseArgs(int argc, char* argv[]);

} // namespace simulator
