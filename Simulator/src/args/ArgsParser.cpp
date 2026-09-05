#include "args/ArgsParser.h"

#include <algorithm>
#include <iostream>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace simulator {

namespace {

struct KeyValue {
    std::string key;
    std::string value;
};

// Comparative / competition key sets (num_threads is optional in both).
constexpr std::string_view kCommonOptionalKeys[] = {"num_threads"};
constexpr std::string_view kComparativeKeys[] = {"simulation", "mission_control_folder", "algorithm"};
constexpr std::string_view kCompetitionKeys[] = {"simulation", "mission_control", "algorithms_folder"};

template <typename Range>
bool contains(const Range& range, std::string_view key) {
    return std::find(std::begin(range), std::end(range), key) != std::end(range);
}

// True when the string is a non-empty run of decimal digits.
bool isUnsignedInteger(std::string_view s) {
    return !s.empty() &&
           std::all_of(s.begin(), s.end(), [](unsigned char c) { return c >= '0' && c <= '9'; });
}

} // namespace

void printUsage(std::ostream& os) {
    os << "Usage:\n"
          "  simulator_322526864_323020347 -comparative simulation=<simulation_composition_yaml>"
          " mission_control_folder=<folder> algorithm=<algorithm_so> [num_threads=<num>] [-verbose]\n"
          "  simulator_322526864_323020347 -competition simulation=<simulation_composition_yaml>"
          " mission_control=<mission_control_so> algorithms_folder=<folder> [num_threads=<num>]"
          " [-verbose]\n"
          "Arguments may appear in any order. '=' must have no spaces around it.\n";
}

std::optional<ParsedArgs> parseArgs(int argc, char* argv[]) {
    bool comparative_flag = false;
    bool competition_flag = false;
    bool verbose = false;
    std::vector<KeyValue> pairs;
    std::vector<std::string> unsupported;

    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];
        if (arg == "-comparative") {
            comparative_flag = true;
        } else if (arg == "-competition") {
            competition_flag = true;
        } else if (arg == "-verbose") {
            verbose = true;
        } else if (const auto eq = arg.find('='); eq != std::string_view::npos && eq > 0) {
            pairs.push_back(KeyValue{std::string(arg.substr(0, eq)), std::string(arg.substr(eq + 1))});
        } else {
            unsupported.push_back(std::string(arg));
        }
    }

    std::vector<std::string> errors;

    // Mode flag: exactly one.
    if (comparative_flag && competition_flag) {
        errors.push_back("both -comparative and -competition were provided; choose exactly one");
    } else if (!comparative_flag && !competition_flag) {
        errors.push_back("missing mode flag: -comparative or -competition");
    }
    const RunMode mode = competition_flag ? RunMode::Competition : RunMode::Comparative;
    const auto& mandatory_keys =
        (mode == RunMode::Competition) ? kCompetitionKeys : kComparativeKeys;

    // Classify key=value pairs; collect duplicates and keys the selected mode does not support.
    ParsedArgs args;
    args.mode = mode;
    args.verbose = verbose;
    std::vector<std::string> seen_keys;
    for (const auto& kv : pairs) {
        if (!contains(mandatory_keys, kv.key) && !contains(kCommonOptionalKeys, kv.key)) {
            unsupported.push_back(kv.key + "=" + kv.value);
            continue;
        }
        if (contains(seen_keys, kv.key)) {
            errors.push_back("argument '" + kv.key + "' was provided more than once");
            continue;
        }
        seen_keys.push_back(kv.key);

        if (kv.value.empty()) {
            errors.push_back("argument '" + kv.key + "' has an empty value");
            continue;
        }

        if (kv.key == "simulation") {
            args.simulation_yaml = kv.value;
        } else if (kv.key == "mission_control_folder") {
            args.mission_control_folder = kv.value;
        } else if (kv.key == "algorithm") {
            args.algorithm_so = kv.value;
        } else if (kv.key == "mission_control") {
            args.mission_control_so = kv.value;
        } else if (kv.key == "algorithms_folder") {
            args.algorithms_folder = kv.value;
        } else if (kv.key == "num_threads") {
            if (!isUnsignedInteger(kv.value)) {
                errors.push_back("num_threads must be a non-negative integer, got '" + kv.value + "'");
            } else {
                try {
                    args.num_threads = std::stoul(kv.value);
                } catch (const std::exception&) {
                    errors.push_back("num_threads value '" + kv.value + "' is out of range");
                }
            }
        }
    }

    if (!unsupported.empty()) {
        std::string msg = "unsupported command line argument(s):";
        for (const auto& u : unsupported) {
            msg += " '" + u + "'";
        }
        errors.push_back(std::move(msg));
    }

    // Missing mandatory arguments — detail every one (spec requirement).
    for (const auto key : mandatory_keys) {
        if (!contains(seen_keys, key)) {
            errors.push_back("missing mandatory argument: " + std::string(key) + "=<value>");
        }
    }

    // Existence checks — only for arguments that were actually provided.
    const auto checkFile = [&errors](const std::filesystem::path& p, std::string_view what) {
        std::error_code ec;
        if (p.empty()) return;
        if (!std::filesystem::exists(p, ec) || ec) {
            errors.push_back(std::string(what) + " '" + p.string() + "' does not exist");
        } else if (!std::filesystem::is_regular_file(p, ec) || ec) {
            errors.push_back(std::string(what) + " '" + p.string() + "' is not a readable file");
        }
    };
    const auto checkFolder = [&errors](const std::filesystem::path& p, std::string_view what) {
        std::error_code ec;
        if (p.empty()) return;
        if (!std::filesystem::exists(p, ec) || ec) {
            errors.push_back(std::string(what) + " '" + p.string() + "' does not exist");
        } else if (!std::filesystem::is_directory(p, ec) || ec) {
            errors.push_back(std::string(what) + " '" + p.string() + "' is not a folder");
        }
    };

    checkFile(args.simulation_yaml, "simulation file");
    if (mode == RunMode::Comparative) {
        checkFolder(args.mission_control_folder, "mission_control_folder");
        checkFile(args.algorithm_so, "algorithm file");
    } else {
        checkFile(args.mission_control_so, "mission_control file");
        checkFolder(args.algorithms_folder, "algorithms_folder");
    }

    if (!errors.empty()) {
        printUsage(std::cerr);
        for (const auto& e : errors) {
            std::cerr << "Error: " << e << "\n";
        }
        return std::nullopt;
    }
    return args;
}

} // namespace simulator
