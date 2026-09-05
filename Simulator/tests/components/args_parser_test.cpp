// The full CLI validation matrix (spec: order-independent args; ALL unsupported args listed; ALL
// missing args detailed; bad file/folder paths rejected with usage; exact message text is ours).
// Plain TEST (no fixture) — the ArgsParser suite already has plain TESTs elsewhere and gtest
// forbids mixing TEST and TEST_F under one suite name.

#include "args/ArgsParser.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;

// On-disk fixtures for the parser's existence checks, (re)created per call.
struct PathFixture {
    fs::path base;
    std::string sim_yaml;   // existing regular file
    std::string algo_so;    // existing regular file
    std::string mc_so;      // existing regular file
    std::string mc_dir;     // existing directory
    std::string algo_dir;   // existing directory
};

PathFixture makePaths() {
    PathFixture f;
    f.base = fs::temp_directory_path() / "args_parser_test";
    fs::create_directories(f.base / "mc_dir");
    fs::create_directories(f.base / "algo_dir");
    for (const char* name : {"sim.yaml", "algo.so", "mc.so"}) {
        std::ofstream out{f.base / name};
        out << "x";
    }
    f.sim_yaml = (f.base / "sim.yaml").string();
    f.algo_so = (f.base / "algo.so").string();
    f.mc_so = (f.base / "mc.so").string();
    f.mc_dir = (f.base / "mc_dir").string();
    f.algo_dir = (f.base / "algo_dir").string();
    return f;
}

// Runs parseArgs over the given tokens (argv[0] added), returning the result and what was
// printed to stderr (usage + error lines on failure).
struct ParseOutcome {
    std::optional<simulator::ParsedArgs> args;
    std::string stderr_text;
};

ParseOutcome parse(std::vector<std::string> tokens) {
    tokens.insert(tokens.begin(), "simulator_322526864_323020347");
    std::vector<char*> argv;
    argv.reserve(tokens.size());
    for (auto& token : tokens) {
        argv.push_back(token.data());
    }
    ::testing::internal::CaptureStderr();
    ParseOutcome outcome;
    outcome.args = simulator::parseArgs(static_cast<int>(argv.size()), argv.data());
    outcome.stderr_text = ::testing::internal::GetCapturedStderr();
    return outcome;
}

// ---- valid invocations ----

TEST(ArgsParser, ComparativeHappyPathParsesAllFields) {
    const auto f = makePaths();
    const auto out = parse({"-comparative", "simulation=" + f.sim_yaml,
                            "mission_control_folder=" + f.mc_dir, "algorithm=" + f.algo_so});
    ASSERT_TRUE(out.args.has_value()) << out.stderr_text;
    EXPECT_EQ(out.args->mode, simulator::RunMode::Comparative);
    EXPECT_EQ(out.args->simulation_yaml, fs::path{f.sim_yaml});
    EXPECT_EQ(out.args->mission_control_folder, fs::path{f.mc_dir});
    EXPECT_EQ(out.args->algorithm_so, fs::path{f.algo_so});
    EXPECT_EQ(out.args->num_threads, 1u); // default when absent
    EXPECT_FALSE(out.args->verbose);
}

TEST(ArgsParser, CompetitionHappyPathParsesAllFields) {
    const auto f = makePaths();
    const auto out = parse({"-competition", "simulation=" + f.sim_yaml,
                            "mission_control=" + f.mc_so, "algorithms_folder=" + f.algo_dir,
                            "num_threads=4", "-verbose"});
    ASSERT_TRUE(out.args.has_value()) << out.stderr_text;
    EXPECT_EQ(out.args->mode, simulator::RunMode::Competition);
    EXPECT_EQ(out.args->mission_control_so, fs::path{f.mc_so});
    EXPECT_EQ(out.args->algorithms_folder, fs::path{f.algo_dir});
    EXPECT_EQ(out.args->num_threads, 4u);
    EXPECT_TRUE(out.args->verbose);
}

TEST(ArgsParser, ArgumentsAreOrderIndependent) {
    const auto f = makePaths();
    const auto out = parse({"algorithm=" + f.algo_so, "-verbose",
                            "mission_control_folder=" + f.mc_dir, "num_threads=3",
                            "simulation=" + f.sim_yaml, "-comparative"});
    ASSERT_TRUE(out.args.has_value()) << out.stderr_text;
    EXPECT_EQ(out.args->num_threads, 3u);
    EXPECT_TRUE(out.args->verbose);
}

TEST(ArgsParser, NumThreadsZeroAndOneMeanMainThreadOnly) {
    const auto f = makePaths();
    for (const char* value : {"0", "1"}) {
        const auto out = parse({"-comparative", "simulation=" + f.sim_yaml,
                                "mission_control_folder=" + f.mc_dir,
                                "algorithm=" + f.algo_so,
                                std::string("num_threads=") + value});
        ASSERT_TRUE(out.args.has_value()) << out.stderr_text;
        EXPECT_EQ(out.args->workerThreads(), 0u) << "num_threads=" << value;
    }
}

// ---- mode-flag errors ----

TEST(ArgsParser, NoArgumentsFailsWithUsage) {
    const auto out = parse({});
    EXPECT_FALSE(out.args.has_value());
    EXPECT_NE(out.stderr_text.find("Usage:"), std::string::npos);
    EXPECT_NE(out.stderr_text.find("missing mode flag"), std::string::npos);
}

TEST(ArgsParser, BothModeFlagsFail) {
    const auto f = makePaths();
    const auto out = parse({"-comparative", "-competition", "simulation=" + f.sim_yaml,
                            "mission_control_folder=" + f.mc_dir, "algorithm=" + f.algo_so});
    EXPECT_FALSE(out.args.has_value());
    EXPECT_NE(out.stderr_text.find("choose exactly one"), std::string::npos);
}

// ---- unsupported arguments (spec: point at ALL of them) ----

TEST(ArgsParser, UnsupportedFlagFails) {
    const auto f = makePaths();
    const auto out = parse({"-comparative", "-bogus", "simulation=" + f.sim_yaml,
                            "mission_control_folder=" + f.mc_dir, "algorithm=" + f.algo_so});
    EXPECT_FALSE(out.args.has_value());
    EXPECT_NE(out.stderr_text.find("-bogus"), std::string::npos);
}

TEST(ArgsParser, AllUnsupportedArgumentsListedTogether) {
    const auto f = makePaths();
    const auto out = parse({"-comparative", "-bogus", "mystery=42", "simulation=" + f.sim_yaml,
                            "mission_control_folder=" + f.mc_dir, "algorithm=" + f.algo_so});
    EXPECT_FALSE(out.args.has_value());
    EXPECT_NE(out.stderr_text.find("-bogus"), std::string::npos);
    EXPECT_NE(out.stderr_text.find("mystery=42"), std::string::npos);
}

TEST(ArgsParser, CrossModeKeyIsUnsupported) {
    const auto f = makePaths();
    // A competition-only key in comparative mode is "unsupported" for that mode.
    const auto out = parse({"-comparative", "simulation=" + f.sim_yaml,
                            "mission_control_folder=" + f.mc_dir, "algorithm=" + f.algo_so,
                            "algorithms_folder=" + f.algo_dir});
    EXPECT_FALSE(out.args.has_value());
    EXPECT_NE(out.stderr_text.find("algorithms_folder"), std::string::npos);
}

TEST(ArgsParser, TokenStartingWithEqualsIsUnsupported) {
    const auto f = makePaths();
    const auto out = parse({"-comparative", "=oops", "simulation=" + f.sim_yaml,
                            "mission_control_folder=" + f.mc_dir, "algorithm=" + f.algo_so});
    EXPECT_FALSE(out.args.has_value());
    EXPECT_NE(out.stderr_text.find("=oops"), std::string::npos);
}

// ---- missing arguments (spec: detail ALL of them) ----

TEST(ArgsParser, MissingSingleMandatoryArgumentDetailed) {
    const auto f = makePaths();
    const auto out = parse({"-comparative", "simulation=" + f.sim_yaml,
                            "mission_control_folder=" + f.mc_dir});
    EXPECT_FALSE(out.args.has_value());
    EXPECT_NE(out.stderr_text.find("missing mandatory argument: algorithm="), std::string::npos);
}

TEST(ArgsParser, MissingMultipleMandatoryArgumentsAllDetailed) {
    const auto out = parse({"-competition"});
    EXPECT_FALSE(out.args.has_value());
    EXPECT_NE(out.stderr_text.find("simulation="), std::string::npos);
    EXPECT_NE(out.stderr_text.find("mission_control="), std::string::npos);
    EXPECT_NE(out.stderr_text.find("algorithms_folder="), std::string::npos);
}

// ---- malformed values ----

TEST(ArgsParser, DuplicateKeyFails) {
    const auto f = makePaths();
    const auto out = parse({"-comparative", "simulation=" + f.sim_yaml,
                            "simulation=" + f.sim_yaml, "mission_control_folder=" + f.mc_dir,
                            "algorithm=" + f.algo_so});
    EXPECT_FALSE(out.args.has_value());
    EXPECT_NE(out.stderr_text.find("more than once"), std::string::npos);
}

TEST(ArgsParser, EmptyValueFails) {
    const auto f = makePaths();
    const auto out = parse({"-comparative", "simulation=",
                            "mission_control_folder=" + f.mc_dir, "algorithm=" + f.algo_so});
    EXPECT_FALSE(out.args.has_value());
    EXPECT_NE(out.stderr_text.find("empty value"), std::string::npos);
}

TEST(ArgsParser, NumThreadsNonNumericFails) {
    const auto f = makePaths();
    const auto out = parse({"-comparative", "simulation=" + f.sim_yaml,
                            "mission_control_folder=" + f.mc_dir, "algorithm=" + f.algo_so,
                            "num_threads=four"});
    EXPECT_FALSE(out.args.has_value());
    EXPECT_NE(out.stderr_text.find("non-negative integer"), std::string::npos);
}

TEST(ArgsParser, NumThreadsNegativeFails) {
    const auto f = makePaths();
    const auto out = parse({"-comparative", "simulation=" + f.sim_yaml,
                            "mission_control_folder=" + f.mc_dir, "algorithm=" + f.algo_so,
                            "num_threads=-3"});
    EXPECT_FALSE(out.args.has_value());
}

TEST(ArgsParser, NumThreadsOverflowFails) {
    const auto f = makePaths();
    const auto out = parse({"-comparative", "simulation=" + f.sim_yaml,
                            "mission_control_folder=" + f.mc_dir, "algorithm=" + f.algo_so,
                            "num_threads=99999999999999999999999999"});
    EXPECT_FALSE(out.args.has_value());
    EXPECT_NE(out.stderr_text.find("out of range"), std::string::npos);
}

// ---- file/folder existence and kind (spec: usage + proper error, then finish) ----

TEST(ArgsParser, SimulationFileMissingFails) {
    const auto f = makePaths();
    const auto out = parse({"-comparative", "simulation=" + (f.base / "nope.yaml").string(),
                            "mission_control_folder=" + f.mc_dir, "algorithm=" + f.algo_so});
    EXPECT_FALSE(out.args.has_value());
    EXPECT_NE(out.stderr_text.find("does not exist"), std::string::npos);
}

TEST(ArgsParser, SimulationPathBeingDirectoryFails) {
    const auto f = makePaths();
    const auto out = parse({"-comparative", "simulation=" + f.mc_dir,
                            "mission_control_folder=" + f.mc_dir, "algorithm=" + f.algo_so});
    EXPECT_FALSE(out.args.has_value());
    EXPECT_NE(out.stderr_text.find("not a readable file"), std::string::npos);
}

TEST(ArgsParser, AlgorithmFileMissingFails) {
    const auto f = makePaths();
    const auto out = parse({"-comparative", "simulation=" + f.sim_yaml,
                            "mission_control_folder=" + f.mc_dir,
                            "algorithm=" + (f.base / "nope.so").string()});
    EXPECT_FALSE(out.args.has_value());
    EXPECT_NE(out.stderr_text.find("nope.so"), std::string::npos);
}

TEST(ArgsParser, MissionControlFolderMissingFails) {
    const auto f = makePaths();
    const auto out = parse({"-comparative", "simulation=" + f.sim_yaml,
                            "mission_control_folder=" + (f.base / "no_dir").string(),
                            "algorithm=" + f.algo_so});
    EXPECT_FALSE(out.args.has_value());
    EXPECT_NE(out.stderr_text.find("does not exist"), std::string::npos);
}

TEST(ArgsParser, MissionControlFolderBeingFileFails) {
    const auto f = makePaths();
    const auto out = parse({"-comparative", "simulation=" + f.sim_yaml,
                            "mission_control_folder=" + f.algo_so, "algorithm=" + f.algo_so});
    EXPECT_FALSE(out.args.has_value());
    EXPECT_NE(out.stderr_text.find("not a folder"), std::string::npos);
}

TEST(ArgsParser, CompetitionMissionControlFileMissingFails) {
    const auto f = makePaths();
    const auto out = parse({"-competition", "simulation=" + f.sim_yaml,
                            "mission_control=" + (f.base / "nope.so").string(),
                            "algorithms_folder=" + f.algo_dir});
    EXPECT_FALSE(out.args.has_value());
}

TEST(ArgsParser, CompetitionAlgorithmsFolderBeingFileFails) {
    const auto f = makePaths();
    const auto out = parse({"-competition", "simulation=" + f.sim_yaml,
                            "mission_control=" + f.mc_so, "algorithms_folder=" + f.mc_so});
    EXPECT_FALSE(out.args.has_value());
    EXPECT_NE(out.stderr_text.find("not a folder"), std::string::npos);
}

TEST(ArgsParser, MultipleErrorCategoriesReportedTogether) {
    // One invocation with an unsupported token, a missing mandatory argument, AND a bad value —
    // all three surface in a single usage printout.
    const auto f = makePaths();
    const auto out = parse({"-comparative", "-bogus", "simulation=" + f.sim_yaml,
                            "mission_control_folder=" + f.mc_dir, "num_threads=four"});
    EXPECT_FALSE(out.args.has_value());
    EXPECT_NE(out.stderr_text.find("Usage:"), std::string::npos);
    EXPECT_NE(out.stderr_text.find("-bogus"), std::string::npos);
    EXPECT_NE(out.stderr_text.find("missing mandatory argument: algorithm="), std::string::npos);
    EXPECT_NE(out.stderr_text.find("non-negative integer"), std::string::npos);
}

} // namespace
