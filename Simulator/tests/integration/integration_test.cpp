// End-to-end integration through the REAL product path: runComparativeMode() with actual dlopen'd
// plugin .so files, staged folders, and YAML output parsing. Fixture-plugin cases always build;
// the real-plugin cases compile in only under the root superbuild (where those targets exist).

#include "args/ArgsParser.h"
#include "modes/ModeRunner.h"
#include "registration/Registrar.h"

#include <yaml-cpp/yaml.h>

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;

fs::path miniComposePath() {
    return fs::path{DRONE_TEST_INPUTS_DIR} / "mini_compose.yaml";
}

// Stages a fresh mission-control folder containing the given .so, returns it.
fs::path stageMcFolder(const std::string& test_name, const fs::path& mc_so) {
    const fs::path dir = fs::temp_directory_path() / "sim_integration" / test_name;
    fs::remove_all(dir);
    fs::create_directories(dir);
    fs::copy_file(mc_so, dir / mc_so.filename());
    return dir;
}

// The single comparative_results_<time> folder created inside dir.
fs::path findResultsDir(const fs::path& dir) {
    for (const auto& entry : fs::directory_iterator{dir}) {
        if (entry.is_directory() &&
            entry.path().filename().string().rfind("comparative_results_", 0) == 0) {
            return entry.path();
        }
    }
    return {};
}

simulator::ParsedArgs makeArgs(const fs::path& mc_folder, const fs::path& algo_so,
                               bool verbose = false) {
    simulator::ParsedArgs args;
    args.mode = simulator::RunMode::Comparative;
    args.simulation_yaml = miniComposePath();
    args.mission_control_folder = mc_folder;
    args.algorithm_so = algo_so;
    args.num_threads = 1;
    args.verbose = verbose;
    return args;
}

simulator::ParsedArgs makeCompetitionArgs(const fs::path& algos_folder, const fs::path& mc_so,
                                          bool verbose = false) {
    simulator::ParsedArgs args;
    args.mode = simulator::RunMode::Competition;
    args.simulation_yaml = miniComposePath();
    args.algorithms_folder = algos_folder;
    args.mission_control_so = mc_so;
    args.num_threads = 1;
    args.verbose = verbose;
    return args;
}

// The single competition_<time> folder created inside dir.
fs::path findCompetitionResultsDir(const fs::path& dir) {
    for (const auto& entry : fs::directory_iterator{dir}) {
        if (entry.is_directory() &&
            entry.path().filename().string().rfind("competition_", 0) == 0) {
            return entry.path();
        }
    }
    return {};
}

class Integration : public ::testing::Test {
protected:
    void TearDown() override {
        // The mode driver owns plugin lifecycles, but keep the process-wide registrars clean in
        // case a failing test aborted mid-flow.
        simulator::AlgorithmRegistrar::instance().clear();
        simulator::MissionControlRegistrar::instance().clear();
    }
};

TEST_F(Integration, FixturePluginsComparativeEndToEnd) {
    const fs::path plugin_dir{DRONE_TEST_PLUGIN_DIR};
    const fs::path mc_folder = stageMcFolder("fixture_e2e", plugin_dir / "test_mc_ok.so");

    const int exit_code =
        simulator::runComparativeMode(makeArgs(mc_folder, plugin_dir / "test_algo_ok.so"));
    ASSERT_EQ(exit_code, 0);

    const fs::path results = findResultsDir(mc_folder);
    ASSERT_FALSE(results.empty()) << "no comparative_results_<time> folder created";

    // The single Comparative Simulation Result Output File.
    const auto report = YAML::LoadFile((results / "comparative_report.yaml").string());
    const auto body = report["comparative_report"];
    ASSERT_TRUE(body);
    ASSERT_TRUE(body["results_summary"] && body["results_summary"].IsSequence());
    ASSERT_EQ(body["results_summary"].size(), 1u);
    const auto group = body["results_summary"][0];
    ASSERT_EQ(group["same_results"].size(), 1u);
    EXPECT_EQ(group["same_results"][0].as<std::string>(), "test_mc_ok.so");
    EXPECT_TRUE(body["errors"] && body["errors"].size() == 0);

    // One per-run score report + one output map for the mini composition's single run.
    EXPECT_TRUE(fs::exists(results / "test_mc_ok_run0_result.yaml"));
    EXPECT_TRUE(fs::exists(results / "test_mc_ok_run0_output.npy"));

    const auto run_report = YAML::LoadFile((results / "test_mc_ok_run0_result.yaml").string());
    const auto score_report = run_report["score_report"];
    ASSERT_TRUE(score_report);
    EXPECT_EQ(score_report["summary"]["total_runs"].as<int>(), 1);
    EXPECT_EQ(score_report["error_score"].as<int>(), -1);
}

TEST_F(Integration, EmptyMissionControlFolderFailsCleanly) {
    const fs::path dir = fs::temp_directory_path() / "sim_integration" / "empty_folder";
    fs::remove_all(dir);
    fs::create_directories(dir);
    const int exit_code = simulator::runComparativeMode(
        makeArgs(dir, fs::path{DRONE_TEST_PLUGIN_DIR} / "test_algo_ok.so"));
    EXPECT_EQ(exit_code, 1);
    EXPECT_TRUE(findResultsDir(dir).empty()); // no results folder for a rejected invocation
}

TEST_F(Integration, UnloadableMissionControlGoesToErrorsList) {
    const fs::path plugin_dir{DRONE_TEST_PLUGIN_DIR};
    const fs::path mc_folder = stageMcFolder("bad_mc", plugin_dir / "test_mc_ok.so");
    // A second "mission control" that registers nothing → must land in errors, not in results.
    fs::copy_file(plugin_dir / "test_algo_none.so", mc_folder / "test_algo_none.so");

    const int exit_code =
        simulator::runComparativeMode(makeArgs(mc_folder, plugin_dir / "test_algo_ok.so"));
    ASSERT_EQ(exit_code, 0);

    const fs::path results = findResultsDir(mc_folder);
    ASSERT_FALSE(results.empty());
    const auto body = YAML::LoadFile((results / "comparative_report.yaml").string())
                          ["comparative_report"];
    ASSERT_EQ(body["errors"].size(), 1u);
    EXPECT_EQ(body["errors"][0].as<std::string>(), "test_algo_none.so");
    ASSERT_EQ(body["results_summary"].size(), 1u); // the good MC still ran
}

TEST_F(Integration, FullErrorsTaxonomyAndGrouping) {
    // The complete three-way errors taxonomy plus same_results grouping in ONE staged folder:
    //   test_mc_ok.so / test_mc_ok2.so — execute identically           → one group of two
    //   test_mc_error.so              — executes, every run scores -1 → its own group, NOT errors
    //   test_mc_ctor_throws.so        — loads, can never instantiate  → errors (executed nothing)
    //   test_algo_none.so             — cannot load (0 registrations) → errors
    const fs::path plugin_dir{DRONE_TEST_PLUGIN_DIR};
    const fs::path mc_folder = stageMcFolder("taxonomy", plugin_dir / "test_mc_ok.so");
    fs::copy_file(plugin_dir / "test_mc_ok.so", mc_folder / "test_mc_ok2.so");
    fs::copy_file(plugin_dir / "test_mc_error.so", mc_folder / "test_mc_error.so");
    fs::copy_file(plugin_dir / "test_mc_ctor_throws.so", mc_folder / "test_mc_ctor_throws.so");
    fs::copy_file(plugin_dir / "test_algo_none.so", mc_folder / "test_algo_none.so");

    const int exit_code =
        simulator::runComparativeMode(makeArgs(mc_folder, plugin_dir / "test_algo_ok.so"));
    ASSERT_EQ(exit_code, 0);

    const fs::path results = findResultsDir(mc_folder);
    ASSERT_FALSE(results.empty());
    const auto body = YAML::LoadFile((results / "comparative_report.yaml").string())
                          ["comparative_report"];

    // Grouping: sorted by size descending — the identical pair first, the always-error MC alone.
    ASSERT_EQ(body["results_summary"].size(), 2u);
    ASSERT_EQ(body["results_summary"][0]["same_results"].size(), 2u);
    EXPECT_EQ(body["results_summary"][0]["same_results"][0].as<std::string>(), "test_mc_ok.so");
    EXPECT_EQ(body["results_summary"][0]["same_results"][1].as<std::string>(), "test_mc_ok2.so");
    ASSERT_EQ(body["results_summary"][1]["same_results"].size(), 1u);
    EXPECT_EQ(body["results_summary"][1]["same_results"][0].as<std::string>(), "test_mc_error.so");
    // The always-error component's totals exclude its -1 runs entirely.
    EXPECT_DOUBLE_EQ(body["results_summary"][1]["total_score"].as<double>(), 0.0);
    EXPECT_EQ(body["results_summary"][1]["total_steps"].as<long long>(), 0);

    // Errors: exactly the unloadable and the never-instantiable components, sorted.
    ASSERT_EQ(body["errors"].size(), 2u);
    EXPECT_EQ(body["errors"][0].as<std::string>(), "test_algo_none.so");
    EXPECT_EQ(body["errors"][1].as<std::string>(), "test_mc_ctor_throws.so");

    // Matrix completeness on files: one per-run report per executed component's run (mini
    // composition = 1 run each), and none at all for the errors-list components.
    EXPECT_TRUE(fs::exists(results / "test_mc_ok_run0_result.yaml"));
    EXPECT_TRUE(fs::exists(results / "test_mc_ok2_run0_result.yaml"));
    EXPECT_TRUE(fs::exists(results / "test_mc_error_run0_result.yaml"));
    EXPECT_FALSE(fs::exists(results / "test_mc_ctor_throws_run0_result.yaml"));

    // The always-error component's per-run report carries the -1 score and its error code.
    const auto err_run =
        YAML::LoadFile((results / "test_mc_error_run0_result.yaml").string())["score_report"];
    const auto run_node = err_run["simulations"][0]["missions"][0]["runs"][0];
    EXPECT_EQ(run_node["status"].as<std::string>(), "error");
    EXPECT_DOUBLE_EQ(run_node["score"].as<double>(), -1.0);
    EXPECT_EQ(run_node["error_ref"]["code"].as<std::string>(), "MC_ALWAYS_FAILS");
}

TEST_F(Integration, ComparativeReportGoldenLayout) {
    // Golden-file pin of the comparative_report: exact keys, order, indentation, and flow style,
    // with the volatile parts (paths, timestamp) normalized. Later phases cannot silently bend
    // the format without failing this test.
    const fs::path plugin_dir{DRONE_TEST_PLUGIN_DIR};
    const fs::path mc_folder = stageMcFolder("golden", plugin_dir / "test_mc_ok.so");

    ASSERT_EQ(simulator::runComparativeMode(makeArgs(mc_folder, plugin_dir / "test_algo_ok.so")),
              0);
    const fs::path results = findResultsDir(mc_folder);
    ASSERT_FALSE(results.empty());

    std::ifstream in{results / "comparative_report.yaml"};
    std::stringstream buffer;
    buffer << in.rdbuf();
    std::string text = buffer.str();

    const auto replaceAll = [&text](const std::string& from, const std::string& to) {
        for (std::size_t pos = text.find(from); pos != std::string::npos;
             pos = text.find(from, pos + to.size())) {
            text.replace(pos, from.size(), to);
        }
    };
    replaceAll(miniComposePath().string(), "<COMPOSITION>");
    replaceAll(mc_folder.string(), "<MC_FOLDER>");
    text = std::regex_replace(
        text, std::regex{R"(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z)"}, "<UTC>");

    const std::string expected =
        "comparative_report:\n"
        "  composition_file: <COMPOSITION>\n"
        "  mission_control_folder: <MC_FOLDER>\n"
        "  generated_at_utc: <UTC>\n"
        "  results_summary:\n"
        "    - same_results: [test_mc_ok.so]\n"
        "      total_score: 0\n"
        "      total_steps: 0\n"
        "  errors: []\n";
    EXPECT_EQ(text, expected);
}

TEST_F(Integration, CompetitionFixtureTaxonomyAndOrdering) {
    // Competition mirror of the taxonomy test: one fixed mission control, four algorithm
    // candidates — the identical pair each gets its OWN entry (no grouping in competition),
    // tied entries ordered by name; the never-instantiable and unloadable ones land in errors.
    const fs::path plugin_dir{DRONE_TEST_PLUGIN_DIR};
    const fs::path algos = fs::temp_directory_path() / "sim_integration" / "competition_taxonomy";
    fs::remove_all(algos);
    fs::create_directories(algos);
    fs::copy_file(plugin_dir / "test_algo_ok.so", algos / "test_algo_ok.so");
    fs::copy_file(plugin_dir / "test_algo_ok.so", algos / "test_algo_ok2.so");
    fs::copy_file(plugin_dir / "test_algo_ctor_throws.so", algos / "test_algo_ctor_throws.so");
    fs::copy_file(plugin_dir / "test_algo_none.so", algos / "test_algo_none.so");

    const int exit_code = simulator::runCompetitionMode(
        makeCompetitionArgs(algos, plugin_dir / "test_mc_ok.so"));
    ASSERT_EQ(exit_code, 0);

    const fs::path results = findCompetitionResultsDir(algos);
    ASSERT_FALSE(results.empty()) << "no competition_<time> folder created";
    const auto body = YAML::LoadFile((results / "competitive_report.yaml").string())
                          ["competitive_report"];
    ASSERT_TRUE(body);
    EXPECT_EQ(body["mission_control"].as<std::string>(), "test_mc_ok.so");

    // Per-algorithm entries (no same_results grouping in this mode), exact tie → name order.
    ASSERT_EQ(body["results_summary"].size(), 2u);
    EXPECT_EQ(body["results_summary"][0]["algorithm"].as<std::string>(), "test_algo_ok.so");
    EXPECT_EQ(body["results_summary"][1]["algorithm"].as<std::string>(), "test_algo_ok2.so");

    ASSERT_EQ(body["errors"].size(), 2u);
    EXPECT_EQ(body["errors"][0].as<std::string>(), "test_algo_ctor_throws.so");
    EXPECT_EQ(body["errors"][1].as<std::string>(), "test_algo_none.so");

    // Per-run reports named by algorithm; none for errors-list components.
    EXPECT_TRUE(fs::exists(results / "test_algo_ok_run0_result.yaml"));
    EXPECT_TRUE(fs::exists(results / "test_algo_ok2_run0_result.yaml"));
    EXPECT_FALSE(fs::exists(results / "test_algo_ctor_throws_run0_result.yaml"));
}

TEST_F(Integration, CompetitiveReportGoldenLayout) {
    const fs::path plugin_dir{DRONE_TEST_PLUGIN_DIR};
    const fs::path algos = fs::temp_directory_path() / "sim_integration" / "competition_golden";
    fs::remove_all(algos);
    fs::create_directories(algos);
    fs::copy_file(plugin_dir / "test_algo_ok.so", algos / "test_algo_ok.so");

    ASSERT_EQ(simulator::runCompetitionMode(
                  makeCompetitionArgs(algos, plugin_dir / "test_mc_ok.so")),
              0);
    const fs::path results = findCompetitionResultsDir(algos);
    ASSERT_FALSE(results.empty());

    std::ifstream in{results / "competitive_report.yaml"};
    std::stringstream buffer;
    buffer << in.rdbuf();
    std::string text = buffer.str();

    const auto replaceAll = [&text](const std::string& from, const std::string& to) {
        for (std::size_t pos = text.find(from); pos != std::string::npos;
             pos = text.find(from, pos + to.size())) {
            text.replace(pos, from.size(), to);
        }
    };
    replaceAll(miniComposePath().string(), "<COMPOSITION>");
    text = std::regex_replace(
        text, std::regex{R"(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z)"}, "<UTC>");

    const std::string expected =
        "competitive_report:\n"
        "  composition_file: <COMPOSITION>\n"
        "  mission_control: test_mc_ok.so\n"
        "  generated_at_utc: <UTC>\n"
        "  results_summary:\n"
        "    - algorithm: test_algo_ok.so\n"
        "      total_score: 0\n"
        "      total_steps: 0\n"
        "  errors: []\n";
    EXPECT_EQ(text, expected);
}

TEST_F(Integration, CompetitionEmptyAlgorithmsFolderFailsCleanly) {
    const fs::path dir = fs::temp_directory_path() / "sim_integration" / "competition_empty";
    fs::remove_all(dir);
    fs::create_directories(dir);
    const int exit_code = simulator::runCompetitionMode(
        makeCompetitionArgs(dir, fs::path{DRONE_TEST_PLUGIN_DIR} / "test_mc_ok.so"));
    EXPECT_EQ(exit_code, 1);
    EXPECT_TRUE(findCompetitionResultsDir(dir).empty());
}

TEST_F(Integration, UnwritableFolderFailsCleanlyWithoutCrashing) {
    // "In case the folder cannot be created, write a proper error to screen" + the global
    // never-crash rule: a valid invocation whose results folder cannot be created exits 1 cleanly.
    const fs::path plugin_dir{DRONE_TEST_PLUGIN_DIR};
    const fs::path mc_folder = stageMcFolder("unwritable", plugin_dir / "test_mc_ok.so");
    fs::permissions(mc_folder, fs::perms::owner_read | fs::perms::owner_exec,
                    fs::perm_options::replace);
    const int exit_code =
        simulator::runComparativeMode(makeArgs(mc_folder, plugin_dir / "test_algo_ok.so"));
    fs::permissions(mc_folder, fs::perms::owner_all, fs::perm_options::replace); // restore
    EXPECT_EQ(exit_code, 1);
    EXPECT_TRUE(findResultsDir(mc_folder).empty());
}

TEST_F(Integration, ThreadedRunMatchesSingleThreadedByteForByte) {
    // The Phase-5 determinism gate, deliberately including FAILING runs (test_mc_error) so error
    // aggregation is proven schedule-independent: identical staged folders run with num_threads=1
    // (0 workers, inline) and num_threads=4; every produced file must match byte-for-byte after
    // normalizing the volatile parts (timestamps + the differing folder paths).
    const fs::path plugin_dir{DRONE_TEST_PLUGIN_DIR};
    const auto stage = [&plugin_dir](const char* name) {
        const fs::path dir = fs::temp_directory_path() / "sim_integration" / name;
        fs::remove_all(dir);
        fs::create_directories(dir);
        fs::copy_file(plugin_dir / "test_mc_ok.so", dir / "test_mc_ok.so");
        fs::copy_file(plugin_dir / "test_mc_ok.so", dir / "test_mc_ok2.so");
        fs::copy_file(plugin_dir / "test_mc_error.so", dir / "test_mc_error.so");
        return dir;
    };
    const fs::path dir_single = stage("determinism_single");
    const fs::path dir_threaded = stage("determinism_threaded");

    auto args_single = makeArgs(dir_single, plugin_dir / "test_algo_ok.so");
    args_single.num_threads = 1;
    auto args_threaded = makeArgs(dir_threaded, plugin_dir / "test_algo_ok.so");
    args_threaded.num_threads = 4;

    ASSERT_EQ(simulator::runComparativeMode(args_single), 0);
    ASSERT_EQ(simulator::runComparativeMode(args_threaded), 0);

    const fs::path res_single = findResultsDir(dir_single);
    const fs::path res_threaded = findResultsDir(dir_threaded);
    ASSERT_FALSE(res_single.empty());
    ASSERT_FALSE(res_threaded.empty());

    const auto normalized = [](const fs::path& file, const fs::path& base) {
        std::ifstream in{file};
        std::stringstream buffer;
        buffer << in.rdbuf();
        std::string text = buffer.str();
        const std::string base_str = base.string();
        for (std::size_t pos = text.find(base_str); pos != std::string::npos;
             pos = text.find(base_str, pos + 6)) {
            text.replace(pos, base_str.size(), "<BASE>");
        }
        return std::regex_replace(
            text, std::regex{R"(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z)"}, "<UTC>");
    };

    // The mode report and every per-run report (including the failing component's).
    EXPECT_EQ(normalized(res_single / "comparative_report.yaml", dir_single),
              normalized(res_threaded / "comparative_report.yaml", dir_threaded));
    for (const char* file : {"test_mc_ok_run0_result.yaml", "test_mc_ok2_run0_result.yaml",
                             "test_mc_error_run0_result.yaml"}) {
        EXPECT_EQ(normalized(res_single / file, dir_single),
                  normalized(res_threaded / file, dir_threaded))
            << "mismatch in " << file;
    }

    // Output maps byte-identical.
    for (const char* file : {"test_mc_ok_run0_output.npy", "test_mc_error_run0_output.npy"}) {
        std::ifstream a{res_single / file, std::ios::binary};
        std::ifstream b{res_threaded / file, std::ios::binary};
        std::stringstream sa, sb;
        sa << a.rdbuf();
        sb << b.rdbuf();
        EXPECT_EQ(sa.str(), sb.str()) << "binary mismatch in " << file;
    }
}

#if defined(DRONE_TEST_REAL_ALGO_SO) && defined(DRONE_TEST_REAL_MC_SO)

TEST_F(Integration, RealPluginsCompetitionEndToEnd) {
    const fs::path algos = fs::temp_directory_path() / "sim_integration" / "competition_real";
    fs::remove_all(algos);
    fs::create_directories(algos);
    const fs::path real_algo{DRONE_TEST_REAL_ALGO_SO};
    fs::copy_file(real_algo, algos / real_algo.filename());

    const int exit_code = simulator::runCompetitionMode(
        makeCompetitionArgs(algos, fs::path{DRONE_TEST_REAL_MC_SO}));
    ASSERT_EQ(exit_code, 0);

    const fs::path results = findCompetitionResultsDir(algos);
    ASSERT_FALSE(results.empty());
    const auto body = YAML::LoadFile((results / "competitive_report.yaml").string())
                          ["competitive_report"];
    ASSERT_EQ(body["results_summary"].size(), 1u);
    EXPECT_GT(body["results_summary"][0]["total_score"].as<double>(), 50.0);
    EXPECT_GT(body["results_summary"][0]["total_steps"].as<long long>(), 0);
}

TEST_F(Integration, RealPluginsComparativeEndToEnd) {
    const fs::path mc_folder = stageMcFolder("real_e2e", fs::path{DRONE_TEST_REAL_MC_SO});

    const int exit_code = simulator::runComparativeMode(
        makeArgs(mc_folder, fs::path{DRONE_TEST_REAL_ALGO_SO}));
    ASSERT_EQ(exit_code, 0);

    const fs::path results = findResultsDir(mc_folder);
    ASSERT_FALSE(results.empty());
    const auto body = YAML::LoadFile((results / "comparative_report.yaml").string())
                          ["comparative_report"];
    ASSERT_EQ(body["results_summary"].size(), 1u);
    // The real algorithm maps the small room well — a meaningful score, not the stub's 0.
    EXPECT_GT(body["results_summary"][0]["total_score"].as<double>(), 50.0);
    EXPECT_GT(body["results_summary"][0]["total_steps"].as<long long>(), 0);
}

TEST_F(Integration, RealPluginsVerboseProducesVerboseFile) {
    const fs::path mc_folder = stageMcFolder("real_verbose", fs::path{DRONE_TEST_REAL_MC_SO});
    const int exit_code = simulator::runComparativeMode(
        makeArgs(mc_folder, fs::path{DRONE_TEST_REAL_ALGO_SO}, /*verbose=*/true));
    ASSERT_EQ(exit_code, 0);

    const fs::path results = findResultsDir(mc_folder);
    ASSERT_FALSE(results.empty());
    bool found_verbose = false;
    for (const auto& entry : fs::directory_iterator{results}) {
        if (entry.path().filename().string().find("_verbose.txt") != std::string::npos) {
            found_verbose = true;
        }
    }
    EXPECT_TRUE(found_verbose);
}

#endif // real-plugin cases

} // namespace

// ---------------------------------------------------------------------------------------------
// Phase 8c bonus — grouped-by-plugin load-once/unload-when-unused lifecycle.
// The instrumented fixture (test_mc_lifecycle.so) appends "load:<name>"/"unload:<name>" to
// DRONE_LIFECYCLE_LOG from its static init/destruction, i.e. at dlopen/dlclose exactly.
// ---------------------------------------------------------------------------------------------

namespace {

std::vector<std::string> readLifecycleLog(const fs::path& log) {
    std::vector<std::string> lines;
    std::ifstream in{log};
    for (std::string line; std::getline(in, line);) {
        if (!line.empty()) {
            lines.push_back(line);
        }
    }
    return lines;
}

} // namespace

TEST_F(Integration, PluginLifecycleGroupedLoadRunUnloadPerPlugin) {
    const fs::path plugin_dir{DRONE_TEST_PLUGIN_DIR};
    // Two "teams" staged as byte-copies of the instrumented fixture, names sorted A then B.
    const fs::path dir = fs::temp_directory_path() / "sim_integration" / "lifecycle_grouped";
    fs::remove_all(dir);
    fs::create_directories(dir);
    fs::copy_file(plugin_dir / "test_mc_lifecycle.so", dir / "MissionControl_A.so");
    fs::copy_file(plugin_dir / "test_mc_lifecycle.so", dir / "MissionControl_B.so");

    const fs::path log = dir / "lifecycle_events.log";
    ::setenv("DRONE_LIFECYCLE_LOG", log.c_str(), 1);
    const int exit_code =
        simulator::runComparativeMode(makeArgs(dir, plugin_dir / "test_algo_ok.so"));
    ::unsetenv("DRONE_LIFECYCLE_LOG");
    ASSERT_EQ(exit_code, 0);

    // The exact grouped sequence: each plugin is loaded ONCE, runs its whole group, and is
    // unloaded BEFORE the next plugin's load — never revisited.
    const std::vector<std::string> expected{
        "load:MissionControl_A.so", "unload:MissionControl_A.so",
        "load:MissionControl_B.so", "unload:MissionControl_B.so"};
    EXPECT_EQ(readLifecycleLog(log), expected);
}

TEST_F(Integration, PluginLifecycleSamePathWhenThreaded) {
    const fs::path plugin_dir{DRONE_TEST_PLUGIN_DIR};
    const fs::path dir = fs::temp_directory_path() / "sim_integration" / "lifecycle_threaded";
    fs::remove_all(dir);
    fs::create_directories(dir);
    fs::copy_file(plugin_dir / "test_mc_lifecycle.so", dir / "MissionControl_A.so");
    fs::copy_file(plugin_dir / "test_mc_lifecycle.so", dir / "MissionControl_B.so");

    const fs::path log = dir / "lifecycle_events.log";
    auto args = makeArgs(dir, plugin_dir / "test_algo_ok.so");
    args.num_threads = 4; // workers run the group's tasks; loads/unloads stay on main
    ::setenv("DRONE_LIFECYCLE_LOG", log.c_str(), 1);
    const int exit_code = simulator::runComparativeMode(args);
    ::unsetenv("DRONE_LIFECYCLE_LOG");
    ASSERT_EQ(exit_code, 0);

    const std::vector<std::string> expected{
        "load:MissionControl_A.so", "unload:MissionControl_A.so",
        "load:MissionControl_B.so", "unload:MissionControl_B.so"};
    EXPECT_EQ(readLifecycleLog(log), expected);
}

TEST_F(Integration, PluginLifecycleUnloadableAndNeverExecutedStillGrouped) {
    const fs::path plugin_dir{DRONE_TEST_PLUGIN_DIR};
    const fs::path dir = fs::temp_directory_path() / "sim_integration" / "lifecycle_mixed";
    fs::remove_all(dir);
    fs::create_directories(dir);
    // A healthy instrumented plugin between a ctor-throws one and a registers-nothing one.
    fs::copy_file(plugin_dir / "test_mc_ctor_throws.so", dir / "MissionControl_A.so");
    fs::copy_file(plugin_dir / "test_mc_lifecycle.so", dir / "MissionControl_B.so");
    fs::copy_file(plugin_dir / "test_algo_none.so", dir / "MissionControl_C.so");

    const fs::path log = dir / "lifecycle_events.log";
    ::setenv("DRONE_LIFECYCLE_LOG", log.c_str(), 1);
    const int exit_code =
        simulator::runComparativeMode(makeArgs(dir, plugin_dir / "test_algo_ok.so"));
    ::unsetenv("DRONE_LIFECYCLE_LOG");
    ASSERT_EQ(exit_code, 0);

    // The healthy plugin's lifecycle is unaffected by broken neighbors, and the report still
    // carries the full errors taxonomy (A executed nothing, C registered nothing).
    const std::vector<std::string> expected{
        "load:MissionControl_B.so", "unload:MissionControl_B.so"};
    EXPECT_EQ(readLifecycleLog(log), expected);

    const fs::path results = findResultsDir(dir);
    ASSERT_FALSE(results.empty());
    const YAML::Node report = YAML::LoadFile((results / "comparative_report.yaml").string());
    const auto errors = report["comparative_report"]["errors"];
    ASSERT_TRUE(errors.IsSequence());
    std::vector<std::string> error_names;
    for (const auto& e : errors) {
        error_names.push_back(e.as<std::string>());
    }
    EXPECT_EQ(error_names, (std::vector<std::string>{
        "MissionControl_A.so", "MissionControl_C.so"}));
}
