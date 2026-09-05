#pragma once

#include <Simulator/SimulationTypes.h>

#include <filesystem>
#include <vector>

namespace simulator {

// Source .yaml paths for a single run (one simulation × mission × drone × lidar combination). The
// frozen SimulationResult does not carry these, so the caller supplies them index-aligned with the
// report's runs to let the writer emit the exact assignment-2 output format.
struct RunPaths {
    std::filesystem::path simulation_config{};
    std::filesystem::path mission_config{};
    std::filesystem::path drone_config{};
    std::filesystem::path lidar_config{};
};

// Emits the assignment-2 `score_report` YAML format. In ex3 (per the forum clarification) one such
// file is generated for every individual Simulation Run, so the primary entry point is
// writeRunReport(); the multi-run write() remains the shared implementation.
class SimulationReportWriter {
public:
    // Writes a score_report YAML for the given report to output_file (parent dirs are created).
    // The composition file path is taken from report.composition_file.
    // run_paths: aligned with report.runs; if the sizes mismatch, paths are emitted empty.
    static void write(const types::SimulationManagerReport& report,
                      const std::filesystem::path& output_file,
                      const std::vector<RunPaths>& run_paths = {});

    // Writes the ex2-format score report for ONE simulation run (ex3's per-run file).
    static void writeRunReport(const types::SimulationResult& run,
                               const RunPaths& paths,
                               const std::filesystem::path& composition_file,
                               const std::string& generated_at_utc,
                               const std::filesystem::path& output_file);
};

} // namespace simulator
