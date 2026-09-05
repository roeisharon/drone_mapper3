#pragma once

#include "report/ResultAggregation.h"

#include <filesystem>
#include <string>
#include <vector>

namespace simulator {

// Emits the Comparative Simulation Result Output File (spec format, top-level key
// `comparative_report`, lowercase per the forum clarification).
class ComparativeReportWriter {
public:
    // components: one aggregate per successfully-executed mission control, in canonical
    //             (sorted-by-name) order. Grouping/sorting happens inside.
    // load_errors: .so filenames that could not be loaded or executed at all (spec `errors:`).
    static void write(const std::filesystem::path& output_file,
                      const std::filesystem::path& composition_file,
                      const std::filesystem::path& mission_control_folder,
                      const std::string& generated_at_utc,
                      const std::vector<ComponentAggregate>& components,
                      const std::vector<std::string>& load_errors);
};

} // namespace simulator
