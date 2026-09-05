#pragma once

#include "report/ResultAggregation.h"

#include <filesystem>
#include <string>
#include <vector>

namespace simulator {

// Emits the Competitive Simulation Result Output File (spec format, top-level key
// `competitive_report`, lowercase per the forum clarification). Unlike the comparative report
// there is no grouping: every executed algorithm gets its own entry, sorted by the spec rule.
class CompetitiveReportWriter {
public:
    // components: one aggregate per successfully-executed algorithm (any order; sorting happens
    //             inside per the ordering policy: score desc, steps asc, name asc).
    // errors: .so filenames that could not be loaded or executed at all (spec `errors:`).
    static void write(const std::filesystem::path& output_file,
                      const std::filesystem::path& composition_file,
                      const std::string& mission_control_name,
                      const std::string& generated_at_utc,
                      const std::vector<ComponentAggregate>& components,
                      const std::vector<std::string>& errors);
};

} // namespace simulator
