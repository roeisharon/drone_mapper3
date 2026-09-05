#include "report/ComparativeReportWriter.h"

#include <yaml-cpp/yaml.h>

#include <fstream>

namespace simulator {

void ComparativeReportWriter::write(const std::filesystem::path& output_file,
                                    const std::filesystem::path& composition_file,
                                    const std::filesystem::path& mission_control_folder,
                                    const std::string& generated_at_utc,
                                    const std::vector<ComponentAggregate>& components,
                                    const std::vector<std::string>& load_errors) {
    const auto groups = groupBySameResults(components);

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "comparative_report" << YAML::Value << YAML::BeginMap;

    out << YAML::Key << "composition_file" << YAML::Value << composition_file.string();
    out << YAML::Key << "mission_control_folder" << YAML::Value << mission_control_folder.string();
    out << YAML::Key << "generated_at_utc" << YAML::Value << generated_at_utc;

    // Sorted by number of agreeing managers, descending (done in groupBySameResults). All members
    // of a group share identical per-run outcomes, so the group's totals are the first member's.
    out << YAML::Key << "results_summary" << YAML::Value << YAML::BeginSeq;
    for (const auto& group : groups) {
        out << YAML::BeginMap;
        out << YAML::Key << "same_results" << YAML::Value << YAML::Flow << YAML::BeginSeq;
        for (const ComponentAggregate* member : group) {
            out << member->name;
        }
        out << YAML::EndSeq;
        out << YAML::Key << "total_score" << YAML::Value << group.front()->total_score;
        out << YAML::Key << "total_steps" << YAML::Value
            << static_cast<unsigned long long>(group.front()->total_steps);
        out << YAML::EndMap;
    }
    out << YAML::EndSeq;

    // Only components that could not be loaded / executed at all (forum clarification).
    out << YAML::Key << "errors" << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (const auto& name : load_errors) {
        out << name;
    }
    out << YAML::EndSeq;

    out << YAML::EndMap; // comparative_report
    out << YAML::EndMap; // document

    if (output_file.has_parent_path()) {
        std::filesystem::create_directories(output_file.parent_path());
    }
    std::ofstream f{output_file};
    f << out.c_str() << "\n";
}

} // namespace simulator
