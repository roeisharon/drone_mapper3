#include "report/CompetitiveReportWriter.h"

#include <yaml-cpp/yaml.h>

#include <fstream>

namespace simulator {

void CompetitiveReportWriter::write(const std::filesystem::path& output_file,
                                    const std::filesystem::path& composition_file,
                                    const std::string& mission_control_name,
                                    const std::string& generated_at_utc,
                                    const std::vector<ComponentAggregate>& components,
                                    const std::vector<std::string>& errors) {
    const auto ordered = orderByScoreDescStepsAsc(components);

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "competitive_report" << YAML::Value << YAML::BeginMap;

    out << YAML::Key << "composition_file" << YAML::Value << composition_file.string();
    out << YAML::Key << "mission_control" << YAML::Value << mission_control_name;
    out << YAML::Key << "generated_at_utc" << YAML::Value << generated_at_utc;

    // Sorted by score descending, then by steps ascending (spec rule; name breaks exact ties for
    // deterministic output).
    out << YAML::Key << "results_summary" << YAML::Value << YAML::BeginSeq;
    for (const ComponentAggregate* component : ordered) {
        out << YAML::BeginMap;
        out << YAML::Key << "algorithm" << YAML::Value << component->name;
        out << YAML::Key << "total_score" << YAML::Value << component->total_score;
        out << YAML::Key << "total_steps" << YAML::Value
            << static_cast<unsigned long long>(component->total_steps);
        out << YAML::EndMap;
    }
    out << YAML::EndSeq;

    // Only components that could not be loaded / executed at all (forum clarification).
    out << YAML::Key << "errors" << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (const auto& name : errors) {
        out << name;
    }
    out << YAML::EndSeq;

    out << YAML::EndMap; // competitive_report
    out << YAML::EndMap; // document

    if (output_file.has_parent_path()) {
        std::filesystem::create_directories(output_file.parent_path());
    }
    std::ofstream f{output_file};
    f << out.c_str() << "\n";
}

} // namespace simulator
