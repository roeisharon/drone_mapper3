#include "config/YamlConfigParser.h"

#include <yaml-cpp/yaml.h>

#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace simulator::config {

using namespace common;

namespace {

// Loads a YAML file safely. On failure, logs the error and returns a null node so subsequent field
// reads fall through to their defaults.
YAML::Node loadFile(const std::filesystem::path& path, uc::ErrorLogger& logger) {
    try {
        return YAML::LoadFile(path.string());
    } catch (const std::exception& e) {
        try { logger.log("YAML_LOAD_ERROR", path.string() + ": " + e.what()); } catch (...) {}
        return YAML::Node{};
    }
}

// Reads a required scalar field. Logs an error and returns dflt if the key is absent, null, or of
// the wrong type.
template <typename T>
T reqField(const YAML::Node& node, const char* key, T dflt,
           const std::string& ctx, uc::ErrorLogger& logger) {
    if (!node || !node[key] || !node[key].IsDefined() || node[key].IsNull()) {
        try {
            logger.log("MISSING_REQUIRED_FIELD",
                       ctx + ": required field '" + key + "' not found, using default");
        } catch (...) {}
        return dflt;
    }
    try {
        return node[key].as<T>();
    } catch (const YAML::Exception& e) {
        try { logger.log("PARSE_ERROR", ctx + ": field '" + key + "': " + e.what()); } catch (...) {}
        return dflt;
    }
}

// Reads an optional scalar field. Returns dflt silently when the key is absent or unparseable.
template <typename T>
T optField(const YAML::Node& node, const char* key, T dflt) {
    if (!node || !node[key] || !node[key].IsDefined() || node[key].IsNull()) {
        return dflt;
    }
    try {
        return node[key].as<T>();
    } catch (...) {
        return dflt;
    }
}

// Resolves a potentially-relative path against base_dir; absolute paths are returned unchanged.
std::filesystem::path resolve(const std::filesystem::path& base_dir, const std::string& rel) {
    const std::filesystem::path p{rel};
    return p.is_absolute() ? p : (base_dir / p);
}

// Returns the wrapper node when the body is nested under a top-level key named after the file kind,
// otherwise the file node unchanged (a flat document).
YAML::Node unwrap(const YAML::Node& file, const char* wrapper_key) {
    if (file && file[wrapper_key] && file[wrapper_key].IsMap()) {
        return file[wrapper_key];
    }
    return file;
}

// Parses a single axis boundary ({min_cm, max_cm}) into doubles. Returns false if the sub-node is
// missing or not a map.
bool readAxisBoundary(const YAML::Node& node, const char* key, double& min_out, double& max_out) {
    if (!node || !node[key] || !node[key].IsMap()) {
        return false;
    }
    const auto& axis = node[key];
    min_out = optField<double>(axis, "min_cm", 0.0);
    max_out = optField<double>(axis, "max_cm", 0.0);
    return true;
}

// Parses a `boundaries` block into a MappingBounds. Returns nullopt when absent or not a map.
// A partial block is accepted; missing axes default to a zero-width range.
std::optional<common::types::MappingBounds> readBoundariesBlock(const YAML::Node& block) {
    if (!block || !block.IsMap()) {
        return std::nullopt;
    }
    common::types::MappingBounds b;
    double mn = 0.0, mx = 0.0;
    if (readAxisBoundary(block, "x_boundary", mn, mx)) {
        b.min_x = mn * x_extent[cm];
        b.max_x = mx * x_extent[cm];
    }
    if (readAxisBoundary(block, "y_boundary", mn, mx)) {
        b.min_y = mn * y_extent[cm];
        b.max_y = mx * y_extent[cm];
    }
    if (readAxisBoundary(block, "height_boundary", mn, mx)) {
        b.min_height = mn * z_extent[cm];
        b.max_height = mx * z_extent[cm];
    }
    return b;
}

} // namespace

common::types::DroneConfigData parseDroneConfig(const std::filesystem::path& path,
                                                uc::ErrorLogger& logger) {
    const std::string ctx = path.string();
    const auto doc = unwrap(loadFile(path, logger), "drone_config");

    common::types::DroneConfigData cfg;
    // The drone is modelled as a sphere. Spec key `dimensions_cm` is its DIAMETER (the opening width
    // it can pass through), so the stored DroneConfigData.radius is half that.
    cfg.radius      = reqField<double>(doc, "dimensions_cm",  0.0, ctx, logger) / 2 * cm;
    cfg.max_rotate  = reqField<double>(doc, "max_rotate_deg", 0.0, ctx, logger) * horizontal_angle[deg];
    cfg.max_advance = reqField<double>(doc, "max_advance_cm", 0.0, ctx, logger) * cm;
    cfg.max_elevate = reqField<double>(doc, "max_elevate_cm", 0.0, ctx, logger) * cm;
    return cfg;
}

common::types::LidarConfigData parseLidarConfig(const std::filesystem::path& path,
                                                uc::ErrorLogger& logger) {
    const std::string ctx = path.string();
    const auto doc = unwrap(loadFile(path, logger), "lidar_config");

    common::types::LidarConfigData cfg;
    cfg.z_min       = reqField<double>(doc, "z_min_cm", 0.0, ctx, logger) * cm;
    cfg.z_max       = reqField<double>(doc, "z_max_cm", 0.0, ctx, logger) * cm;
    cfg.d           = reqField<double>(doc, "d_cm",     0.0, ctx, logger) * cm;
    cfg.fov_circles = reqField<std::size_t>(doc, "fov_circles", 0u, ctx, logger);
    return cfg;
}

common::types::MissionConfigData parseMissionConfig(const std::filesystem::path& path,
                                                    uc::ErrorLogger& logger) {
    const std::string ctx = path.string();
    const auto doc = unwrap(loadFile(path, logger), "mission_config");

    common::types::MissionConfigData cfg;
    cfg.max_steps      = reqField<std::size_t>(doc, "max_steps",        0u,  ctx, logger);
    cfg.gps_resolution = reqField<double>(doc, "gps_resolution_cm",     0.0, ctx, logger) * cm;
    // Optional; spec: integer, defaults to 1 when absent. Values are stored as-is; the resolution
    // request classification happens in the simulation run.
    cfg.output_mapping_resolution_factor =
        optField<double>(doc, "output_mapping_resolution_factor", 1.0);
    // The `boundaries` block lives on MissionConfigData.mission_bounds. Absent → default (all-zero
    // = no restriction).
    if (const auto bounds = readBoundariesBlock(doc["boundaries"])) {
        cfg.mission_bounds = *bounds;
    }
    return cfg;
}

simulator::types::SimulationConfigData
parseSimulationConfig(const std::filesystem::path& path,
                      uc::ErrorLogger& logger,
                      const std::filesystem::path* fallback_base_dir) {
    const std::string ctx = path.string();
    const auto file = loadFile(path, logger);
    const auto doc  = unwrap(file, "simulation_config");
    const auto base_dir = path.parent_path();

    simulator::types::SimulationConfigData cfg;

    // map_filename is required. Resolved relative to this config file's directory; when the file
    // is not there and a fallback base (the composition file's directory) was provided, resolve
    // against that instead — the ex3 skeleton inputs use composition-relative map paths.
    const auto map_rel = reqField<std::string>(doc, "map_filename", "", ctx, logger);
    if (!map_rel.empty()) {
        std::filesystem::path resolved = resolve(base_dir, map_rel);
        if (fallback_base_dir != nullptr) {
            std::error_code ec;
            if (!std::filesystem::exists(resolved, ec)) {
                const std::filesystem::path alt = resolve(*fallback_base_dir, map_rel);
                if (std::filesystem::exists(alt, ec)) {
                    resolved = alt;
                }
            }
        }
        cfg.map_filename = resolved;
    }
    cfg.map_resolution = reqField<double>(doc, "map_resolution_cm", 0.0, ctx, logger) * cm;

    // map_axes_offset is optional. The spec places it at the document root, beside (not inside)
    // simulation_config; accept it in either location. Benchmark-style configs spell the key
    // `map_axes_offset_cm`, the spec spells it `map_axes_offset` — accept both, or a non-zero offset
    // would be silently dropped (and the map treated as if it started at world 0).
    const auto findOffset = [](const YAML::Node& n) -> YAML::Node {
        for (const char* key : {"map_axes_offset", "map_axes_offset_cm"}) {
            if (n[key] && n[key].IsMap()) {
                return n[key];
            }
        }
        return YAML::Node{};
    };
    YAML::Node offset_node = findOffset(file);
    if (!(offset_node && offset_node.IsMap())) {
        offset_node = findOffset(doc);
    }
    if (offset_node && offset_node.IsMap()) {
        cfg.map_offset.x = optField<double>(offset_node, "x_offset",      0.0) * x_extent[cm];
        cfg.map_offset.y = optField<double>(offset_node, "y_offset",      0.0) * y_extent[cm];
        cfg.map_offset.z = optField<double>(offset_node, "height_offset", 0.0) * z_extent[cm];
    }

    // initial_drone_position is optional; defaults to world origin. The spec's vertical key is
    // `height_cm`; `z_cm` is accepted as a fallback alias.
    if (doc["initial_drone_position"] && doc["initial_drone_position"].IsMap()) {
        const auto& pos = doc["initial_drone_position"];
        cfg.initial_drone_position.x = optField<double>(pos, "x_cm", 0.0) * x_extent[cm];
        cfg.initial_drone_position.y = optField<double>(pos, "y_cm", 0.0) * y_extent[cm];
        const double height_cm =
            (pos["height_cm"] && pos["height_cm"].IsDefined() && !pos["height_cm"].IsNull())
                ? optField<double>(pos, "height_cm", 0.0)
                : optField<double>(pos, "z_cm", 0.0);
        cfg.initial_drone_position.z = height_cm * z_extent[cm];
    }

    cfg.initial_angle = optField<double>(doc, "initial_angle_deg", 0.0) * horizontal_angle[deg];
    return cfg;
}

simulator::types::SimulationCompositionData
parseSimulationCompositions(const std::filesystem::path& path, uc::ErrorLogger& logger,
                            CompositionPaths* out_paths) {
    const std::string ctx = path.string();
    const auto file = loadFile(path, logger);

    simulator::types::SimulationCompositionData result;
    result.composition_file = path;

    if (!file || !file.IsDefined() || file.IsNull()) {
        return result; // loadFile already logged
    }

    // The spec wraps the body under `simulation_compositions:`; a flat document is also accepted.
    const YAML::Node doc =
        (file["simulation_compositions"] && file["simulation_compositions"].IsMap())
            ? file["simulation_compositions"]
            : file;

    const auto base_dir = path.parent_path();

    // Global drone configs (shared across every simulation group).
    std::vector<std::filesystem::path> drone_paths;
    if (doc["drone_configs"] && doc["drone_configs"].IsSequence()) {
        for (const auto& item : doc["drone_configs"]) {
            const auto sub = resolve(base_dir, item.as<std::string>());
            result.drone_configs.push_back(parseDroneConfig(sub, logger));
            drone_paths.push_back(sub);
        }
    } else {
        try {
            logger.log("MISSING_REQUIRED_FIELD", ctx + ": 'drone_configs' is missing or not a sequence");
        } catch (...) {}
    }

    // Global lidar configs (shared across every simulation group).
    std::vector<std::filesystem::path> lidar_paths;
    if (doc["lidar_configs"] && doc["lidar_configs"].IsSequence()) {
        for (const auto& item : doc["lidar_configs"]) {
            const auto sub = resolve(base_dir, item.as<std::string>());
            result.lidar_configs.push_back(parseLidarConfig(sub, logger));
            lidar_paths.push_back(sub);
        }
    } else {
        try {
            logger.log("MISSING_REQUIRED_FIELD", ctx + ": 'lidar_configs' is missing or not a sequence");
        } catch (...) {}
    }

    if (out_paths != nullptr) {
        out_paths->drone_configs = drone_paths;
        out_paths->lidar_configs = lidar_paths;
    }

    // Per-simulation groups: each pairs one simulation_config with its own mission list.
    if (!doc["simulations"] || !doc["simulations"].IsSequence()) {
        try {
            logger.log("MISSING_REQUIRED_FIELD", ctx + ": 'simulations' is missing or not a sequence");
        } catch (...) {}
        return result;
    }

    for (const auto& entry : doc["simulations"]) {
        if (!entry["simulation_config"] || entry["simulation_config"].IsNull()) {
            try {
                logger.log("MISSING_REQUIRED_FIELD",
                           ctx + ": a simulation entry is missing 'simulation_config', skipping");
            } catch (...) {}
            continue;
        }

        const auto sim_rel  = entry["simulation_config"].as<std::string>();
        const auto sim_path = resolve(base_dir, sim_rel);
        auto sim_cfg = parseSimulationConfig(sim_path, logger, &base_dir);

        std::vector<common::types::MissionConfigData> missions;
        std::vector<std::filesystem::path>            mission_paths;
        if (entry["mission_configs"] && entry["mission_configs"].IsSequence()) {
            for (const auto& mc : entry["mission_configs"]) {
                const auto mp = resolve(base_dir, mc.as<std::string>());
                missions.push_back(parseMissionConfig(mp, logger));
                mission_paths.push_back(mp);
            }
        } else {
            try {
                logger.log("MISSING_REQUIRED_FIELD",
                           ctx + ": simulation entry '" + sim_rel + "' has no 'mission_configs'");
            } catch (...) {}
        }

        result.simulation_mission_groups.emplace_back(std::move(sim_cfg), std::move(missions));
        if (out_paths != nullptr) {
            out_paths->groups.push_back(CompositionGroupPaths{sim_path, std::move(mission_paths)});
        }
    }

    return result;
}

} // namespace simulator::config
