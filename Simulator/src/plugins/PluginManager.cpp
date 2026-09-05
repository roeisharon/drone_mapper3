#include "plugins/PluginManager.h"

#include <utility>

namespace simulator {

namespace {

// Shared load protocol for both plugin kinds. On success the new factory is moved out of the
// registrar into a Plugin record (single ownership) and the handle joins `handles`.
template <typename Registrar, typename PluginT>
bool loadPlugin(const std::filesystem::path& so_path,
                std::vector<PluginT>& plugins,
                std::vector<DlHandle>& handles,
                std::string& error) {
    auto& registrar = Registrar::instance();
    const std::size_t before = registrar.size();

    // Absolute path: dlopen treats a plain filename (no slash) as a library-search-path lookup,
    // which is never what we mean here.
    const std::filesystem::path abs_path = std::filesystem::absolute(so_path);
    DlHandle handle{dlopen(abs_path.c_str(), RTLD_LAZY | RTLD_LOCAL)};
    if (!handle) {
        const char* msg = dlerror();
        error = msg != nullptr ? msg : "dlopen failed";
        registrar.truncate(before); // defensive: a failed dlopen must leave no entries behind
        return false;
    }

    const std::size_t after = registrar.size();
    if (after != before + 1) {
        error = "expected exactly 1 registration from " + so_path.filename().string() +
                ", got " + std::to_string(after - before);
        // Roll back BEFORE the handle's destructor dlcloses: the discarded std::functions hold
        // lambdas whose code lives inside this .so — destroying them after dlclose would be UB.
        registrar.truncate(before);
        return false;
    }

    PluginT plugin;
    plugin.name = so_path.filename().string();
    plugin.path = abs_path;
    plugin.factory = registrar.releaseLast(); // registrar returns to its pre-load size
    plugins.push_back(std::move(plugin));
    handles.push_back(std::move(handle));
    return true;
}

} // namespace

bool PluginManager::loadAlgorithm(const std::filesystem::path& so_path, std::string& error) {
    return loadPlugin<AlgorithmRegistrar>(so_path, algorithms_, handles_, error);
}

bool PluginManager::loadMissionControl(const std::filesystem::path& so_path, std::string& error) {
    return loadPlugin<MissionControlRegistrar>(so_path, mission_controls_, handles_, error);
}

void PluginManager::unloadAll() {
    // Order is the whole point (see header): factories die while their code is still mapped,
    // then the (normally already-empty) registrars are cleared, and only then dlclose runs.
    algorithms_.clear();
    mission_controls_.clear();
    AlgorithmRegistrar::instance().clear();
    MissionControlRegistrar::instance().clear();
    handles_.clear();
}

} // namespace simulator
