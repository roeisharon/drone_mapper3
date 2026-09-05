#pragma once

#include "plugins/DlHandle.h"
#include "registration/Registrar.h"

#include <filesystem>
#include <string>
#include <vector>

namespace simulator {

// One successfully loaded plugin. The record is the single owner of the plugin's factory (it is
// moved out of the registrar immediately after dlopen), so factory lifetime is exactly the record's
// lifetime — provably shorter than the DlHandle that keeps the code mapped.
template <typename Factory>
struct Plugin {
    std::string name;           // the .so filename, e.g. "MissionControl_1_2.so" (report identity)
    std::filesystem::path path;
    Factory factory;
};

using AlgorithmPlugin = Plugin<common::MappingAlgorithmFactory>;
using MissionControlPlugin = Plugin<common::MissionControlFactory>;

// Owns every dlopen'd plugin for the process. Load-time protocol (count-delta name binding):
// record the registrar size, dlopen, and require the size to have grown by EXACTLY one — the
// macros carry no name string, so the delta is the only way to bind a registration to its file.
// A delta of 0 or ≥2 is a load failure: the added entries are destroyed (they hold lambdas whose
// code lives in the plugin) and the handle is closed immediately.
//
// Not thread-safe by design: all loads happen on the main thread before workers exist, and
// teardown happens after they are joined (see Registrar.h).
class PluginManager {
public:
    PluginManager() = default;
    PluginManager(const PluginManager&) = delete;
    PluginManager& operator=(const PluginManager&) = delete;
    ~PluginManager() { unloadAll(); }

    // Loads one .so expected to register exactly one mapping-algorithm factory.
    // On failure fills `error` and returns false; the process state is as if never loaded.
    bool loadAlgorithm(const std::filesystem::path& so_path, std::string& error);

    // Same for a mission-control plugin.
    bool loadMissionControl(const std::filesystem::path& so_path, std::string& error);

    [[nodiscard]] const std::vector<AlgorithmPlugin>& algorithms() const noexcept {
        return algorithms_;
    }
    [[nodiscard]] const std::vector<MissionControlPlugin>& missionControls() const noexcept {
        return mission_controls_;
    }

    // Teardown in the dlclose-safe order: plugin records (factories) die first, the registrars are
    // defensively cleared, and only then are the libraries closed. The caller must ensure every
    // instance created FROM the factories was already destroyed (the mode drivers guarantee this
    // by scoping). Also run by the destructor.
    void unloadAll();

private:
    std::vector<AlgorithmPlugin> algorithms_;
    std::vector<MissionControlPlugin> mission_controls_;
    std::vector<DlHandle> handles_;
};

} // namespace simulator
