#pragma once

#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>

#include <cstddef>
#include <utility>
#include <vector>

namespace simulator {

// Process-wide collection of plugin factories, filled as a side effect of dlopen:
// each plugin's global REGISTER_* object constructs a common::*Registration, whose
// constructor (defined in Registration.cpp, Simulator-only) appends the factory here.
//
// Thread-safety by invariant, not by locking: all dlopen calls (and therefore all
// mutations of this registrar) happen on the main thread before any worker is spawned,
// and truncate()/clear() run after the workers are joined.
template <typename Factory>
class Registrar {
public:
    static Registrar& instance() {
        static Registrar registrar;
        return registrar;
    }

    void add(Factory factory) { factories_.push_back(std::move(factory)); }

    [[nodiscard]] std::size_t size() const noexcept { return factories_.size(); }

    [[nodiscard]] const std::vector<Factory>& entries() const noexcept { return factories_; }

    // Moves the most recently registered factory out (and removes its slot). The PluginManager
    // calls this right after a successful dlopen so each factory has exactly one owner — its
    // plugin record — and the registrar acts as a short-lived mailbox between the plugin's
    // static initialization and the loader.
    [[nodiscard]] Factory releaseLast() {
        Factory factory = std::move(factories_.back());
        factories_.pop_back();
        return factory;
    }

    // Destroys every entry past new_size. Used to roll back a dlopen that registered an
    // unexpected number of factories (0 or >1) before that .so is dlclosed — the discarded
    // std::functions hold lambdas whose code lives inside the plugin.
    void truncate(std::size_t new_size) {
        if (new_size < factories_.size()) {
            factories_.resize(new_size);
        }
    }

    // Destroys all entries. Must run before the final dlclose of the plugins that
    // registered them, for the same lifetime reason as truncate().
    void clear() noexcept { factories_.clear(); }

    Registrar(const Registrar&) = delete;
    Registrar& operator=(const Registrar&) = delete;

private:
    Registrar() = default;

    std::vector<Factory> factories_;
};

using AlgorithmRegistrar = Registrar<common::MappingAlgorithmFactory>;
using MissionControlRegistrar = Registrar<common::MissionControlFactory>;

} // namespace simulator
