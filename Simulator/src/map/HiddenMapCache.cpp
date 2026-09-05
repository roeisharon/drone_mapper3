#include "map/HiddenMapCache.h"

#include <TinyNPY.h>

#include <exception>
#include <tuple>
#include <utility>

namespace simulator {

HiddenMapCache HiddenMapCache::build(const types::SimulationCompositionData& composition) {
    HiddenMapCache cache;
    cache.entries_.reserve(composition.simulation_mission_groups.size());

    for (const auto& [simulation, missions] : composition.simulation_mission_groups) {
        Entry entry;
        try {
            NpyArray npy;
            const std::string path = simulation.map_filename.string();
            const char* error = npy.LoadNPY(path);
            if (error != nullptr) {
                entry.error = "Failed to load NPY file '" + path + "': " + error;
            } else {
                const common::types::MapConfig config{
                    common::types::MappingBounds{},
                    simulation.map_offset,
                    simulation.map_resolution,
                };
                auto map = std::make_unique<const Map3DImpl>(npy, config);
                entry.map = map.get();
                entry.resolved_config = map->getMapConfig();
                cache.storage_.push_back(std::move(map));
            }
        } catch (const std::exception& e) {
            entry.map = nullptr;
            entry.error = e.what();
        }
        cache.entries_.push_back(std::move(entry));
    }
    return cache;
}

} // namespace simulator
