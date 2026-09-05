// The dlopen load protocol against REAL fixture .so files: count-delta binding, rollback on
// unexpected registration counts, factory hand-over, and registrar mailbox behavior.

#include "map/Map3DImpl.h"
#include "plugins/PluginManager.h"
#include "registration/Registrar.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <string>

namespace {

using simulator::AlgorithmRegistrar;
using simulator::MissionControlRegistrar;
using simulator::PluginManager;

std::filesystem::path pluginPath(const std::string& name) {
    return std::filesystem::path{DRONE_TEST_PLUGIN_DIR} / name;
}

// Every test must leave the process-wide registrars empty; the fixture enforces it.
class PluginLoaderTest : public ::testing::Test {
protected:
    void SetUp() override {
        AlgorithmRegistrar::instance().clear();
        MissionControlRegistrar::instance().clear();
    }
    void TearDown() override {
        AlgorithmRegistrar::instance().clear();
        MissionControlRegistrar::instance().clear();
    }
};

TEST_F(PluginLoaderTest, LoadOkRegistersExactlyOnePlugin) {
    PluginManager manager;
    std::string error;
    ASSERT_TRUE(manager.loadAlgorithm(pluginPath("test_algo_ok.so"), error)) << error;

    ASSERT_EQ(manager.algorithms().size(), 1u);
    EXPECT_EQ(manager.algorithms().front().name, "test_algo_ok.so");
    EXPECT_TRUE(static_cast<bool>(manager.algorithms().front().factory));
    // Mailbox behavior: the factory was moved out of the registrar into the plugin record.
    EXPECT_EQ(AlgorithmRegistrar::instance().size(), 0u);
}

TEST_F(PluginLoaderTest, LoadMissingFileFails) {
    PluginManager manager;
    std::string error;
    EXPECT_FALSE(manager.loadAlgorithm(pluginPath("does_not_exist.so"), error));
    EXPECT_FALSE(error.empty());
    EXPECT_TRUE(manager.algorithms().empty());
}

TEST_F(PluginLoaderTest, LoadWithZeroRegistrationsFails) {
    PluginManager manager;
    std::string error;
    EXPECT_FALSE(manager.loadAlgorithm(pluginPath("test_algo_none.so"), error));
    EXPECT_NE(error.find("expected exactly 1"), std::string::npos) << error;
    EXPECT_TRUE(manager.algorithms().empty());
    EXPECT_EQ(AlgorithmRegistrar::instance().size(), 0u);
}

TEST_F(PluginLoaderTest, LoadWithDoubleRegistrationRollsBack) {
    PluginManager manager;
    std::string error;
    EXPECT_FALSE(manager.loadAlgorithm(pluginPath("test_algo_double.so"), error));
    EXPECT_NE(error.find("got 2"), std::string::npos) << error;
    EXPECT_TRUE(manager.algorithms().empty());
    // The critical rollback invariant: both orphan entries were destroyed BEFORE the dlclose
    // (leaving them behind would be destroyed-after-unmap UB at shutdown).
    EXPECT_EQ(AlgorithmRegistrar::instance().size(), 0u);
}

TEST_F(PluginLoaderTest, MissionControlLoadOk) {
    PluginManager manager;
    std::string error;
    ASSERT_TRUE(manager.loadMissionControl(pluginPath("test_mc_ok.so"), error)) << error;
    ASSERT_EQ(manager.missionControls().size(), 1u);
    EXPECT_EQ(manager.missionControls().front().name, "test_mc_ok.so");
    EXPECT_EQ(MissionControlRegistrar::instance().size(), 0u);
}

TEST_F(PluginLoaderTest, MixedLoadsKeepKindsSeparate) {
    PluginManager manager;
    std::string error;
    ASSERT_TRUE(manager.loadAlgorithm(pluginPath("test_algo_ok.so"), error)) << error;
    ASSERT_TRUE(manager.loadMissionControl(pluginPath("test_mc_ok.so"), error)) << error;
    EXPECT_EQ(manager.algorithms().size(), 1u);
    EXPECT_EQ(manager.missionControls().size(), 1u);
}

TEST_F(PluginLoaderTest, LoadedFactoryCreatesWorkingInstance) {
    PluginManager manager;
    std::string error;
    ASSERT_TRUE(manager.loadAlgorithm(pluginPath("test_algo_ok.so"), error)) << error;

    // A real (tiny) map + configs to satisfy the dependencies' references.
    common::types::MapConfig cfg;
    cfg.resolution = 10.0 * common::cm;
    cfg.boundaries = {0.0 * common::x_extent[common::cm], 40.0 * common::x_extent[common::cm],
                      0.0 * common::y_extent[common::cm], 40.0 * common::y_extent[common::cm],
                      0.0 * common::z_extent[common::cm], 40.0 * common::z_extent[common::cm]};
    simulator::Map3DImpl map{cfg};
    const common::types::MissionConfigData mission{};
    const common::types::LidarConfigData lidar{};
    const common::types::DroneConfigData drone{};

    {
        // Instance scope: it must die before the manager (and its dlclose) — mirroring the
        // production lifetime rules.
        auto instance = manager.algorithms().front().factory(
            common::MappingAlgorithmDependencies{mission, lidar, drone, map});
        ASSERT_NE(instance, nullptr);
        const auto command = instance->nextStep(common::types::DroneState{}, nullptr);
        EXPECT_EQ(command.status, common::types::AlgorithmStatus::Finished);
    }
}

TEST_F(PluginLoaderTest, UnloadAllLeavesRegistrarsEmpty) {
    {
        PluginManager manager;
        std::string error;
        ASSERT_TRUE(manager.loadAlgorithm(pluginPath("test_algo_ok.so"), error)) << error;
        ASSERT_TRUE(manager.loadMissionControl(pluginPath("test_mc_ok.so"), error)) << error;
        // Destructor performs unloadAll: records → registrars → dlclose.
    }
    EXPECT_EQ(AlgorithmRegistrar::instance().size(), 0u);
    EXPECT_EQ(MissionControlRegistrar::instance().size(), 0u);
}

TEST_F(PluginLoaderTest, RegistrarTruncateDropsOnlyNewestEntries) {
    auto& registrar = AlgorithmRegistrar::instance();
    registrar.add(common::MappingAlgorithmFactory{});
    registrar.add(common::MappingAlgorithmFactory{});
    registrar.add(common::MappingAlgorithmFactory{});
    registrar.truncate(1);
    EXPECT_EQ(registrar.size(), 1u);
    registrar.truncate(5); // larger than size: no-op
    EXPECT_EQ(registrar.size(), 1u);
    registrar.clear();
    EXPECT_EQ(registrar.size(), 0u);
}

} // namespace
