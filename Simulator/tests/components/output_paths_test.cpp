// Output-folder creation and the two time formats (the 9-digit collision token vs the
// human-readable UTC stamp), including the spec's "in case the folder cannot be created, write a
// proper error to screen" path.

#include "report/OutputPaths.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <regex>

namespace {

namespace fs = std::filesystem;

TEST(OutputPaths, TimeTokenIsNineDigits) {
    const std::string token = simulator::timeToken();
    EXPECT_EQ(token.size(), 9u);
    EXPECT_TRUE(std::all_of(token.begin(), token.end(),
                            [](unsigned char c) { return std::isdigit(c) != 0; }))
        << token;
}

TEST(OutputPaths, UtcTimestampHasIso8601Format) {
    EXPECT_TRUE(std::regex_match(simulator::utcTimestamp(),
                                 std::regex{R"(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z)"}));
}

TEST(OutputPaths, CreateOutputFolderCreatesPrefixedDirectory) {
    const fs::path base = fs::temp_directory_path() / "output_paths_test" / "ok";
    fs::create_directories(base);
    const auto dir = simulator::createOutputFolder(base, "unit_prefix_");
    ASSERT_TRUE(dir.has_value());
    EXPECT_TRUE(fs::is_directory(*dir));
    EXPECT_EQ(dir->parent_path(), base);
    EXPECT_EQ(dir->filename().string().rfind("unit_prefix_", 0), 0u);
}

TEST(OutputPaths, CreateOutputFolderFailsCleanlyWithoutWritePermission) {
    const fs::path base = fs::temp_directory_path() / "output_paths_test" / "readonly";
    fs::create_directories(base);
    fs::permissions(base, fs::perms::owner_read | fs::perms::owner_exec,
                    fs::perm_options::replace);
    ::testing::internal::CaptureStderr();
    const auto dir = simulator::createOutputFolder(base, "unit_prefix_");
    const std::string err = ::testing::internal::GetCapturedStderr();
    fs::permissions(base, fs::perms::owner_all, fs::perm_options::replace); // restore for cleanup
    EXPECT_FALSE(dir.has_value());
    EXPECT_NE(err.find("cannot create output directory"), std::string::npos) << err;
}

} // namespace
