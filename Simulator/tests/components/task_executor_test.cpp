// The lock-free task executor and the spec's num_threads semantics: 0 workers = inline on main
// with zero spawns; n workers capped by task count; every task executed exactly once under
// concurrency.

#include "args/ArgsParser.h"
#include "exec/TaskExecutor.h"

#include <gtest/gtest.h>

#include <atomic>
#include <cstddef>
#include <vector>

namespace {

using simulator::TaskExecutor;

TEST(TaskExecutor, ZeroWorkersRunsInlineAndSpawnsNone) {
    std::vector<int> hits(5, 0);
    const std::size_t spawned =
        TaskExecutor::runAll(hits.size(), 0, [&hits](std::size_t i) { ++hits[i]; });
    EXPECT_EQ(spawned, 0u);
    for (const int h : hits) {
        EXPECT_EQ(h, 1);
    }
}

TEST(TaskExecutor, WorkerCountCappedByTaskCount) {
    // "Do not open threads which cannot be utilized": 8 requested, 3 tasks → 3 workers.
    std::vector<std::atomic<int>> hits(3);
    const std::size_t spawned =
        TaskExecutor::runAll(hits.size(), 8, [&hits](std::size_t i) { ++hits[i]; });
    EXPECT_EQ(spawned, 3u);
    for (const auto& h : hits) {
        EXPECT_EQ(h.load(), 1);
    }
}

TEST(TaskExecutor, EveryTaskExecutedExactlyOnceUnderConcurrency) {
    constexpr std::size_t kTasks = 1000;
    std::vector<std::atomic<int>> hits(kTasks);
    const std::size_t spawned =
        TaskExecutor::runAll(kTasks, 4, [&hits](std::size_t i) { ++hits[i]; });
    EXPECT_EQ(spawned, 4u);
    for (const auto& h : hits) {
        EXPECT_EQ(h.load(), 1);
    }
}

TEST(TaskExecutor, EmptyTaskListSpawnsNothing) {
    bool called = false;
    const std::size_t spawned =
        TaskExecutor::runAll(0, 4, [&called](std::size_t) { called = true; });
    EXPECT_EQ(spawned, 0u);
    EXPECT_FALSE(called);
}

// The spec's thread-count mapping, normalized in ParsedArgs::workerThreads():
// missing/0/1 → main only (0 workers); n >= 2 → n workers in addition to main.
// Total thread count is therefore 1 or >= 3 — never exactly 2.
TEST(ArgsParser, WorkerThreadsMappingNeverYieldsTwoTotalThreads) {
    simulator::ParsedArgs args;
    args.num_threads = 0;
    EXPECT_EQ(args.workerThreads(), 0u);
    args.num_threads = 1;
    EXPECT_EQ(args.workerThreads(), 0u); // 1 total thread
    args.num_threads = 2;
    EXPECT_EQ(args.workerThreads(), 2u); // 3 total threads — never 2
    args.num_threads = 8;
    EXPECT_EQ(args.workerThreads(), 8u);
}

} // namespace
