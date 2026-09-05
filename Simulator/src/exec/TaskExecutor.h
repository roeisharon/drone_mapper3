#pragma once

#include <cstddef>
#include <functional>

namespace simulator {

// Lock-free work distribution over a pre-built task list. The spec's num_threads semantics are
// applied by the caller (ParsedArgs::workerThreads(): missing/0/1 → 0 workers): with 0 workers
// every task runs inline on the calling (main) thread and NO thread is ever spawned; with n ≥ 2
// workers, min(n, task_count) threads are spawned ("do not open threads which cannot be
// utilized") and the main thread blocks in join (explicitly blessed by the spec), so the total
// thread count is 1 or ≥ 3 — never 2.
//
// Distribution is a single std::atomic index (fetch_add): no queue, no mutex, no condition
// variable. execute(i) must not let exceptions escape (workers cannot rethrow to main; the
// driver's task body converts failures into error results).
class TaskExecutor {
public:
    // Runs execute(0..task_count-1), each exactly once. Returns the number of worker threads
    // actually spawned (for tests and the README's thread-count documentation).
    static std::size_t runAll(std::size_t task_count,
                              std::size_t worker_threads,
                              const std::function<void(std::size_t)>& execute);
};

} // namespace simulator
