#pragma once

#include <dlfcn.h>

#include <utility>

namespace simulator {

// Move-only RAII owner of a dlopen handle. The destructor dlcloses; whoever owns a DlHandle is
// responsible for ensuring nothing from the library (instances, factories, registrar entries)
// is still alive when the handle dies — see PluginManager's teardown order.
class DlHandle {
public:
    DlHandle() = default;
    explicit DlHandle(void* handle) : handle_(handle) {}

    DlHandle(DlHandle&& other) noexcept : handle_(std::exchange(other.handle_, nullptr)) {}
    DlHandle& operator=(DlHandle&& other) noexcept {
        if (this != &other) {
            close();
            handle_ = std::exchange(other.handle_, nullptr);
        }
        return *this;
    }

    DlHandle(const DlHandle&) = delete;
    DlHandle& operator=(const DlHandle&) = delete;

    ~DlHandle() { close(); }

    [[nodiscard]] void* get() const noexcept { return handle_; }
    [[nodiscard]] explicit operator bool() const noexcept { return handle_ != nullptr; }

private:
    void close() noexcept {
        if (handle_ != nullptr) {
            dlclose(handle_);
            handle_ = nullptr;
        }
    }

    void* handle_ = nullptr;
};

} // namespace simulator
