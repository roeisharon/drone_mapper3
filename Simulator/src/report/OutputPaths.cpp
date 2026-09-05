#include "report/OutputPaths.h"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <system_error>

namespace simulator {

std::string timeToken() {
    using namespace std::chrono;
    const auto now = system_clock::now();
    const duration<double> ts = now.time_since_epoch();
    // Faithful to the spec's sample: the 9 sub-second digits of the epoch time, zero-padded.
    constexpr std::size_t kNumDigits = 9;
    constexpr std::size_t kModulus = 1'000'000'000; // 10^kNumDigits
    const auto value = static_cast<std::size_t>(ts.count() * static_cast<double>(kModulus)) % kModulus;
    std::ostringstream oss;
    oss << std::setw(kNumDigits) << std::setfill('0') << value;
    return oss.str();
}

std::string utcTimestamp() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_utc{};
#ifdef _WIN32
    gmtime_s(&tm_utc, &t);
#else
    gmtime_r(&t, &tm_utc);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm_utc, "%Y-%m-%dT%H:%M:%SZ");
    return oss.str();
}

std::optional<std::filesystem::path> createOutputFolder(const std::filesystem::path& base,
                                                        const std::string& prefix) {
    constexpr int kMaxAttempts = 5;
    std::error_code ec;
    for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
        const std::filesystem::path dir = base / (prefix + timeToken());
        // create_directory: true only when this call created it — an existing directory of the
        // same name is a token collision, so retry with a fresh token.
        if (std::filesystem::create_directory(dir, ec) && !ec) {
            return dir;
        }
        if (ec) {
            std::cerr << "Error: cannot create output directory '" << dir.string()
                      << "': " << ec.message() << "\n";
            return std::nullopt;
        }
    }
    std::cerr << "Error: cannot create a unique output directory under '" << base.string()
              << "' after " << kMaxAttempts << " attempts\n";
    return std::nullopt;
}

} // namespace simulator
