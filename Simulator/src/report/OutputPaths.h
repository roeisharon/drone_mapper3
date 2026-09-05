#pragma once

#include <filesystem>
#include <optional>
#include <string>

namespace simulator {

// The 9-digit zero-padded sub-second time token from the spec's sample code — used only for
// collision-free output folder names (distinct from the human-readable generated_at_utc).
[[nodiscard]] std::string timeToken();

// Current wall-clock time as "YYYY-MM-DDTHH:MM:SSZ" (the reports' generated_at_utc format).
[[nodiscard]] std::string utcTimestamp();

// Creates `<base>/<prefix><time>/` (e.g. prefix "comparative_results_"). Retries with a fresh
// token on a name collision. On failure prints a proper error to the screen (spec requirement)
// and returns nullopt.
[[nodiscard]] std::optional<std::filesystem::path>
createOutputFolder(const std::filesystem::path& base, const std::string& prefix);

} // namespace simulator
