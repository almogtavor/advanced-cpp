#pragma once

// A tiny append-only error log. Every call to log() writes and flushes
// immediately, satisfying the requirement that errors are never deferred.
// A process-global instance lets deeply nested components report errors the
// moment they occur without threading a logger through fixed interfaces.

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <string>

namespace user_common_323084962_212223036 {

class ErrorLog {
public:
    ErrorLog() = default;
    explicit ErrorLog(const std::filesystem::path& path);

    // Redirects the log to a new file (created/truncated). Also resets count.
    void open(const std::filesystem::path& path);

    void log(const std::string& code, const std::string& message);
    void log(const std::string& message);

    [[nodiscard]] std::size_t count() const { return count_; }
    [[nodiscard]] const std::filesystem::path& path() const { return path_; }

private:
    std::filesystem::path path_{};
    std::ofstream out_{};
    std::size_t count_ = 0;
};

// Shared logger used by simulation components. SimulationManager points it at
// the run's error-log file; unconfigured it simply counts without writing.
[[nodiscard]] ErrorLog& globalErrorLog();

} // namespace user_common_323084962_212223036
