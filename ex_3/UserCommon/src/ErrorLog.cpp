#include <UserCommon/ErrorLog.h>

#include <chrono>
#include <ctime>

namespace user_common_323084962_212223036 {

namespace {

[[nodiscard]] std::string utcTimestamp() {
    using clock = std::chrono::system_clock;
    const std::time_t t = clock::to_time_t(clock::now());
    std::tm tm_utc{};
#if defined(_WIN32)
    gmtime_s(&tm_utc, &t);
#else
    gmtime_r(&t, &tm_utc);
#endif
    char buffer[32];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &tm_utc);
    return buffer;
}

} // namespace

ErrorLog::ErrorLog(const std::filesystem::path& path) {
    open(path);
}

void ErrorLog::open(const std::filesystem::path& path) {
    if (path.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(path.parent_path(), ec);
    }
    out_.close();
    out_.clear();
    out_.open(path, std::ios::out | std::ios::trunc);
    path_ = path;
    count_ = 0;
}

void ErrorLog::log(const std::string& code, const std::string& message) {
    ++count_;
    if (out_.is_open()) {
        out_ << utcTimestamp() << " [" << code << "] " << message << '\n';
        out_.flush();
    }
}

void ErrorLog::log(const std::string& message) {
    log("ERROR", message);
}

ErrorLog& globalErrorLog() {
    static ErrorLog instance;
    return instance;
}

} // namespace user_common_323084962_212223036
