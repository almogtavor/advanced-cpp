#pragma once

#include <fstream>
#include <iostream>
#include <string>

namespace drone {

enum class LogLevel { Debug = 0, Info = 1, Warning = 2, Error = 3, None = 4 };

// Simple singleton logger that writes to a file and optionally to stderr.
// The log level can be set at runtime. Default: Info.
class Logger {
public:
    static Logger& instance() {
        static Logger inst;
        return inst;
    }

    void set_level(LogLevel level) { level_ = level; }
    LogLevel level() const { return level_; }

    void set_file(const std::string& path) {
        file_.open(path, std::ios::trunc);
        has_file_ = file_.is_open();
    }

    void set_stderr(bool on) { to_stderr_ = on; }

    void log(LogLevel lvl, const std::string& msg) {
        if (lvl < level_) return;
        const char* tag = "";
        switch (lvl) {
            case LogLevel::Debug:   tag = "[DEBUG]   "; break;
            case LogLevel::Info:    tag = "[INFO]    "; break;
            case LogLevel::Warning: tag = "[WARNING] "; break;
            case LogLevel::Error:   tag = "[ERROR]   "; break;
            default: break;
        }
        if (has_file_) file_ << tag << msg << "\n";
        if (to_stderr_) std::cerr << tag << msg << "\n";
    }

    void debug(const std::string& msg)   { log(LogLevel::Debug, msg);   }
    void info(const std::string& msg)    { log(LogLevel::Info, msg);    }
    void warning(const std::string& msg) { log(LogLevel::Warning, msg); }
    void error(const std::string& msg)   { log(LogLevel::Error, msg);   }

private:
    Logger() = default;
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    LogLevel level_{LogLevel::Info};
    std::ofstream file_{};
    bool has_file_{false};
    bool to_stderr_{false};
};

} // namespace drone

// Convenience macros.
#define LOG_DEBUG(msg)   ::drone::Logger::instance().debug(msg)
#define LOG_INFO(msg)    ::drone::Logger::instance().info(msg)
#define LOG_WARNING(msg) ::drone::Logger::instance().warning(msg)
#define LOG_ERROR(msg)   ::drone::Logger::instance().error(msg)
