#include "arborvia/backends/SpdlogBackend.h"
#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace arborvia {

SpdlogBackend::SpdlogBackend(const std::string& logDir, bool logToFile) {
    if (logDir.empty() && !logToFile) {
        // Console only
        logger_ = spdlog::stdout_color_mt("arborvia");
        logger_->set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");
    } else {
        // Console + file
        std::vector<spdlog::sink_ptr> sinks;

        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");
        sinks.push_back(console_sink);

        if (logToFile && !logDir.empty()) {
            std::filesystem::create_directories(logDir);
            std::filesystem::path logPath = std::filesystem::path(logDir) / "arborvia.log";

            auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
                logPath.string(), true);
            file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] [%t] %v");
            sinks.push_back(file_sink);
        }

        logger_ = std::make_shared<spdlog::logger>("arborvia", sinks.begin(), sinks.end());
        spdlog::register_logger(logger_);
    }

    // Default level
    logger_->set_level(spdlog::level::debug);

    // Check LOG_LEVEL environment variable
    const char* env_level = std::getenv("LOG_LEVEL");
    if (!env_level) {
        env_level = std::getenv("SPDLOG_LEVEL");
    }

    if (env_level) {
        std::string level_str(env_level);
        std::transform(level_str.begin(), level_str.end(), level_str.begin(), ::tolower);

        if (level_str == "trace") logger_->set_level(spdlog::level::trace);
        else if (level_str == "debug") logger_->set_level(spdlog::level::debug);
        else if (level_str == "info") logger_->set_level(spdlog::level::info);
        else if (level_str == "warn" || level_str == "warning") logger_->set_level(spdlog::level::warn);
        else if (level_str == "err" || level_str == "error") logger_->set_level(spdlog::level::err);
        else if (level_str == "critical") logger_->set_level(spdlog::level::critical);
        else if (level_str == "off") logger_->set_level(spdlog::level::off);
    }
}

void SpdlogBackend::log(LogLevel level, const std::string& message,
                        [[maybe_unused]] const std::source_location& loc) {
    if (logger_) {
        logger_->log(convertLevel(level), message);
    }
}

void SpdlogBackend::setLevel(LogLevel level) {
    if (logger_) {
        logger_->set_level(convertLevel(level));
    }
}

void SpdlogBackend::flush() {
    if (logger_) {
        logger_->flush();
    }
}

spdlog::level::level_enum SpdlogBackend::convertLevel(LogLevel level) {
    switch (level) {
        case LogLevel::Trace: return spdlog::level::trace;
        case LogLevel::Debug: return spdlog::level::debug;
        case LogLevel::Info: return spdlog::level::info;
        case LogLevel::Warn: return spdlog::level::warn;
        case LogLevel::Error: return spdlog::level::err;
        case LogLevel::Critical: return spdlog::level::critical;
        case LogLevel::Off: return spdlog::level::off;
        default: return spdlog::level::debug;
    }
}

}  // namespace arborvia
