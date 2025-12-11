#pragma once

#include "arborvia/common/ILoggerBackend.h"
#include <format>
#include <memory>
#include <source_location>
#include <string>
#include <vector>

namespace arborvia {

/**
 * @brief Centralized logging facade with dependency injection support
 *
 * Supports three usage patterns:
 * 1. Default mode: Uses built-in backend (spdlog if available, DefaultBackend otherwise)
 * 2. Custom mode: Users inject their own ILoggerBackend implementation
 * 3. Capture mode: Enable log capture for programmatic retrieval
 *
 * Thread-safe: All operations are thread-safe via backend implementation.
 *
 * Example:
 * @code
 * arborvia::Logger::initialize();
 * arborvia::Logger::enableCapture(true);  // Enable log capture
 * LOG_INFO("Layout completed in {} ms", elapsed);
 * auto logs = arborvia::Logger::getCapturedLogs();  // Retrieve captured logs
 * @endcode
 */
class Logger {
public:
    /**
     * @brief Inject custom logger backend
     * @param backend User's logger backend (ownership transferred)
     */
    static void setBackend(std::unique_ptr<ILoggerBackend> backend);

    /**
     * @brief Initialize default logger (stdout only)
     */
    static void initialize();

    /**
     * @brief Initialize default logger with file output
     * @param logDir Directory for log files
     * @param logToFile Enable file logging
     */
    static void initialize(const std::string& logDir, bool logToFile = true);

    /**
     * @brief Set minimum log level
     */
    static void setLevel(LogLevel level);

    // Logging methods
    static void trace(const std::string& message,
                      const std::source_location& loc = std::source_location::current());
    static void debug(const std::string& message,
                      const std::source_location& loc = std::source_location::current());
    static void info(const std::string& message,
                     const std::source_location& loc = std::source_location::current());
    static void warn(const std::string& message,
                     const std::source_location& loc = std::source_location::current());
    static void error(const std::string& message,
                      const std::source_location& loc = std::source_location::current());

    /**
     * @brief Flush log buffers
     */
    static void flush();

    // ===== Log Capture API =====
    
    /**
     * @brief Enable or disable log capture
     * @param enable true to start capturing, false to stop
     * 
     * When enabled, all log messages are stored in memory in addition to
     * being sent to the backend. Use getCapturedLogs() to retrieve them.
     */
    static void enableCapture(bool enable);
    
    /**
     * @brief Check if log capture is enabled
     */
    static bool isCaptureEnabled();
    
    /**
     * @brief Get captured logs with optional filtering
     * @param pattern Optional substring filter (empty = all logs)
     * @param maxLines Maximum number of lines to return (0 = unlimited)
     * @return Vector of log lines matching the filter
     */
    static std::vector<std::string> getCapturedLogs(
        const std::string& pattern = "", 
        size_t maxLines = 0);
    
    /**
     * @brief Clear captured logs buffer
     */
    static void clearCapturedLogs();

private:
    static std::unique_ptr<ILoggerBackend> backend_;
    static void ensureBackend();
    static std::string extractFunctionName(const std::source_location& loc);
    static void captureLog(const std::string& message);
};

}  // namespace arborvia

// Logging macros with std::format support
#define LOG_TRACE(...) arborvia::Logger::trace(std::format(__VA_ARGS__), std::source_location::current())
#define LOG_DEBUG(...) arborvia::Logger::debug(std::format(__VA_ARGS__), std::source_location::current())
#define LOG_INFO(...)  arborvia::Logger::info(std::format(__VA_ARGS__), std::source_location::current())
#define LOG_WARN(...)  arborvia::Logger::warn(std::format(__VA_ARGS__), std::source_location::current())
#define LOG_ERROR(...) arborvia::Logger::error(std::format(__VA_ARGS__), std::source_location::current())
