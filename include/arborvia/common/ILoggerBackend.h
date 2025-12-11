#pragma once

#include <source_location>
#include <string>

namespace arborvia {

/**
 * @brief Log level enumeration
 */
enum class LogLevel {
    Trace = 0,
    Debug = 1,
    Info = 2,
    Warn = 3,
    Error = 4,
    Critical = 5,
    Off = 6
};

/**
 * @brief Logger backend interface for dependency injection
 *
 * Implement this interface to integrate custom logging systems.
 * Allows arborvia to use any logging framework without compile-time dependencies.
 *
 * Example:
 * @code
 * class MyLogger : public arborvia::ILoggerBackend {
 * public:
 *     void log(LogLevel level, const std::string& message,
 *              const std::source_location& loc) override {
 *         mySystem->write(level, message, loc.file_name(), loc.line());
 *     }
 *     void setLevel(LogLevel level) override { mySystem->setMinLevel(level); }
 *     void flush() override { mySystem->flush(); }
 * };
 *
 * arborvia::Logger::setBackend(std::make_unique<MyLogger>());
 * @endcode
 */
class ILoggerBackend {
public:
    virtual ~ILoggerBackend() = default;

    /**
     * @brief Log a message with source location
     * @param level Log level
     * @param message Pre-formatted message
     * @param loc Source location (file, line, function)
     */
    virtual void log(LogLevel level, const std::string& message,
                     const std::source_location& loc) = 0;

    /**
     * @brief Set minimum log level
     */
    virtual void setLevel(LogLevel level) = 0;

    /**
     * @brief Flush log buffers
     */
    virtual void flush() = 0;
};

}  // namespace arborvia
