#pragma once

#include "arborvia/common/ILoggerBackend.h"
#include <mutex>

namespace arborvia {

/**
 * @brief Simple stdout logger with no external dependencies
 *
 * Provides basic logging to stdout with:
 * - Thread-safe output (std::mutex)
 * - Timestamp (HH:MM:SS.mmm)
 * - Log level coloring (ANSI codes)
 *
 * For production use with advanced features (file logging, rotation),
 * use SpdlogBackend or inject a custom ILoggerBackend.
 */
class DefaultBackend : public ILoggerBackend {
public:
    DefaultBackend();

    void log(LogLevel level, const std::string& message,
             const std::source_location& loc) override;
    void setLevel(LogLevel level) override;
    void flush() override;

private:
    LogLevel currentLevel_;
    std::mutex mutex_;

    const char* levelToString(LogLevel level);
    const char* levelToColor(LogLevel level);
    std::string getTimestamp();
};

}  // namespace arborvia
