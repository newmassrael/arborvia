#pragma once

#include "arborvia/common/ILoggerBackend.h"
#include <memory>
#include <spdlog/spdlog.h>

namespace arborvia {

/**
 * @brief spdlog-based logger backend
 *
 * Provides full spdlog features:
 * - Multiple sinks (console, file)
 * - File rotation
 * - High performance
 *
 * Default backend when ARBORVIA_USE_SPDLOG=ON.
 */
class SpdlogBackend : public ILoggerBackend {
public:
    SpdlogBackend(const std::string& logDir = "", bool logToFile = false);

    void log(LogLevel level, const std::string& message,
             const std::source_location& loc) override;
    void setLevel(LogLevel level) override;
    void flush() override;

private:
    std::shared_ptr<spdlog::logger> logger_;
    spdlog::level::level_enum convertLevel(LogLevel level);
};

}  // namespace arborvia
