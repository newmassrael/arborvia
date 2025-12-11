#include "arborvia/common/Logger.h"

#ifdef ARBORVIA_USE_SPDLOG
#include "arborvia/backends/SpdlogBackend.h"
#else
#include "arborvia/backends/DefaultBackend.h"
#endif

#include <algorithm>
#include <mutex>
#include <sstream>

namespace arborvia {

std::unique_ptr<ILoggerBackend> Logger::backend_;
static std::mutex backend_mutex;

// Log capture state
static bool capture_enabled_ = false;
static std::vector<std::string> captured_logs_;
static std::mutex capture_mutex_;

void Logger::setBackend(std::unique_ptr<ILoggerBackend> backend) {
    std::lock_guard<std::mutex> lock(backend_mutex);
    backend_ = std::move(backend);
}

void Logger::initialize() {
    std::lock_guard<std::mutex> lock(backend_mutex);
    if (!backend_) {
#ifdef ARBORVIA_USE_SPDLOG
        backend_ = std::make_unique<SpdlogBackend>();
#else
        backend_ = std::make_unique<DefaultBackend>();
#endif
    }
}

void Logger::initialize([[maybe_unused]] const std::string& logDir,
                        [[maybe_unused]] bool logToFile) {
    std::lock_guard<std::mutex> lock(backend_mutex);
    if (!backend_) {
#ifdef ARBORVIA_USE_SPDLOG
        backend_ = std::make_unique<SpdlogBackend>(logDir, logToFile);
#else
        backend_ = std::make_unique<DefaultBackend>();
#endif
    }
}

void Logger::setLevel(LogLevel level) {
    ensureBackend();
    backend_->setLevel(level);
}

void Logger::trace(const std::string& message, const std::source_location& loc) {
    ensureBackend();
    std::string enhanced = extractFunctionName(loc) + "() - " + message;
    backend_->log(LogLevel::Trace, enhanced, loc);
    captureLog("[trace] " + enhanced);
}

void Logger::debug(const std::string& message, const std::source_location& loc) {
    ensureBackend();
    std::string enhanced = extractFunctionName(loc) + "() - " + message;
    backend_->log(LogLevel::Debug, enhanced, loc);
    captureLog("[debug] " + enhanced);
}

void Logger::info(const std::string& message, const std::source_location& loc) {
    ensureBackend();
    std::string enhanced = extractFunctionName(loc) + "() - " + message;
    backend_->log(LogLevel::Info, enhanced, loc);
    captureLog("[info] " + enhanced);
}

void Logger::warn(const std::string& message, const std::source_location& loc) {
    ensureBackend();
    std::string enhanced = extractFunctionName(loc) + "() - " + message;
    backend_->log(LogLevel::Warn, enhanced, loc);
    captureLog("[warn] " + enhanced);
}

void Logger::error(const std::string& message, const std::source_location& loc) {
    ensureBackend();
    std::string enhanced = extractFunctionName(loc) + "() - " + message;
    backend_->log(LogLevel::Error, enhanced, loc);
    captureLog("[error] " + enhanced);
}

void Logger::flush() {
    ensureBackend();
    backend_->flush();
}

void Logger::ensureBackend() {
    if (!backend_) {
        initialize();
    }
}

// ===== Log Capture Implementation =====

void Logger::enableCapture(bool enable) {
    std::lock_guard<std::mutex> lock(capture_mutex_);
    capture_enabled_ = enable;
}

bool Logger::isCaptureEnabled() {
    std::lock_guard<std::mutex> lock(capture_mutex_);
    return capture_enabled_;
}

std::vector<std::string> Logger::getCapturedLogs(const std::string& pattern, size_t maxLines) {
    std::lock_guard<std::mutex> lock(capture_mutex_);
    
    std::vector<std::string> result;
    
    // Filter by pattern
    for (const auto& line : captured_logs_) {
        if (pattern.empty() || line.find(pattern) != std::string::npos) {
            result.push_back(line);
        }
    }
    
    // Apply maxLines limit (return last N lines)
    if (maxLines > 0 && result.size() > maxLines) {
        result.erase(result.begin(), result.begin() + (result.size() - maxLines));
    }
    
    return result;
}

void Logger::clearCapturedLogs() {
    std::lock_guard<std::mutex> lock(capture_mutex_);
    captured_logs_.clear();
}

void Logger::captureLog(const std::string& message) {
    std::lock_guard<std::mutex> lock(capture_mutex_);
    if (capture_enabled_) {
        captured_logs_.push_back(message);
    }
}

std::string Logger::extractFunctionName(const std::source_location& loc) {
    std::string full_name = loc.function_name();

    // Find opening parenthesis
    size_t paren_pos = full_name.find('(');
    if (paren_pos == std::string::npos) {
        return "Unknown";
    }

    // Find start of function name (after last space before parenthesis)
    size_t name_end = paren_pos;
    size_t name_start = 0;

    // Skip template parameters and find last space
    int angle_count = 0;
    size_t last_space = std::string::npos;
    for (size_t i = 0; i < name_end; ++i) {
        char c = full_name[i];
        if (c == '<') angle_count++;
        else if (c == '>') angle_count--;
        else if (c == ' ' && angle_count == 0) last_space = i;
    }

    if (last_space != std::string::npos) {
        name_start = last_space + 1;
    }

    std::string qualified = full_name.substr(name_start, name_end - name_start);

    // Remove template parameters from result
    std::string result;
    angle_count = 0;
    for (char c : qualified) {
        if (c == '<') angle_count++;
        else if (c == '>') angle_count--;
        else if (angle_count == 0) result += c;
    }

    // Trim whitespace and pointer/reference symbols
    while (!result.empty() && (std::isspace(result[0]) || result[0] == '*' || result[0] == '&')) {
        result.erase(0, 1);
    }
    while (!result.empty() && std::isspace(result.back())) {
        result.pop_back();
    }

    return result.empty() ? "Unknown" : result;
}

}  // namespace arborvia
