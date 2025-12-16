#pragma once

#include <iostream>
#include <sstream>
#include <mutex>
#include <string>
#include <streambuf>

namespace arborvia {

/// Captures std::cout output for TCP retrieval
///
/// Singleton class that redirects std::cout to an internal buffer while
/// also forwarding to the original output stream.
class LogCapture {
public:
    static LogCapture& instance() {
        static LogCapture inst;
        return inst;
    }

    void install() {
        if (!installed_) {
            originalBuf_ = std::cout.rdbuf(&captureBuf_);
            installed_ = true;
        }
    }

    void uninstall() {
        if (installed_) {
            std::cout.rdbuf(originalBuf_);
            installed_ = false;
        }
    }

    std::string getAndClear() {
        std::lock_guard<std::mutex> lock(mutex_);
        std::string result = buffer_.str();
        buffer_.str("");
        buffer_.clear();
        return result;
    }

    std::string get() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return buffer_.str();
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        buffer_.str("");
        buffer_.clear();
    }

    void append(const std::string& s) {
        std::lock_guard<std::mutex> lock(mutex_);
        buffer_ << s;
    }

private:
    LogCapture() : captureBuf_(buffer_, originalBuf_, mutex_) {}
    ~LogCapture() { uninstall(); }

    // Custom streambuf that captures output and also forwards to original
    class CaptureStreamBuf : public std::streambuf {
    public:
        CaptureStreamBuf(std::ostringstream& buffer, std::streambuf*& original, std::mutex& mutex)
            : buffer_(buffer), original_(original), mutex_(mutex) {}

    protected:
        int overflow(int c) override {
            if (c != EOF) {
                std::lock_guard<std::mutex> lock(mutex_);
                buffer_.put(static_cast<char>(c));
                if (original_) {
                    original_->sputc(static_cast<char>(c));
                }
            }
            return c;
        }

        std::streamsize xsputn(const char* s, std::streamsize n) override {
            std::lock_guard<std::mutex> lock(mutex_);
            buffer_.write(s, n);
            if (original_) {
                original_->sputn(s, n);
            }
            return n;
        }

        int sync() override {
            if (original_) {
                original_->pubsync();
            }
            return 0;
        }

    private:
        std::ostringstream& buffer_;
        std::streambuf*& original_;
        std::mutex& mutex_;
    };

    std::ostringstream buffer_;
    std::streambuf* originalBuf_ = nullptr;
    CaptureStreamBuf captureBuf_;
    mutable std::mutex mutex_;
    bool installed_ = false;
};

}  // namespace arborvia
