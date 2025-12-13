#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace arborvia {

/// Interface for task execution strategies
/// Allows swapping between jthread, thread pool, or other implementations
class ITaskExecutor {
public:
    virtual ~ITaskExecutor() = default;

    /// Submit a task for execution
    /// @param task The task to execute
    virtual void submit(std::function<void()> task) = 0;

    /// Shutdown the executor and wait for pending tasks
    virtual void shutdown() = 0;

    /// Check if executor is running
    virtual bool isRunning() const = 0;
};

/// Simple executor using std::jthread for each task
/// Each task runs in its own thread with automatic join on destruction
class JThreadExecutor : public ITaskExecutor {
public:
    JThreadExecutor() = default;

    ~JThreadExecutor() override {
        shutdown();
    }

    // Non-copyable
    JThreadExecutor(const JThreadExecutor&) = delete;
    JThreadExecutor& operator=(const JThreadExecutor&) = delete;

    void submit(std::function<void()> task) override {
        if (!running_.load()) {
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        // Create jthread that auto-joins on destruction
        threads_.emplace_back([task = std::move(task)]() {
            task();
        });
    }

    void shutdown() override {
        running_.store(false);
        std::lock_guard<std::mutex> lock(mutex_);
        // jthreads auto-join in destructor when cleared
        threads_.clear();
    }

    bool isRunning() const override {
        return running_.load();
    }

private:
    std::vector<std::jthread> threads_;
    std::mutex mutex_;
    std::atomic<bool> running_{true};
};

}  // namespace arborvia
