#pragma once

#include <arborvia/arborvia.h>
#include <arborvia/layout/config/LayoutOptions.h>
#include <arborvia/layout/interactive/PathRoutingCoordinator.h>
#include <arborvia/core/TaskExecutor.h>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace arborvia {

/// Manages asynchronous edge optimization for the interactive demo
///
/// Extracts async optimization logic from InteractiveDemo for better separation of concerns.
/// Handles worker thread submission, main thread queue, and result application.
class AsyncOptimizer {
public:
    /// Snapshot of layout state for async processing
    struct LayoutSnapshot {
        std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;
        std::unordered_map<NodeId, NodeLayout> nodeLayouts;
        LayoutOptions options;
    };

    /// Callback types
    using GetSnapshotCallback = std::function<LayoutSnapshot()>;
    using ApplyResultCallback = std::function<void(const std::unordered_map<EdgeId, EdgeLayout>&)>;

    /// Constructor
    /// @param routingCoordinator For generation tracking and state management
    explicit AsyncOptimizer(std::shared_ptr<PathRoutingCoordinator> routingCoordinator);

    /// Destructor - ensures executor shutdown
    ~AsyncOptimizer();

    /// Set callback to get current layout snapshot
    void setGetSnapshotCallback(GetSnapshotCallback cb) { getSnapshotCallback_ = std::move(cb); }

    /// Set callback to apply optimization result
    void setApplyResultCallback(ApplyResultCallback cb) { applyResultCallback_ = std::move(cb); }

    /// Start async optimization for affected edges
    /// @param movedNodes Set of nodes that were moved (triggers optimization)
    void startOptimization(const std::unordered_set<NodeId>& movedNodes);

    /// Process pending results on main thread (call from main loop)
    void processMainThreadQueue();

    /// Wait for optimization to become idle
    /// @param timeoutMs Timeout in milliseconds
    /// @return true if idle, false if timeout
    bool waitForIdle(int timeoutMs);

    /// Shutdown the executor (call before destruction if needed)
    void shutdown();

private:
    void postToMainThread(std::function<void()> fn);
    void onOptimizationComplete(
        const std::unordered_map<EdgeId, EdgeLayout>& result,
        uint64_t generation);

    std::shared_ptr<PathRoutingCoordinator> routingCoordinator_;
    std::unique_ptr<ITaskExecutor> executor_;
    std::queue<std::function<void()>> mainThreadQueue_;
    std::mutex queueMutex_;

    GetSnapshotCallback getSnapshotCallback_;
    ApplyResultCallback applyResultCallback_;
};

}  // namespace arborvia
