#pragma once

#include "IPathFinder.h"
#include "../core/Types.h"

#include <memory>
#include <vector>
#include <cstdint>
#include <functional>

namespace arborvia {

// Forward declarations
struct EdgeLayout;
struct NodeLayout;

/// State of the routing coordinator
enum class RoutingState {
    Idle,      ///< No drag in progress
    Dragging,  ///< Drag in progress, using fast pathfinder
    Pending    ///< Drag ended, waiting for debounce timeout to run optimization
};

/// Listener interface for routing events
class IRoutingListener {
public:
    virtual ~IRoutingListener() = default;

    /// Called when optimization is complete after debounce delay
    /// @param optimizedEdges List of edges that were re-routed with optimal algorithm
    virtual void onOptimizationComplete(const std::vector<EdgeId>& optimizedEdges) = 0;
};

/// Coordinates between fast drag-time and optimal drop-time pathfinding algorithms
///
/// This coordinator manages two pathfinding algorithms:
/// - Drag-time: Fast, deterministic algorithm (e.g., LShapedPathFinder) for smooth drag
/// - Drop-time: Optimal algorithm (e.g., AStarPathFinder) that runs after drag ends
///
/// The coordinator implements a polling-based debounce mechanism:
/// 1. During drag: Uses fast pathfinder via currentPathFinder()
/// 2. On drag end: Transitions to Pending state, starts debounce timer
/// 3. On update(): If debounce delay elapsed, executes optimization
/// 4. If drag restarts during pending: Cancels pending optimization
///
/// Usage:
/// @code
/// auto coordinator = std::make_shared<PathRoutingCoordinator>();
/// coordinator->setDebounceDelay(300);  // 300ms delay
/// coordinator->setListener(myListener);
///
/// // On drag start
/// coordinator->onDragStart(affectedEdges);
///
/// // During drag (each frame)
/// auto& pathFinder = coordinator->currentPathFinder();
/// // Use pathFinder for routing...
///
/// // On drag end
/// coordinator->onDragEnd();
///
/// // Each frame in main loop
/// coordinator->update(getCurrentTimeMs());
/// @endcode
class PathRoutingCoordinator {
public:
    /// Default debounce delay in milliseconds
    static constexpr uint32_t DEFAULT_DEBOUNCE_DELAY_MS = 300;

    /// Construct coordinator with optional custom pathfinders
    /// @param dragFinder Pathfinder for drag operations (default: LShapedPathFinder)
    /// @param dropFinder Pathfinder for post-drop optimization (default: AStarPathFinder)
    explicit PathRoutingCoordinator(
        std::shared_ptr<IPathFinder> dragFinder = nullptr,
        std::shared_ptr<IPathFinder> dropFinder = nullptr);

    ~PathRoutingCoordinator() = default;

    // Non-copyable, movable
    PathRoutingCoordinator(const PathRoutingCoordinator&) = delete;
    PathRoutingCoordinator& operator=(const PathRoutingCoordinator&) = delete;
    PathRoutingCoordinator(PathRoutingCoordinator&&) = default;
    PathRoutingCoordinator& operator=(PathRoutingCoordinator&&) = default;

    // === Algorithm Access ===

    /// Set the pathfinder used during drag operations
    void setDragPathFinder(std::shared_ptr<IPathFinder> finder);

    /// Set the pathfinder used for post-drop optimization
    void setDropPathFinder(std::shared_ptr<IPathFinder> finder);

    /// Get the drag-time pathfinder
    IPathFinder& dragPathFinder() const;

    /// Get the drop-time pathfinder
    IPathFinder& dropPathFinder() const;

    /// Get the appropriate pathfinder based on current state
    /// - Dragging/Pending: Returns drag pathfinder
    /// - Idle: Returns drop pathfinder
    IPathFinder& currentPathFinder() const;

    // === State Management ===

    /// Call when drag operation starts
    /// @param affectedEdges Edges that will need re-routing
    void onDragStart(const std::vector<EdgeId>& affectedEdges);

    /// Call when drag operation ends (mouse released)
    /// Transitions to Pending state and starts debounce timer
    void onDragEnd();

    /// Add additional edges to the pending optimization queue
    /// Useful for adding edges that penetrate a newly positioned node
    /// @param edges Additional edges to optimize
    void addPendingEdges(const std::vector<EdgeId>& edges);

    /// Call each frame to check debounce timer and execute pending optimization
    /// @param currentTimeMs Current time in milliseconds
    void update(uint64_t currentTimeMs);

    /// Cancel any pending optimization and return to Idle state
    void cancelPendingOptimization();

    // === Configuration ===

    /// Set the debounce delay before optimization runs after drag end
    /// @param ms Delay in milliseconds (0 = run immediately)
    void setDebounceDelay(uint32_t ms);

    /// Get the current debounce delay
    uint32_t debounceDelay() const;

    // === Listener ===

    /// Set the listener for routing events
    /// @param listener Listener to notify (not owned, must outlive coordinator)
    void setListener(IRoutingListener* listener);

    // === State Query ===

    /// Get current routing state
    RoutingState state() const;

    /// Check if there's a pending optimization waiting to run
    bool isPendingOptimization() const;

    /// Get edges affected by current/pending drag operation
    const std::vector<EdgeId>& pendingEdges() const;

    // === Optimization Execution ===

    /// Callback type for optimization execution
    /// The callback receives the edges to optimize and should use dropPathFinder()
    using OptimizationCallback = std::function<void(const std::vector<EdgeId>&)>;

    /// Set callback for executing the actual optimization
    /// This allows the coordinator to trigger re-routing through EdgeRouting
    void setOptimizationCallback(OptimizationCallback callback);

private:
    std::shared_ptr<IPathFinder> dragPathFinder_;
    std::shared_ptr<IPathFinder> dropPathFinder_;

    RoutingState state_ = RoutingState::Idle;
    uint64_t dropTimeMs_ = 0;
    uint32_t debounceDelayMs_ = DEFAULT_DEBOUNCE_DELAY_MS;

    std::vector<EdgeId> pendingEdges_;
    IRoutingListener* listener_ = nullptr;
    OptimizationCallback optimizationCallback_;

    void executeOptimization();
};

}  // namespace arborvia
