#include "arborvia/layout/interactive/PathRoutingCoordinator.h"
#include "../pathfinding/LShapedPathFinder.h"
#include "../pathfinding/AStarPathFinder.h"

namespace arborvia {


PathRoutingCoordinator::PathRoutingCoordinator(
    std::shared_ptr<IPathFinder> dragFinder,
    std::shared_ptr<IPathFinder> dropFinder)
    : dragPathFinder_(dragFinder ? std::move(dragFinder)
                                 : std::make_shared<LShapedPathFinder>())
    , dropPathFinder_(dropFinder ? std::move(dropFinder)
                                 : std::make_shared<AStarPathFinder>())
{
}

// === Algorithm Access ===

void PathRoutingCoordinator::setDragPathFinder(std::shared_ptr<IPathFinder> finder) {
    if (finder) {
        dragPathFinder_ = std::move(finder);
    }
}

void PathRoutingCoordinator::setDropPathFinder(std::shared_ptr<IPathFinder> finder) {
    if (finder) {
        dropPathFinder_ = std::move(finder);
    }
}

IPathFinder& PathRoutingCoordinator::dragPathFinder() const {
    return *dragPathFinder_;
}

IPathFinder& PathRoutingCoordinator::dropPathFinder() const {
    return *dropPathFinder_;
}

IPathFinder& PathRoutingCoordinator::currentPathFinder() const {
    // During drag or pending, use fast pathfinder
    // In idle state, use optimal pathfinder
    if (state_ == RoutingState::Idle) {
        return *dropPathFinder_;
    }
    return *dragPathFinder_;
}

// === State Management ===

void PathRoutingCoordinator::onDragStart(const std::vector<EdgeId>& affectedEdges) {
    // If we were pending, cancel the pending optimization
    if (state_ == RoutingState::Pending) {
        // Cancel pending - new drag started
    }

    // Invalidate previous async results
    ++generation_;

    state_ = RoutingState::Dragging;
    pendingEdges_ = affectedEdges;
    dropTimeMs_ = 0;
}

void PathRoutingCoordinator::onDragEnd(const std::unordered_set<NodeId>& movedNodes) {
    if (state_ != RoutingState::Dragging) {
        return;  // Not dragging, nothing to do
    }

    // Store moved nodes for optimization callback
    movedNodes_ = movedNodes;

    if (debounceDelayMs_ == 0) {
        // No delay, execute immediately
        state_ = RoutingState::Idle;
        executeOptimization();
    } else {
        // Start debounce timer - will be checked in update()
        state_ = RoutingState::Pending;
        // Note: dropTimeMs_ will be set in update() on first call
        // This allows the caller to provide accurate time
        dropTimeMs_ = 0;  // Marker: needs timestamp on first update
    }
}

void PathRoutingCoordinator::addPendingEdges(const std::vector<EdgeId>& edges) {
    for (const auto& edgeId : edges) {
        // Add only if not already in pending list
        if (std::find(pendingEdges_.begin(), pendingEdges_.end(), edgeId) == pendingEdges_.end()) {
            pendingEdges_.push_back(edgeId);
        }
    }
}

void PathRoutingCoordinator::update(uint64_t currentTimeMs) {
    if (state_ != RoutingState::Pending) {
        return;  // Nothing to do
    }

    // First update after onDragEnd - record the drop time
    if (dropTimeMs_ == 0) {
        dropTimeMs_ = currentTimeMs;
        return;
    }

    // Check if debounce delay has elapsed
    uint64_t elapsed = currentTimeMs - dropTimeMs_;
    if (elapsed >= debounceDelayMs_) {
        state_ = RoutingState::Idle;
        executeOptimization();
    }
}

void PathRoutingCoordinator::cancelPendingOptimization() {
    if (state_ == RoutingState::Pending) {
        state_ = RoutingState::Idle;
        pendingEdges_.clear();
        movedNodes_.clear();
        dropTimeMs_ = 0;
    }
}

// === Configuration ===

void PathRoutingCoordinator::setDebounceDelay(uint32_t ms) {
    debounceDelayMs_ = ms;
}

uint32_t PathRoutingCoordinator::debounceDelay() const {
    return debounceDelayMs_;
}

// === Listener ===

void PathRoutingCoordinator::setListener(IRoutingListener* listener) {
    listener_ = listener;
}

// === State Query ===

RoutingState PathRoutingCoordinator::state() const {
    return state_;
}

bool PathRoutingCoordinator::isPendingOptimization() const {
    return state_ == RoutingState::Pending;
}

const std::vector<EdgeId>& PathRoutingCoordinator::pendingEdges() const {
    return pendingEdges_;
}

// === Optimization Execution ===

void PathRoutingCoordinator::setOptimizationCallback(OptimizationCallback callback) {
    optimizationCallback_ = std::move(callback);
}

void PathRoutingCoordinator::executeOptimization() {
    if (pendingEdges_.empty()) {
        return;
    }

    // Call optimization callback (both sync and async modes)
    if (optimizationCallback_) {
        optimizationCallback_(pendingEdges_, movedNodes_);
    }

    // Sync mode: notify listener immediately
    // Async mode: caller must call notifyOptimizationComplete() after applying results
    if (!asyncMode_ && listener_) {
        listener_->onOptimizationComplete(pendingEdges_);
    }

    // Clear pending state
    pendingEdges_.clear();
    movedNodes_.clear();
    dropTimeMs_ = 0;
}


// === Async Mode ===

void PathRoutingCoordinator::setAsyncMode(bool enabled) {
    asyncMode_ = enabled;
}

bool PathRoutingCoordinator::isAsyncMode() const {
    return asyncMode_;
}

uint64_t PathRoutingCoordinator::currentGeneration() const {
    return generation_.load();
}

bool PathRoutingCoordinator::canApplyResult(uint64_t generation) const {
    // Result is valid only if:
    // 1. Generation matches (no new drag started)
    // 2. Not currently dragging
    return generation == generation_.load() && state_ != RoutingState::Dragging;
}

void PathRoutingCoordinator::notifyOptimizationComplete(const std::vector<EdgeId>& optimizedEdges) {
    // For async mode: caller notifies listener after applying results
    if (listener_) {
        listener_->onOptimizationComplete(optimizedEdges);
    }
}

}  // namespace arborvia
