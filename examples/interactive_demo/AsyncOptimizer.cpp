#include "AsyncOptimizer.h"
#include <arborvia/layout/util/LayoutUtils.h>
#include <chrono>
#include <iostream>
#include <thread>

namespace arborvia {

AsyncOptimizer::AsyncOptimizer(std::shared_ptr<PathRoutingCoordinator> routingCoordinator)
    : routingCoordinator_(std::move(routingCoordinator))
    , executor_(std::make_unique<JThreadExecutor>()) {
}

AsyncOptimizer::~AsyncOptimizer() {
    shutdown();
}

void AsyncOptimizer::shutdown() {
    if (executor_) {
        executor_->shutdown();
    }
}

void AsyncOptimizer::startOptimization(const std::unordered_set<NodeId>& movedNodes) {
    if (!getSnapshotCallback_) {
        std::cerr << "[AsyncOptimizer] No snapshot callback set" << std::endl;
        return;
    }

    // Get current generation for this optimization
    uint64_t gen = routingCoordinator_->currentGeneration();

    // Get snapshot via callback
    auto snapshot = getSnapshotCallback_();

    // Collect all edges for optimization
    std::vector<EdgeId> allEdges;
    allEdges.reserve(snapshot.edgeLayouts.size());
    for (const auto& [edgeId, layout] : snapshot.edgeLayouts) {
        allEdges.push_back(edgeId);
    }

    std::cout << "[Async] Starting optimization gen=" << gen
              << " edges=" << allEdges.size()
              << " movedNodes=" << movedNodes.size() << std::endl;

    // Start async optimization via executor
    executor_->submit([this, gen, allEdges, snapshot, movedNodes]() {
        // Worker thread: run optimization on snapshot
        auto workingEdges = snapshot.edgeLayouts;

        LayoutUtils::updateEdgePositions(
            workingEdges, snapshot.nodeLayouts, allEdges,
            snapshot.options, movedNodes);

        // Post result to main thread
        postToMainThread([this, gen, workingEdges = std::move(workingEdges)]() {
            onOptimizationComplete(workingEdges, gen);
        });
    });
}

void AsyncOptimizer::processMainThreadQueue() {
    std::queue<std::function<void()>> toProcess;
    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        std::swap(toProcess, mainThreadQueue_);
    }
    while (!toProcess.empty()) {
        toProcess.front()();
        toProcess.pop();
    }
}

bool AsyncOptimizer::waitForIdle(int timeoutMs) {
    auto startTime = std::chrono::steady_clock::now();
    while (routingCoordinator_->state() != RoutingState::Idle) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - startTime).count();
        if (elapsed >= timeoutMs) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return true;
}

void AsyncOptimizer::postToMainThread(std::function<void()> fn) {
    std::lock_guard<std::mutex> lock(queueMutex_);
    mainThreadQueue_.push(std::move(fn));
}

void AsyncOptimizer::onOptimizationComplete(
    const std::unordered_map<EdgeId, EdgeLayout>& result,
    uint64_t generation)
{
    // Check if result is still valid
    if (!routingCoordinator_->canApplyResult(generation)) {
        std::cout << "[Async] Result discarded (gen=" << generation
                  << ", current=" << routingCoordinator_->currentGeneration()
                  << ", state=" << static_cast<int>(routingCoordinator_->state())
                  << ")" << std::endl;
        return;
    }

    // Apply result via callback
    if (applyResultCallback_) {
        applyResultCallback_(result);
    }

    // Notify coordinator (triggers listener callback)
    std::vector<EdgeId> optimizedEdges;
    optimizedEdges.reserve(result.size());
    for (const auto& [id, layout] : result) {
        optimizedEdges.push_back(id);
    }
    routingCoordinator_->notifyOptimizationComplete(optimizedEdges);

    std::cout << "[Async] Result applied (gen=" << generation
              << ", edges=" << result.size() << ")" << std::endl;
}

}  // namespace arborvia
