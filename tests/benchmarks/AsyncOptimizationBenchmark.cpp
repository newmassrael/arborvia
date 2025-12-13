#include <arborvia/arborvia.h>
#include <arborvia/core/TaskExecutor.h>
#include <arborvia/layout/interactive/PathRoutingCoordinator.h>

// Internal headers for detailed benchmarking
#include "pathfinding/ObstacleMap.h"
#include "pathfinding/AStarPathFinder.h"

#include <benchmark/benchmark.h>

#include <memory>
#include <queue>
#include <random>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace arborvia;

// ============================================================================
// Test Data Generation
// ============================================================================

class BenchmarkFixture : public benchmark::Fixture {
protected:
    void SetUp(const ::benchmark::State& state) override {
        const int numNodes = state.range(0);
        const int numEdges = state.range(1);
        generateTestData(numNodes, numEdges);
    }

    void TearDown(const ::benchmark::State& /*state*/) override {
        nodeLayouts_.clear();
        edgeLayouts_.clear();
        affectedEdges_.clear();
    }

    void generateTestData(int numNodes, int numEdges) {
        std::mt19937 rng(42);  // Fixed seed for reproducibility
        std::uniform_real_distribution<float> posDist(0.0f, 1000.0f);
        std::uniform_real_distribution<float> sizeDist(50.0f, 150.0f);

        // Generate nodes
        for (int i = 0; i < numNodes; ++i) {
            NodeId id = static_cast<NodeId>(i);
            NodeLayout layout;
            layout.position = {posDist(rng), posDist(rng)};
            layout.size = {sizeDist(rng), sizeDist(rng)};
            nodeLayouts_[id] = layout;
        }

        // Generate edges
        std::uniform_int_distribution<int> nodeDist(0, numNodes - 1);
        for (int i = 0; i < numEdges; ++i) {
            EdgeId id = static_cast<EdgeId>(i);
            EdgeLayout layout;
            layout.from = static_cast<NodeId>(nodeDist(rng));
            layout.to = static_cast<NodeId>(nodeDist(rng));
            while (layout.to == layout.from) {
                layout.to = static_cast<NodeId>(nodeDist(rng));
            }

            // Generate some bend points
            int numBends = 2 + (i % 3);
            for (int j = 0; j < numBends; ++j) {
                layout.bendPoints.push_back({{posDist(rng), posDist(rng)}});
            }

            edgeLayouts_[id] = layout;
        }

        // Select ~20% of edges as "affected"
        int numAffected = std::max(1, numEdges / 5);
        for (int i = 0; i < numAffected; ++i) {
            affectedEdges_.push_back(static_cast<EdgeId>(i));
        }
    }

    std::unordered_map<NodeId, NodeLayout> nodeLayouts_;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts_;
    std::vector<EdgeId> affectedEdges_;
};

// ============================================================================
// Snapshot Copy Benchmarks
// ============================================================================

// Benchmark: Copy entire edge layouts (current implementation)
BENCHMARK_DEFINE_F(BenchmarkFixture, SnapshotCopy_Full)(benchmark::State& state) {
    for (auto _ : state) {
        auto snapshot = edgeLayouts_;
        benchmark::DoNotOptimize(snapshot);
    }

    state.SetItemsProcessed(state.iterations() * edgeLayouts_.size());
    state.SetLabel("Full copy: " + std::to_string(edgeLayouts_.size()) + " edges");
}

// Benchmark: Copy only affected edge layouts (proposed optimization)
BENCHMARK_DEFINE_F(BenchmarkFixture, SnapshotCopy_Partial)(benchmark::State& state) {
    for (auto _ : state) {
        std::unordered_map<EdgeId, EdgeLayout> snapshot;
        snapshot.reserve(affectedEdges_.size());
        for (const auto& id : affectedEdges_) {
            auto it = edgeLayouts_.find(id);
            if (it != edgeLayouts_.end()) {
                snapshot[id] = it->second;
            }
        }
        benchmark::DoNotOptimize(snapshot);
    }

    state.SetItemsProcessed(state.iterations() * affectedEdges_.size());
    state.SetLabel("Partial copy: " + std::to_string(affectedEdges_.size()) + " edges");
}

// Register with different sizes: (nodes, edges)
BENCHMARK_REGISTER_F(BenchmarkFixture, SnapshotCopy_Full)
    ->Args({10, 20})    // Small
    ->Args({50, 100})   // Medium
    ->Args({100, 300}); // Large

BENCHMARK_REGISTER_F(BenchmarkFixture, SnapshotCopy_Partial)
    ->Args({10, 20})
    ->Args({50, 100})
    ->Args({100, 300});

// ============================================================================
// Edge Collection Benchmarks
// ============================================================================

// Benchmark: Collect all edges (current implementation)
BENCHMARK_DEFINE_F(BenchmarkFixture, EdgeCollection_All)(benchmark::State& state) {
    for (auto _ : state) {
        std::vector<EdgeId> allEdges;
        allEdges.reserve(edgeLayouts_.size());
        for (const auto& [edgeId, layout] : edgeLayouts_) {
            allEdges.push_back(edgeId);
        }
        benchmark::DoNotOptimize(allEdges);
    }

    state.SetItemsProcessed(state.iterations() * edgeLayouts_.size());
}

// Benchmark: Use affected edges directly (proposed optimization)
BENCHMARK_DEFINE_F(BenchmarkFixture, EdgeCollection_Affected)(benchmark::State& state) {
    for (auto _ : state) {
        std::vector<EdgeId> edges = affectedEdges_;
        benchmark::DoNotOptimize(edges);
    }

    state.SetItemsProcessed(state.iterations() * affectedEdges_.size());
}

BENCHMARK_REGISTER_F(BenchmarkFixture, EdgeCollection_All)
    ->Args({10, 20})
    ->Args({50, 100})
    ->Args({100, 300});

BENCHMARK_REGISTER_F(BenchmarkFixture, EdgeCollection_Affected)
    ->Args({10, 20})
    ->Args({50, 100})
    ->Args({100, 300});

// ============================================================================
// Thread Creation Benchmarks
// ============================================================================

// Benchmark: JThreadExecutor task submission
static void BM_ThreadCreation_JThread(benchmark::State& state) {
    JThreadExecutor executor;
    std::atomic<int> counter{0};

    for (auto _ : state) {
        executor.submit([&counter]() {
            counter.fetch_add(1, std::memory_order_relaxed);
        });
    }

    // Wait for all tasks to complete
    executor.shutdown();

    state.SetItemsProcessed(state.iterations());
    state.SetLabel("JThread creation + execution");
}

BENCHMARK(BM_ThreadCreation_JThread)->Iterations(1000);

// Benchmark: Direct std::jthread creation (baseline)
static void BM_ThreadCreation_Direct(benchmark::State& state) {
    std::atomic<int> counter{0};
    std::vector<std::jthread> threads;
    threads.reserve(state.max_iterations);

    for (auto _ : state) {
        threads.emplace_back([&counter]() {
            counter.fetch_add(1, std::memory_order_relaxed);
        });
    }

    // jthreads auto-join on destruction
    threads.clear();

    state.SetItemsProcessed(state.iterations());
    state.SetLabel("Direct jthread creation");
}

BENCHMARK(BM_ThreadCreation_Direct)->Iterations(1000);

// ============================================================================
// Main Thread Queue Benchmarks
// ============================================================================

static void BM_MainThreadQueue_Push(benchmark::State& state) {
    std::queue<std::function<void()>> queue;
    std::mutex mutex;

    for (auto _ : state) {
        std::lock_guard<std::mutex> lock(mutex);
        queue.push([]() {});
    }

    state.SetItemsProcessed(state.iterations());
}

BENCHMARK(BM_MainThreadQueue_Push);

static void BM_MainThreadQueue_Process(benchmark::State& state) {
    const int batchSize = state.range(0);

    for (auto _ : state) {
        state.PauseTiming();
        std::queue<std::function<void()>> queue;
        std::mutex mutex;

        // Fill queue
        for (int i = 0; i < batchSize; ++i) {
            queue.push([]() {});
        }
        state.ResumeTiming();

        // Process queue (simulating main thread processing)
        std::queue<std::function<void()>> toProcess;
        {
            std::lock_guard<std::mutex> lock(mutex);
            std::swap(toProcess, queue);
        }
        while (!toProcess.empty()) {
            toProcess.front()();
            toProcess.pop();
        }
    }

    state.SetItemsProcessed(state.iterations() * batchSize);
}

BENCHMARK(BM_MainThreadQueue_Process)->Arg(1)->Arg(10)->Arg(100);

// ============================================================================
// Quick Benchmarks (for CI/CD)
// ============================================================================

static void BM_Quick_SnapshotCopy(benchmark::State& state) {
    std::unordered_map<EdgeId, EdgeLayout> layouts;
    for (int i = 0; i < 50; ++i) {
        EdgeLayout layout;
        layout.from = static_cast<NodeId>(i);
        layout.to = static_cast<NodeId>(i + 1);
        layouts[static_cast<EdgeId>(i)] = layout;
    }

    for (auto _ : state) {
        auto copy = layouts;
        benchmark::DoNotOptimize(copy);
    }
}

BENCHMARK(BM_Quick_SnapshotCopy)->Name("Quick/SnapshotCopy");

static void BM_Quick_ThreadSubmit(benchmark::State& state) {
    JThreadExecutor executor;
    std::atomic<int> counter{0};

    for (auto _ : state) {
        executor.submit([&counter]() {
            counter.fetch_add(1, std::memory_order_relaxed);
        });
    }

    executor.shutdown();
}

BENCHMARK(BM_Quick_ThreadSubmit)->Name("Quick/ThreadSubmit")->Iterations(100);

// ============================================================================
// ObstacleMap Benchmarks
// ============================================================================

// Benchmark: Build ObstacleMap from nodes (current: rebuild every time)
static void BM_ObstacleMap_Build(benchmark::State& state) {
    const int numNodes = state.range(0);
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> posDist(0.0f, 1000.0f);
    std::uniform_real_distribution<float> sizeDist(50.0f, 150.0f);

    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    for (int i = 0; i < numNodes; ++i) {
        NodeId id = static_cast<NodeId>(i);
        NodeLayout layout;
        layout.position = {posDist(rng), posDist(rng)};
        layout.size = {sizeDist(rng), sizeDist(rng)};
        nodeLayouts[id] = layout;
    }

    for (auto _ : state) {
        ObstacleMap obstacles;
        obstacles.buildFromNodes(nodeLayouts, 10.0f, 1);
        benchmark::DoNotOptimize(obstacles);
    }

    state.SetLabel(std::to_string(numNodes) + " nodes");
}

BENCHMARK(BM_ObstacleMap_Build)
    ->Arg(10)
    ->Arg(50)
    ->Arg(100)
    ->Arg(200);

// Benchmark: Build once and reuse (proposed caching)
static void BM_ObstacleMap_Reuse(benchmark::State& state) {
    const int numNodes = state.range(0);
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> posDist(0.0f, 1000.0f);
    std::uniform_real_distribution<float> sizeDist(50.0f, 150.0f);

    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    for (int i = 0; i < numNodes; ++i) {
        NodeId id = static_cast<NodeId>(i);
        NodeLayout layout;
        layout.position = {posDist(rng), posDist(rng)};
        layout.size = {sizeDist(rng), sizeDist(rng)};
        nodeLayouts[id] = layout;
    }

    // Build once
    ObstacleMap obstacles;
    obstacles.buildFromNodes(nodeLayouts, 10.0f, 1);

    for (auto _ : state) {
        // Simulate reuse - just access the cached map
        benchmark::DoNotOptimize(obstacles.width());
        benchmark::DoNotOptimize(obstacles.height());
    }

    state.SetLabel(std::to_string(numNodes) + " nodes (cached)");
}

BENCHMARK(BM_ObstacleMap_Reuse)
    ->Arg(10)
    ->Arg(50)
    ->Arg(100)
    ->Arg(200);

// Benchmark: Multiple rebuilds (simulating current N edges optimization)
static void BM_ObstacleMap_MultipleBuilds(benchmark::State& state) {
    const int numNodes = state.range(0);
    const int numRebuilds = state.range(1);
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> posDist(0.0f, 1000.0f);
    std::uniform_real_distribution<float> sizeDist(50.0f, 150.0f);

    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    for (int i = 0; i < numNodes; ++i) {
        NodeId id = static_cast<NodeId>(i);
        NodeLayout layout;
        layout.position = {posDist(rng), posDist(rng)};
        layout.size = {sizeDist(rng), sizeDist(rng)};
        nodeLayouts[id] = layout;
    }

    for (auto _ : state) {
        for (int r = 0; r < numRebuilds; ++r) {
            ObstacleMap obstacles;
            obstacles.buildFromNodes(nodeLayouts, 10.0f, 1);
            benchmark::DoNotOptimize(obstacles);
        }
    }

    state.SetLabel(std::to_string(numNodes) + " nodes x " + std::to_string(numRebuilds) + " rebuilds");
}

BENCHMARK(BM_ObstacleMap_MultipleBuilds)
    ->Args({50, 8})    // 50 nodes, 8 edges (typical small graph)
    ->Args({100, 20})  // 100 nodes, 20 edges (medium graph)
    ->Args({100, 50}); // 100 nodes, 50 edges (dense graph)

// ============================================================================
// A* PathFinder Benchmarks
// ============================================================================

// Benchmark: A* pathfinding single path
static void BM_AStar_SinglePath(benchmark::State& state) {
    const int numNodes = state.range(0);
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> posDist(100.0f, 900.0f);
    std::uniform_real_distribution<float> sizeDist(50.0f, 100.0f);

    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    for (int i = 0; i < numNodes; ++i) {
        NodeId id = static_cast<NodeId>(i);
        NodeLayout layout;
        layout.position = {posDist(rng), posDist(rng)};
        layout.size = {sizeDist(rng), sizeDist(rng)};
        nodeLayouts[id] = layout;
    }

    ObstacleMap obstacles;
    obstacles.buildFromNodes(nodeLayouts, 10.0f, 1);

    AStarPathFinder pathFinder;
    GridPoint start{5, 5};
    GridPoint goal{static_cast<int>(obstacles.width()) - 5,
                   static_cast<int>(obstacles.height()) - 5};

    for (auto _ : state) {
        auto result = pathFinder.findPath(start, goal, obstacles,
                                          static_cast<NodeId>(0),
                                          static_cast<NodeId>(1),
                                          NodeEdge::Bottom,
                                          NodeEdge::Top);
        benchmark::DoNotOptimize(result);
    }

    state.SetLabel(std::to_string(numNodes) + " nodes");
}

BENCHMARK(BM_AStar_SinglePath)
    ->Arg(10)
    ->Arg(50)
    ->Arg(100);

// ============================================================================
// Hot Path Allocation Benchmarks
// ============================================================================

// Benchmark: Creating unordered_set in loop (current)
static void BM_HotPath_SetCreation(benchmark::State& state) {
    const int iterations = state.range(0);

    for (auto _ : state) {
        for (int i = 0; i < iterations; ++i) {
            std::unordered_set<NodeId> excludeSet;
            excludeSet.insert(static_cast<NodeId>(0));
            excludeSet.insert(static_cast<NodeId>(1));
            benchmark::DoNotOptimize(excludeSet);
        }
    }

    state.SetItemsProcessed(state.iterations() * iterations);
    state.SetLabel("new set per iteration");
}

BENCHMARK(BM_HotPath_SetCreation)
    ->Arg(100)
    ->Arg(1000)
    ->Arg(10000);

// Benchmark: Reusing pre-allocated set (proposed)
static void BM_HotPath_SetReuse(benchmark::State& state) {
    const int iterations = state.range(0);

    for (auto _ : state) {
        std::unordered_set<NodeId> excludeSet;
        excludeSet.reserve(4);  // Pre-allocate

        for (int i = 0; i < iterations; ++i) {
            excludeSet.clear();  // Reuse allocation
            excludeSet.insert(static_cast<NodeId>(0));
            excludeSet.insert(static_cast<NodeId>(1));
            benchmark::DoNotOptimize(excludeSet);
        }
    }

    state.SetItemsProcessed(state.iterations() * iterations);
    state.SetLabel("reused set");
}

BENCHMARK(BM_HotPath_SetReuse)
    ->Arg(100)
    ->Arg(1000)
    ->Arg(10000);

// ============================================================================
// ObstacleMap Copy Benchmarks (for caching optimization validation)
// ============================================================================

// Benchmark: Copy ObstacleMap (simulating the cached approach)
static void BM_ObstacleMap_Copy(benchmark::State& state) {
    const int numNodes = state.range(0);
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> posDist(0.0f, 1000.0f);
    std::uniform_real_distribution<float> sizeDist(50.0f, 150.0f);

    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    for (int i = 0; i < numNodes; ++i) {
        NodeId id = static_cast<NodeId>(i);
        NodeLayout layout;
        layout.position = {posDist(rng), posDist(rng)};
        layout.size = {sizeDist(rng), sizeDist(rng)};
        nodeLayouts[id] = layout;
    }

    // Build base once
    ObstacleMap base;
    base.buildFromNodes(nodeLayouts, 10.0f, 1);

    for (auto _ : state) {
        ObstacleMap copy = base;  // Copy
        benchmark::DoNotOptimize(copy);
    }

    state.SetLabel(std::to_string(numNodes) + " nodes copy");
}

BENCHMARK(BM_ObstacleMap_Copy)
    ->Arg(10)
    ->Arg(50)
    ->Arg(100)
    ->Arg(200);

// Benchmark: Build once + N copies (new optimized approach)
static void BM_ObstacleMap_BuildOnceCopyN(benchmark::State& state) {
    const int numNodes = state.range(0);
    const int numCopies = state.range(1);
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> posDist(0.0f, 1000.0f);
    std::uniform_real_distribution<float> sizeDist(50.0f, 150.0f);

    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    for (int i = 0; i < numNodes; ++i) {
        NodeId id = static_cast<NodeId>(i);
        NodeLayout layout;
        layout.position = {posDist(rng), posDist(rng)};
        layout.size = {sizeDist(rng), sizeDist(rng)};
        nodeLayouts[id] = layout;
    }

    for (auto _ : state) {
        // Build once
        ObstacleMap base;
        base.buildFromNodes(nodeLayouts, 10.0f, 1);

        // Copy N times (simulating loop over edges)
        for (int i = 0; i < numCopies; ++i) {
            ObstacleMap copy = base;
            benchmark::DoNotOptimize(copy);
        }
    }

    state.SetLabel(std::to_string(numNodes) + " nodes: 1 build + " +
                   std::to_string(numCopies) + " copies");
}

BENCHMARK(BM_ObstacleMap_BuildOnceCopyN)
    ->Args({50, 8})
    ->Args({100, 20})
    ->Args({100, 50});
