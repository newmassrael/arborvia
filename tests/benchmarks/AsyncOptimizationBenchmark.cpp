#include <arborvia/arborvia.h>
#include <arborvia/core/TaskExecutor.h>
#include <arborvia/layout/interactive/PathRoutingCoordinator.h>

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
