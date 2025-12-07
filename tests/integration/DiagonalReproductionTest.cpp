#include <gtest/gtest.h>
#include <arborvia/arborvia.h>
#include <iostream>
#include <unordered_map>
#include <unordered_set>

using namespace arborvia;

/**
 * DiagonalReproductionTest
 *
 * TDD test that reproduces the diagonal edge bug observed in interactive demo.
 *
 * Bug scenario from logs:
 * - Edge 2 (paused -> running) creates DIAGONAL after multiple drags
 * - A* pathfinding fails: "ALL RETRIES FAILED! src=(100,400) tgt=(400,120)"
 * - Result: Edge with bends=0 and dx=300, dy=280 (diagonal)
 *
 * This test should FAIL initially (TDD red phase), then pass after fix.
 */
class DiagonalReproductionTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Setup identical to interactive demo (state machine graph)
        idle_ = graph_.addNode(Size{200, 100}, "Idle");
        running_ = graph_.addNode(Size{200, 100}, "Running");
        paused_ = graph_.addNode(Size{200, 100}, "Paused");
        stopped_ = graph_.addNode(Size{200, 100}, "Stopped");
        error_ = graph_.addNode(Size{200, 100}, "Error");

        e_start_ = graph_.addEdge(idle_, running_, "start");       // Edge 0
        e_pause_ = graph_.addEdge(running_, paused_, "pause");     // Edge 1
        e_resume_ = graph_.addEdge(paused_, running_, "resume");   // Edge 2 - THE FAILING EDGE
        e_stop1_ = graph_.addEdge(running_, stopped_, "stop");     // Edge 3
        e_stop2_ = graph_.addEdge(paused_, stopped_, "stop");      // Edge 4
        e_fail_ = graph_.addEdge(running_, error_, "fail");        // Edge 5
        e_reset_ = graph_.addEdge(error_, idle_, "reset");         // Edge 6
        e_retry_ = graph_.addEdge(error_, error_, "retry");        // Edge 7 (self-loop)

        // Setup layout options (same as interactive demo)
        options_.direction = Direction::TopToBottom;
        options_.nodeSpacingHorizontal = 100.0f;
        options_.nodeSpacingVertical = 100.0f;
        options_.gridConfig.cellSize = 20.0f;
        options_.autoSnapPoints = true;

        manager_ = std::make_shared<ManualLayoutManager>();
    }

    // Helper: Check if edge has diagonal segment (non-orthogonal)
    static bool hasDiagonalSegment(const EdgeLayout& edge) {
        std::vector<Point> path;
        path.push_back(edge.sourcePoint);
        for (const auto& bp : edge.bendPoints) {
            path.push_back(bp.position);
        }
        path.push_back(edge.targetPoint);

        for (size_t i = 0; i + 1 < path.size(); ++i) {
            float dx = std::abs(path[i + 1].x - path[i].x);
            float dy = std::abs(path[i + 1].y - path[i].y);

            // Segment is orthogonal if either dx ≈ 0 (vertical) or dy ≈ 0 (horizontal)
            bool isHorizontal = dy < 1.0f;
            bool isVertical = dx < 1.0f;

            if (!isHorizontal && !isVertical) {
                return true;  // Diagonal found
            }
        }
        return false;
    }

    // Helper: Simulate drag and update edges
    void simulateDrag(NodeId nodeId, float dx, float dy) {
        nodeLayouts_[nodeId].position.x += dx;
        nodeLayouts_[nodeId].position.y += dy;
        manager_->setNodePosition(nodeId, nodeLayouts_[nodeId].position);

        std::vector<EdgeId> affected = graph_.getConnectedEdges(nodeId);
        LayoutUtils::updateEdgePositions(
            edgeLayouts_, nodeLayouts_, affected,
            options_, {nodeId});
    }

    // Helper: Do initial layout
    void doLayout() {
        SugiyamaLayout layout;
        layout.setOptions(options_);
        layout.setManualLayoutManager(manager_);
        auto result = layout.layout(graph_);

        nodeLayouts_.clear();
        edgeLayouts_.clear();
        for (const auto& [id, nl] : result.nodeLayouts()) {
            nodeLayouts_[id] = nl;
        }
        for (const auto& [id, el] : result.edgeLayouts()) {
            edgeLayouts_[id] = el;
        }
    }

    // Helper: Print edge info for debugging
    void printEdgeInfo(EdgeId edgeId, const std::string& label = "") {
        const auto& el = edgeLayouts_[edgeId];
        std::cout << label << "Edge " << edgeId
                  << ": src=(" << el.sourcePoint.x << "," << el.sourcePoint.y << ")"
                  << " tgt=(" << el.targetPoint.x << "," << el.targetPoint.y << ")"
                  << " bends=" << el.bendPoints.size();

        float dx = std::abs(el.targetPoint.x - el.sourcePoint.x);
        float dy = std::abs(el.targetPoint.y - el.sourcePoint.y);
        if (el.bendPoints.empty() && dx > 1 && dy > 1) {
            std::cout << " [DIAGONAL! dx=" << dx << " dy=" << dy << "]";
        }
        std::cout << std::endl;
    }

    Graph graph_;
    NodeId idle_, running_, paused_, stopped_, error_;
    EdgeId e_start_, e_pause_, e_resume_, e_stop1_, e_stop2_, e_fail_, e_reset_, e_retry_;
    LayoutOptions options_;
    std::shared_ptr<ManualLayoutManager> manager_;
    std::unordered_map<NodeId, NodeLayout> nodeLayouts_;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts_;
};

/**
 * Test: Exact reproduction of diagonal bug from interactive demo logs
 *
 * Log sequence that caused Edge 2 DIAGONAL:
 * 1. Initial layout
 * 2. drag 1 100 0 (4-5 times, total ~500px right)
 * 3. drag 1 0 -200 (up)
 *
 * Result: Edge 2 (paused->running) becomes diagonal with:
 *   src=(100,400) tgt=(400,120) bends=0 [dx=300 dy=280]
 */
TEST_F(DiagonalReproductionTest, DragRunningFarRight_Edge2ShouldNotBeDiagonal) {
    std::cout << "\n=== Diagonal Reproduction Test ===" << std::endl;

    doLayout();

    std::cout << "\n--- Initial layout ---" << std::endl;
    for (const auto& [edgeId, el] : edgeLayouts_) {
        printEdgeInfo(edgeId);
    }

    // Verify initial layout has no diagonals
    for (const auto& [edgeId, el] : edgeLayouts_) {
        ASSERT_FALSE(hasDiagonalSegment(el))
            << "Initial layout: Edge " << edgeId << " should not be diagonal";
    }

    std::cout << "\n--- Simulating drag sequence from logs ---" << std::endl;

    // Drag sequence that reproduced the bug:
    // Multiple drags of node 1 (Running) to the right, then up
    std::vector<std::pair<NodeId, Point>> dragSequence = {
        {running_, {100, 0}},   // Step 1: Running right 100
        {running_, {100, 0}},   // Step 2: Running right 100 more (total +200)
        {running_, {100, 0}},   // Step 3: Running right 100 more (total +300)
        {running_, {100, 0}},   // Step 4: Running right 100 more (total +400)
        {running_, {100, 0}},   // Step 5: Running right 100 more (total +500)
        {running_, {0, -200}},  // Step 6: Running up 200
    };

    int step = 0;
    for (const auto& [node, offset] : dragSequence) {
        step++;
        std::cout << "\nStep " << step << ": drag node " << node
                  << " by (" << offset.x << ", " << offset.y << ")" << std::endl;

        simulateDrag(node, offset.x, offset.y);

        // Print Edge 2 status (the problematic edge)
        printEdgeInfo(e_resume_, "  After drag: ");

        // Check for diagonal after each step
        for (const auto& [edgeId, el] : edgeLayouts_) {
            if (hasDiagonalSegment(el)) {
                std::cout << "  WARNING: Edge " << edgeId << " became diagonal!" << std::endl;
                printEdgeInfo(edgeId, "  ");
            }
        }
    }

    std::cout << "\n--- Final state ---" << std::endl;
    for (const auto& [edgeId, el] : edgeLayouts_) {
        printEdgeInfo(edgeId);
    }

    // Final assertion: NO edge should be diagonal
    int diagonalCount = 0;
    for (const auto& [edgeId, el] : edgeLayouts_) {
        if (hasDiagonalSegment(el)) {
            diagonalCount++;
            std::cout << "FAIL: Edge " << edgeId << " is DIAGONAL!" << std::endl;
        }
    }

    EXPECT_EQ(diagonalCount, 0)
        << "No edges should be diagonal after drag sequence. Found " << diagonalCount << " diagonal edges.";

    // Specifically check Edge 2 (the edge that was diagonal in logs)
    EXPECT_FALSE(hasDiagonalSegment(edgeLayouts_[e_resume_]))
        << "Edge 2 (paused->running) should NOT be diagonal! "
        << "src=(" << edgeLayouts_[e_resume_].sourcePoint.x << "," << edgeLayouts_[e_resume_].sourcePoint.y << ") "
        << "tgt=(" << edgeLayouts_[e_resume_].targetPoint.x << "," << edgeLayouts_[e_resume_].targetPoint.y << ") "
        << "bends=" << edgeLayouts_[e_resume_].bendPoints.size();
}

/**
 * Test: Alternative drag sequence - drag node 1 very far right in one step
 */
TEST_F(DiagonalReproductionTest, DragRunningSingleLargeOffset_NoEdgeShouldBeDiagonal) {
    std::cout << "\n=== Single Large Drag Test ===" << std::endl;

    doLayout();

    std::cout << "\n--- Initial layout ---" << std::endl;
    printEdgeInfo(e_resume_, "Edge 2 (resume): ");

    // Single large drag (equivalent to cumulative 500px right, 200px up)
    std::cout << "\n--- Dragging Running node by (500, -200) ---" << std::endl;
    simulateDrag(running_, 500, -200);

    std::cout << "\n--- After drag ---" << std::endl;
    printEdgeInfo(e_resume_, "Edge 2 (resume): ");

    // Check all edges for diagonal
    int diagonalCount = 0;
    for (const auto& [edgeId, el] : edgeLayouts_) {
        if (hasDiagonalSegment(el)) {
            diagonalCount++;
            printEdgeInfo(edgeId, "DIAGONAL: ");
        }
    }

    EXPECT_EQ(diagonalCount, 0)
        << "No edges should be diagonal. Found " << diagonalCount << " diagonal edges.";
}

/**
 * Test: Extended drag sequence from user's test script
 *
 * Sequence from background bash script:
 * 1. drag 1 500 0
 * 2. drag 1 0 -200
 * 3. drag 2 -200 0
 * 4. drag 0 300 0
 */
TEST_F(DiagonalReproductionTest, ExtendedDragSequence_NoEdgeShouldBeDiagonal) {
    std::cout << "\n=== Extended Drag Sequence Test ===" << std::endl;

    doLayout();

    std::cout << "\n--- Initial layout ---" << std::endl;
    for (const auto& [edgeId, el] : edgeLayouts_) {
        printEdgeInfo(edgeId);
    }

    // Sequence from user's test
    std::vector<std::tuple<std::string, NodeId, Point>> sequence = {
        {"drag 1 500 0", running_, {500, 0}},
        {"drag 1 0 -200", running_, {0, -200}},
        {"drag 2 -200 0", paused_, {-200, 0}},
        {"drag 0 300 0", idle_, {300, 0}},
    };

    for (const auto& [desc, node, offset] : sequence) {
        std::cout << "\n--- " << desc << " ---" << std::endl;
        simulateDrag(node, offset.x, offset.y);

        // Check for diagonals after each step
        for (const auto& [edgeId, el] : edgeLayouts_) {
            if (hasDiagonalSegment(el)) {
                std::cout << "  DIAGONAL detected: Edge " << edgeId << std::endl;
                printEdgeInfo(edgeId, "    ");
            }
        }
    }

    std::cout << "\n--- Final state ---" << std::endl;
    for (const auto& [edgeId, el] : edgeLayouts_) {
        printEdgeInfo(edgeId);
    }

    // Final check
    int diagonalCount = 0;
    for (const auto& [edgeId, el] : edgeLayouts_) {
        if (hasDiagonalSegment(el)) {
            diagonalCount++;
        }
    }

    EXPECT_EQ(diagonalCount, 0)
        << "No edges should be diagonal after extended drag sequence. Found " << diagonalCount << " diagonal edges.";
}

/**
 * Test: Stress test with random large drags
 * Should never produce diagonal edges
 */
TEST_F(DiagonalReproductionTest, StressTest_RandomLargeDrags_NoEdgeShouldBeDiagonal) {
    std::cout << "\n=== Stress Test: Random Large Drags ===" << std::endl;

    doLayout();

    std::vector<NodeId> nodes = {idle_, running_, paused_, stopped_, error_};
    std::vector<Point> offsets = {
        {300, 0}, {-300, 0}, {0, 200}, {0, -200},
        {200, -100}, {-200, 100}, {400, 0}, {-400, 0},
        {0, 300}, {0, -300}
    };

    int diagonalCount = 0;
    int testCount = 0;

    for (NodeId node : nodes) {
        for (const auto& offset : offsets) {
            testCount++;

            // Save current state
            auto savedNodeLayouts = nodeLayouts_;
            auto savedEdgeLayouts = edgeLayouts_;

            // Apply drag
            simulateDrag(node, offset.x, offset.y);

            // Check for diagonals
            for (const auto& [edgeId, el] : edgeLayouts_) {
                if (hasDiagonalSegment(el)) {
                    diagonalCount++;
                    std::cout << "DIAGONAL: Node " << node
                              << " drag (" << offset.x << "," << offset.y << "): "
                              << "Edge " << edgeId << std::endl;
                }
            }

            // Restore for next test
            nodeLayouts_ = savedNodeLayouts;
            edgeLayouts_ = savedEdgeLayouts;
        }
    }

    std::cout << "\nCompleted " << testCount << " drag tests." << std::endl;
    std::cout << "Diagonal count: " << diagonalCount << std::endl;

    EXPECT_EQ(diagonalCount, 0)
        << "No edges should be diagonal in stress test. Found " << diagonalCount << " instances.";
}

/**
 * Test: Cumulative drags without restore - simulates real user interaction
 */
TEST_F(DiagonalReproductionTest, CumulativeDrags_RealisticUserInteraction) {
    std::cout << "\n=== Cumulative Drags Test ===" << std::endl;

    doLayout();

    // Simulate a realistic user interaction: multiple small drags in sequence
    // without resetting between drags
    std::vector<std::pair<NodeId, Point>> drags = {
        {running_, {50, 0}},
        {running_, {50, 0}},
        {running_, {50, 0}},
        {running_, {50, 0}},
        {running_, {50, 0}},   // Total: 250 right
        {running_, {0, -50}},
        {running_, {0, -50}},
        {running_, {0, -50}},
        {running_, {0, -50}},  // Total: 200 up
        {paused_, {-100, 0}},
        {idle_, {100, 0}},
        {error_, {0, 50}},
        {stopped_, {-50, 25}},
    };

    int step = 0;
    int diagonalCount = 0;

    for (const auto& [node, offset] : drags) {
        step++;
        simulateDrag(node, offset.x, offset.y);

        for (const auto& [edgeId, el] : edgeLayouts_) {
            if (hasDiagonalSegment(el)) {
                std::cout << "Step " << step << ": DIAGONAL Edge " << edgeId
                          << " after dragging node " << node
                          << " by (" << offset.x << "," << offset.y << ")" << std::endl;
                diagonalCount++;
            }
        }
    }

    std::cout << "\nCompleted " << step << " cumulative drags." << std::endl;

    EXPECT_EQ(diagonalCount, 0)
        << "No diagonal edges should appear during realistic user interaction.";
}
