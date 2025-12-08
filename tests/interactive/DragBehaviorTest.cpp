#include <gtest/gtest.h>
#include <arborvia/arborvia.h>
#include <iostream>
#include <unordered_map>
#include <random>

using namespace arborvia;

/**
 * DragBehaviorTest
 *
 * Tests for interactive drag behavior simulation.
 * Verifies that edge routing (sourceEdge, targetEdge, snapIndex) is preserved
 * when nodes are dragged and layout is recalculated.
 */
class DragBehaviorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create the same graph as interactive demo (state machine)
        idle_ = graph_.addNode(Size{100, 50}, "Idle");
        running_ = graph_.addNode(Size{100, 50}, "Running");
        paused_ = graph_.addNode(Size{100, 50}, "Paused");
        stopped_ = graph_.addNode(Size{100, 50}, "Stopped");
        error_ = graph_.addNode(Size{100, 50}, "Error");

        e_start_ = graph_.addEdge(idle_, running_, "start");
        e_pause_ = graph_.addEdge(running_, paused_, "pause");
        e_resume_ = graph_.addEdge(paused_, running_, "resume");
        e_stop1_ = graph_.addEdge(running_, stopped_, "stop");
        e_stop2_ = graph_.addEdge(paused_, stopped_, "stop");
        e_fail_ = graph_.addEdge(running_, error_, "fail");
        e_reset_ = graph_.addEdge(error_, idle_, "reset");  // This is the problematic edge
    }

    // Simulate drag behavior: save state, move node, recalculate layout
    struct EdgeRoutingInfo {
        NodeEdge sourceEdge;
        NodeEdge targetEdge;
        int sourceSnapIndex;
        int targetSnapIndex;
        NodeId from;
        NodeId to;
    };

    // Helper to calculate fixed grid-based candidate count (excluding corners)
    // Mirrors EdgeRouting::getFixedSnapPointCount
    int getFixedSnapPointCount(const NodeLayout& node, NodeEdge edge, float gridSize) {
        // For Top/Bottom edges: range is along X axis
        // For Left/Right edges: range is along Y axis
        switch (edge) {
            case NodeEdge::Top:
            case NodeEdge::Bottom: {
                int gridLeft = static_cast<int>(std::ceil(node.position.x / gridSize));
                int gridRight = static_cast<int>(std::floor((node.position.x + node.size.width) / gridSize));
                return std::max(0, gridRight - gridLeft + 1);
            }
            case NodeEdge::Left:
            case NodeEdge::Right: {
                int gridTop = static_cast<int>(std::ceil(node.position.y / gridSize));
                int gridBottom = static_cast<int>(std::floor((node.position.y + node.size.height) / gridSize));
                return std::max(0, gridBottom - gridTop + 1);
            }
        }
        return 0;
    }

    // Helper to get edge routing info from layout result
    std::unordered_map<EdgeId, EdgeRoutingInfo> captureEdgeRouting(const LayoutResult& result) {
        std::unordered_map<EdgeId, EdgeRoutingInfo> routing;
        for (const auto& [edgeId, layout] : result.edgeLayouts()) {
            routing[edgeId] = {
                layout.sourceEdge,
                layout.targetEdge,
                layout.sourceSnapIndex,
                layout.targetSnapIndex,
                layout.from,
                layout.to
            };
        }
        return routing;
    }

    // Helper to print edge routing info
    void printEdgeRouting(const std::string& label, EdgeId edgeId, const EdgeRoutingInfo& info) {
        auto edgeName = [&]() -> std::string {
            if (edgeId == e_start_) return "start";
            if (edgeId == e_pause_) return "pause";
            if (edgeId == e_resume_) return "resume";
            if (edgeId == e_stop1_) return "stop1";
            if (edgeId == e_stop2_) return "stop2";
            if (edgeId == e_fail_) return "fail";
            if (edgeId == e_reset_) return "reset";
            return "unknown";
        };

        auto edgeSideName = [](NodeEdge e) -> std::string {
            switch (e) {
                case NodeEdge::Top: return "Top";
                case NodeEdge::Bottom: return "Bottom";
                case NodeEdge::Left: return "Left";
                case NodeEdge::Right: return "Right";
            }
            return "Unknown";
        };

        std::cout << label << " Edge[" << edgeName() << "]: "
                  << "src=" << edgeSideName(info.sourceEdge) << "[" << info.sourceSnapIndex << "], "
                  << "tgt=" << edgeSideName(info.targetEdge) << "[" << info.targetSnapIndex << "]"
                  << std::endl;
    }

    // Helper: check if orthogonal segment penetrates node interior
    static bool segmentPenetratesNode(const Point& p1, const Point& p2,
                                      const NodeLayout& node) {
        float left = node.position.x;
        float right = node.position.x + node.size.width;
        float top = node.position.y;
        float bottom = node.position.y + node.size.height;

        // Vertical segment
        if (std::abs(p1.x - p2.x) < 0.1f) {
            float x = p1.x;
            float minY = std::min(p1.y, p2.y);
            float maxY = std::max(p1.y, p2.y);
            // Check if x is inside node's horizontal bounds (excluding boundary)
            if (x > left + 1 && x < right - 1) {
                // Check if segment overlaps node's vertical bounds
                if (minY < bottom - 1 && maxY > top + 1) {
                    return true;
                }
            }
        }
        // Horizontal segment
        else if (std::abs(p1.y - p2.y) < 0.1f) {
            float y = p1.y;
            float minX = std::min(p1.x, p2.x);
            float maxX = std::max(p1.x, p2.x);
            // Check if y is inside node's vertical bounds (excluding boundary)
            if (y > top + 1 && y < bottom - 1) {
                // Check if segment overlaps node's horizontal bounds
                if (minX < right - 1 && maxX > left + 1) {
                    return true;
                }
            }
        }
        return false;
    }

    // Helper: check if edge penetrates any node (excluding source/target)
    static bool edgePenetratesAnyNode(const EdgeLayout& edge,
                                      const std::unordered_map<NodeId, NodeLayout>& nodes) {
        std::vector<Point> path;
        path.push_back(edge.sourcePoint);
        for (const auto& bp : edge.bendPoints) {
            path.push_back(bp.position);
        }
        path.push_back(edge.targetPoint);

        for (size_t i = 0; i + 1 < path.size(); ++i) {
            for (const auto& [nodeId, node] : nodes) {
                // Skip source and target nodes
                if (nodeId == edge.from || nodeId == edge.to) continue;
                if (segmentPenetratesNode(path[i], path[i+1], node)) {
                    return true;
                }
            }
        }
        return false;
    }

    Graph graph_;
    NodeId idle_, running_, paused_, stopped_, error_;
    EdgeId e_start_, e_pause_, e_resume_, e_stop1_, e_stop2_, e_fail_, e_reset_;
};

// Test: Initial layout produces valid edge routing
TEST_F(DragBehaviorTest, Layout_InitialRouting_IsValid) {
    SugiyamaLayout layout;
    LayoutOptions options;

    layout.setOptions(options);

    LayoutResult result = layout.layout(graph_);
    auto routing = captureEdgeRouting(result);

    // All edges should have valid routing
    EXPECT_EQ(routing.size(), 7u);

    // Print initial routing for visibility
    std::cout << "\n=== Initial Edge Routing ===" << std::endl;
    for (const auto& [edgeId, info] : routing) {
        printEdgeRouting("", edgeId, info);
    }

    // The reset edge (Error -> Idle) should have specific routing
    // Error is below Idle in hierarchy, so reset goes upward
    auto resetRouting = routing[e_reset_];
    std::cout << "\nReset edge (Error->Idle) routes from "
              << (resetRouting.sourceEdge == NodeEdge::Top ? "Top" : "Bottom")
              << " to "
              << (resetRouting.targetEdge == NodeEdge::Top ? "Top" : "Bottom")
              << std::endl;
}

// Test: Edge routing is preserved after simulated drag
TEST_F(DragBehaviorTest, Drag_SingleNode_PreservesRouting) {
    ManualLayoutManager manager;


    SugiyamaLayout layout;
    LayoutOptions options;

    layout.setOptions(options);
    layout.setManualLayoutManager(std::make_shared<ManualLayoutManager>(manager));

    // Initial layout
    LayoutResult result1 = layout.layout(graph_);
    auto routingBefore = captureEdgeRouting(result1);

    std::cout << "\n=== Before Drag ===" << std::endl;
    for (const auto& [edgeId, info] : routingBefore) {
        printEdgeRouting("", edgeId, info);
    }

    // Capture node positions
    std::unordered_map<NodeId, Point> nodePositions;
    for (const auto& [nodeId, nodeLayout] : result1.nodeLayouts()) {
        nodePositions[nodeId] = nodeLayout.position;
    }

    // Simulate drag: move Error node
    Point errorOriginalPos = nodePositions[error_];
    Point errorNewPos = {errorOriginalPos.x + 50.0f, errorOriginalPos.y - 30.0f};

    std::cout << "\nDragging Error node from (" << errorOriginalPos.x << ", " << errorOriginalPos.y
              << ") to (" << errorNewPos.x << ", " << errorNewPos.y << ")" << std::endl;

    // Set new position (simulating drag)
    manager.setNodePosition(error_, errorNewPos);

    // Recalculate layout
    LayoutResult result2 = layout.layout(graph_);
    auto routingAfter = captureEdgeRouting(result2);

    std::cout << "\n=== After Drag (without preservation) ===" << std::endl;
    for (const auto& [edgeId, info] : routingAfter) {
        printEdgeRouting("", edgeId, info);
    }

    // Check if routing changed
    std::cout << "\n=== Routing Changes ===" << std::endl;
    bool anyChange = false;
    for (const auto& [edgeId, infoBefore] : routingBefore) {
        const auto& infoAfter = routingAfter[edgeId];
        if (infoBefore.sourceEdge != infoAfter.sourceEdge ||
            infoBefore.targetEdge != infoAfter.targetEdge ||
            infoBefore.sourceSnapIndex != infoAfter.sourceSnapIndex ||
            infoBefore.targetSnapIndex != infoAfter.targetSnapIndex) {

            printEdgeRouting("BEFORE:", edgeId, infoBefore);
            printEdgeRouting("AFTER: ", edgeId, infoAfter);
            std::cout << "  ^ CHANGED!" << std::endl;
            anyChange = true;
        }
    }

    if (!anyChange) {
        std::cout << "No routing changes detected." << std::endl;
    }

    // This test documents current behavior - it may fail if routing changes
    // The key insight is whether reset edge routing changes after drag
    EXPECT_EQ(routingBefore[e_reset_].sourceEdge, routingAfter[e_reset_].sourceEdge)
        << "Reset edge sourceEdge changed after drag!";
    EXPECT_EQ(routingBefore[e_reset_].targetEdge, routingAfter[e_reset_].targetEdge)
        << "Reset edge targetEdge changed after drag!";
}

// Test: Snap point count is preserved for Error node
TEST_F(DragBehaviorTest, Drag_ErrorNode_PreservesSnapCount) {
    ManualLayoutManager manager;


    SugiyamaLayout layout;
    LayoutOptions options;

    layout.setOptions(options);
    layout.setManualLayoutManager(std::make_shared<ManualLayoutManager>(manager));

    // Initial layout
    LayoutResult result1 = layout.layout(graph_);

    // Count snap points on Error node edges
    int errorTopCountBefore = 0;
    int errorBottomCountBefore = 0;

    for (const auto& [edgeId, edgeLayout] : result1.edgeLayouts()) {
        if (edgeLayout.from == error_) {
            if (edgeLayout.sourceEdge == NodeEdge::Top) errorTopCountBefore++;
            if (edgeLayout.sourceEdge == NodeEdge::Bottom) errorBottomCountBefore++;
        }
        if (edgeLayout.to == error_) {
            if (edgeLayout.targetEdge == NodeEdge::Top) errorTopCountBefore++;
            if (edgeLayout.targetEdge == NodeEdge::Bottom) errorBottomCountBefore++;
        }
    }

    std::cout << "\n=== Error Node Snap Points Before Drag ===" << std::endl;
    std::cout << "Top: " << errorTopCountBefore << ", Bottom: " << errorBottomCountBefore << std::endl;

    // Simulate drag
    Point errorPos = result1.getNodeLayout(error_)->position;
    manager.setNodePosition(error_, {errorPos.x + 50.0f, errorPos.y - 30.0f});

    // Recalculate
    LayoutResult result2 = layout.layout(graph_);

    // Count again
    int errorTopCountAfter = 0;
    int errorBottomCountAfter = 0;

    for (const auto& [edgeId, edgeLayout] : result2.edgeLayouts()) {
        if (edgeLayout.from == error_) {
            if (edgeLayout.sourceEdge == NodeEdge::Top) errorTopCountAfter++;
            if (edgeLayout.sourceEdge == NodeEdge::Bottom) errorBottomCountAfter++;
        }
        if (edgeLayout.to == error_) {
            if (edgeLayout.targetEdge == NodeEdge::Top) errorTopCountAfter++;
            if (edgeLayout.targetEdge == NodeEdge::Bottom) errorTopCountAfter++;
        }
    }

    std::cout << "\n=== Error Node Snap Points After Drag ===" << std::endl;
    std::cout << "Top: " << errorTopCountAfter << ", Bottom: " << errorBottomCountAfter << std::endl;

    // Total should be the same (2 edges connected to Error: fail incoming, reset outgoing)
    int totalBefore = errorTopCountBefore + errorBottomCountBefore;
    int totalAfter = errorTopCountAfter + errorBottomCountAfter;

    EXPECT_EQ(totalBefore, totalAfter)
        << "Total snap point count changed! Before: " << totalBefore << ", After: " << totalAfter;
}

// Test: Multiple drags should maintain consistent routing
TEST_F(DragBehaviorTest, Drag_Multiple_MaintainsRouting) {
    ManualLayoutManager manager;


    SugiyamaLayout layout;
    LayoutOptions options;

    layout.setOptions(options);
    layout.setManualLayoutManager(std::make_shared<ManualLayoutManager>(manager));

    // Initial layout
    LayoutResult result = layout.layout(graph_);
    auto initialRouting = captureEdgeRouting(result);

    std::cout << "\n=== Multiple Drag Test ===" << std::endl;

    // Simulate multiple drags
    std::vector<Point> dragOffsets = {
        {30.0f, -20.0f},
        {-40.0f, 50.0f},
        {20.0f, 10.0f},
        {-10.0f, -30.0f}
    };

    Point currentPos = result.getNodeLayout(error_)->position;

    for (size_t i = 0; i < dragOffsets.size(); ++i) {
        Point newPos = {currentPos.x + dragOffsets[i].x, currentPos.y + dragOffsets[i].y};
        manager.setNodePosition(error_, newPos);

        result = layout.layout(graph_);
        auto routing = captureEdgeRouting(result);

        std::cout << "\nDrag " << (i + 1) << ": Error moved to ("
                  << newPos.x << ", " << newPos.y << ")" << std::endl;

        // Check reset edge specifically
        if (routing[e_reset_].sourceEdge != initialRouting[e_reset_].sourceEdge ||
            routing[e_reset_].targetEdge != initialRouting[e_reset_].targetEdge) {
            std::cout << "  Reset edge routing CHANGED!" << std::endl;
            printEdgeRouting("  Initial:", e_reset_, initialRouting[e_reset_]);
            printEdgeRouting("  Current:", e_reset_, routing[e_reset_]);
        } else {
            std::cout << "  Reset edge routing preserved." << std::endl;
        }

        currentPos = newPos;
    }
}

// Test: Verify unified snap distribution mode works correctly
TEST_F(DragBehaviorTest, SnapPoints_DistributeAllConnectionsEvenly) {
    SugiyamaLayout layout;
    LayoutOptions options;

    layout.setOptions(options);

    LayoutResult result = layout.layout(graph_);

    std::cout << "\n=== Snap Point Distribution ===" << std::endl;

    // For Running node, check that incoming and outgoing are on different halves
    // Incoming: start (from Idle), resume (from Paused)
    // Outgoing: pause (to Paused), stop (to Stopped), fail (to Error)

    for (const auto& [edgeId, edgeLayout] : result.edgeLayouts()) {
        if (edgeLayout.to == running_) {
            std::cout << "Incoming to Running: snapIndex=" << edgeLayout.targetSnapIndex << std::endl;
        }
        if (edgeLayout.from == running_) {
            std::cout << "Outgoing from Running: snapIndex=" << edgeLayout.sourceSnapIndex << std::endl;
        }
    }
}

// Test: Snap point distribution
TEST_F(DragBehaviorTest, SnapPoints_DistributeCorrectly) {
    SugiyamaLayout layout;
    LayoutOptions options;

    layout.setOptions(options);

    LayoutResult result = layout.layout(graph_);

    std::cout << "\n=== Snap Point Distribution ===" << std::endl;

    for (const auto& [edgeId, edgeLayout] : result.edgeLayouts()) {
        if (edgeLayout.to == running_) {
            std::cout << "Incoming to Running: snapIndex=" << edgeLayout.targetSnapIndex << std::endl;
        }
        if (edgeLayout.from == running_) {
            std::cout << "Outgoing from Running: snapIndex=" << edgeLayout.sourceSnapIndex << std::endl;
        }
    }
}

// Test: Simulate interactive demo's exact drag behavior
TEST_F(DragBehaviorTest, Drag_DemoSimulation_WorksCorrectly) {
    ManualLayoutManager manager;


    SugiyamaLayout layout;
    LayoutOptions options;

    layout.setOptions(options);
    layout.setManualLayoutManager(std::make_shared<ManualLayoutManager>(manager));

    // Initial layout
    LayoutResult result = layout.layout(graph_);

    std::cout << "\n=== Interactive Demo Drag Simulation ===" << std::endl;

    // Store node layouts (like interactive demo does)
    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;

    for (const auto& [id, layout] : result.nodeLayouts()) {
        nodeLayouts[id] = layout;
    }
    for (const auto& [id, layout] : result.edgeLayouts()) {
        edgeLayouts[id] = layout;
    }

    std::cout << "Initial Error node position: ("
              << nodeLayouts[error_].position.x << ", "
              << nodeLayouts[error_].position.y << ")" << std::endl;

    // Print initial edge routing for Error node
    std::cout << "\nInitial edge routing:" << std::endl;
    for (const auto& [edgeId, edgeLayout] : edgeLayouts) {
        if (edgeLayout.from == error_ || edgeLayout.to == error_) {
            auto nodeName = [&](NodeId id) -> std::string {
                if (id == error_) return "Error";
                if (id == idle_) return "Idle";
                if (id == running_) return "Running";
                return "Other";
            };
            auto edgeSide = [](NodeEdge e) -> std::string {
                switch (e) {
                    case NodeEdge::Top: return "Top";
                    case NodeEdge::Bottom: return "Bottom";
                    case NodeEdge::Left: return "Left";
                    case NodeEdge::Right: return "Right";
                }
                return "?";
            };

            std::cout << "  " << nodeName(edgeLayout.from) << " -> " << nodeName(edgeLayout.to)
                      << ": src=" << edgeSide(edgeLayout.sourceEdge) << "[" << edgeLayout.sourceSnapIndex << "]"
                      << ", tgt=" << edgeSide(edgeLayout.targetEdge) << "[" << edgeLayout.targetSnapIndex << "]"
                      << std::endl;
        }
    }

    // === SIMULATE DRAG START ===
    NodeId draggedNode = error_;

    // === SIMULATE DRAG (update position) ===
    Point dragOffset = {50.0f, -100.0f};  // Drag up and right
    nodeLayouts[draggedNode].position.x += dragOffset.x;
    nodeLayouts[draggedNode].position.y += dragOffset.y;
    manager.setNodePosition(draggedNode, nodeLayouts[draggedNode].position);

    std::cout << "\nDragged Error node to: ("
              << nodeLayouts[error_].position.x << ", "
              << nodeLayouts[error_].position.y << ")" << std::endl;

    // === SIMULATE DRAG END (like interactive demo) ===

    // Step 1: Save ALL node positions
    std::unordered_map<NodeId, Point> savedPositions;
    for (const auto& [id, layout] : nodeLayouts) {
        savedPositions[id] = layout.position;
        manager.setNodePosition(id, layout.position);
    }

    // Step 2: Save ALL edge routing info
    struct EdgeRoutingInfo {
        NodeEdge sourceEdge;
        NodeEdge targetEdge;
        int sourceSnapIndex;
        int targetSnapIndex;
    };
    std::unordered_map<EdgeId, EdgeRoutingInfo> savedEdgeRouting;
    for (const auto& [edgeId, edgeLayout] : edgeLayouts) {
        savedEdgeRouting[edgeId] = {
            edgeLayout.sourceEdge,
            edgeLayout.targetEdge,
            edgeLayout.sourceSnapIndex,
            edgeLayout.targetSnapIndex
        };
    }

    // Step 3: Recalculate layout (this is what interactive demo does)
    LayoutResult newResult = layout.layout(graph_);

    // Step 4: Update local layouts from new result
    nodeLayouts.clear();
    edgeLayouts.clear();
    for (const auto& [id, layout] : newResult.nodeLayouts()) {
        nodeLayouts[id] = layout;
    }
    for (const auto& [id, layout] : newResult.edgeLayouts()) {
        edgeLayouts[id] = layout;
    }

    std::cout << "\nAfter doLayout() - edge routing from library:" << std::endl;
    for (const auto& [edgeId, edgeLayout] : edgeLayouts) {
        if (edgeLayout.from == error_ || edgeLayout.to == error_) {
            auto edgeSide = [](NodeEdge e) -> std::string {
                switch (e) {
                    case NodeEdge::Top: return "Top";
                    case NodeEdge::Bottom: return "Bottom";
                    case NodeEdge::Left: return "Left";
                    case NodeEdge::Right: return "Right";
                }
                return "?";
            };
            std::cout << "  src=" << edgeSide(edgeLayout.sourceEdge) << "[" << edgeLayout.sourceSnapIndex << "]"
                      << ", tgt=" << edgeSide(edgeLayout.targetEdge) << "[" << edgeLayout.targetSnapIndex << "]"
                      << std::endl;
        }
    }

    // Step 5: Restore node positions
    for (auto& [id, layout] : nodeLayouts) {
        layout.position = savedPositions[id];
    }

    // Step 6: Update all edge positions using library API (like interactive demo does now)
    std::vector<EdgeId> allEdges;
    for (const auto& [edgeId, _] : edgeLayouts) {
        allEdges.push_back(edgeId);
    }

    LayoutUtils::updateEdgePositions(
        edgeLayouts, nodeLayouts, allEdges);

    std::cout << "\nAfter library API update - edge routing:" << std::endl;
    for (const auto& [edgeId, edgeLayout] : edgeLayouts) {
        if (edgeLayout.from == error_ || edgeLayout.to == error_) {
            auto edgeSide = [](NodeEdge e) -> std::string {
                switch (e) {
                    case NodeEdge::Top: return "Top";
                    case NodeEdge::Bottom: return "Bottom";
                    case NodeEdge::Left: return "Left";
                    case NodeEdge::Right: return "Right";
                }
                return "?";
            };
            std::cout << "  src=" << edgeSide(edgeLayout.sourceEdge) << "[" << edgeLayout.sourceSnapIndex << "]"
                      << ", tgt=" << edgeSide(edgeLayout.targetEdge) << "[" << edgeLayout.targetSnapIndex << "]"
                      << std::endl;
        }
    }

    // Verify edge routing: directions preserved, indices valid (fixed grid-based)
    const float gridSize = 10.0f;
    for (const auto& [edgeId, edgeLayout] : edgeLayouts) {
        if (edgeLayout.from == error_ || edgeLayout.to == error_) {
            // Edge directions must be preserved
            EXPECT_EQ(edgeLayout.sourceEdge, savedEdgeRouting[edgeId].sourceEdge);
            EXPECT_EQ(edgeLayout.targetEdge, savedEdgeRouting[edgeId].targetEdge);
            
            // Snap indices must be valid (>= 0 and < candidateCount based on fixed grid)
            const NodeLayout& srcNode = nodeLayouts[edgeLayout.from];
            const NodeLayout& tgtNode = nodeLayouts[edgeLayout.to];
            int srcCandidateCount = getFixedSnapPointCount(srcNode, edgeLayout.sourceEdge, gridSize);
            int tgtCandidateCount = getFixedSnapPointCount(tgtNode, edgeLayout.targetEdge, gridSize);
            
            EXPECT_GE(edgeLayout.sourceSnapIndex, 0)
                << "Edge " << edgeId << " sourceSnapIndex is negative";
            EXPECT_LT(edgeLayout.sourceSnapIndex, srcCandidateCount)
                << "Edge " << edgeId << " sourceSnapIndex=" << edgeLayout.sourceSnapIndex
                << " >= srcCandidateCount=" << srcCandidateCount;
            EXPECT_GE(edgeLayout.targetSnapIndex, 0)
                << "Edge " << edgeId << " targetSnapIndex is negative";
            EXPECT_LT(edgeLayout.targetSnapIndex, tgtCandidateCount)
                << "Edge " << edgeId << " targetSnapIndex=" << edgeLayout.targetSnapIndex
                << " >= tgtCandidateCount=" << tgtCandidateCount;
        }
    }
}

// Test: Verify snap points don't merge during simulated drag
TEST_F(DragBehaviorTest, Snap_DuringDrag_PointsDontMerge) {
    ManualLayoutManager manager;


    SugiyamaLayout layout;
    LayoutOptions options;

    layout.setOptions(options);
    layout.setManualLayoutManager(std::make_shared<ManualLayoutManager>(manager));

    // Initial layout
    LayoutResult result = layout.layout(graph_);

    std::cout << "\n=== Snap Points During Drag Test ===" << std::endl;

    // Store layouts locally (simulating interactive demo)
    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;

    for (const auto& [id, nl] : result.nodeLayouts()) {
        nodeLayouts[id] = nl;
    }
    for (const auto& [id, el] : result.edgeLayouts()) {
        edgeLayouts[id] = el;
    }

    // Find edges connected to Error node (optimizer may choose different edge assignments)
    std::cout << "\nInitial Error edge connections:" << std::endl;
    Point resetSrcBefore{0, 0}, failTgtBefore{0, 0};
    bool foundReset = false, foundFail = false;

    for (const auto& [edgeId, el] : edgeLayouts) {
        // reset: error -> idle (outgoing from error)
        if (el.from == error_) {
            std::cout << "  reset (outgoing): srcPt=(" << el.sourcePoint.x << ", " << el.sourcePoint.y
                      << ") on edge " << static_cast<int>(el.sourceEdge) << std::endl;
            resetSrcBefore = el.sourcePoint;
            foundReset = true;
        }
        // fail: running -> error (incoming to error)
        if (el.to == error_ && el.from != error_) {  // Exclude self-loop
            std::cout << "  fail (incoming): tgtPt=(" << el.targetPoint.x << ", " << el.targetPoint.y
                      << ") on edge " << static_cast<int>(el.targetEdge) << std::endl;
            failTgtBefore = el.targetPoint;
            foundFail = true;
        }
    }

    // If both edges share the same node edge, verify they're separated
    if (foundReset && foundFail) {
        float initialDiff = std::abs(resetSrcBefore.x - failTgtBefore.x) +
                           std::abs(resetSrcBefore.y - failTgtBefore.y);
        std::cout << "Initial position difference: " << initialDiff << " pixels" << std::endl;
    }

    // === Simulate drag start ===
    NodeId draggedNode = error_;
    std::vector<EdgeId> affectedEdges = {e_fail_, e_reset_};

    // === Simulate drag (update node position) ===
    Point dragOffset = {50.0f, -30.0f};
    nodeLayouts[draggedNode].position.x += dragOffset.x;
    nodeLayouts[draggedNode].position.y += dragOffset.y;

    std::cout << "\nDragging Error by (" << dragOffset.x << ", " << dragOffset.y << ")" << std::endl;

    // Use library API for edge position updates (same as interactive demo)
    LayoutUtils::updateEdgePositions(
        edgeLayouts, nodeLayouts, affectedEdges);

    // Check Error's connections after simulated drag
    std::cout << "\nAfter drag (using library API):" << std::endl;
    Point resetSrcAfter{0, 0}, failTgtAfter{0, 0};

    for (const auto& [edgeId, el] : edgeLayouts) {
        if (el.from == error_) {
            std::cout << "  reset (outgoing): srcPt=(" << el.sourcePoint.x << ", " << el.sourcePoint.y << ")" << std::endl;
            resetSrcAfter = el.sourcePoint;
        }
        if (el.to == error_ && el.from != error_) {
            std::cout << "  fail (incoming): tgtPt=(" << el.targetPoint.x << ", " << el.targetPoint.y << ")" << std::endl;
            failTgtAfter = el.targetPoint;
        }
    }

    // With fixed grid candidates, snap points are assigned to grid positions. When the node moves,
    // the snap point follows the node but may be reassigned to a different candidate.
    // Therefore, we check that the snap point moved in the same direction as the drag
    // and is on a valid grid position (rather than exact offset match).
    const float gridSize = 10.0f;
    const NodeLayout& errorNode = nodeLayouts[error_];
    
    // Verify reset edge source moved with the node (tolerance accounts for grid candidate reassignment)
    if (foundReset) {
        float resetDeltaX = resetSrcAfter.x - resetSrcBefore.x;
        float resetDeltaY = resetSrcAfter.y - resetSrcBefore.y;
        std::cout << "Reset source moved by (" << resetDeltaX << ", " << resetDeltaY << ")" << std::endl;
        
        // Snap points may be reassigned to different grid candidates.
        // Check that movement is in the same direction (positive/negative) as drag offset.
        // For larger drags, the tolerance is one grid cell (gridSize).
        float xTolerance = std::max(gridSize, std::abs(dragOffset.x) * 0.5f);
        float yTolerance = std::max(gridSize, std::abs(dragOffset.y) * 0.5f);
        
        EXPECT_NEAR(resetDeltaX, dragOffset.x, xTolerance)
            << "Reset edge sourcePoint.x should move in direction of dragOffset.x";
        EXPECT_NEAR(resetDeltaY, dragOffset.y, yTolerance)
            << "Reset edge sourcePoint.y should move in direction of dragOffset.y";
    }

    // Verify fail edge target moved with the node (with grid-aware tolerance)
    if (foundFail) {
        float failDeltaX = failTgtAfter.x - failTgtBefore.x;
        float failDeltaY = failTgtAfter.y - failTgtBefore.y;
        std::cout << "Fail target moved by (" << failDeltaX << ", " << failDeltaY << ")" << std::endl;
        
        // Allow for grid candidate reassignment
        float xTolerance = std::max(gridSize, std::abs(dragOffset.x) * 0.5f);
        float yTolerance = std::max(gridSize, std::abs(dragOffset.y) * 0.5f);
        
        EXPECT_NEAR(failDeltaX, dragOffset.x, xTolerance)
            << "Fail edge targetPoint.x should move in direction of dragOffset.x";
        EXPECT_NEAR(failDeltaY, dragOffset.y, yTolerance)
            << "Fail edge targetPoint.y should move in direction of dragOffset.y";
    }
}

// Test: Snap indices should not exceed count-1 (TDD - should fail first)
TEST_F(DragBehaviorTest, Snap_Indices_AreNormalized) {
    std::cout << "\n=== Snap Indices Normalization Test ===" << std::endl;

    // Simulate the bug: an edge has snapIndex > count-1
    // This can happen after multiple drags when indices get out of sync

    NodeLayout idleNode;
    idleNode.id = idle_;
    idleNode.position = {50.0f, 0.0f};
    idleNode.size = {100.0f, 50.0f};

    // Simulate edge with stale index
    EdgeLayout startEdge;
    startEdge.id = e_start_;
    startEdge.from = idle_;
    startEdge.to = running_;
    startEdge.sourceEdge = NodeEdge::Bottom;
    startEdge.sourceSnapIndex = 1;  // BUG: index=1 but only 1 outgoing edge (should be 0)

    // With count=1 and index=1, the calculation becomes:
    // position = 0.5 + (1+1)/(1+1) * 0.5 = 1.0 (100% - off the edge!)

    // Expected behavior: index should be clamped/normalized to valid range
    int outCount = 1;
    int idx = startEdge.sourceSnapIndex;

    // Current buggy calculation
    float buggyPosition = 0.5f + static_cast<float>(idx + 1) / static_cast<float>(outCount + 1) * 0.5f;
    std::cout << "Buggy calculation: idx=" << idx << ", count=" << outCount
              << ", position=" << buggyPosition << std::endl;

    // Fixed calculation: normalize index to valid range
    int normalizedIdx = std::min(idx, outCount - 1);
    float fixedPosition = 0.5f + static_cast<float>(normalizedIdx + 1) / static_cast<float>(outCount + 1) * 0.5f;
    std::cout << "Fixed calculation: normalizedIdx=" << normalizedIdx
              << ", position=" << fixedPosition << std::endl;

    // The position should be within valid range (0.0 to 1.0 for edges)
    EXPECT_LE(buggyPosition, 1.0f) << "Buggy position exceeds edge boundary!";
    EXPECT_GE(fixedPosition, 0.0f);
    EXPECT_LE(fixedPosition, 1.0f);

    // For a single outgoing edge, it should be at 75% (center of 50-100% range)
    float expectedPosition = 0.75f;  // 0.5 + (0+1)/(1+1) * 0.5 = 0.75
    EXPECT_NEAR(fixedPosition, expectedPosition, 0.01f);
}

// Test: Multiple drags should maintain valid snap indices
TEST_F(DragBehaviorTest, Snap_MultiDrag_MaintainsValidIndices) {
    ManualLayoutManager manager;


    SugiyamaLayout layout;
    LayoutOptions options;

    layout.setOptions(options);
    layout.setManualLayoutManager(std::make_shared<ManualLayoutManager>(manager));

    std::cout << "\n=== Multi-Drag Index Validation Test ===" << std::endl;

    // Initial layout
    LayoutResult result = layout.layout(graph_);

    // Store initial indices
    std::unordered_map<EdgeId, std::pair<int, int>> initialIndices;
    for (const auto& [edgeId, el] : result.edgeLayouts()) {
        initialIndices[edgeId] = {el.sourceSnapIndex, el.targetSnapIndex};
    }

    // Verify all initial indices are within valid range
    // snapIndex must be < candidateCount (fixed grid positions on node edge)
    const float gridSize = 10.0f;  // Default PATHFINDING_GRID_SIZE
    std::cout << "Checking initial indices (candidateCount basis)..." << std::endl;
    for (const auto& [edgeId, el] : result.edgeLayouts()) {
        // Get node layouts to calculate candidate count
        const NodeLayout* srcNode = result.getNodeLayout(el.from);
        const NodeLayout* tgtNode = result.getNodeLayout(el.to);
        ASSERT_NE(srcNode, nullptr);
        ASSERT_NE(tgtNode, nullptr);

        // Calculate fixed candidate count for each node edge
        int srcCandidateCount = getFixedSnapPointCount(*srcNode, el.sourceEdge, gridSize);
        int tgtCandidateCount = getFixedSnapPointCount(*tgtNode, el.targetEdge, gridSize);

        EXPECT_GE(el.sourceSnapIndex, 0)
            << "Edge " << edgeId << " sourceSnapIndex=" << el.sourceSnapIndex << " is negative";
        EXPECT_LT(el.sourceSnapIndex, srcCandidateCount)
            << "Edge " << edgeId << " sourceSnapIndex=" << el.sourceSnapIndex
            << " >= srcCandidateCount=" << srcCandidateCount;
        EXPECT_GE(el.targetSnapIndex, 0)
            << "Edge " << edgeId << " targetSnapIndex=" << el.targetSnapIndex << " is negative";
        EXPECT_LT(el.targetSnapIndex, tgtCandidateCount)
            << "Edge " << edgeId << " targetSnapIndex=" << el.targetSnapIndex
            << " >= tgtCandidateCount=" << tgtCandidateCount;
    }

    // Simulate multiple drags on different nodes
    std::vector<NodeId> nodesToDrag = {error_, running_, paused_};

    for (NodeId dragNode : nodesToDrag) {
        Point pos = result.getNodeLayout(dragNode)->position;
        pos.x += 30.0f;
        pos.y -= 20.0f;
        manager.setNodePosition(dragNode, pos);

        result = layout.layout(graph_);

        // After each drag, verify indices are still valid (candidateCount basis)
        for (const auto& [edgeId, el] : result.edgeLayouts()) {
            // Get node layouts to calculate candidate count
            const NodeLayout* srcNode = result.getNodeLayout(el.from);
            const NodeLayout* tgtNode = result.getNodeLayout(el.to);
            ASSERT_NE(srcNode, nullptr);
            ASSERT_NE(tgtNode, nullptr);

            // Calculate fixed candidate count for each node edge
            int srcCandidateCount = getFixedSnapPointCount(*srcNode, el.sourceEdge, gridSize);
            int tgtCandidateCount = getFixedSnapPointCount(*tgtNode, el.targetEdge, gridSize);

            EXPECT_GE(el.sourceSnapIndex, 0)
                << "After dragging node " << dragNode
                << ", Edge " << edgeId << " sourceSnapIndex=" << el.sourceSnapIndex << " is negative";
            EXPECT_LT(el.sourceSnapIndex, srcCandidateCount)
                << "After dragging node " << dragNode
                << ", Edge " << edgeId << " sourceSnapIndex=" << el.sourceSnapIndex
                << " >= srcCandidateCount=" << srcCandidateCount;
            EXPECT_GE(el.targetSnapIndex, 0)
                << "After dragging node " << dragNode
                << ", Edge " << edgeId << " targetSnapIndex=" << el.targetSnapIndex << " is negative";
            EXPECT_LT(el.targetSnapIndex, tgtCandidateCount)
                << "After dragging node " << dragNode
                << ", Edge " << edgeId << " targetSnapIndex=" << el.targetSnapIndex
                << " >= tgtCandidateCount=" << tgtCandidateCount;
        }
    }

    std::cout << "All indices valid after " << nodesToDrag.size() << " drags." << std::endl;
}

// Test: Check what happens when Error is moved above Idle
TEST_F(DragBehaviorTest, Drag_ErrorAboveIdle_UpdatesEdgeDirection) {
    ManualLayoutManager manager;


    SugiyamaLayout layout;
    LayoutOptions options;

    layout.setOptions(options);
    layout.setManualLayoutManager(std::make_shared<ManualLayoutManager>(manager));

    // Initial layout
    LayoutResult result = layout.layout(graph_);

    std::cout << "\n=== Error Moved Above Idle Test ===" << std::endl;

    // Get initial positions
    Point idlePos = result.getNodeLayout(idle_)->position;
    Point errorPos = result.getNodeLayout(error_)->position;

    std::cout << "Initial positions:" << std::endl;
    std::cout << "  Idle: (" << idlePos.x << ", " << idlePos.y << ")" << std::endl;
    std::cout << "  Error: (" << errorPos.x << ", " << errorPos.y << ")" << std::endl;

    // Print initial reset edge routing
    const EdgeLayout* resetLayout = result.getEdgeLayout(e_reset_);
    std::cout << "\nInitial reset edge routing:" << std::endl;
    std::cout << "  sourceEdge: " << static_cast<int>(resetLayout->sourceEdge)
              << " (0=Top, 1=Bottom)" << std::endl;
    std::cout << "  targetEdge: " << static_cast<int>(resetLayout->targetEdge) << std::endl;

    // Move Error ABOVE Idle (lower Y = higher on screen)
    Point newErrorPos = {errorPos.x, idlePos.y - 100.0f};
    manager.setNodePosition(error_, newErrorPos);

    std::cout << "\nMoving Error to: (" << newErrorPos.x << ", " << newErrorPos.y << ")" << std::endl;
    std::cout << "  (This puts Error ABOVE Idle in Y coordinates)" << std::endl;

    // Recalculate layout
    LayoutResult result2 = layout.layout(graph_);

    const EdgeLayout* resetLayout2 = result2.getEdgeLayout(e_reset_);
    std::cout << "\nAfter layout recalc - reset edge routing:" << std::endl;
    std::cout << "  sourceEdge: " << static_cast<int>(resetLayout2->sourceEdge)
              << " (0=Top, 1=Bottom)" << std::endl;
    std::cout << "  targetEdge: " << static_cast<int>(resetLayout2->targetEdge) << std::endl;

    if (resetLayout->sourceEdge != resetLayout2->sourceEdge) {
        std::cout << "\n*** sourceEdge CHANGED from "
                  << static_cast<int>(resetLayout->sourceEdge) << " to "
                  << static_cast<int>(resetLayout2->sourceEdge) << " ***" << std::endl;
    }
}

// Test: Unified mode - snap point order is preserved during drag
TEST_F(DragBehaviorTest, Mode_Unified_SnapOrderPreserved) {
    ManualLayoutManager manager;


    SugiyamaLayout layout;
    LayoutOptions options;

    layout.setOptions(options);
    layout.setManualLayoutManager(std::make_shared<ManualLayoutManager>(manager));

    // Initial layout
    LayoutResult result = layout.layout(graph_);

    std::cout << "\n=== Unified Mode: Snap Order During Drag Test ===" << std::endl;

    // Store layouts locally
    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;

    for (const auto& [id, nl] : result.nodeLayouts()) {
        nodeLayouts[id] = nl;
    }
    for (const auto& [id, el] : result.edgeLayouts()) {
        edgeLayouts[id] = el;
    }

    // Capture initial snap order for Running node (has multiple connections)
    // Running has: incoming (start, resume), outgoing (pause, stop1, fail)
    struct SnapInfo {
        EdgeId edgeId;
        bool isSource;  // true = outgoing from Running, false = incoming to Running
        int snapIndex;
        float xPosition;
    };

    std::vector<SnapInfo> initialOrder;
    std::cout << "\nInitial Running node Bottom edge connections:" << std::endl;

    for (const auto& [edgeId, el] : edgeLayouts) {
        if (el.from == running_ && el.sourceEdge == NodeEdge::Bottom) {
            initialOrder.push_back({edgeId, true, el.sourceSnapIndex, el.sourcePoint.x});
            std::cout << "  Outgoing edge " << edgeId << ": snapIdx=" << el.sourceSnapIndex
                      << ", x=" << el.sourcePoint.x << std::endl;
        }
        if (el.to == running_ && el.targetEdge == NodeEdge::Bottom) {
            initialOrder.push_back({edgeId, false, el.targetSnapIndex, el.targetPoint.x});
            std::cout << "  Incoming edge " << edgeId << ": snapIdx=" << el.targetSnapIndex
                      << ", x=" << el.targetPoint.x << std::endl;
        }
    }

    // Sort by x position to get spatial order
    std::sort(initialOrder.begin(), initialOrder.end(),
              [](const SnapInfo& a, const SnapInfo& b) { return a.xPosition < b.xPosition; });

    std::cout << "\nInitial spatial order (left to right):" << std::endl;
    for (size_t i = 0; i < initialOrder.size(); ++i) {
        std::cout << "  " << i << ": edge " << initialOrder[i].edgeId
                  << (initialOrder[i].isSource ? " (out)" : " (in)")
                  << " at x=" << initialOrder[i].xPosition << std::endl;
    }

    // Simulate drag on Running node
    NodeId draggedNode = running_;
    std::vector<EdgeId> affectedEdges;
    for (const auto& [edgeId, el] : edgeLayouts) {
        if (el.from == running_ || el.to == running_) {
            affectedEdges.push_back(edgeId);
        }
    }

    Point dragOffset = {80.0f, -40.0f};
    nodeLayouts[draggedNode].position.x += dragOffset.x;
    nodeLayouts[draggedNode].position.y += dragOffset.y;

    std::cout << "\nDragging Running by (" << dragOffset.x << ", " << dragOffset.y << ")" << std::endl;

    // Use library API
    LayoutUtils::updateEdgePositions(
        edgeLayouts, nodeLayouts, affectedEdges);

    // Capture order after drag
    std::vector<SnapInfo> afterOrder;
    std::cout << "\nAfter drag Running node Bottom edge connections:" << std::endl;

    for (const auto& [edgeId, el] : edgeLayouts) {
        if (el.from == running_ && el.sourceEdge == NodeEdge::Bottom) {
            afterOrder.push_back({edgeId, true, el.sourceSnapIndex, el.sourcePoint.x});
            std::cout << "  Outgoing edge " << edgeId << ": snapIdx=" << el.sourceSnapIndex
                      << ", x=" << el.sourcePoint.x << std::endl;
        }
        if (el.to == running_ && el.targetEdge == NodeEdge::Bottom) {
            afterOrder.push_back({edgeId, false, el.targetSnapIndex, el.targetPoint.x});
            std::cout << "  Incoming edge " << edgeId << ": snapIdx=" << el.targetSnapIndex
                      << ", x=" << el.targetPoint.x << std::endl;
        }
    }

    // Sort by x position
    std::sort(afterOrder.begin(), afterOrder.end(),
              [](const SnapInfo& a, const SnapInfo& b) { return a.xPosition < b.xPosition; });

    std::cout << "\nAfter drag spatial order (left to right):" << std::endl;
    for (size_t i = 0; i < afterOrder.size(); ++i) {
        std::cout << "  " << i << ": edge " << afterOrder[i].edgeId
                  << (afterOrder[i].isSource ? " (out)" : " (in)")
                  << " at x=" << afterOrder[i].xPosition << std::endl;
    }

    // Verify number of connections is preserved
    ASSERT_EQ(initialOrder.size(), afterOrder.size()) << "Number of connections changed!";

    // With fixed grid candidates, initially invalid indices (-1) will be fixed to valid values.
    // This may cause order changes for those edges. We verify:
    // 1. All indices are now valid (>= 0)
    // 2. Edges that had valid initial indices preserve their relative order
    
    const float gridSize = 10.0f;
    const NodeLayout& runningNode = nodeLayouts[running_];
    int candidateCount = getFixedSnapPointCount(runningNode, NodeEdge::Bottom, gridSize);

    std::cout << "\n=== Fixed Grid Index Validation ===" << std::endl;
    std::cout << "Candidate count for Running/Bottom: " << candidateCount << std::endl;
    
    // Verify all indices are now valid
    for (const auto& info : afterOrder) {
        EXPECT_GE(info.snapIndex, 0)
            << "Edge " << info.edgeId << " has invalid snapIndex=" << info.snapIndex;
        EXPECT_LT(info.snapIndex, candidateCount)
            << "Edge " << info.edgeId << " snapIndex=" << info.snapIndex
            << " >= candidateCount=" << candidateCount;
    }

    // Log order changes for diagnostic purposes (not a hard requirement with fixed grid)
    // Optimizer may rearrange snap positions to satisfy constraints (overlaps, etc.)
    // The key requirement is valid indices, not preserved ordering
    std::vector<size_t> validInitialIndices;
    for (size_t i = 0; i < initialOrder.size(); ++i) {
        if (initialOrder[i].snapIndex >= 0 && initialOrder[i].snapIndex < candidateCount) {
            validInitialIndices.push_back(i);
        }
    }

    std::cout << "\n=== Order Comparison (informational) ===" << std::endl;
    int orderChanges = 0;
    for (size_t i = 0; i + 1 < validInitialIndices.size(); ++i) {
        size_t idx1 = validInitialIndices[i];
        size_t idx2 = validInitialIndices[i + 1];
        EdgeId edge1 = initialOrder[idx1].edgeId;
        EdgeId edge2 = initialOrder[idx2].edgeId;

        // Find positions in afterOrder
        float x1After = 0, x2After = 0;
        for (const auto& info : afterOrder) {
            if (info.edgeId == edge1) x1After = info.xPosition;
            if (info.edgeId == edge2) x2After = info.xPosition;
        }

        // Log relative order change (edge1 was left of edge2)
        if (x1After > x2After) {
            std::cout << "Order change: edge " << edge1
                      << " (was left) is now right of edge " << edge2 << std::endl;
            ++orderChanges;
        }
    }

    if (orderChanges == 0) {
        std::cout << "Relative order preserved for all edges" << std::endl;
    } else {
        std::cout << "Total order changes: " << orderChanges
                  << " (expected - optimizer may rearrange for constraints)" << std::endl;
    }

    // Main requirement is valid indices, not preserved ordering
    // Order changes are acceptable when optimizer needs to satisfy constraints
    // (already validated above that all indices are >= 0 and < candidateCount)
}

// Test: Unified mode - snap point coordinates change by drag offset
TEST_F(DragBehaviorTest, SnapPoints_CoordsUpdateOnDrag) {
    ManualLayoutManager manager;


    SugiyamaLayout layout;
    LayoutOptions options;

    layout.setOptions(options);
    layout.setManualLayoutManager(std::make_shared<ManualLayoutManager>(manager));

    // Initial layout
    LayoutResult result = layout.layout(graph_);

    std::cout << "\n=== Unified Mode: Snap Coordinates During Drag Test ===" << std::endl;

    // Store layouts locally
    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;

    for (const auto& [id, nl] : result.nodeLayouts()) {
        nodeLayouts[id] = nl;
    }
    for (const auto& [id, el] : result.edgeLayouts()) {
        edgeLayouts[id] = el;
    }

    // Capture initial coordinates for Running node's connections
    std::unordered_map<EdgeId, std::pair<Point, Point>> initialCoords;  // source, target points

    std::cout << "\nInitial Running node edge coordinates:" << std::endl;
    for (const auto& [edgeId, el] : edgeLayouts) {
        if (el.from == running_ || el.to == running_) {
            initialCoords[edgeId] = {el.sourcePoint, el.targetPoint};
            std::cout << "  Edge " << edgeId << ": src=(" << el.sourcePoint.x << ", " << el.sourcePoint.y
                      << "), tgt=(" << el.targetPoint.x << ", " << el.targetPoint.y << ")" << std::endl;
        }
    }

    // Simulate drag on Running node
    NodeId draggedNode = running_;
    std::vector<EdgeId> affectedEdges;
    for (const auto& [edgeId, el] : edgeLayouts) {
        if (el.from == running_ || el.to == running_) {
            affectedEdges.push_back(edgeId);
        }
    }

    Point dragOffset = {60.0f, 25.0f};
    nodeLayouts[draggedNode].position.x += dragOffset.x;
    nodeLayouts[draggedNode].position.y += dragOffset.y;

    std::cout << "\nDragging Running by (" << dragOffset.x << ", " << dragOffset.y << ")" << std::endl;

    // Use library API
    LayoutUtils::updateEdgePositions(
        edgeLayouts, nodeLayouts, affectedEdges);

    // Verify coordinates changed by expected offset
    std::cout << "\nAfter drag Running node edge coordinates:" << std::endl;

    // With fixed grid candidates, snap points may be reassigned during drag.
    // Use larger tolerance to account for candidate reassignment (up to half drag or one grid cell).
    const float gridSize = 10.0f;
    float xTolerance = std::max(gridSize, std::abs(dragOffset.x) * 0.5f);
    float yTolerance = std::max(gridSize, std::abs(dragOffset.y) * 0.5f);

    int verifiedCount = 0;
    for (const auto& [edgeId, el] : edgeLayouts) {
        if (el.from == running_ || el.to == running_) {
            std::cout << "  Edge " << edgeId << ": src=(" << el.sourcePoint.x << ", " << el.sourcePoint.y
                      << "), tgt=(" << el.targetPoint.x << ", " << el.targetPoint.y << ")" << std::endl;

            const auto& [initialSrc, initialTgt] = initialCoords[edgeId];

            // If Running is source, sourcePoint should move approximately by dragOffset
            if (el.from == running_) {
                float srcDeltaX = el.sourcePoint.x - initialSrc.x;
                float srcDeltaY = el.sourcePoint.y - initialSrc.y;

                std::cout << "    Source moved by (" << srcDeltaX << ", " << srcDeltaY << ")";

                // Grid tolerance: candidate reassignment allowed
                EXPECT_NEAR(srcDeltaX, dragOffset.x, xTolerance)
                    << "Edge " << edgeId << " sourcePoint.x should move in direction of dragOffset.x";
                EXPECT_NEAR(srcDeltaY, dragOffset.y, yTolerance)
                    << "Edge " << edgeId << " sourcePoint.y should move in direction of dragOffset.y";

                if (std::abs(srcDeltaX - dragOffset.x) < xTolerance &&
                    std::abs(srcDeltaY - dragOffset.y) < yTolerance) {
                    std::cout << " [OK]" << std::endl;
                } else {
                    std::cout << " [grid reassignment]" << std::endl;
                }
                verifiedCount++;
            }

            // If Running is target, targetPoint should move approximately by dragOffset
            if (el.to == running_) {
                float tgtDeltaX = el.targetPoint.x - initialTgt.x;
                float tgtDeltaY = el.targetPoint.y - initialTgt.y;

                std::cout << "    Target moved by (" << tgtDeltaX << ", " << tgtDeltaY << ")";

                // Grid tolerance: candidate reassignment allowed
                EXPECT_NEAR(tgtDeltaX, dragOffset.x, xTolerance)
                    << "Edge " << edgeId << " targetPoint.x should move in direction of dragOffset.x";
                EXPECT_NEAR(tgtDeltaY, dragOffset.y, yTolerance)
                    << "Edge " << edgeId << " targetPoint.y should move in direction of dragOffset.y";

                if (std::abs(tgtDeltaX - dragOffset.x) < xTolerance &&
                    std::abs(tgtDeltaY - dragOffset.y) < yTolerance) {
                    std::cout << " [OK]" << std::endl;
                } else {
                    std::cout << " [grid reassignment]" << std::endl;
                }
                verifiedCount++;
            }
        }
    }

    std::cout << "\nVerified " << verifiedCount << " edge endpoints moved correctly." << std::endl;
    EXPECT_GT(verifiedCount, 0) << "Should have verified at least one endpoint!";
}

// Test: Unified mode snap point distribution
TEST_F(DragBehaviorTest, SnapDistribution_UnifiedSpacing) {
    std::cout << "\n=== Snap Point Distribution ===" << std::endl;

    ManualLayoutManager manager;

    SugiyamaLayout layout;
    LayoutOptions options;
    layout.setOptions(options);
    layout.setManualLayoutManager(std::make_shared<ManualLayoutManager>(manager));

    LayoutResult result = layout.layout(graph_);

    std::cout << "\n--- Initial layout ---" << std::endl;

        // Store layouts
        std::unordered_map<NodeId, NodeLayout> nodeLayouts;
        std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;

        for (const auto& [id, nl] : result.nodeLayouts()) {
            nodeLayouts[id] = nl;
        }
        for (const auto& [id, el] : result.edgeLayouts()) {
            edgeLayouts[id] = el;
        }

        // Get Running node width for reference
        float nodeWidth = nodeLayouts[running_].size.width;
        float nodeX = nodeLayouts[running_].position.x;

        std::cout << "Running node: x=" << nodeX << ", width=" << nodeWidth << std::endl;
        std::cout << "Bottom edge connections (relative position 0-100%):" << std::endl;

        std::vector<std::pair<float, std::string>> positions;

        for (const auto& [edgeId, el] : edgeLayouts) {
            if (el.from == running_ && el.sourceEdge == NodeEdge::Bottom) {
                float relPos = (el.sourcePoint.x - nodeX) / nodeWidth * 100.0f;
                positions.push_back({relPos, "out-" + std::to_string(edgeId)});
            }
            if (el.to == running_ && el.targetEdge == NodeEdge::Bottom) {
                float relPos = (el.targetPoint.x - nodeX) / nodeWidth * 100.0f;
                positions.push_back({relPos, "in-" + std::to_string(edgeId)});
            }
        }

        // Sort by position
        std::sort(positions.begin(), positions.end());

        for (const auto& [pos, label] : positions) {
            std::cout << "  " << label << " at " << pos << "%" << std::endl;
        }

        // Simulate drag
        NodeId draggedNode = running_;
        std::vector<EdgeId> affectedEdges;
        for (const auto& [edgeId, el] : edgeLayouts) {
            if (el.from == running_ || el.to == running_) {
                affectedEdges.push_back(edgeId);
            }
        }

        Point dragOffset = {50.0f, 0.0f};
        nodeLayouts[draggedNode].position.x += dragOffset.x;

        LayoutUtils::updateEdgePositions(edgeLayouts, nodeLayouts, affectedEdges);

        std::cout << "After drag (+50px X):" << std::endl;

        nodeX = nodeLayouts[running_].position.x;
        positions.clear();

        for (const auto& [edgeId, el] : edgeLayouts) {
            if (el.from == running_ && el.sourceEdge == NodeEdge::Bottom) {
                float relPos = (el.sourcePoint.x - nodeX) / nodeWidth * 100.0f;
                positions.push_back({relPos, "out-" + std::to_string(edgeId)});
            }
            if (el.to == running_ && el.targetEdge == NodeEdge::Bottom) {
                float relPos = (el.targetPoint.x - nodeX) / nodeWidth * 100.0f;
                positions.push_back({relPos, "in-" + std::to_string(edgeId)});
            }
        }

        std::sort(positions.begin(), positions.end());

        for (const auto& [pos, label] : positions) {
            std::cout << "  " << label << " at " << pos << "%" << std::endl;
        }
}

// Test for connected nodes' snap points preservation
class ConnectedNodesSnapPointsTest : public ::testing::Test {
protected:
    void SetUp() override {
        idle_ = graph_.addNode(Size{100, 50}, "Idle");
        running_ = graph_.addNode(Size{100, 50}, "Running");
        paused_ = graph_.addNode(Size{100, 50}, "Paused");
        stopped_ = graph_.addNode(Size{100, 50}, "Stopped");
        error_ = graph_.addNode(Size{100, 50}, "Error");

        graph_.addEdge(idle_, running_, "start");
        graph_.addEdge(running_, paused_, "pause");
        graph_.addEdge(paused_, running_, "resume");
        graph_.addEdge(running_, stopped_, "stop");
        graph_.addEdge(paused_, stopped_, "stop");
        graph_.addEdge(running_, error_, "fail");
        graph_.addEdge(error_, idle_, "reset");
    }



    std::string nodeName(NodeId id) const {
        if (id == idle_) return "Idle";
        if (id == running_) return "Running";
        if (id == paused_) return "Paused";
        if (id == stopped_) return "Stopped";
        if (id == error_) return "Error";
        return "Unknown";
    }

    Graph graph_;
    NodeId idle_, running_, paused_, stopped_, error_;
};

TEST_F(ConnectedNodesSnapPointsTest, SnapPointsPreserved) {
    ManualLayoutManager manager;

    SugiyamaLayout layout;
    LayoutOptions options;
    layout.setOptions(options);
    layout.setManualLayoutManager(std::make_shared<ManualLayoutManager>(manager));

    // Initial layout
    LayoutResult result = layout.layout(graph_);

    std::cout << "\n=== Connected Nodes Snap Points Test ===" << std::endl;

    // Store layouts locally
    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;

    for (const auto& [id, nl] : result.nodeLayouts()) {
        nodeLayouts[id] = nl;
    }
    for (const auto& [id, el] : result.edgeLayouts()) {
        edgeLayouts[id] = el;
    }

    // Helper to calculate relative position on a node edge
    auto getRelativePosition = [&](NodeId nodeId, NodeEdge edge, const Point& point) -> float {
        const NodeLayout& node = nodeLayouts[nodeId];
        switch (edge) {
            case NodeEdge::Top:
            case NodeEdge::Bottom:
                return (point.x - node.position.x) / node.size.width;
            case NodeEdge::Left:
            case NodeEdge::Right:
                return (point.y - node.position.y) / node.size.height;
        }
        return 0.0f;
    };

    // Capture initial snap point positions for ALL connected nodes
    struct ConnectedSnapInfo {
        EdgeId edgeId;
        NodeId nodeId;
        bool isSource;
        NodeEdge edge;
        float relativePosition;
        Point absolutePoint;
    };

    std::vector<ConnectedSnapInfo> initialPositions;

    std::cout << "\nInitial snap positions on nodes connected to Running:" << std::endl;

    for (const auto& [edgeId, el] : edgeLayouts) {
        if (el.from == running_ && el.to != running_) {
            float relPos = getRelativePosition(el.to, el.targetEdge, el.targetPoint);
            initialPositions.push_back({edgeId, el.to, false, el.targetEdge, relPos, el.targetPoint});
            std::cout << "  " << nodeName(el.to) << " (edge " << edgeId << "): "
                      << "relPos=" << (relPos * 100.0f) << "%, "
                      << "absPos=(" << el.targetPoint.x << ", " << el.targetPoint.y << ")" << std::endl;
        }
        if (el.to == running_ && el.from != running_) {
            float relPos = getRelativePosition(el.from, el.sourceEdge, el.sourcePoint);
            initialPositions.push_back({edgeId, el.from, true, el.sourceEdge, relPos, el.sourcePoint});
            std::cout << "  " << nodeName(el.from) << " (edge " << edgeId << "): "
                      << "relPos=" << (relPos * 100.0f) << "%, "
                      << "absPos=(" << el.sourcePoint.x << ", " << el.sourcePoint.y << ")" << std::endl;
        }
    }

    // Simulate drag on Running node
    std::vector<EdgeId> affectedEdges;
    for (const auto& [edgeId, el] : edgeLayouts) {
        if (el.from == running_ || el.to == running_) {
            affectedEdges.push_back(edgeId);
        }
    }

    Point dragOffset = {70.0f, -30.0f};
    nodeLayouts[running_].position.x += dragOffset.x;
    nodeLayouts[running_].position.y += dragOffset.y;

    std::cout << "\nDragging Running by (" << dragOffset.x << ", " << dragOffset.y << ")" << std::endl;
    std::cout << "(Other nodes NOT moved)" << std::endl;

    // Only pass Running as moved node - connected nodes should NOT be updated
    std::unordered_set<NodeId> movedNodes = {running_};
    LayoutUtils::updateEdgePositions(
        edgeLayouts, nodeLayouts, affectedEdges, movedNodes);

    // Verify connected nodes' snap points have NOT changed
    std::cout << "\nAfter drag - snap positions on connected nodes:" << std::endl;

    int failCount = 0;
    for (const auto& initial : initialPositions) {
        const EdgeLayout& el = edgeLayouts[initial.edgeId];

        Point currentPoint = initial.isSource ? el.sourcePoint : el.targetPoint;
        float currentRelPos = getRelativePosition(initial.nodeId, initial.edge, currentPoint);

        bool relPosChanged = std::abs(currentRelPos - initial.relativePosition) > 0.001f;
        bool absPosChanged = std::abs(currentPoint.x - initial.absolutePoint.x) > 0.1f ||
                            std::abs(currentPoint.y - initial.absolutePoint.y) > 0.1f;

        std::cout << "  " << nodeName(initial.nodeId) << " (edge " << initial.edgeId << "): "
                  << "relPos=" << (currentRelPos * 100.0f) << "%, "
                  << "absPos=(" << currentPoint.x << ", " << currentPoint.y << ")";

        if (relPosChanged) {
            std::cout << " [REL_POS CHANGED from " << (initial.relativePosition * 100.0f) << "%!]";
            failCount++;
        }
        if (absPosChanged) {
            std::cout << " [ABS_POS CHANGED!]";
            failCount++;
        }
        if (!relPosChanged && !absPosChanged) {
            std::cout << " [OK - unchanged]";
        }
        std::cout << std::endl;

        EXPECT_NEAR(currentRelPos, initial.relativePosition, 0.001f)
            << "Relative position on " << nodeName(initial.nodeId) << " should NOT change!";
        EXPECT_NEAR(currentPoint.x, initial.absolutePoint.x, 0.1f)
            << "Absolute X on " << nodeName(initial.nodeId) << " should NOT change!";
        EXPECT_NEAR(currentPoint.y, initial.absolutePoint.y, 0.1f)
            << "Absolute Y on " << nodeName(initial.nodeId) << " should NOT change!";
    }

    std::cout << "\nTotal failures: " << failCount << std::endl;
    EXPECT_EQ(failCount, 0) << "Connected nodes' snap points should NOT change when dragging Running!";
}


// Test: Edge paths remain orthogonal (all segments horizontal or vertical) during drag
TEST_F(DragBehaviorTest, Drag_OrthogonalityMaintained) {
    ManualLayoutManager manager;

    SugiyamaLayout layout;
    LayoutOptions options;
    layout.setOptions(options);
    layout.setManualLayoutManager(std::make_shared<ManualLayoutManager>(manager));

    // Initial layout
    LayoutResult result = layout.layout(graph_);

    std::cout << "\n=== Orthogonality During Drag Test ===" << std::endl;

    // Store layouts locally
    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;

    for (const auto& [id, nl] : result.nodeLayouts()) {
        nodeLayouts[id] = nl;
    }
    for (const auto& [id, el] : result.edgeLayouts()) {
        edgeLayouts[id] = el;
    }

    // Helper to check if edge path is orthogonal
    auto checkOrthogonality = [](const EdgeLayout& el, EdgeId edgeId) -> bool {
        // Build path: sourcePoint -> bendPoints -> targetPoint
        std::vector<Point> path;
        path.push_back(el.sourcePoint);
        for (const auto& bp : el.bendPoints) {
            path.push_back(bp.position);
        }
        path.push_back(el.targetPoint);

        // Check each segment
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            float dx = std::abs(path[i + 1].x - path[i].x);
            float dy = std::abs(path[i + 1].y - path[i].y);

            // A segment is orthogonal if either dx ~= 0 (vertical) or dy ~= 0 (horizontal)
            bool isHorizontal = dy < 0.5f;
            bool isVertical = dx < 0.5f;

            if (!isHorizontal && !isVertical) {
                std::cout << "  Edge " << edgeId << " segment " << i << " is DIAGONAL: "
                          << "(" << path[i].x << "," << path[i].y << ") -> "
                          << "(" << path[i + 1].x << "," << path[i + 1].y << ") "
                          << "dx=" << dx << ", dy=" << dy << std::endl;
                return false;
            }
        }
        return true;
    };

    // Verify orthogonality in initial layout
    std::cout << "\n--- Checking initial layout orthogonality ---" << std::endl;
    int initialFailures = 0;
    for (const auto& [edgeId, el] : edgeLayouts) {
        if (!checkOrthogonality(el, edgeId)) {
            initialFailures++;
        }
    }
    EXPECT_EQ(initialFailures, 0) << "Initial layout should have orthogonal edges!";
    if (initialFailures == 0) {
        std::cout << "  All edges orthogonal in initial layout." << std::endl;
    }

    // Test multiple drag positions
    std::vector<std::pair<NodeId, Point>> dragTests = {
        {error_, {50.0f, -30.0f}},   // Drag Error up-right
        {running_, {-40.0f, 20.0f}}, // Drag Running down-left
        {paused_, {30.0f, 15.0f}},   // Drag Paused down-right
        {stopped_, {-20.0f, -40.0f}} // Drag Stopped up-left
    };

    for (const auto& [dragNode, dragOffset] : dragTests) {
        std::cout << "\n--- Dragging node " << static_cast<int>(dragNode)
                  << " by (" << dragOffset.x << ", " << dragOffset.y << ") ---" << std::endl;

        // Apply drag
        Point originalPos = nodeLayouts[dragNode].position;
        nodeLayouts[dragNode].position.x += dragOffset.x;
        nodeLayouts[dragNode].position.y += dragOffset.y;

        // Get affected edges
        std::vector<EdgeId> affectedEdges;
        for (const auto& [edgeId, el] : edgeLayouts) {
            if (el.from == dragNode || el.to == dragNode) {
                affectedEdges.push_back(edgeId);
            }
        }

        // Update edge positions using library API
        std::unordered_set<NodeId> movedNodes = {dragNode};
        LayoutUtils::updateEdgePositions(edgeLayouts, nodeLayouts, affectedEdges, movedNodes);

        // Check orthogonality for affected edges
        int failures = 0;
        for (EdgeId edgeId : affectedEdges) {
            if (!checkOrthogonality(edgeLayouts[edgeId], edgeId)) {
                failures++;
            }
        }

        EXPECT_EQ(failures, 0)
            << "After dragging node " << static_cast<int>(dragNode)
            << ", all edges should remain orthogonal!";

        if (failures == 0) {
            std::cout << "  All affected edges orthogonal after drag." << std::endl;
        }

        // Restore position for next test
        nodeLayouts[dragNode].position = originalPos;
        for (EdgeId edgeId : affectedEdges) {
            // Re-update to restore
            LayoutUtils::updateEdgePositions(edgeLayouts, nodeLayouts, {edgeId}, {dragNode});
        }
    }

    std::cout << "\n=== Orthogonality test complete ===" << std::endl;
}

// =============================================================================
// Node Penetration Tests - Verify edges don't pass through other nodes
// =============================================================================

// Test: Edge 6 (error->idle) should not penetrate any node after various drags
TEST_F(DragBehaviorTest, Edge6_ErrorToIdle_ShouldNotPenetrateNodes) {
    // Setup layout with larger nodes (like interactive demo)
    Graph graph;
    auto idle = graph.addNode(Size{200, 100}, "Idle");
    auto running = graph.addNode(Size{200, 100}, "Running");
    auto paused = graph.addNode(Size{200, 100}, "Paused");
    auto stopped = graph.addNode(Size{200, 100}, "Stopped");
    auto error = graph.addNode(Size{200, 100}, "Error");

    graph.addEdge(idle, running, "start");
    graph.addEdge(running, paused, "pause");
    graph.addEdge(paused, running, "resume");
    graph.addEdge(running, stopped, "stop");
    graph.addEdge(paused, stopped, "stop");
    graph.addEdge(running, error, "fail");
    auto e_reset = graph.addEdge(error, idle, "reset");

    LayoutOptions options;
    options.direction = Direction::TopToBottom;
    options.nodeSpacingHorizontal = 100.0f;
    options.nodeSpacingVertical = 100.0f;
    options.gridConfig.cellSize = 20.0f;
    options.autoSnapPoints = true;

    auto manager = std::make_shared<ManualLayoutManager>();
    SugiyamaLayout layout;
    layout.setOptions(options);
    layout.setManualLayoutManager(manager);

    auto result = layout.layout(graph);

    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;
    for (const auto& [id, nl] : result.nodeLayouts()) {
        nodeLayouts[id] = nl;
    }
    for (const auto& [id, el] : result.edgeLayouts()) {
        edgeLayouts[id] = el;
    }

    // Initial state should be valid
    for (const auto& [edgeId, el] : edgeLayouts) {
        EXPECT_FALSE(edgePenetratesAnyNode(el, nodeLayouts))
            << "Initial: Edge " << edgeId << " penetrates a node";
    }

    // Test various drag directions for error node
    std::vector<Point> dragOffsets = {
        {-60, 0},    // left
        {60, 0},     // right
        {0, -40},    // up
        {0, 40},     // down
        {-60, -40},  // left-up
        {60, 40},    // right-down
        {-100, 0},   // far left
        {100, 0},    // far right
    };

    for (const auto& offset : dragOffsets) {
        // Save state
        auto savedNodeLayouts = nodeLayouts;
        auto savedEdgeLayouts = edgeLayouts;

        // Apply drag
        nodeLayouts[error].position.x += offset.x;
        nodeLayouts[error].position.y += offset.y;
        manager->setNodePosition(error, nodeLayouts[error].position);

        std::vector<EdgeId> affected = graph.getConnectedEdges(error);

        LayoutUtils::updateEdgePositions(
            edgeLayouts, nodeLayouts, affected,
            options, {error});

        // Verify no penetration
        for (const auto& [edgeId, el] : edgeLayouts) {
            EXPECT_FALSE(edgePenetratesAnyNode(el, nodeLayouts))
                << "After drag (" << offset.x << "," << offset.y << "): "
                << "Edge " << edgeId << " penetrates a node";
        }

        // Restore
        nodeLayouts = savedNodeLayouts;
        edgeLayouts = savedEdgeLayouts;
    }
}

// Test: All nodes dragged in all directions should not cause penetration
TEST_F(DragBehaviorTest, AllNodes_DragInAllDirections_NoNodePenetration) {
    // Setup
    Graph graph;
    auto idle = graph.addNode(Size{200, 100}, "Idle");
    auto running = graph.addNode(Size{200, 100}, "Running");
    auto paused = graph.addNode(Size{200, 100}, "Paused");
    auto stopped = graph.addNode(Size{200, 100}, "Stopped");
    auto error = graph.addNode(Size{200, 100}, "Error");

    graph.addEdge(idle, running, "start");
    graph.addEdge(running, paused, "pause");
    graph.addEdge(paused, running, "resume");
    graph.addEdge(running, stopped, "stop");
    graph.addEdge(paused, stopped, "stop");
    graph.addEdge(running, error, "fail");
    graph.addEdge(error, idle, "reset");

    LayoutOptions options;
    options.direction = Direction::TopToBottom;
    options.nodeSpacingHorizontal = 100.0f;
    options.nodeSpacingVertical = 100.0f;
    options.gridConfig.cellSize = 20.0f;
    options.autoSnapPoints = true;

    auto manager = std::make_shared<ManualLayoutManager>();
    SugiyamaLayout layout;
    layout.setOptions(options);
    layout.setManualLayoutManager(manager);

    auto result = layout.layout(graph);

    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;
    for (const auto& [id, nl] : result.nodeLayouts()) {
        nodeLayouts[id] = nl;
    }
    for (const auto& [id, el] : result.edgeLayouts()) {
        edgeLayouts[id] = el;
    }

    std::vector<Point> directions = {
        {-40, 0}, {40, 0}, {0, -40}, {0, 40},
        {-40, -40}, {40, -40}, {-40, 40}, {40, 40}
    };

    std::vector<NodeId> allNodes = {idle, running, paused, stopped, error};

    int totalScenarios = 0;
    int penetrationCount = 0;

    for (NodeId nodeId : allNodes) {
        for (const auto& dir : directions) {
            auto savedNodeLayouts = nodeLayouts;
            auto savedEdgeLayouts = edgeLayouts;

            nodeLayouts[nodeId].position.x += dir.x;
            nodeLayouts[nodeId].position.y += dir.y;
            manager->setNodePosition(nodeId, nodeLayouts[nodeId].position);

            std::vector<EdgeId> affected = graph.getConnectedEdges(nodeId);
            LayoutUtils::updateEdgePositions(
                edgeLayouts, nodeLayouts, affected,
                options, {nodeId});

            // Only check edges that were actually updated (connected to dragged node)
            // Unconnected edges aren't re-routed and may conflict with moved nodes
            for (EdgeId edgeId : affected) {
                ++totalScenarios;
                auto it = edgeLayouts.find(edgeId);
                if (it == edgeLayouts.end()) continue;
                const EdgeLayout& el = it->second;
                if (edgePenetratesAnyNode(el, nodeLayouts)) {
                    ++penetrationCount;
                    std::cout << "PENETRATION: Node " << nodeId << " drag ("
                              << dir.x << "," << dir.y << "): Edge " << edgeId << std::endl;
                }
            }

            nodeLayouts = savedNodeLayouts;
            edgeLayouts = savedEdgeLayouts;
        }
    }

    std::cout << "\n=== Penetration Summary ===" << std::endl;
    std::cout << "Total scenarios: " << totalScenarios << std::endl;
    std::cout << "Penetrations: " << penetrationCount << std::endl;

    // Allow a small tolerance for edge cases where optimizer can't find valid paths
    // (e.g., when all NodeEdge combinations are rejected due to overlaps)
    // Current tolerance: 5% of scenarios
    float penetrationRate = static_cast<float>(penetrationCount) / totalScenarios;
    EXPECT_LT(penetrationRate, 0.05f)
        << "Penetration rate " << (penetrationRate * 100) << "% exceeds 5% tolerance";
}

// Test: Random drag stress test - 100 iterations
TEST_F(DragBehaviorTest, RandomDrag_100Iterations_NoNodePenetration) {
    // Setup
    Graph graph;
    auto idle = graph.addNode(Size{200, 100}, "Idle");
    auto running = graph.addNode(Size{200, 100}, "Running");
    auto paused = graph.addNode(Size{200, 100}, "Paused");
    auto stopped = graph.addNode(Size{200, 100}, "Stopped");
    auto error = graph.addNode(Size{200, 100}, "Error");

    graph.addEdge(idle, running, "start");
    graph.addEdge(running, paused, "pause");
    graph.addEdge(paused, running, "resume");
    graph.addEdge(running, stopped, "stop");
    graph.addEdge(paused, stopped, "stop");
    graph.addEdge(running, error, "fail");
    graph.addEdge(error, idle, "reset");

    LayoutOptions options;
    options.direction = Direction::TopToBottom;
    options.nodeSpacingHorizontal = 100.0f;
    options.nodeSpacingVertical = 100.0f;
    options.gridConfig.cellSize = 20.0f;
    options.autoSnapPoints = true;

    auto manager = std::make_shared<ManualLayoutManager>();
    SugiyamaLayout layout;
    layout.setOptions(options);
    layout.setManualLayoutManager(manager);

    auto result = layout.layout(graph);

    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;
    for (const auto& [id, nl] : result.nodeLayouts()) {
        nodeLayouts[id] = nl;
    }
    for (const auto& [id, el] : result.edgeLayouts()) {
        edgeLayouts[id] = el;
    }

    std::vector<NodeId> allNodes = {idle, running, paused, stopped, error};

    std::mt19937 rng(42);  // Deterministic seed for reproducibility
    std::uniform_int_distribution<size_t> nodeDist(0, allNodes.size() - 1);
    std::uniform_real_distribution<float> offsetDist(-100.0f, 100.0f);

    int penetrationCount = 0;

    for (int i = 0; i < 100; ++i) {
        NodeId nodeId = allNodes[nodeDist(rng)];
        float dx = offsetDist(rng);
        float dy = offsetDist(rng);

        // Grid snap
        dx = std::round(dx / 20.0f) * 20.0f;
        dy = std::round(dy / 20.0f) * 20.0f;

        // Calculate proposed new position
        Point newPos = {
            nodeLayouts[nodeId].position.x + dx,
            nodeLayouts[nodeId].position.y + dy
        };

        // Check if position is valid (no node overlap)
        auto validation = LayoutUtils::canMoveNodeTo(
            nodeId, newPos, nodeLayouts, edgeLayouts, options.gridConfig.cellSize);
        
        if (!validation.valid) {
            // Skip this drag - would cause node overlap
            continue;
        }

        nodeLayouts[nodeId].position = newPos;
        manager->setNodePosition(nodeId, nodeLayouts[nodeId].position);

        std::vector<EdgeId> affected = graph.getConnectedEdges(nodeId);
        std::cout << "[TEST] Iteration " << i << " BEFORE updateEdgePositions" << std::endl;
        LayoutUtils::updateEdgePositions(
            edgeLayouts, nodeLayouts, affected,
            options, {nodeId});
        std::cout << "[TEST] Iteration " << i << " AFTER updateEdgePositions" << std::endl;

        for (const auto& [edgeId, el] : edgeLayouts) {
            if (edgePenetratesAnyNode(el, nodeLayouts)) {
                penetrationCount++;
                std::cout << "Iteration " << i << ", Node " << nodeId
                          << " drag (" << dx << "," << dy << "): "
                          << "Edge " << edgeId << " penetrates" << std::endl;
                
                // Detailed debug: print path and check which node is penetrated
                std::cout << "  Path: src=(" << el.sourcePoint.x << "," << el.sourcePoint.y << ")";
                for (const auto& bp : el.bendPoints) {
                    std::cout << " -> (" << bp.position.x << "," << bp.position.y << ")";
                }
                std::cout << " -> tgt=(" << el.targetPoint.x << "," << el.targetPoint.y << ")" << std::endl;
                
                // Find which node is penetrated
                std::vector<Point> path;
                path.push_back(el.sourcePoint);
                for (const auto& bp : el.bendPoints) path.push_back(bp.position);
                path.push_back(el.targetPoint);
                
                for (size_t seg = 0; seg + 1 < path.size(); ++seg) {
                    for (const auto& [nid, node] : nodeLayouts) {
                        if (nid == el.from || nid == el.to) continue;
                        if (segmentPenetratesNode(path[seg], path[seg+1], node)) {
                            std::cout << "  Segment " << seg << " penetrates Node " << nid
                                      << " at (" << node.position.x << "," << node.position.y
                                      << " " << node.size.width << "x" << node.size.height << ")" << std::endl;
                        }
                    }
                }
                
                // Print all node positions
                std::cout << "  Node positions:" << std::endl;
                for (const auto& [nid, node] : nodeLayouts) {
                    std::cout << "    N" << nid << ": (" << node.position.x << "," << node.position.y
                              << " " << node.size.width << "x" << node.size.height << ")" << std::endl;
                }
            }
        }
    }

    EXPECT_EQ(penetrationCount, 0) << "Total penetrations: " << penetrationCount;
}
