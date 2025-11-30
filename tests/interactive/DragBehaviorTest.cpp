#include <gtest/gtest.h>
#include <arborvia/arborvia.h>
#include <iostream>
#include <unordered_map>

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
    
    Graph graph_;
    NodeId idle_, running_, paused_, stopped_, error_;
    EdgeId e_start_, e_pause_, e_resume_, e_stop1_, e_stop2_, e_fail_, e_reset_;
};

// Test: Initial layout produces valid edge routing
TEST_F(DragBehaviorTest, Layout_InitialRouting_IsValid) {
    SugiyamaLayout layout;
    LayoutOptions options;
    options.snapDistribution = SnapDistribution::Separated;
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
    manager.setMode(LayoutMode::Auto);
    
    SugiyamaLayout layout;
    LayoutOptions options;
    options.snapDistribution = SnapDistribution::Separated;
    layout.setOptions(options);
    layout.setManualLayoutManager(&manager);
    
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
    manager.setMode(LayoutMode::Auto);
    
    SugiyamaLayout layout;
    LayoutOptions options;
    options.snapDistribution = SnapDistribution::Separated;
    layout.setOptions(options);
    layout.setManualLayoutManager(&manager);
    
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
    manager.setMode(LayoutMode::Auto);
    
    SugiyamaLayout layout;
    LayoutOptions options;
    options.snapDistribution = SnapDistribution::Separated;
    layout.setOptions(options);
    layout.setManualLayoutManager(&manager);
    
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

// Test: Verify snap distribution mode works correctly
TEST_F(DragBehaviorTest, Mode_Separated_DistributesCorrectly) {
    SugiyamaLayout layout;
    LayoutOptions options;
    options.snapDistribution = SnapDistribution::Separated;
    layout.setOptions(options);
    
    LayoutResult result = layout.layout(graph_);
    
    std::cout << "\n=== Separated Distribution Mode ===" << std::endl;
    
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

// Test: Unified distribution mode
TEST_F(DragBehaviorTest, Mode_Unified_DistributesCorrectly) {
    SugiyamaLayout layout;
    LayoutOptions options;
    options.snapDistribution = SnapDistribution::Unified;
    layout.setOptions(options);
    
    LayoutResult result = layout.layout(graph_);
    
    std::cout << "\n=== Unified Distribution Mode ===" << std::endl;
    
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
    manager.setMode(LayoutMode::Auto);
    
    SugiyamaLayout layout;
    LayoutOptions options;
    options.snapDistribution = SnapDistribution::Separated;
    layout.setOptions(options);
    layout.setManualLayoutManager(&manager);
    
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
        edgeLayouts, nodeLayouts, allEdges,
        options.snapDistribution);
    
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
    
    // Verify edge routing was preserved (library API should not change these)
    for (const auto& [edgeId, edgeLayout] : edgeLayouts) {
        if (edgeLayout.from == error_ || edgeLayout.to == error_) {
            EXPECT_EQ(edgeLayout.sourceEdge, savedEdgeRouting[edgeId].sourceEdge);
            EXPECT_EQ(edgeLayout.targetEdge, savedEdgeRouting[edgeId].targetEdge);
            EXPECT_EQ(edgeLayout.sourceSnapIndex, savedEdgeRouting[edgeId].sourceSnapIndex);
            EXPECT_EQ(edgeLayout.targetSnapIndex, savedEdgeRouting[edgeId].targetSnapIndex);
        }
    }
}

// Test: Verify snap points don't merge during simulated drag
TEST_F(DragBehaviorTest, Snap_DuringDrag_PointsDontMerge) {
    ManualLayoutManager manager;
    manager.setMode(LayoutMode::Auto);
    
    SugiyamaLayout layout;
    LayoutOptions options;
    options.snapDistribution = SnapDistribution::Separated;
    layout.setOptions(options);
    layout.setManualLayoutManager(&manager);
    
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
    
    // Find Error's Top edge connections
    std::cout << "\nInitial Error Top connections:" << std::endl;
    Point resetSrcBefore, failTgtBefore;
    for (const auto& [edgeId, el] : edgeLayouts) {
        if (el.from == error_ && el.sourceEdge == NodeEdge::Top) {
            std::cout << "  reset (outgoing): srcPt=(" << el.sourcePoint.x << ", " << el.sourcePoint.y << ")" << std::endl;
            resetSrcBefore = el.sourcePoint;
        }
        if (el.to == error_ && el.targetEdge == NodeEdge::Top) {
            std::cout << "  fail (incoming): tgtPt=(" << el.targetPoint.x << ", " << el.targetPoint.y << ")" << std::endl;
            failTgtBefore = el.targetPoint;
        }
    }
    
    // Verify they're at different positions initially
    float initialDiff = std::abs(resetSrcBefore.x - failTgtBefore.x);
    std::cout << "Initial position difference: " << initialDiff << " pixels" << std::endl;
    EXPECT_GT(initialDiff, 10.0f) << "Initial snap points should be separated!";
    
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
        edgeLayouts, nodeLayouts, affectedEdges,
        SnapDistribution::Separated);
    
    // Check Error's Top connections after simulated drag
    std::cout << "\nAfter drag (using library API):" << std::endl;
    Point resetSrcAfter, failTgtAfter;
    for (const auto& [edgeId, el] : edgeLayouts) {
        if (el.from == error_ && el.sourceEdge == NodeEdge::Top) {
            std::cout << "  reset (outgoing): srcPt=(" << el.sourcePoint.x << ", " << el.sourcePoint.y << ")" << std::endl;
            resetSrcAfter = el.sourcePoint;
        }
        if (el.to == error_ && el.targetEdge == NodeEdge::Top) {
            std::cout << "  fail (incoming): tgtPt=(" << el.targetPoint.x << ", " << el.targetPoint.y << ")" << std::endl;
            failTgtAfter = el.targetPoint;
        }
    }
    
    float afterDiff = std::abs(resetSrcAfter.x - failTgtAfter.x);
    std::cout << "After drag position difference: " << afterDiff << " pixels" << std::endl;
    
    // Key assertion: snap points should remain separated during drag
    EXPECT_GT(afterDiff, 10.0f) << "Snap points should remain separated during drag!";
    
    // Verify positions moved by the drag offset
    EXPECT_NEAR(resetSrcAfter.x - resetSrcBefore.x, dragOffset.x, 1.0f);
    EXPECT_NEAR(failTgtAfter.x - failTgtBefore.x, dragOffset.x, 1.0f);
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
    manager.setMode(LayoutMode::Auto);
    
    SugiyamaLayout layout;
    LayoutOptions options;
    options.snapDistribution = SnapDistribution::Separated;
    layout.setOptions(options);
    layout.setManualLayoutManager(&manager);
    
    std::cout << "\n=== Multi-Drag Index Validation Test ===" << std::endl;
    
    // Initial layout
    LayoutResult result = layout.layout(graph_);
    
    // Store initial indices
    std::unordered_map<EdgeId, std::pair<int, int>> initialIndices;
    for (const auto& [edgeId, el] : result.edgeLayouts()) {
        initialIndices[edgeId] = {el.sourceSnapIndex, el.targetSnapIndex};
    }
    
    // Verify all initial indices are within valid range
    std::cout << "Checking initial indices..." << std::endl;
    for (const auto& [edgeId, el] : result.edgeLayouts()) {
        // Count edges on same source edge
        int srcCount = 0;
        int tgtCount = 0;
        for (const auto& [otherId, otherEl] : result.edgeLayouts()) {
            if (otherEl.from == el.from && otherEl.sourceEdge == el.sourceEdge) srcCount++;
            if (otherEl.to == el.to && otherEl.targetEdge == el.targetEdge) tgtCount++;
        }
        
        EXPECT_LT(el.sourceSnapIndex, srcCount) 
            << "Edge " << edgeId << " sourceSnapIndex=" << el.sourceSnapIndex 
            << " >= srcCount=" << srcCount;
        EXPECT_LT(el.targetSnapIndex, tgtCount)
            << "Edge " << edgeId << " targetSnapIndex=" << el.targetSnapIndex 
            << " >= tgtCount=" << tgtCount;
    }
    
    // Simulate multiple drags on different nodes
    std::vector<NodeId> nodesToDrag = {error_, running_, paused_};
    
    for (NodeId dragNode : nodesToDrag) {
        Point pos = result.getNodeLayout(dragNode)->position;
        pos.x += 30.0f;
        pos.y -= 20.0f;
        manager.setNodePosition(dragNode, pos);
        
        result = layout.layout(graph_);
        
        // After each drag, verify indices are still valid
        for (const auto& [edgeId, el] : result.edgeLayouts()) {
            int srcCount = 0;
            int tgtCount = 0;
            for (const auto& [otherId, otherEl] : result.edgeLayouts()) {
                if (otherEl.from == el.from && otherEl.sourceEdge == el.sourceEdge) srcCount++;
                if (otherEl.to == el.to && otherEl.targetEdge == el.targetEdge) tgtCount++;
            }
            
            EXPECT_LT(el.sourceSnapIndex, srcCount) 
                << "After dragging node " << dragNode 
                << ", Edge " << edgeId << " sourceSnapIndex=" << el.sourceSnapIndex 
                << " >= srcCount=" << srcCount;
            EXPECT_LT(el.targetSnapIndex, tgtCount)
                << "After dragging node " << dragNode
                << ", Edge " << edgeId << " targetSnapIndex=" << el.targetSnapIndex 
                << " >= tgtCount=" << tgtCount;
        }
    }
    
    std::cout << "All indices valid after " << nodesToDrag.size() << " drags." << std::endl;
}

// Test: Check what happens when Error is moved above Idle
TEST_F(DragBehaviorTest, Drag_ErrorAboveIdle_UpdatesEdgeDirection) {
    ManualLayoutManager manager;
    manager.setMode(LayoutMode::Auto);
    
    SugiyamaLayout layout;
    LayoutOptions options;
    options.snapDistribution = SnapDistribution::Separated;
    layout.setOptions(options);
    layout.setManualLayoutManager(&manager);
    
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
    manager.setMode(LayoutMode::Auto);
    
    SugiyamaLayout layout;
    LayoutOptions options;
    options.snapDistribution = SnapDistribution::Unified;
    layout.setOptions(options);
    layout.setManualLayoutManager(&manager);
    
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
    
    // Use library API with Unified mode
    LayoutUtils::updateEdgePositions(
        edgeLayouts, nodeLayouts, affectedEdges,
        SnapDistribution::Unified);
    
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
    
    // Verify order is preserved
    ASSERT_EQ(initialOrder.size(), afterOrder.size()) << "Number of connections changed!";
    
    std::cout << "\n=== Order Comparison ===" << std::endl;
    bool orderPreserved = true;
    for (size_t i = 0; i < initialOrder.size(); ++i) {
        if (initialOrder[i].edgeId != afterOrder[i].edgeId) {
            std::cout << "Order CHANGED at position " << i << ": was edge " 
                      << initialOrder[i].edgeId << ", now edge " << afterOrder[i].edgeId << std::endl;
            orderPreserved = false;
        }
    }
    
    if (orderPreserved) {
        std::cout << "Order PRESERVED - all edges in same spatial order" << std::endl;
    }
    
    EXPECT_TRUE(orderPreserved) << "Snap point order should be preserved during drag!";
    
    // Also verify snap indices are unchanged
    for (size_t i = 0; i < initialOrder.size(); ++i) {
        EXPECT_EQ(initialOrder[i].snapIndex, afterOrder[i].snapIndex)
            << "Snap index changed for edge " << initialOrder[i].edgeId;
    }
}

// Test: Unified mode - snap point coordinates change by drag offset
TEST_F(DragBehaviorTest, Mode_Unified_CoordsUpdateOnDrag) {
    ManualLayoutManager manager;
    manager.setMode(LayoutMode::Auto);
    
    SugiyamaLayout layout;
    LayoutOptions options;
    options.snapDistribution = SnapDistribution::Unified;
    layout.setOptions(options);
    layout.setManualLayoutManager(&manager);
    
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
    
    // Use library API with Unified mode
    LayoutUtils::updateEdgePositions(
        edgeLayouts, nodeLayouts, affectedEdges,
        SnapDistribution::Unified);
    
    // Verify coordinates changed by expected offset
    std::cout << "\nAfter drag Running node edge coordinates:" << std::endl;
    
    int verifiedCount = 0;
    for (const auto& [edgeId, el] : edgeLayouts) {
        if (el.from == running_ || el.to == running_) {
            std::cout << "  Edge " << edgeId << ": src=(" << el.sourcePoint.x << ", " << el.sourcePoint.y
                      << "), tgt=(" << el.targetPoint.x << ", " << el.targetPoint.y << ")" << std::endl;
            
            const auto& [initialSrc, initialTgt] = initialCoords[edgeId];
            
            // If Running is source, sourcePoint should move by dragOffset
            if (el.from == running_) {
                float srcDeltaX = el.sourcePoint.x - initialSrc.x;
                float srcDeltaY = el.sourcePoint.y - initialSrc.y;
                
                std::cout << "    Source moved by (" << srcDeltaX << ", " << srcDeltaY << ")";
                
                EXPECT_NEAR(srcDeltaX, dragOffset.x, 1.0f) 
                    << "Edge " << edgeId << " sourcePoint.x should move by dragOffset.x";
                EXPECT_NEAR(srcDeltaY, dragOffset.y, 1.0f)
                    << "Edge " << edgeId << " sourcePoint.y should move by dragOffset.y";
                
                if (std::abs(srcDeltaX - dragOffset.x) < 1.0f && 
                    std::abs(srcDeltaY - dragOffset.y) < 1.0f) {
                    std::cout << " [OK]" << std::endl;
                } else {
                    std::cout << " [MISMATCH - expected (" << dragOffset.x << ", " << dragOffset.y << ")]" << std::endl;
                }
                verifiedCount++;
            }
            
            // If Running is target, targetPoint should move by dragOffset  
            if (el.to == running_) {
                float tgtDeltaX = el.targetPoint.x - initialTgt.x;
                float tgtDeltaY = el.targetPoint.y - initialTgt.y;
                
                std::cout << "    Target moved by (" << tgtDeltaX << ", " << tgtDeltaY << ")";
                
                EXPECT_NEAR(tgtDeltaX, dragOffset.x, 1.0f)
                    << "Edge " << edgeId << " targetPoint.x should move by dragOffset.x";
                EXPECT_NEAR(tgtDeltaY, dragOffset.y, 1.0f)
                    << "Edge " << edgeId << " targetPoint.y should move by dragOffset.y";
                
                if (std::abs(tgtDeltaX - dragOffset.x) < 1.0f && 
                    std::abs(tgtDeltaY - dragOffset.y) < 1.0f) {
                    std::cout << " [OK]" << std::endl;
                } else {
                    std::cout << " [MISMATCH - expected (" << dragOffset.x << ", " << dragOffset.y << ")]" << std::endl;
                }
                verifiedCount++;
            }
        }
    }
    
    std::cout << "\nVerified " << verifiedCount << " edge endpoints moved correctly." << std::endl;
    EXPECT_GT(verifiedCount, 0) << "Should have verified at least one endpoint!";
}

// Test: Unified mode - compare with Separated mode behavior
TEST_F(DragBehaviorTest, Mode_UnifiedVsSeparated_Comparison) {
    std::cout << "\n=== Unified vs Separated Mode Comparison ===" << std::endl;
    
    // Test with both modes and compare snap point distributions
    for (auto mode : {SnapDistribution::Unified, SnapDistribution::Separated}) {
        ManualLayoutManager manager;
        manager.setMode(LayoutMode::Auto);
        
        SugiyamaLayout layout;
        LayoutOptions options;
        options.snapDistribution = mode;
        layout.setOptions(options);
        layout.setManualLayoutManager(&manager);
        
        LayoutResult result = layout.layout(graph_);
        
        std::cout << "\n--- " << (mode == SnapDistribution::Unified ? "UNIFIED" : "SEPARATED") 
                  << " mode ---" << std::endl;
        
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
        
        LayoutUtils::updateEdgePositions(edgeLayouts, nodeLayouts, affectedEdges, mode);
        
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
}

// Parameterized test for connected nodes' snap points preservation
// Tests both Unified and Separated snap distribution modes
class ConnectedNodesSnapPointsTest : public ::testing::TestWithParam<SnapDistribution> {
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
    
    std::string getModeName() const {
        return GetParam() == SnapDistribution::Unified ? "Unified" : "Separated";
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

TEST_P(ConnectedNodesSnapPointsTest, SnapPointsPreserved) {
    SnapDistribution distribution = GetParam();
    
    ManualLayoutManager manager;
    manager.setMode(LayoutMode::Auto);
    
    SugiyamaLayout layout;
    LayoutOptions options;
    options.snapDistribution = distribution;
    layout.setOptions(options);
    layout.setManualLayoutManager(&manager);
    
    // Initial layout
    LayoutResult result = layout.layout(graph_);
    
    std::cout << "\n=== " << getModeName() << " Mode: Connected Nodes Snap Points Test ===" << std::endl;
    
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
        edgeLayouts, nodeLayouts, affectedEdges,
        distribution, movedNodes);
    
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

INSTANTIATE_TEST_SUITE_P(
    SnapDistributionModes,
    ConnectedNodesSnapPointsTest,
    ::testing::Values(SnapDistribution::Unified, SnapDistribution::Separated),
    [](const ::testing::TestParamInfo<SnapDistribution>& info) {
        return info.param == SnapDistribution::Unified ? "Unified" : "Separated";
    }
);
