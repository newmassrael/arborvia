#include <gtest/gtest.h>
#include <gtest/gtest.h>
#include <arborvia/arborvia.h>
#include <arborvia/layout/api/LayoutController.h>
#include <arborvia/layout/api/IEdgeOptimizer.h>
#include <arborvia/layout/config/OptimizerConfig.h>
#include "../../src/layout/optimization/OptimizerRegistry.h"
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <filesystem>

// SCXML test infrastructure
#include "../infrastructure/scxml/SCXMLTestLoader.h"
#include "../infrastructure/scxml/SCXMLGraph.h"
#include "../infrastructure/scxml/SCXMLTypes.h"

using namespace arborvia;
using arborvia::test::scxml::SCXMLTestLoader;
using arborvia::test::scxml::SCXMLGraph;
using arborvia::test::scxml::ConvertOptions;

/**
 * SnapCollisionReproductionTest
 *
 * TDD test that reproduces the snap position collision bug observed in interactive demo.
 *
 * Bug scenario from logs:
 * - Edge 1 and Edge 2 share the same sourcePoint (60, 160)
 * - Edge 1 snap: src=(60,160) snapIdx=2 conn=1/2
 * - Edge 2 path: (60,160)->(60,340)->...
 * - Both edges exit from the same NodeEdge but have identical sourcePoint
 *
 * Root cause:
 * - recalculateSourceEdgesForMovedNodes() only recalculates edges whose nodes were moved
 * - When Edge 1 joins Edge 2's NodeEdge, Edge 2 is not recalculated
 * - Edge 2 keeps its old position, which may collide with Edge 1's new position
 *
 * This test should FAIL initially (TDD red phase), then pass after fix.
 */
class SnapCollisionReproductionTest : public ::testing::Test {
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
        e_resume_ = graph_.addEdge(paused_, running_, "resume");   // Edge 2
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

    // Helper: Check if two edges on same NodeEdge have duplicate sourcePoints
    bool hasSourcePointCollision(const EdgeLayout& e1, const EdgeLayout& e2, float tolerance = 1.0f) const {
        // Must be from same source node and same source edge
        if (e1.from != e2.from) return false;
        if (e1.sourceEdge != e2.sourceEdge) return false;
        
        // Check if sourcePoints are the same (within tolerance)
        float dx = std::abs(e1.sourcePoint.x - e2.sourcePoint.x);
        float dy = std::abs(e1.sourcePoint.y - e2.sourcePoint.y);
        return dx < tolerance && dy < tolerance;
    }

    // Helper: Check if two edges on same NodeEdge have duplicate targetPoints
    bool hasTargetPointCollision(const EdgeLayout& e1, const EdgeLayout& e2, float tolerance = 1.0f) const {
        // Must be to same target node and same target edge
        if (e1.to != e2.to) return false;
        if (e1.targetEdge != e2.targetEdge) return false;
        
        // Check if targetPoints are the same (within tolerance)
        float dx = std::abs(e1.targetPoint.x - e2.targetPoint.x);
        float dy = std::abs(e1.targetPoint.y - e2.targetPoint.y);
        return dx < tolerance && dy < tolerance;
    }

    // Helper: Find all snap point collisions in current layout
    std::vector<std::pair<EdgeId, EdgeId>> findAllSnapCollisions() const {
        std::vector<std::pair<EdgeId, EdgeId>> collisions;
        std::vector<EdgeId> edgeIds;
        for (const auto& [id, _] : edgeLayouts_) {
            edgeIds.push_back(id);
        }
        
        for (size_t i = 0; i < edgeIds.size(); ++i) {
            for (size_t j = i + 1; j < edgeIds.size(); ++j) {
                const auto& e1 = edgeLayouts_.at(edgeIds[i]);
                const auto& e2 = edgeLayouts_.at(edgeIds[j]);
                
                // Skip self-loops
                if (e1.from == e1.to || e2.from == e2.to) continue;
                
                if (hasSourcePointCollision(e1, e2) || hasTargetPointCollision(e1, e2)) {
                    collisions.push_back({edgeIds[i], edgeIds[j]});
                }
            }
        }
        return collisions;
    }

    // Helper: Simulate drag and update edges using LayoutController
    void simulateDrag(NodeId nodeId, float dx, float dy) {
        Point newPosition = nodeLayouts_[nodeId].position;
        newPosition.x += dx;
        newPosition.y += dy;

        // Initialize controller with current state
        LayoutController controller(graph_, options_);
        controller.initializeFrom(nodeLayouts_, edgeLayouts_);

        // Use LayoutController::moveNode() with full constraint validation
        auto result = controller.moveNode(nodeId, newPosition);

        if (result.success) {
            // Sync layouts from controller
            for (const auto& [id, layout] : controller.nodeLayouts()) {
                nodeLayouts_[id] = layout;
            }
            for (const auto& [id, layout] : controller.edgeLayouts()) {
                edgeLayouts_[id] = layout;
            }
            // Update manager with actual position
            manager_->setNodePosition(nodeId, result.actualPosition);
        }
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

    // Helper: Print edge snap info for debugging
    void printEdgeSnapInfo(EdgeId edgeId, const std::string& label = "") const {
        const auto& el = edgeLayouts_.at(edgeId);
        std::cout << label << "Edge " << edgeId
                  << ": from=" << el.from << " to=" << el.to
                  << " srcEdge=" << static_cast<int>(el.sourceEdge)
                  << " srcPoint=(" << el.sourcePoint.x << "," << el.sourcePoint.y << ")"
                  << " tgtEdge=" << static_cast<int>(el.targetEdge)
                  << " tgtPoint=(" << el.targetPoint.x << "," << el.targetPoint.y << ")"
                  << std::endl;
    }

    // Helper: Print collision info
    void printCollisionInfo(const std::vector<std::pair<EdgeId, EdgeId>>& collisions) const {
        if (collisions.empty()) {
            std::cout << "No snap point collisions found." << std::endl;
            return;
        }
        std::cout << "Found " << collisions.size() << " snap point collision(s):" << std::endl;
        for (const auto& [e1, e2] : collisions) {
            const auto& el1 = edgeLayouts_.at(e1);
            const auto& el2 = edgeLayouts_.at(e2);
            
            bool srcCollision = hasSourcePointCollision(el1, el2);
            bool tgtCollision = hasTargetPointCollision(el1, el2);
            
            std::cout << "  Edge " << e1 << " and Edge " << e2 << ": ";
            if (srcCollision) {
                std::cout << "SOURCE collision at (" << el1.sourcePoint.x << "," << el1.sourcePoint.y << ") ";
            }
            if (tgtCollision) {
                std::cout << "TARGET collision at (" << el1.targetPoint.x << "," << el1.targetPoint.y << ")";
            }
            std::cout << std::endl;
        }
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
 * Test: Initial layout should have no snap point collisions
 */
TEST_F(SnapCollisionReproductionTest, InitialLayout_NoSnapCollisions) {
    std::cout << "\n=== Initial Layout Snap Collision Test ===" << std::endl;

    doLayout();

    std::cout << "\n--- All edges ---" << std::endl;
    for (const auto& [edgeId, _] : edgeLayouts_) {
        printEdgeSnapInfo(edgeId);
    }

    auto collisions = findAllSnapCollisions();
    printCollisionInfo(collisions);

    EXPECT_EQ(collisions.size(), 0u)
        << "Initial layout should have no snap point collisions";
}

/**
 * Test: After drag sequence, edges on same NodeEdge should have different snap points
 *
 * This test simulates a scenario where:
 * 1. Two edges initially exit from different NodeEdges of the same node
 * 2. After dragging, one edge's optimal NodeEdge changes
 * 3. Both edges now exit from the same NodeEdge
 * 4. They should have DIFFERENT sourcePoints (distributed snap positions)
 *
 * Bug: If recalculateSourceEdgesForMovedNodes() only recalculates moved edges,
 * the non-moved edge keeps its old position, causing collision.
 */
TEST_F(SnapCollisionReproductionTest, DragSequence_NoSnapCollisionsAfterNodeEdgeChange) {
    std::cout << "\n=== Drag Sequence Snap Collision Test ===" << std::endl;

    doLayout();

    std::cout << "\n--- Initial state ---" << std::endl;
    for (const auto& [edgeId, _] : edgeLayouts_) {
        printEdgeSnapInfo(edgeId);
    }

    // Verify no initial collisions
    auto initialCollisions = findAllSnapCollisions();
    ASSERT_EQ(initialCollisions.size(), 0u) << "Should start with no collisions";

    std::cout << "\n--- Drag sequence to trigger NodeEdge changes ---" << std::endl;

    // This drag sequence is designed to cause edges to change their NodeEdge
    // and potentially collide with existing edges on the new NodeEdge
    std::vector<std::tuple<std::string, NodeId, Point>> sequence = {
        {"drag running right", running_, {300, 0}},    // Running moves far right
        {"drag running up", running_, {0, -150}},       // Running moves up
        {"drag paused right", paused_, {200, 0}},       // Paused moves right
        {"drag idle right", idle_, {100, 0}},           // Idle moves right
    };

    for (const auto& [desc, node, offset] : sequence) {
        std::cout << "\n--- " << desc << " (node " << node << " by " 
                  << offset.x << "," << offset.y << ") ---" << std::endl;
        
        simulateDrag(node, offset.x, offset.y);

        // Check for collisions after each step
        auto collisions = findAllSnapCollisions();
        if (!collisions.empty()) {
            std::cout << "  COLLISION detected after this drag!" << std::endl;
            printCollisionInfo(collisions);
        }
    }

    std::cout << "\n--- Final state ---" << std::endl;
    for (const auto& [edgeId, _] : edgeLayouts_) {
        printEdgeSnapInfo(edgeId);
    }

    // Final assertion: NO snap point collisions
    auto finalCollisions = findAllSnapCollisions();
    printCollisionInfo(finalCollisions);

    EXPECT_EQ(finalCollisions.size(), 0u)
        << "No edges should share snap points after drag sequence. "
        << "Found " << finalCollisions.size() << " collision(s).";
}

/**
 * Test: Random drag sequence should maintain snap point uniqueness
 *
 * This test uses more aggressive drags to increase the chance of
 * triggering the bug where edges join the same NodeEdge.
 */
TEST_F(SnapCollisionReproductionTest, AggressiveDragSequence_NoSnapCollisions) {
    std::cout << "\n=== Aggressive Drag Sequence Test ===" << std::endl;

    doLayout();

    auto initialCollisions = findAllSnapCollisions();
    ASSERT_EQ(initialCollisions.size(), 0u) << "Should start with no collisions";

    // More aggressive drag sequence
    std::vector<std::tuple<std::string, NodeId, Point>> sequence = {
        // Move running far right to change edge directions
        {"running +500,0", running_, {500, 0}},
        // Move running back and up
        {"running -200,-200", running_, {-200, -200}},
        // Move paused to overlap area
        {"paused +300,+100", paused_, {300, 100}},
        // Move stopped
        {"stopped -100,+50", stopped_, {-100, 50}},
        // Move running again
        {"running +100,+100", running_, {100, 100}},
        // Move idle
        {"idle +200,-50", idle_, {200, -50}},
    };

    int step = 0;
    int totalCollisions = 0;

    for (const auto& [desc, node, offset] : sequence) {
        step++;
        std::cout << "\nStep " << step << ": " << desc << std::endl;
        
        simulateDrag(node, offset.x, offset.y);

        auto collisions = findAllSnapCollisions();
        if (!collisions.empty()) {
            totalCollisions += static_cast<int>(collisions.size());
            std::cout << "  WARNING: " << collisions.size() << " collision(s) detected!" << std::endl;
            printCollisionInfo(collisions);
            
            // Print all edges for debugging
            for (const auto& [edgeId, _] : edgeLayouts_) {
                printEdgeSnapInfo(edgeId, "    ");
            }
        }
    }

    std::cout << "\n--- Final state ---" << std::endl;
    for (const auto& [edgeId, _] : edgeLayouts_) {
        printEdgeSnapInfo(edgeId);
    }

    auto finalCollisions = findAllSnapCollisions();
    printCollisionInfo(finalCollisions);

    // This is the key assertion that should FAIL if the bug exists
    EXPECT_EQ(finalCollisions.size(), 0u)
        << "BUG: Edges share snap points after drag sequence!\n"
        << "This indicates recalculateSourceEdgesForMovedNodes() is not "
        << "recalculating ALL edges on affected NodeEdges.\n"
        << "Found " << finalCollisions.size() << " collision(s) at final state.";
}

/**
 * Test: Specific reproduction of user's bug report
 *
 * User reported:
 *   Edge 1 snap: src=(60,160) snapIdx=2 conn=1/2
 *   Edge 2 path: (60,160)->(60,340)->...
 *
 * Both edges share sourcePoint (60,160).
 */
TEST_F(SnapCollisionReproductionTest, UserReportedBug_Edge1And2ShareSourcePoint) {
    std::cout << "\n=== User Reported Bug Reproduction Test ===" << std::endl;

    doLayout();

    std::cout << "\n--- Initial layout ---" << std::endl;
    // Focus on edges that might collide (edges from same node)
    // Edge 1 (e_pause_): running -> paused
    // Edge 3 (e_stop1_): running -> stopped
    // Edge 5 (e_fail_): running -> error
    // These all exit from 'running' node and could share snap points
    
    printEdgeSnapInfo(e_pause_, "Edge 1 (pause): ");
    printEdgeSnapInfo(e_stop1_, "Edge 3 (stop1): ");
    printEdgeSnapInfo(e_fail_, "Edge 5 (fail): ");

    // Simulate the drag that caused the issue
    // The user's logs suggest s1 (running) was moved
    std::cout << "\n--- Simulating user's drag sequence ---" << std::endl;
    
    // Drag running node significantly
    simulateDrag(running_, 420, -140);
    
    std::cout << "\n--- After drag ---" << std::endl;
    printEdgeSnapInfo(e_pause_, "Edge 1 (pause): ");
    printEdgeSnapInfo(e_stop1_, "Edge 3 (stop1): ");
    printEdgeSnapInfo(e_fail_, "Edge 5 (fail): ");

    // Check for collisions among edges from 'running' node
    const auto& e1 = edgeLayouts_[e_pause_];
    const auto& e3 = edgeLayouts_[e_stop1_];
    const auto& e5 = edgeLayouts_[e_fail_];

    bool collision_1_3 = hasSourcePointCollision(e1, e3);
    bool collision_1_5 = hasSourcePointCollision(e1, e5);
    bool collision_3_5 = hasSourcePointCollision(e3, e5);

    if (collision_1_3) {
        std::cout << "COLLISION: Edge 1 and Edge 3 share sourcePoint!" << std::endl;
    }
    if (collision_1_5) {
        std::cout << "COLLISION: Edge 1 and Edge 5 share sourcePoint!" << std::endl;
    }
    if (collision_3_5) {
        std::cout << "COLLISION: Edge 3 and Edge 5 share sourcePoint!" << std::endl;
    }

    EXPECT_FALSE(collision_1_3)
        << "Edge 1 and Edge 3 should NOT share sourcePoint";
    EXPECT_FALSE(collision_1_5)
        << "Edge 1 and Edge 5 should NOT share sourcePoint";
    EXPECT_FALSE(collision_3_5)
        << "Edge 3 and Edge 5 should NOT share sourcePoint";
}

/**
 * Test: SCXML test144 - Multiple edges to same node should have unique snap points
 *
 * test144.scxml graph:
 *   s0 -> s1 (foo)
 *   s0 -> fail (*)
 *   s1 -> pass (bar)
 *   s1 -> fail (*)
 *
 * s0 and s1 both have edges to 'fail' node.
 * After dragging, these edges might share the same targetPoint on fail node.
 */
class SnapCollisionTest144 : public ::testing::Test {
protected:
    std::string resourcePath_;

    void SetUp() override {
        // Find SCXML resources directory
        std::vector<std::string> searchPaths = {
            "resources/scxml",
            "../resources/scxml",
            "../../resources/scxml",
            "/home/coin/arborvia/resources/scxml"
        };

        for (const auto& path : searchPaths) {
            if (std::filesystem::exists(path)) {
                resourcePath_ = path;
                break;
            }
        }
    }

    // Helper: Check snap point collision between edges
    // NOTE: Point nodes (size=0) are excluded - all edges SHOULD converge at center
    bool hasSnapCollision(const EdgeLayout& e1, const EdgeLayout& e2, 
                          const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
                          float tolerance = 1.0f) const {
        // Source collision: same source node and sourceEdge
        // Skip if source node is a Point node
        auto srcNodeIt = nodeLayouts.find(e1.from);
        bool srcIsPoint = (srcNodeIt != nodeLayouts.end() && srcNodeIt->second.isPointNode());
        
        bool srcCollision = false;
        if (!srcIsPoint) {
            srcCollision = (e1.from == e2.from && e1.sourceEdge == e2.sourceEdge &&
                           std::abs(e1.sourcePoint.x - e2.sourcePoint.x) < tolerance &&
                           std::abs(e1.sourcePoint.y - e2.sourcePoint.y) < tolerance);
        }

        // Target collision: same target node and targetEdge
        // Skip if target node is a Point node
        auto tgtNodeIt = nodeLayouts.find(e1.to);
        bool tgtIsPoint = (tgtNodeIt != nodeLayouts.end() && tgtNodeIt->second.isPointNode());
        
        bool tgtCollision = false;
        if (!tgtIsPoint) {
            tgtCollision = (e1.to == e2.to && e1.targetEdge == e2.targetEdge &&
                           std::abs(e1.targetPoint.x - e2.targetPoint.x) < tolerance &&
                           std::abs(e1.targetPoint.y - e2.targetPoint.y) < tolerance);
        }

        return srcCollision || tgtCollision;
    }

    // Helper: Find all collisions in a layout (excluding Point nodes)
    std::vector<std::pair<EdgeId, EdgeId>> findCollisions(
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) const {
        
        std::vector<std::pair<EdgeId, EdgeId>> collisions;
        std::vector<EdgeId> ids;
        for (const auto& [id, _] : edgeLayouts) {
            ids.push_back(id);
        }

        for (size_t i = 0; i < ids.size(); ++i) {
            for (size_t j = i + 1; j < ids.size(); ++j) {
                const auto& e1 = edgeLayouts.at(ids[i]);
                const auto& e2 = edgeLayouts.at(ids[j]);
                // Skip self-loops
                if (e1.from == e1.to || e2.from == e2.to) continue;
                if (hasSnapCollision(e1, e2, nodeLayouts)) {
                    collisions.push_back({ids[i], ids[j]});
                }
            }
        }
        return collisions;
    }

    void printEdgeInfo(EdgeId id, const EdgeLayout& el) const {
        std::cout << "  Edge " << id << ": from=" << el.from << " to=" << el.to
                  << " srcEdge=" << static_cast<int>(el.sourceEdge)
                  << " srcPt=(" << el.sourcePoint.x << "," << el.sourcePoint.y << ")"
                  << " tgtEdge=" << static_cast<int>(el.targetEdge)
                  << " tgtPt=(" << el.targetPoint.x << "," << el.targetPoint.y << ")"
                  << std::endl;
    }
};

/**
 * Test: test144.scxml - edges to 'fail' node should have unique snap points
 *
 * Bug scenario:
 * - s0 -> fail and s1 -> fail both target the 'fail' node
 * - If fail is a Point node, both edges should reach its center
 * - But they should have different snap distribution if on same targetEdge
 *
 * After dragging s1, its edge to fail might change NodeEdge and collide with s0's edge.
 */
TEST_F(SnapCollisionTest144, Test144_EdgesToFailNode_NoSnapCollision) {
    if (resourcePath_.empty()) {
        GTEST_SKIP() << "SCXML resources directory not found";
    }

    SCXMLTestLoader loader(resourcePath_);
    ASSERT_TRUE(loader.loadIndex()) << loader.getLastError();

    auto graph = loader.loadGraphById("144");
    ASSERT_NE(graph, nullptr) << loader.getLastError();

    // Find nodes
    auto s0Opt = graph->findByScxmlId("s0");
    auto s1Opt = graph->findByScxmlId("s1");
    auto failOpt = graph->findByScxmlId("fail");
    
    ASSERT_TRUE(s0Opt.has_value()) << "s0 not found";
    ASSERT_TRUE(s1Opt.has_value()) << "s1 not found";
    ASSERT_TRUE(failOpt.has_value()) << "fail not found";

    NodeId s0 = *s0Opt;
    NodeId s1 = *s1Opt;
    NodeId fail = *failOpt;

    // Find edges to fail
    auto s0ToFailOpt = graph->findEdge(s0, fail);
    auto s1ToFailOpt = graph->findEdge(s1, fail);

    ASSERT_TRUE(s0ToFailOpt.has_value()) << "s0->fail edge not found";
    ASSERT_TRUE(s1ToFailOpt.has_value()) << "s1->fail edge not found";

    EdgeId e_s0_fail = *s0ToFailOpt;
    EdgeId e_s1_fail = *s1ToFailOpt;

    // Initial layout
    SugiyamaLayout layout;
    LayoutOptions options;
    options.direction = Direction::TopToBottom;
    options.gridConfig.cellSize = 20.0f;
    options.autoSnapPoints = true;
    layout.setOptions(options);

    auto manager = std::make_shared<ManualLayoutManager>();
    layout.setManualLayoutManager(manager);

    LayoutResult result = layout.layout(*graph);

    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;
    for (const auto& [id, nl] : result.nodeLayouts()) {
        nodeLayouts[id] = nl;
    }
    for (const auto& [id, el] : result.edgeLayouts()) {
        edgeLayouts[id] = el;
    }

    std::cout << "\n=== Test144 Initial Layout ===" << std::endl;
    for (const auto& [id, el] : edgeLayouts) {
        printEdgeInfo(id, el);
    }

    // Check initial collisions (excluding Point nodes where edges should converge)
    auto initialCollisions = findCollisions(edgeLayouts, nodeLayouts);
    std::cout << "Initial collisions: " << initialCollisions.size() << std::endl;

    // Now simulate drag that causes NodeEdge change
    // Drag s1 to trigger edge recalculation
    std::cout << "\n--- Dragging s1 by (200, -100) ---" << std::endl;

    Point s1NewPos = nodeLayouts[s1].position;
    s1NewPos.x += 200;
    s1NewPos.y -= 100;

    LayoutController controller(*graph, options);
    controller.initializeFrom(nodeLayouts, edgeLayouts);
    auto moveResult = controller.moveNode(s1, s1NewPos);

    if (moveResult.success) {
        for (const auto& [id, nl] : controller.nodeLayouts()) {
            nodeLayouts[id] = nl;
        }
        for (const auto& [id, el] : controller.edgeLayouts()) {
            edgeLayouts[id] = el;
        }
    }

    std::cout << "\n=== After Drag ===" << std::endl;
    for (const auto& [id, el] : edgeLayouts) {
        printEdgeInfo(id, el);
    }

    // Check for snap collisions (excluding Point nodes)
    auto finalCollisions = findCollisions(edgeLayouts, nodeLayouts);

    // Focus on the two edges to fail node
    const auto& el_s0_fail = edgeLayouts[e_s0_fail];
    const auto& el_s1_fail = edgeLayouts[e_s1_fail];

    std::cout << "\n=== Edges to 'fail' node ===" << std::endl;
    printEdgeInfo(e_s0_fail, el_s0_fail);
    printEdgeInfo(e_s1_fail, el_s1_fail);

    // Check if fail node is a Point node (all edges to Point nodes should converge at center)
    auto failNodeIt = nodeLayouts.find(fail);
    bool failIsPoint = (failNodeIt != nodeLayouts.end() && failNodeIt->second.isPointNode());

    if (failIsPoint) {
        std::cout << "Note: 'fail' is a Point node - edges should converge at center (no collision check)" << std::endl;
    }

    // Check if they collide on targetPoint (only for non-Point nodes)
    bool targetCollision = false;
    if (!failIsPoint) {
        targetCollision = (el_s0_fail.to == el_s1_fail.to &&
                          el_s0_fail.targetEdge == el_s1_fail.targetEdge &&
                          std::abs(el_s0_fail.targetPoint.x - el_s1_fail.targetPoint.x) < 1.0f &&
                          std::abs(el_s0_fail.targetPoint.y - el_s1_fail.targetPoint.y) < 1.0f);

        if (targetCollision) {
            std::cout << "\nBUG DETECTED: s0->fail and s1->fail share targetPoint!" << std::endl;
            std::cout << "  s0->fail targetPoint: (" << el_s0_fail.targetPoint.x << "," << el_s0_fail.targetPoint.y << ")" << std::endl;
            std::cout << "  s1->fail targetPoint: (" << el_s1_fail.targetPoint.x << "," << el_s1_fail.targetPoint.y << ")" << std::endl;
        }
    }

    EXPECT_FALSE(targetCollision)
        << "BUG: s0->fail and s1->fail share the same targetPoint!\n"
        << "This happens when recalculateSourceEdgesForMovedNodes() doesn't "
        << "recalculate ALL edges on the affected NodeEdge.";
}

/**
 * Test: test144 with aggressive drag sequence
 */
TEST_F(SnapCollisionTest144, Test144_AggressiveDrag_NoSnapCollision) {
    if (resourcePath_.empty()) {
        GTEST_SKIP() << "SCXML resources directory not found";
    }

    SCXMLTestLoader loader(resourcePath_);
    ASSERT_TRUE(loader.loadIndex()) << loader.getLastError();

    auto graph = loader.loadGraphById("144");
    ASSERT_NE(graph, nullptr) << loader.getLastError();

    // Layout
    SugiyamaLayout layout;
    LayoutOptions options;
    options.direction = Direction::TopToBottom;
    options.gridConfig.cellSize = 20.0f;
    options.autoSnapPoints = true;
    layout.setOptions(options);

    auto manager = std::make_shared<ManualLayoutManager>();
    layout.setManualLayoutManager(manager);

    LayoutResult result = layout.layout(*graph);

    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;
    for (const auto& [id, nl] : result.nodeLayouts()) {
        nodeLayouts[id] = nl;
    }
    for (const auto& [id, el] : result.edgeLayouts()) {
        edgeLayouts[id] = el;
    }

    // Find nodes by scxml id
    auto s0Opt = graph->findByScxmlId("s0");
    auto s1Opt = graph->findByScxmlId("s1");
    ASSERT_TRUE(s0Opt.has_value() && s1Opt.has_value());
    NodeId s0 = *s0Opt;
    NodeId s1 = *s1Opt;

    std::cout << "\n=== Test144 Aggressive Drag Test ===" << std::endl;

    // Drag sequence
    std::vector<std::tuple<std::string, NodeId, Point>> sequence = {
        {"s1 +300,0", s1, {300, 0}},
        {"s1 0,-150", s1, {0, -150}},
        {"s0 +200,0", s0, {200, 0}},
        {"s1 -100,+200", s1, {-100, 200}},
    };

    int collisionCount = 0;

    for (const auto& [desc, nodeId, offset] : sequence) {
        std::cout << "\n--- " << desc << " ---" << std::endl;

        Point newPos = nodeLayouts[nodeId].position;
        newPos.x += offset.x;
        newPos.y += offset.y;

        LayoutController controller(*graph, options);
        controller.initializeFrom(nodeLayouts, edgeLayouts);
        auto moveResult = controller.moveNode(nodeId, newPos);

        if (moveResult.success) {
            for (const auto& [id, nl] : controller.nodeLayouts()) {
                nodeLayouts[id] = nl;
            }
            for (const auto& [id, el] : controller.edgeLayouts()) {
                edgeLayouts[id] = el;
            }
            manager->setNodePosition(nodeId, moveResult.actualPosition);
        }

        // Check for collisions (excluding Point nodes)
        auto collisions = findCollisions(edgeLayouts, nodeLayouts);
        if (!collisions.empty()) {
            collisionCount += static_cast<int>(collisions.size());
            std::cout << "  COLLISION: " << collisions.size() << " collision(s) found!" << std::endl;
            for (const auto& [e1, e2] : collisions) {
                printEdgeInfo(e1, edgeLayouts[e1]);
                printEdgeInfo(e2, edgeLayouts[e2]);
            }
        }
    }

    // Final check (excluding Point nodes)
    auto finalCollisions = findCollisions(edgeLayouts, nodeLayouts);

    std::cout << "\n=== Final State ===" << std::endl;
    for (const auto& [id, el] : edgeLayouts) {
        printEdgeInfo(id, el);
    }

    EXPECT_EQ(finalCollisions.size(), 0u)
        << "BUG: Found " << finalCollisions.size() << " snap point collision(s) after drag sequence!\n"
        << "This indicates recalculateSourceEdgesForMovedNodes() is not properly "
        << "distributing snap points when edges join the same NodeEdge.";
}

/**
 * =============================================================================
 * TDD Red Test: Interactive Demo Snap Collision Bug
 * =============================================================================
 *
 * This test reproduces the EXACT bug observed in the running interactive demo:
 *
 * Current demo state (verified via get_layout API):
 *   Node 1 (Running): pos=(0,100) size=(120,60)
 *   Edge 1: 1->2 srcEdge=bottom srcPt=(60,160)
 *   Edge 2: 1->4 srcEdge=bottom srcPt=(60,160)  <-- SAME! BUG!
 *
 * Two edges from the same normal node (size > 0) share identical sourcePoint.
 * This causes A* pathfinding to fail because both paths start from the same cell.
 *
 * Expected behavior:
 *   Edge 1 and Edge 2 should have DIFFERENT sourcePoints when they share
 *   the same sourceEdge on a normal-sized node.
 */
class DemoSnapCollisionTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Exact graph structure from interactive demo
        // Node sizes: some are Point nodes (0,0), some are normal (120,60)
        node0_ = graph_.addNode(Size{0, 0}, "Node0");      // Point node
        node1_ = graph_.addNode(Size{120, 60}, "Node1");   // Normal node (Running)
        node2_ = graph_.addNode(Size{120, 60}, "Node2");   // Normal node (Paused)
        node3_ = graph_.addNode(Size{0, 0}, "Node3");      // Point node
        node4_ = graph_.addNode(Size{0, 0}, "Node4");      // Point node

        // Edges matching demo
        edge0_ = graph_.addEdge(node0_, node1_, "e0");  // 0->1
        edge1_ = graph_.addEdge(node1_, node2_, "e1");  // 1->2 (Running -> Paused)
        edge2_ = graph_.addEdge(node1_, node4_, "e2");  // 1->4 (Running -> Error)
        edge3_ = graph_.addEdge(node2_, node3_, "e3");  // 2->3
        edge4_ = graph_.addEdge(node2_, node4_, "e4");  // 2->4

        options_.direction = Direction::TopToBottom;
        options_.gridConfig.cellSize = 20.0f;
        options_.autoSnapPoints = true;

        manager_ = std::make_shared<ManualLayoutManager>();
    }

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

    // Check if two edges share the same snap point
    bool haveSnapCollision(const EdgeLayout& e1, const EdgeLayout& e2, float tolerance = 1.0f) const {
        // Source collision: same source node and sourceEdge
        if (e1.from == e2.from && e1.sourceEdge == e2.sourceEdge) {
            float dx = std::abs(e1.sourcePoint.x - e2.sourcePoint.x);
            float dy = std::abs(e1.sourcePoint.y - e2.sourcePoint.y);
            if (dx < tolerance && dy < tolerance) {
                return true;
            }
        }
        // Target collision: same target node and targetEdge
        if (e1.to == e2.to && e1.targetEdge == e2.targetEdge) {
            float dx = std::abs(e1.targetPoint.x - e2.targetPoint.x);
            float dy = std::abs(e1.targetPoint.y - e2.targetPoint.y);
            if (dx < tolerance && dy < tolerance) {
                // For Point nodes (size=0), collision at center is expected
                auto targetIt = nodeLayouts_.find(e1.to);
                if (targetIt != nodeLayouts_.end()) {
                    const auto& targetNode = targetIt->second;
                    if (targetNode.size.width > 0 && targetNode.size.height > 0) {
                        return true;  // Only flag as collision for normal nodes
                    }
                }
            }
        }
        return false;
    }

    void printEdge(EdgeId id, const EdgeLayout& el) const {
        std::cout << "  Edge " << id << ": " << el.from << "->" << el.to
                  << " srcEdge=" << static_cast<int>(el.sourceEdge)
                  << " srcPt=(" << el.sourcePoint.x << "," << el.sourcePoint.y << ")"
                  << " tgtEdge=" << static_cast<int>(el.targetEdge)
                  << " tgtPt=(" << el.targetPoint.x << "," << el.targetPoint.y << ")"
                  << std::endl;
    }

    void printNode(NodeId id, const NodeLayout& nl) const {
        std::cout << "  Node " << id << ": pos=(" << nl.position.x << "," << nl.position.y << ")"
                  << " size=(" << nl.size.width << "," << nl.size.height << ")"
                  << std::endl;
    }

    Graph graph_;
    NodeId node0_, node1_, node2_, node3_, node4_;
    EdgeId edge0_, edge1_, edge2_, edge3_, edge4_;
    LayoutOptions options_;
    std::shared_ptr<ManualLayoutManager> manager_;
    std::unordered_map<NodeId, NodeLayout> nodeLayouts_;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts_;
};

/**
 * TDD Red Test: Edges from same normal node must have different sourcePoints
 *
 * Bug scenario from demo:
 *   Edge 1 (1->2): srcEdge=bottom, srcPt=(60,160)
 *   Edge 2 (1->4): srcEdge=bottom, srcPt=(60,160)
 *
 * Node 1 is a normal node (size=120x60), not a Point node.
 * Therefore, edges should be distributed along the bottom edge with different snap points.
 *
 * This test MUST FAIL initially (TDD Red) to prove the bug exists.
 */
TEST_F(DemoSnapCollisionTest, EdgesFromSameNormalNode_MustHaveDifferentSourcePoints) {
    std::cout << "\n=== TDD Red: Demo Snap Collision Bug ===" << std::endl;

    doLayout();

    std::cout << "\n--- Nodes ---" << std::endl;
    for (const auto& [id, nl] : nodeLayouts_) {
        printNode(id, nl);
    }

    std::cout << "\n--- Edges ---" << std::endl;
    for (const auto& [id, el] : edgeLayouts_) {
        printEdge(id, el);
    }

    // Find edges from node1_ (the normal-sized node)
    std::vector<EdgeId> edgesFromNode1;
    for (const auto& [id, el] : edgeLayouts_) {
        if (el.from == node1_) {
            edgesFromNode1.push_back(id);
        }
    }

    std::cout << "\n--- Edges from Node 1 (size=120x60) ---" << std::endl;
    for (EdgeId id : edgesFromNode1) {
        printEdge(id, edgeLayouts_[id]);
    }

    // Check for snap collisions among edges from node1_
    bool foundCollision = false;
    for (size_t i = 0; i < edgesFromNode1.size(); ++i) {
        for (size_t j = i + 1; j < edgesFromNode1.size(); ++j) {
            EdgeId e1 = edgesFromNode1[i];
            EdgeId e2 = edgesFromNode1[j];
            const auto& el1 = edgeLayouts_[e1];
            const auto& el2 = edgeLayouts_[e2];

            // Only check if they share the same sourceEdge
            if (el1.sourceEdge == el2.sourceEdge) {
                float dx = std::abs(el1.sourcePoint.x - el2.sourcePoint.x);
                float dy = std::abs(el1.sourcePoint.y - el2.sourcePoint.y);
                bool sameSourcePoint = (dx < 1.0f && dy < 1.0f);

                if (sameSourcePoint) {
                    foundCollision = true;
                    std::cout << "\n  BUG DETECTED!" << std::endl;
                    std::cout << "  Edge " << e1 << " and Edge " << e2
                              << " share same sourceEdge=" << static_cast<int>(el1.sourceEdge)
                              << " and same sourcePoint=(" << el1.sourcePoint.x
                              << "," << el1.sourcePoint.y << ")" << std::endl;
                }
            }
        }
    }

    // Verify node1_ is a normal node (not Point node)
    const auto& node1Layout = nodeLayouts_[node1_];
    ASSERT_GT(node1Layout.size.width, 0) << "Node 1 should be a normal node, not Point node";
    ASSERT_GT(node1Layout.size.height, 0) << "Node 1 should be a normal node, not Point node";

    // THE KEY ASSERTION: Edges from normal node must have different sourcePoints
    EXPECT_FALSE(foundCollision)
        << "BUG: Edges from Node 1 (normal node, size=" << node1Layout.size.width
        << "x" << node1Layout.size.height << ") share the same sourcePoint!\n"
        << "Expected: Edges on same NodeEdge should have different snap points.\n"
        << "Root cause: recalculateSourceEdgesForMovedNodes() or snap distribution bug.";
}

/**
 * TDD Red Test: After node drag, sibling edges must be redistributed
 *
 * This test simulates the actual bug trigger:
 * 1. Initial layout: edges correctly distributed
 * 2. Drag a node that affects edge routing
 * 3. After drag, sibling edges on same NodeEdge should still have different snap points
 *
 * Bug: recalculateSourceEdgesForMovedNodes() only recalculates edges of moved nodes,
 * not ALL edges on affected NodeEdges. When one edge joins another's NodeEdge after
 * a drag, the existing edge is not redistributed, causing snap collision.
 */
TEST_F(DemoSnapCollisionTest, AfterNodeDrag_SiblingEdges_MustBeRedistributed) {
    std::cout << "\n=== TDD Red: After Node Drag, Sibling Edges Must Be Redistributed ===" << std::endl;

    doLayout();

    std::cout << "\n--- Initial Layout ---" << std::endl;
    std::cout << "Nodes:" << std::endl;
    for (const auto& [id, nl] : nodeLayouts_) {
        printNode(id, nl);
    }

    std::cout << "\nEdges from Node 1:" << std::endl;
    const auto& el1 = edgeLayouts_[edge1_];  // 1->2
    const auto& el2 = edgeLayouts_[edge2_];  // 1->4
    printEdge(edge1_, el1);
    printEdge(edge2_, el2);

    // Verify initial state: edges should have different sourcePoints
    bool initialCollision = false;
    if (el1.sourceEdge == el2.sourceEdge) {
        float dx = std::abs(el1.sourcePoint.x - el2.sourcePoint.x);
        float dy = std::abs(el1.sourcePoint.y - el2.sourcePoint.y);
        initialCollision = (dx < 1.0f && dy < 1.0f);
    }
    ASSERT_FALSE(initialCollision) << "Initial layout should have no collision";
    std::cout << "Initial state: NO collision (edges correctly distributed)" << std::endl;

    // Now simulate a node drag that triggers recalculateSourceEdgesForMovedNodes
    std::cout << "\n--- Simulating Node Drag ---" << std::endl;

    // Move node2 (Paused) significantly to trigger edge recalculation
    Point node2NewPos = nodeLayouts_[node2_].position;
    node2NewPos.x += 300;  // Move right
    node2NewPos.y -= 100;  // Move up

    std::cout << "Dragging node2 from (" << nodeLayouts_[node2_].position.x
              << "," << nodeLayouts_[node2_].position.y << ") to ("
              << node2NewPos.x << "," << node2NewPos.y << ")" << std::endl;

    // Use LayoutController to move node (this calls recalculateSourceEdgesForMovedNodes)
    LayoutController controller(graph_, options_);
    controller.initializeFrom(nodeLayouts_, edgeLayouts_);
    auto moveResult = controller.moveNode(node2_, node2NewPos);

    if (moveResult.success) {
        for (const auto& [id, nl] : controller.nodeLayouts()) {
            nodeLayouts_[id] = nl;
        }
        for (const auto& [id, el] : controller.edgeLayouts()) {
            edgeLayouts_[id] = el;
        }
        manager_->setNodePosition(node2_, moveResult.actualPosition);
        std::cout << "Move result: SUCCESS, actual position=("
                  << moveResult.actualPosition.x << "," << moveResult.actualPosition.y << ")" << std::endl;
    } else {
        std::cout << "Move result: FAILED - " << moveResult.reason << std::endl;
    }

    // Second drag: move node4 (Error) to force edges to reorganize
    Point node4NewPos = nodeLayouts_[node4_].position;
    node4NewPos.x -= 200;  // Move left
    node4NewPos.y += 50;   // Move down

    std::cout << "\nDragging node4 from (" << nodeLayouts_[node4_].position.x
              << "," << nodeLayouts_[node4_].position.y << ") to ("
              << node4NewPos.x << "," << node4NewPos.y << ")" << std::endl;

    LayoutController controller2(graph_, options_);
    controller2.initializeFrom(nodeLayouts_, edgeLayouts_);
    auto moveResult2 = controller2.moveNode(node4_, node4NewPos);

    if (moveResult2.success) {
        for (const auto& [id, nl] : controller2.nodeLayouts()) {
            nodeLayouts_[id] = nl;
        }
        for (const auto& [id, el] : controller2.edgeLayouts()) {
            edgeLayouts_[id] = el;
        }
        manager_->setNodePosition(node4_, moveResult2.actualPosition);
        std::cout << "Move result: SUCCESS" << std::endl;
    }

    // Third drag: move node1 (Running) to trigger its edges recalculation
    Point node1NewPos = nodeLayouts_[node1_].position;
    node1NewPos.x += 150;  // Move right

    std::cout << "\nDragging node1 from (" << nodeLayouts_[node1_].position.x
              << "," << nodeLayouts_[node1_].position.y << ") to ("
              << node1NewPos.x << "," << node1NewPos.y << ")" << std::endl;

    LayoutController controller3(graph_, options_);
    controller3.initializeFrom(nodeLayouts_, edgeLayouts_);
    auto moveResult3 = controller3.moveNode(node1_, node1NewPos);

    if (moveResult3.success) {
        for (const auto& [id, nl] : controller3.nodeLayouts()) {
            nodeLayouts_[id] = nl;
        }
        for (const auto& [id, el] : controller3.edgeLayouts()) {
            edgeLayouts_[id] = el;
        }
        manager_->setNodePosition(node1_, moveResult3.actualPosition);
        std::cout << "Move result: SUCCESS" << std::endl;
    }

    std::cout << "\n--- After Drag Sequence ---" << std::endl;
    std::cout << "Nodes:" << std::endl;
    for (const auto& [id, nl] : nodeLayouts_) {
        printNode(id, nl);
    }

    std::cout << "\nEdges from Node 1:" << std::endl;
    const auto& el1_after = edgeLayouts_[edge1_];  // 1->2
    const auto& el2_after = edgeLayouts_[edge2_];  // 1->4
    printEdge(edge1_, el1_after);
    printEdge(edge2_, el2_after);

    // Check for collision after drag
    bool afterDragCollision = false;
    if (el1_after.sourceEdge == el2_after.sourceEdge) {
        float dx = std::abs(el1_after.sourcePoint.x - el2_after.sourcePoint.x);
        float dy = std::abs(el1_after.sourcePoint.y - el2_after.sourcePoint.y);
        afterDragCollision = (dx < 1.0f && dy < 1.0f);

        if (afterDragCollision) {
            std::cout << "\nBUG DETECTED: After drag, edges share same sourcePoint!" << std::endl;
            std::cout << "  Edge " << edge1_ << ": srcPt=(" << el1_after.sourcePoint.x
                      << "," << el1_after.sourcePoint.y << ")" << std::endl;
            std::cout << "  Edge " << edge2_ << ": srcPt=(" << el2_after.sourcePoint.x
                      << "," << el2_after.sourcePoint.y << ")" << std::endl;
        }
    }

    // THE KEY ASSERTION: After drag, sibling edges must still have different snap points
    EXPECT_FALSE(afterDragCollision)
        << "BUG: After node drag, Edge " << edge1_ << " and Edge " << edge2_
        << " share the same sourcePoint!\n"
        << "This proves recalculateSourceEdgesForMovedNodes() is not redistributing "
        << "ALL sibling edges on affected NodeEdges.\n"
        << "Expected: When one edge's path changes to join another's NodeEdge, "
        << "both edges should be redistributed with different snap points.";
}

/**
 * TDD Red Test: Exact reproduction of demo collision bug
 *
 * Demo state when collision was observed:
 * - Node 2 was moved to (400, 520)
 * - Edge 1 (1->2) and Edge 2 (1->4) both had srcPt=(60,160)
 *
 * This test reproduces the exact scenario by moving Node 2 significantly
 * to trigger edge direction recalculation and snap point collision.
 */
TEST_F(DemoSnapCollisionTest, ExactDemoScenario_Node2MovedFarRight) {
    std::cout << "\n=== TDD Red: Exact Demo Scenario Reproduction ===" << std::endl;

    doLayout();

    std::cout << "\n--- Initial Layout ---" << std::endl;
    for (const auto& [id, nl] : nodeLayouts_) {
        printNode(id, nl);
    }
    std::cout << "\nEdges from Node 1:" << std::endl;
    printEdge(edge1_, edgeLayouts_[edge1_]);
    printEdge(edge2_, edgeLayouts_[edge2_]);

    // Verify initial state has no collision
    const auto& el1_init = edgeLayouts_[edge1_];
    const auto& el2_init = edgeLayouts_[edge2_];
    bool initCollision = (el1_init.sourceEdge == el2_init.sourceEdge &&
                          std::abs(el1_init.sourcePoint.x - el2_init.sourcePoint.x) < 1.0f &&
                          std::abs(el1_init.sourcePoint.y - el2_init.sourcePoint.y) < 1.0f);
    ASSERT_FALSE(initCollision) << "Initial layout should have no collision";

    // Simulate demo scenario: Move Node 2 far right (to 400, 520)
    // This matches the demo's observed state when collision occurred
    std::cout << "\n--- Moving Node 2 to match demo state (400, 520) ---" << std::endl;
    Point targetPos{400, 520};
    Point currentPos = nodeLayouts_[node2_].position;

    // Move in large step to approximate demo's final position
    LayoutController controller(graph_, options_);
    controller.initializeFrom(nodeLayouts_, edgeLayouts_);
    auto moveResult = controller.moveNode(node2_, targetPos);

    std::cout << "Move result: " << (moveResult.success ? "SUCCESS" : "FAILED") << std::endl;
    if (!moveResult.success) {
        std::cout << "Reason: " << moveResult.reason << std::endl;
    } else {
        std::cout << "Actual position: (" << moveResult.actualPosition.x
                  << ", " << moveResult.actualPosition.y << ")" << std::endl;
    }

    if (moveResult.success) {
        for (const auto& [id, nl] : controller.nodeLayouts()) {
            nodeLayouts_[id] = nl;
        }
        for (const auto& [id, el] : controller.edgeLayouts()) {
            edgeLayouts_[id] = el;
        }
        manager_->setNodePosition(node2_, moveResult.actualPosition);
    }

    std::cout << "\n--- State after Node 2 move ---" << std::endl;
    for (const auto& [id, nl] : nodeLayouts_) {
        printNode(id, nl);
    }
    std::cout << "\nEdges from Node 1:" << std::endl;
    printEdge(edge1_, edgeLayouts_[edge1_]);
    printEdge(edge2_, edgeLayouts_[edge2_]);

    // Check for collision (THE BUG)
    const auto& el1_after = edgeLayouts_[edge1_];
    const auto& el2_after = edgeLayouts_[edge2_];

    bool afterCollision = false;
    if (el1_after.sourceEdge == el2_after.sourceEdge) {
        float dx = std::abs(el1_after.sourcePoint.x - el2_after.sourcePoint.x);
        float dy = std::abs(el1_after.sourcePoint.y - el2_after.sourcePoint.y);
        afterCollision = (dx < 1.0f && dy < 1.0f);

        if (afterCollision) {
            std::cout << "\nBUG DETECTED: Edges share same sourcePoint!" << std::endl;
            std::cout << "  Edge " << edge1_ << ": srcPt=(" << el1_after.sourcePoint.x
                      << "," << el1_after.sourcePoint.y << ")" << std::endl;
            std::cout << "  Edge " << edge2_ << ": srcPt=(" << el2_after.sourcePoint.x
                      << "," << el2_after.sourcePoint.y << ")" << std::endl;
        }
    }

    // This MUST FAIL for TDD Red if the bug exists
    EXPECT_FALSE(afterCollision)
        << "BUG: After moving Node 2, Edge " << edge1_ << " and Edge " << edge2_
        << " share the same sourcePoint at (" << el1_after.sourcePoint.x
        << "," << el1_after.sourcePoint.y << ")!\n"
        << "This reproduces the exact bug observed in interactive demo.";
}

/**
 * TDD Red Test: Direct Optimizer Call
 *
 * This test reproduces the EXACT demo bug path:
 * 1. Demo's startAsyncOptimization() calls LayoutUtils::updateEdgePositions()
 * 2. Which calls DragOptimizationHandler::updateEdgeRoutingWithOptimization()
 * 3. Which calls optimizer->optimize() with movedNodes set
 * 4. AStarEdgeOptimizer::optimize() calls recalculateSourceEdgesForMovedNodes()
 * 5. This function bypasses SnapPositionUpdater and directly assigns snap points
 *
 * The bug: recalculateSourceEdgesForMovedNodes() uses unstable unordered_map
 * iteration order for connection index, causing multiple edges to get the
 * same snap index and thus the same snap point.
 *
 * This test MUST FAIL (TDD Red) to prove the bug exists in the demo path.
 */
TEST_F(DemoSnapCollisionTest, DirectOptimizerCall_MustNotHaveSnapCollision) {
    std::cout << "\n=== TDD Red: Direct Optimizer Call Test ===" << std::endl;

    doLayout();

    std::cout << "\n--- Initial Layout ---" << std::endl;
    for (const auto& [id, nl] : nodeLayouts_) {
        printNode(id, nl);
    }
    std::cout << "\nEdges from Node 1:" << std::endl;
    printEdge(edge1_, edgeLayouts_[edge1_]);
    printEdge(edge2_, edgeLayouts_[edge2_]);

    // Store initial state
    const auto& el1_init = edgeLayouts_[edge1_];
    const auto& el2_init = edgeLayouts_[edge2_];
    std::cout << "Initial snap points: e1=(" << el1_init.sourcePoint.x << ","
              << el1_init.sourcePoint.y << ") e2=(" << el2_init.sourcePoint.x
              << "," << el2_init.sourcePoint.y << ")" << std::endl;

    // Move node2 to match demo scenario (simulates drag)
    Point oldPos = nodeLayouts_[node2_].position;
    nodeLayouts_[node2_].position = Point{400, 520};  // Demo's final position
    std::cout << "\nMoved Node 2: (" << oldPos.x << "," << oldPos.y
              << ") -> (400, 520)" << std::endl;

    // Create AStar optimizer (same as demo path)
    OptimizerConfig config = OptimizerConfig::aggressive();
    config.preserveDirections = false;
    config.penaltySystem = EdgePenaltySystem::createDefault();
    auto optimizer = OptimizerRegistry::instance().create("AStar", config);
    ASSERT_NE(optimizer, nullptr) << "Failed to create AStar optimizer";

    // Collect all edges
    std::vector<EdgeId> allEdges;
    for (const auto& [id, _] : edgeLayouts_) {
        allEdges.push_back(id);
    }

    // Call optimizer->optimize() directly (this is the demo path!)
    // movedNodes = {node2_} triggers recalculateSourceEdgesForMovedNodes()
    std::unordered_set<NodeId> movedNodes = {node2_};
    float gridSize = options_.gridConfig.cellSize;

    std::cout << "\nCalling optimizer->optimize() with movedNodes={" << node2_ << "}..." << std::endl;
    auto optimizedLayouts = optimizer->optimize(
        allEdges, edgeLayouts_, nodeLayouts_, gridSize, movedNodes);

    // Apply optimized layouts
    for (const auto& [id, layout] : optimizedLayouts) {
        edgeLayouts_[id] = layout;
    }

    std::cout << "\n--- After Optimizer ---" << std::endl;
    std::cout << "Edges from Node 1 (sourceEdge=bottom):" << std::endl;
    printEdge(edge1_, edgeLayouts_[edge1_]);
    printEdge(edge2_, edgeLayouts_[edge2_]);

    // Check for snap collision (THE BUG)
    const auto& el1_after = edgeLayouts_[edge1_];
    const auto& el2_after = edgeLayouts_[edge2_];

    bool hasCollision = false;
    if (el1_after.sourceEdge == el2_after.sourceEdge) {
        float dx = std::abs(el1_after.sourcePoint.x - el2_after.sourcePoint.x);
        float dy = std::abs(el1_after.sourcePoint.y - el2_after.sourcePoint.y);
        hasCollision = (dx < 1.0f && dy < 1.0f);

        if (hasCollision) {
            std::cout << "\nBUG DETECTED: Direct optimizer call caused snap collision!" << std::endl;
            std::cout << "  Edge " << edge1_ << ": srcPt=(" << el1_after.sourcePoint.x
                      << "," << el1_after.sourcePoint.y << ")" << std::endl;
            std::cout << "  Edge " << edge2_ << ": srcPt=(" << el2_after.sourcePoint.x
                      << "," << el2_after.sourcePoint.y << ")" << std::endl;
        } else {
            std::cout << "\nNo collision detected (snap points are different)" << std::endl;
        }
    }

    // TDD Red: This test MUST FAIL if the bug exists
    EXPECT_FALSE(hasCollision)
        << "BUG: Direct optimizer call caused snap collision!\n"
        << "Edge " << edge1_ << " and Edge " << edge2_ << " share sourcePoint ("
        << el1_after.sourcePoint.x << "," << el1_after.sourcePoint.y << ")\n"
        << "This reproduces the exact bug in demo's startAsyncOptimization() path.";
}
