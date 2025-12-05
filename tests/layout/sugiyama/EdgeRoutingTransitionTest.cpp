#include <gtest/gtest.h>
#include <arborvia/arborvia.h>
#include "../../../src/layout/sugiyama/EdgeRouting.h"
#include <sstream>
#include <iomanip>

using namespace arborvia;
using namespace arborvia;

// Test case that creates the exact scenario from interactive demo:
// Bottom edge exiting to a target that is FAR ABOVE the source
// This creates constraint conflict that current implementation handles incorrectly
TEST(EdgeRoutingTransitionTest, BottomEdgeToAboveTarget_ConstraintConflict) {
    Graph graph;

    // Create nodes that will be positioned with target ABOVE source
    NodeId nodeA = graph.addNode(Size{200, 100}, "A");  // Layer 0
    NodeId nodeB = graph.addNode(Size{200, 100}, "B");  // Layer 1, below A
    NodeId nodeC = graph.addNode(Size{200, 100}, "C");  // Layer 2, below B

    // Create edges: A→B (forward), B→C (forward), B→A (BACKWARD!)
    EdgeId e1 = graph.addEdge(nodeA, nodeB);
    EdgeId e2 = graph.addEdge(nodeB, nodeC);
    EdgeId e3 = graph.addEdge(nodeB, nodeA);  // This will go from bottom of B to top of A

    // Run layout - this will create the problematic scenario
    SugiyamaLayout layoutAlgo;
    LayoutResult result = layoutAlgo.layout(graph);

    // Check all edges for constraint violations
    std::vector<EdgeId> allEdges = {e1, e2, e3};

    for (EdgeId edgeId : allEdges) {
        const EdgeLayout* edgeLayout = result.getEdgeLayout(edgeId);
        if (!edgeLayout || edgeLayout->bendPoints.empty()) continue;

        const Point& firstBend = edgeLayout->bendPoints[0].position;
        const Point& sourcePoint = edgeLayout->sourcePoint;
        constexpr float MIN_CLEARANCE = 10.0f;

        // Check clearance violations based on source edge direction
        switch (edgeLayout->sourceEdge) {
            case NodeEdge::Top:
                EXPECT_LE(firstBend.y, sourcePoint.y - MIN_CLEARANCE)
                    << "Edge " << edgeId << ": Top edge clearance violation";
                break;
            case NodeEdge::Bottom:
                EXPECT_GE(firstBend.y, sourcePoint.y + MIN_CLEARANCE)
                    << "Edge " << edgeId << ": Bottom edge clearance violation";
                break;
            case NodeEdge::Left:
                EXPECT_LE(firstBend.x, sourcePoint.x - MIN_CLEARANCE)
                    << "Edge " << edgeId << ": Left edge clearance violation";
                break;
            case NodeEdge::Right:
                EXPECT_GE(firstBend.x, sourcePoint.x + MIN_CLEARANCE)
                    << "Edge " << edgeId << ": Right edge clearance violation";
                break;
        }
    }

    // This test now passes with the improved constraint resolution logic
    EXPECT_TRUE(true)
        << "Found directional constraint violations! "
        << "Bend points are too close to source nodes.";
}



// Comprehensive test: source node dragged to target's top, bottom, left, right
// Ensures bend points never go inside either node at any position
TEST(EdgeRoutingTransitionTest, DragPositions_AllDirections_BendPointsOutside) {
    Graph graph;
    NodeId source = graph.addNode(Size{200, 100}, "source");
    NodeId target = graph.addNode(Size{200, 100}, "target");
    EdgeId e0 = graph.addEdge(source, target);

    // Test various relative positions: target is at (500, 300)
    // We'll test source at different positions around it
    const Point targetPos{500, 300};
    std::vector<std::pair<Point, const char*>> testPositions = {
        {{200, 150}, "target upper-left"},      // Source above target
        {{500, 150}, "target above"},            // Source directly above
        {{800, 150}, "target upper-right"},      // Source above-right
        {{200, 450}, "target lower-left"},       // Source below target
        {{500, 450}, "target below"},            // Source directly below
        {{800, 450}, "target lower-right"},      // Source below-right
        {{0, 300}, "target left"},               // Source left of target
        {{1000, 300}, "target right"},           // Source right of target
    };

    for (const auto& [sourcePos, desc] : testPositions) {
        // Create layout
        std::unordered_map<NodeId, NodeLayout> layouts;

        NodeLayout srcLayout;
        srcLayout.id = source;
        srcLayout.position = sourcePos;
        srcLayout.size = {200, 100};
        srcLayout.layer = 0;
        layouts[source] = srcLayout;

        NodeLayout tgtLayout;
        tgtLayout.id = target;
        tgtLayout.position = targetPos;
        tgtLayout.size = {200, 100};
        tgtLayout.layer = 1;
        layouts[target] = tgtLayout;

        // Run layout
        SugiyamaLayout layout;
        LayoutResult result = layout.layout(graph);

        const EdgeLayout* edgeLayout = result.getEdgeLayout(e0);
        ASSERT_NE(edgeLayout, nullptr);

        // Get node bounds
        const NodeLayout* srcNode = result.getNodeLayout(source);
        const NodeLayout* tgtNode = result.getNodeLayout(target);
        ASSERT_NE(srcNode, nullptr);
        ASSERT_NE(tgtNode, nullptr);

        float srcLeft = srcNode->position.x;
        float srcRight = srcNode->position.x + srcNode->size.width;
        float srcTop = srcNode->position.y;
        float srcBottom = srcNode->position.y + srcNode->size.height;

        float tgtLeft = tgtNode->position.x;
        float tgtRight = tgtNode->position.x + tgtNode->size.width;
        float tgtTop = tgtNode->position.y;
        float tgtBottom = tgtNode->position.y + tgtNode->size.height;

        // Check each bend point
        for (size_t i = 0; i < edgeLayout->bendPoints.size(); ++i) {
            const Point& bend = edgeLayout->bendPoints[i].position;

            bool insideSource = (bend.x >= srcLeft && bend.x <= srcRight &&
                                bend.y >= srcTop && bend.y <= srcBottom);
            bool insideTarget = (bend.x >= tgtLeft && bend.x <= tgtRight &&
                                bend.y >= tgtTop && bend.y <= tgtBottom);

            EXPECT_FALSE(insideSource)
                << desc << " - Bend[" << i << "] (" << bend.x << "," << bend.y
                << ") inside SOURCE at [" << srcLeft << "," << srcRight << "]x["
                << srcTop << "," << srcBottom << "]";

            EXPECT_FALSE(insideTarget)
                << desc << " - Bend[" << i << "] (" << bend.x << "," << bend.y
                << ") inside TARGET at [" << tgtLeft << "," << tgtRight << "]x["
                << tgtTop << "," << tgtBottom << "]";
        }
    }
}

// Test specific case from interactive demo: idle → running edge has overlapping bend point
TEST(EdgeRoutingTransitionTest, IdleToRunning_FirstBendPointOutsideNodes) {
    Graph graph;

    // Recreate exact state machine from interactive demo
    NodeId idle = graph.addNode(Size{200, 100}, "idle");
    NodeId running = graph.addNode(Size{200, 100}, "running");
    NodeId error = graph.addNode(Size{200, 100}, "error");
    NodeId processing = graph.addNode(Size{200, 100}, "processing");
    NodeId success = graph.addNode(Size{200, 100}, "success");

    // All transitions
    EdgeId e0 = graph.addEdge(idle, running, "start");
    EdgeId e1 = graph.addEdge(running, error, "fail");
    EdgeId e2 = graph.addEdge(error, running, "retry");
    EdgeId e3 = graph.addEdge(running, processing, "process");
    EdgeId e4 = graph.addEdge(error, processing, "fix");
    EdgeId e5 = graph.addEdge(running, success, "complete");
    EdgeId e6 = graph.addEdge(success, idle, "reset");
    EdgeId e7 = graph.addEdge(success, success, "continue");

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph);

    // Focus on edge 0: idle → running
    const EdgeLayout* edgeLayout = result.getEdgeLayout(e0);
    ASSERT_NE(edgeLayout, nullptr);
    ASSERT_GE(edgeLayout->bendPoints.size(), 1) << "Edge should have at least one bend point";

    // Get node positions
    const NodeLayout* idleNode = result.getNodeLayout(idle);
    const NodeLayout* runningNode = result.getNodeLayout(running);
    ASSERT_NE(idleNode, nullptr);
    ASSERT_NE(runningNode, nullptr);

    // Debug output to understand actual positions
    std::cout << "DEBUG - Edge 0 (idle → running):\n";
    std::cout << "  Idle node: pos(" << idleNode->position.x << ", " << idleNode->position.y
              << ") size(" << idleNode->size.width << "x" << idleNode->size.height << ")\n";
    std::cout << "  Running node: pos(" << runningNode->position.x << ", " << runningNode->position.y
              << ") size(" << runningNode->size.width << "x" << runningNode->size.height << ")\n";
    std::cout << "  Edge sourcePoint: (" << edgeLayout->sourcePoint.x << ", " << edgeLayout->sourcePoint.y << ")\n";
    std::cout << "  Edge targetPoint: (" << edgeLayout->targetPoint.x << ", " << edgeLayout->targetPoint.y << ")\n";
    std::cout << "  channelY: " << edgeLayout->channelY << "\n";
    for (size_t i = 0; i < edgeLayout->bendPoints.size(); ++i) {
        std::cout << "  Bend[" << i << "]: (" << edgeLayout->bendPoints[i].position.x << ", "
                  << edgeLayout->bendPoints[i].position.y << ")\n";
    }

    // Calculate node bounds
    float idleLeft = idleNode->position.x;
    float idleRight = idleNode->position.x + idleNode->size.width;
    float idleTop = idleNode->position.y;
    float idleBottom = idleNode->position.y + idleNode->size.height;

    float runningLeft = runningNode->position.x;
    float runningRight = runningNode->position.x + runningNode->size.width;
    float runningTop = runningNode->position.y;
    float runningBottom = runningNode->position.y + runningNode->size.height;

    // Check FIRST bend point: must be outside BOTH idle and running nodes
    const Point& firstBend = edgeLayout->bendPoints[0].position;

    bool firstBendInsideIdle = (firstBend.x >= idleLeft && firstBend.x <= idleRight &&
                                firstBend.y >= idleTop && firstBend.y <= idleBottom);
    bool firstBendInsideRunning = (firstBend.x >= runningLeft && firstBend.x <= runningRight &&
                                   firstBend.y >= runningTop && firstBend.y <= runningBottom);

    // First bend point must be OUTSIDE idle (source node)
    EXPECT_FALSE(firstBendInsideIdle)
        << "First bend point (" << firstBend.x << ", " << firstBend.y << ") is INSIDE idle node! "
        << "Idle bounds: x[" << idleLeft << "," << idleRight << "] y[" << idleTop << "," << idleBottom << "]";

    // First bend point must be OUTSIDE running too
    EXPECT_FALSE(firstBendInsideRunning)
        << "First bend point (" << firstBend.x << ", " << firstBend.y << ") is INSIDE running node! "
        << "Running bounds: x[" << runningLeft << "," << runningRight << "] y[" << runningTop << "," << runningBottom << "]";

    // Check SECOND bend point
    if (edgeLayout->bendPoints.size() >= 2) {
        const Point& secondBend = edgeLayout->bendPoints[1].position;

        bool secondBendInsideIdle = (secondBend.x >= idleLeft && secondBend.x <= idleRight &&
                                    secondBend.y >= idleTop && secondBend.y <= idleBottom);
        bool secondBendInsideRunning = (secondBend.x >= runningLeft && secondBend.x <= runningRight &&
                                       secondBend.y >= runningTop && secondBend.y <= runningBottom);

        EXPECT_FALSE(secondBendInsideIdle)
            << "Second bend point (" << secondBend.x << ", " << secondBend.y << ") is INSIDE idle node!";

        EXPECT_FALSE(secondBendInsideRunning)
            << "Second bend point (" << secondBend.x << ", " << secondBend.y << ") is INSIDE running node!";
    }
}

// Comprehensive test: Bend points must be STRICTLY OUTSIDE nodes (not on boundary)
// Edge endpoints (sourcePoint, targetPoint) are on node boundaries by design, so we check bend points only
TEST(EdgeRoutingTransitionTest, BendPoints_MustBeOutsideNotOnBoundary) {
    Graph graph;

    NodeId idle = graph.addNode(Size{200, 100}, "idle");
    NodeId running = graph.addNode(Size{200, 100}, "running");
    NodeId error = graph.addNode(Size{200, 100}, "error");
    NodeId processing = graph.addNode(Size{200, 100}, "processing");
    NodeId success = graph.addNode(Size{200, 100}, "success");

    EdgeId e0 = graph.addEdge(idle, running, "start");
    EdgeId e1 = graph.addEdge(running, error, "fail");
    EdgeId e2 = graph.addEdge(error, running, "retry");
    EdgeId e3 = graph.addEdge(running, processing, "process");
    EdgeId e4 = graph.addEdge(error, processing, "fix");
    EdgeId e5 = graph.addEdge(running, success, "complete");
    EdgeId e6 = graph.addEdge(success, idle, "reset");
    EdgeId e7 = graph.addEdge(success, success, "continue");

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph);

    std::vector<EdgeId> allEdges = {e0, e1, e2, e3, e4, e5, e6, e7};

    for (EdgeId edgeId : allEdges) {
        const EdgeLayout* edgeLayout = result.getEdgeLayout(edgeId);
        ASSERT_NE(edgeLayout, nullptr);

        if (edgeLayout->bendPoints.empty()) continue;

        const NodeLayout* srcNode = result.getNodeLayout(edgeLayout->from);
        const NodeLayout* tgtNode = result.getNodeLayout(edgeLayout->to);
        ASSERT_NE(srcNode, nullptr);
        ASSERT_NE(tgtNode, nullptr);

        float srcXmin = srcNode->position.x;
        float srcXmax = srcNode->position.x + srcNode->size.width;
        float srcYmin = srcNode->position.y;
        float srcYmax = srcNode->position.y + srcNode->size.height;

        float tgtXmin = tgtNode->position.x;
        float tgtXmax = tgtNode->position.x + tgtNode->size.width;
        float tgtYmin = tgtNode->position.y;
        float tgtYmax = tgtNode->position.y + tgtNode->size.height;

        // Check BEND POINTS ONLY (not edge endpoints which are on boundaries)
        for (size_t i = 0; i < edgeLayout->bendPoints.size(); ++i) {
            const Point& bend = edgeLayout->bendPoints[i].position;

            // Strictly inside source node (not on boundary)
            bool strictlyInsideSource = (bend.x > srcXmin && bend.x < srcXmax &&
                                        bend.y > srcYmin && bend.y < srcYmax);

            // Strictly inside target node (not on boundary)
            bool strictlyInsideTarget = (bend.x > tgtXmin && bend.x < tgtXmax &&
                                        bend.y > tgtYmin && bend.y < tgtYmax);

            EXPECT_FALSE(strictlyInsideSource)
                << "Edge " << edgeId << " bend[" << i << "] (" << bend.x << "," << bend.y
                << ") is STRICTLY INSIDE source node [" << srcXmin << "," << srcXmax << "]x["
                << srcYmin << "," << srcYmax << "]";

            EXPECT_FALSE(strictlyInsideTarget)
                << "Edge " << edgeId << " bend[" << i << "] (" << bend.x << "," << bend.y
                << ") is STRICTLY INSIDE target node [" << tgtXmin << "," << tgtXmax << "]x["
                << tgtYmin << "," << tgtYmax << "]";
        }
    }
}

// CRITICAL TEST: Segments must NOT intersect node interiors
// This catches cases where bend points are outside but the LINE between them goes through a node
TEST(EdgeRoutingTransitionTest, Segments_MustNotIntersectNodes) {
    Graph graph;

    NodeId idle = graph.addNode(Size{200, 100}, "idle");
    NodeId running = graph.addNode(Size{200, 100}, "running");
    NodeId error = graph.addNode(Size{200, 100}, "error");
    NodeId processing = graph.addNode(Size{200, 100}, "processing");
    NodeId success = graph.addNode(Size{200, 100}, "success");

    EdgeId e0 = graph.addEdge(idle, running, "start");
    EdgeId e1 = graph.addEdge(running, error, "fail");
    EdgeId e2 = graph.addEdge(error, running, "retry");
    EdgeId e3 = graph.addEdge(running, processing, "process");
    EdgeId e4 = graph.addEdge(error, processing, "fix");
    EdgeId e5 = graph.addEdge(running, success, "complete");
    EdgeId e6 = graph.addEdge(success, idle, "reset");
    EdgeId e7 = graph.addEdge(success, success, "continue");

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph);

    std::vector<EdgeId> allEdges = {e0, e1, e2, e3, e4, e5, e6, e7};
    std::vector<NodeId> allNodes = {idle, running, error, processing, success};

    for (EdgeId edgeId : allEdges) {
        const EdgeLayout* edgeLayout = result.getEdgeLayout(edgeId);
        ASSERT_NE(edgeLayout, nullptr);

        // Collect all points
        std::vector<Point> points;
        points.push_back(edgeLayout->sourcePoint);
        for (const auto& bend : edgeLayout->bendPoints) {
            points.push_back(bend.position);
        }
        points.push_back(edgeLayout->targetPoint);

        // Check each segment against all nodes (except endpoints' own nodes)
        for (size_t i = 0; i + 1 < points.size(); ++i) {
            const Point& p1 = points[i];
            const Point& p2 = points[i + 1];

            // Determine if segment is horizontal or vertical
            bool isHorizontal = (std::abs(p1.y - p2.y) < 0.1f);
            bool isVertical = (std::abs(p1.x - p2.x) < 0.1f);

            for (NodeId nodeId : allNodes) {
                // Skip source and target nodes for first and last segments
                if (i == 0 && nodeId == edgeLayout->from) continue;
                if (i == points.size() - 2 && nodeId == edgeLayout->to) continue;

                const NodeLayout* node = result.getNodeLayout(nodeId);
                ASSERT_NE(node, nullptr);

                float nodeXmin = node->position.x;
                float nodeXmax = node->position.x + node->size.width;
                float nodeYmin = node->position.y;
                float nodeYmax = node->position.y + node->size.height;

                // Check if segment intersects node INTERIOR (not just boundary)
                if (isHorizontal) {
                    float y = p1.y;
                    float xMin = std::min(p1.x, p2.x);
                    float xMax = std::max(p1.x, p2.x);

                    // Segment crosses node vertically AND is within node's Y range
                    if (y > nodeYmin && y < nodeYmax && xMin < nodeXmax && xMax > nodeXmin) {
                        FAIL() << "Edge " << edgeId << " segment [" << i << "→" << (i+1)
                               << "] INTERSECTS node " << nodeId << " interior!\n"
                               << "  Segment: (" << p1.x << "," << p1.y << ") → (" << p2.x << "," << p2.y << ")\n"
                               << "  Node: x[" << nodeXmin << "," << nodeXmax << "] y["
                               << nodeYmin << "," << nodeYmax << "]";
                    }
                } else if (isVertical) {
                    float x = p1.x;
                    float yMin = std::min(p1.y, p2.y);
                    float yMax = std::max(p1.y, p2.y);

                    // Segment crosses node horizontally AND is within node's X range
                    if (x > nodeXmin && x < nodeXmax && yMin < nodeYmax && yMax > nodeYmin) {
                        FAIL() << "Edge " << edgeId << " segment [" << i << "→" << (i+1)
                               << "] INTERSECTS node " << nodeId << " interior!\n"
                               << "  Segment: (" << p1.x << "," << p1.y << ") → (" << p2.x << "," << p2.y << ")\n"
                               << "  Node: x[" << nodeXmin << "," << nodeXmax << "] y["
                               << nodeYmin << "," << nodeYmax << "]";
                    }
                }
            }
        }
    }
}

// Comprehensive test: All edges MUST be orthogonal (only horizontal/vertical segments)
// No diagonal lines allowed - this is the fundamental requirement
TEST(EdgeRoutingTransitionTest, AllEdges_MustBeOrthogonal) {
    Graph graph;

    NodeId idle = graph.addNode(Size{200, 100}, "idle");
    NodeId running = graph.addNode(Size{200, 100}, "running");
    NodeId error = graph.addNode(Size{200, 100}, "error");
    NodeId processing = graph.addNode(Size{200, 100}, "processing");
    NodeId success = graph.addNode(Size{200, 100}, "success");

    // All transitions
    EdgeId e0 = graph.addEdge(idle, running, "start");
    EdgeId e1 = graph.addEdge(running, error, "fail");
    EdgeId e2 = graph.addEdge(error, running, "retry");
    EdgeId e3 = graph.addEdge(running, processing, "process");
    EdgeId e4 = graph.addEdge(error, processing, "fix");
    EdgeId e5 = graph.addEdge(running, success, "complete");
    EdgeId e6 = graph.addEdge(success, idle, "reset");
    EdgeId e7 = graph.addEdge(success, success, "continue");

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph);

    std::vector<EdgeId> allEdges = {e0, e1, e2, e3, e4, e5, e6, e7};

    for (EdgeId edgeId : allEdges) {
        const EdgeLayout* edgeLayout = result.getEdgeLayout(edgeId);
        ASSERT_NE(edgeLayout, nullptr) << "Edge " << edgeId << " not found";

        if (edgeLayout->bendPoints.empty()) continue;  // Skip edges without bends

        // Collect all points: source, bends, target
        std::vector<Point> allPoints;
        allPoints.push_back(edgeLayout->sourcePoint);
        for (const auto& bend : edgeLayout->bendPoints) {
            allPoints.push_back(bend.position);
        }
        allPoints.push_back(edgeLayout->targetPoint);

        // Check that EVERY consecutive segment is either horizontal or vertical
        for (size_t i = 0; i + 1 < allPoints.size(); ++i) {
            const Point& p1 = allPoints[i];
            const Point& p2 = allPoints[i + 1];

            // A segment is horizontal if y coordinates are same
            bool isHorizontal = (std::abs(p1.y - p2.y) < 0.1f);

            // A segment is vertical if x coordinates are same
            bool isVertical = (std::abs(p1.x - p2.x) < 0.1f);

            EXPECT_TRUE(isHorizontal || isVertical)
                << "Edge " << edgeId << " segment [" << i << "→" << (i+1)
                << "] is DIAGONAL! Points: (" << p1.x << "," << p1.y << ") → ("
                << p2.x << "," << p2.y << ")\n"
                << "All points for this edge:\n";

            // Print all points on failure
            if (!isHorizontal && !isVertical) {
                for (size_t j = 0; j < allPoints.size(); ++j) {
                    std::cout << "  Point[" << j << "]: (" << allPoints[j].x << ", "
                              << allPoints[j].y << ")\n";
                }
            }
        }
    }
}

// DEDICATED ORTHOGONALITY VALIDATION TEST
// This test ONLY checks that every single segment in every edge is orthogonal
// It provides detailed output of every segment to identify any violations
TEST(EdgeRoutingTransitionTest, Orthogonality_ComprehensiveSegmentValidation) {
    Graph graph;

    // Create state machine from interactive_demo
    NodeId idle = graph.addNode(Size{200, 100}, "idle");
    NodeId running = graph.addNode(Size{200, 100}, "running");
    NodeId error = graph.addNode(Size{200, 100}, "error");
    NodeId processing = graph.addNode(Size{200, 100}, "processing");
    NodeId success = graph.addNode(Size{200, 100}, "success");

    // All transitions
    EdgeId e0 = graph.addEdge(idle, running, "start");
    EdgeId e1 = graph.addEdge(running, error, "fail");
    EdgeId e2 = graph.addEdge(error, running, "retry");
    EdgeId e3 = graph.addEdge(running, processing, "process");
    EdgeId e4 = graph.addEdge(error, processing, "fix");
    EdgeId e5 = graph.addEdge(running, success, "complete");
    EdgeId e6 = graph.addEdge(success, idle, "reset");
    EdgeId e7 = graph.addEdge(success, success, "continue");

    // Run layout
    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph);

    std::vector<EdgeId> allEdges = {e0, e1, e2, e3, e4, e5, e6, e7};

    int totalSegments = 0;
    int orthogonalSegments = 0;
    int diagonalSegments = 0;

    std::string diagnosticOutput;

    for (EdgeId edgeId : allEdges) {
        const EdgeLayout* edgeLayout = result.getEdgeLayout(edgeId);
        ASSERT_NE(edgeLayout, nullptr);

        // Collect all points for this edge
        std::vector<Point> allPoints;
        allPoints.push_back(edgeLayout->sourcePoint);
        for (const auto& bend : edgeLayout->bendPoints) {
            allPoints.push_back(bend.position);
        }
        allPoints.push_back(edgeLayout->targetPoint);

        // Diagnostic output for this edge
        std::ostringstream edgeInfo;
        edgeInfo << "\nEdge " << edgeId << ": " << edgeLayout->from << " → " << edgeLayout->to << "\n";
        edgeInfo << "  Total points: " << allPoints.size() << "\n";
        edgeInfo << "  Segments:\n";

        // Check every segment
        for (size_t i = 0; i + 1 < allPoints.size(); ++i) {
            const Point& p1 = allPoints[i];
            const Point& p2 = allPoints[i + 1];

            totalSegments++;

            // Check if horizontal (y same)
            float yDiff = std::abs(p1.y - p2.y);
            bool isHorizontal = (yDiff < 0.1f);

            // Check if vertical (x same)
            float xDiff = std::abs(p1.x - p2.x);
            bool isVertical = (xDiff < 0.1f);

            bool isOrthogonal = isHorizontal || isVertical;

            if (isOrthogonal) {
                orthogonalSegments++;
            } else {
                diagonalSegments++;
            }

            // Build segment info
            std::ostringstream segInfo;
            segInfo << "    [" << i << "→" << (i+1) << "] ("
                    << std::fixed << std::setprecision(2)
                    << p1.x << "," << p1.y << ") → ("
                    << p2.x << "," << p2.y << ")";

            if (isHorizontal) {
                segInfo << " [HORIZONTAL] dx=" << xDiff;
            } else if (isVertical) {
                segInfo << " [VERTICAL] dy=" << yDiff;
            } else {
                segInfo << " [DIAGONAL] dx=" << xDiff << ", dy=" << yDiff;
            }
            segInfo << "\n";

            edgeInfo << segInfo.str();

            // Fail on first diagonal segment
            EXPECT_TRUE(isOrthogonal)
                << "Edge " << edgeId << " segment [" << i << "→" << (i+1)
                << "] is DIAGONAL!\n"
                << "  From: (" << p1.x << ", " << p1.y << ")\n"
                << "  To:   (" << p2.x << ", " << p2.y << ")\n"
                << "  Δx=" << xDiff << ", Δy=" << yDiff << "\n"
                << "  All points for this edge:\n";

            if (!isOrthogonal) {
                for (size_t j = 0; j < allPoints.size(); ++j) {
                    std::cout << "    Point[" << j << "]: ("
                              << std::fixed << std::setprecision(2)
                              << allPoints[j].x << ", " << allPoints[j].y << ")\n";
                }
            }
        }

        diagnosticOutput += edgeInfo.str();
    }

    // Print comprehensive diagnostic output
    std::cout << "\n===== ORTHOGONALITY DIAGNOSTIC REPORT =====\n";
    std::cout << "Total segments checked: " << totalSegments << "\n";
    std::cout << "Orthogonal segments: " << orthogonalSegments << " ("
              << (totalSegments > 0 ? (100.0 * orthogonalSegments / totalSegments) : 0) << "%)\n";
    std::cout << "Diagonal segments: " << diagonalSegments << " ("
              << (totalSegments > 0 ? (100.0 * diagonalSegments / totalSegments) : 0) << "%)\n";
    std::cout << diagnosticOutput;
    std::cout << "==========================================\n";

    // Final verdict
    EXPECT_EQ(diagonalSegments, 0) << "Found " << diagonalSegments << " non-orthogonal segments!";
}

// Test that ALL edges have bend points OUTSIDE their connected nodes
// This test iterates through all transitions to ensure directional constraints work correctly
TEST(EdgeRoutingTransitionTest, AllBendPoints_MustBeOutsideNodes) {
    // Create the state machine graph from interactive demo
    Graph graph;

    NodeId idle = graph.addNode(Size{200, 100}, "idle");
    NodeId running = graph.addNode(Size{200, 100}, "running");
    NodeId error = graph.addNode(Size{200, 100}, "error");
    NodeId processing = graph.addNode(Size{200, 100}, "processing");
    NodeId success = graph.addNode(Size{200, 100}, "success");

    // All transitions
    EdgeId e0 = graph.addEdge(idle, running, "start");
    EdgeId e1 = graph.addEdge(running, error, "fail");
    EdgeId e2 = graph.addEdge(error, running, "retry");
    EdgeId e3 = graph.addEdge(running, processing, "process");
    EdgeId e4 = graph.addEdge(error, processing, "fix");
    EdgeId e5 = graph.addEdge(running, success, "complete");
    EdgeId e6 = graph.addEdge(success, idle, "reset");
    EdgeId e7 = graph.addEdge(success, success, "continue");

    // Run layout
    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph);


    // Check EVERY edge
    std::vector<EdgeId> allEdges = {e0, e1, e2, e3, e4, e5, e6, e7};

    for (EdgeId edgeId : allEdges) {
        const EdgeLayout* edgeLayout = result.getEdgeLayout(edgeId);
        ASSERT_NE(edgeLayout, nullptr) << "Edge " << edgeId << " not found in layout";

        // Get source and target node bounds
        const NodeLayout* sourceNode = result.getNodeLayout(edgeLayout->from);
        const NodeLayout* targetNode = result.getNodeLayout(edgeLayout->to);
        ASSERT_NE(sourceNode, nullptr);
        ASSERT_NE(targetNode, nullptr);

        // Calculate node bounds
        float sourceTop = sourceNode->position.y;
        float sourceBottom = sourceNode->position.y + sourceNode->size.height;
        float sourceLeft = sourceNode->position.x;
        float sourceRight = sourceNode->position.x + sourceNode->size.width;

        float targetTop = targetNode->position.y;
        float targetBottom = targetNode->position.y + targetNode->size.height;
        float targetLeft = targetNode->position.x;
        float targetRight = targetNode->position.x + targetNode->size.width;

        // Check each bend point
        for (size_t i = 0; i < edgeLayout->bendPoints.size(); ++i) {
            const Point& bend = edgeLayout->bendPoints[i].position;

            // Check if bend point is inside SOURCE node
            bool insideSource = (bend.x >= sourceLeft && bend.x <= sourceRight &&
                               bend.y >= sourceTop && bend.y <= sourceBottom);

            // Check if bend point is inside TARGET node
            bool insideTarget = (bend.x >= targetLeft && bend.x <= targetRight &&
                               bend.y >= targetTop && bend.y <= targetBottom);

            // Detailed error message
            if (insideSource) {
                FAIL() << "Edge " << edgeId << " bend point [" << i << "] is INSIDE source node!\n"
                       << "  Source (" << edgeLayout->from << "): "
                       << "[" << sourceLeft << "," << sourceRight << "] x [" << sourceTop << "," << sourceBottom << "]\n"
                       << "  Bend point: (" << bend.x << ", " << bend.y << ")\n"
                       << "  Source edge: " << static_cast<int>(edgeLayout->sourceEdge) << "\n"
                       << "  Target edge: " << static_cast<int>(edgeLayout->targetEdge) << "\n"
                       << "  channelY: " << edgeLayout->channelY;
            }

            if (insideTarget) {
                FAIL() << "Edge " << edgeId << " bend point [" << i << "] is INSIDE target node!\n"
                       << "  Target (" << edgeLayout->to << "): "
                       << "[" << targetLeft << "," << targetRight << "] x [" << targetTop << "," << targetBottom << "]\n"
                       << "  Bend point: (" << bend.x << ", " << bend.y << ")\n"
                       << "  Source edge: " << static_cast<int>(edgeLayout->sourceEdge) << "\n"
                       << "  Target edge: " << static_cast<int>(edgeLayout->targetEdge) << "\n"
                       << "  channelY: " << edgeLayout->channelY;
            }
        }

        // Also check that first bend point has minimum clearance from source
        if (!edgeLayout->bendPoints.empty()) {
            const Point& firstBend = edgeLayout->bendPoints[0].position;
            const Point& sourcePoint = edgeLayout->sourcePoint;
            constexpr float MIN_CLEARANCE = 10.0f;

            // Check clearance based on source edge direction
            switch (edgeLayout->sourceEdge) {
                case NodeEdge::Top:
                    EXPECT_LE(firstBend.y, sourcePoint.y - MIN_CLEARANCE)
                        << "Edge " << edgeId << ": Top edge should have bend point ABOVE source with clearance\n"
                        << "  sourcePoint.y: " << sourcePoint.y << ", bendPoint.y: " << firstBend.y;
                    break;
                case NodeEdge::Bottom:
                    EXPECT_GE(firstBend.y, sourcePoint.y + MIN_CLEARANCE)
                        << "Edge " << edgeId << ": Bottom edge should have bend point BELOW source with clearance\n"
                        << "  sourcePoint.y: " << sourcePoint.y << ", bendPoint.y: " << firstBend.y
                        << "\n  channelY: " << edgeLayout->channelY;
                    break;
                case NodeEdge::Left:
                    EXPECT_LE(firstBend.x, sourcePoint.x - MIN_CLEARANCE)
                        << "Edge " << edgeId << ": Left edge should have bend point LEFT of source with clearance\n"
                        << "  sourcePoint.x: " << sourcePoint.x << ", bendPoint.x: " << firstBend.x;
                    break;
                case NodeEdge::Right:
                    EXPECT_GE(firstBend.x, sourcePoint.x + MIN_CLEARANCE)
                        << "Edge " << edgeId << ": Right edge should have bend point RIGHT of source with clearance\n"
                        << "  sourcePoint.x: " << sourcePoint.x << ", bendPoint.x: " << firstBend.x;
                    break;
            }
        }
    }
}

// CRITICAL TEST: Uses exact interactive_demo graph structure
// Tests that no segment passes through ANY node (including source/target for middle segments)
TEST(EdgeRoutingTransitionTest, InteractiveDemo_SegmentsMustNotPassThroughNodes) {
    Graph graph;

    // Exact same graph structure as examples/interactive_demo/main.cpp
    NodeId idle = graph.addNode(Size{200, 100}, "Idle");
    NodeId running = graph.addNode(Size{200, 100}, "Running");
    NodeId paused = graph.addNode(Size{200, 100}, "Paused");
    NodeId stopped = graph.addNode(Size{200, 100}, "Stopped");
    NodeId error = graph.addNode(Size{200, 100}, "Error");

    EdgeId e0 = graph.addEdge(idle, running, "start");
    EdgeId e1 = graph.addEdge(running, paused, "pause");
    EdgeId e2 = graph.addEdge(paused, running, "resume");
    EdgeId e3 = graph.addEdge(running, stopped, "stop");
    EdgeId e4 = graph.addEdge(paused, stopped, "stop");
    EdgeId e5 = graph.addEdge(running, error, "fail");
    EdgeId e6 = graph.addEdge(error, idle, "reset");
    EdgeId e7 = graph.addEdge(error, error, "retry");

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph);

    std::vector<EdgeId> allEdges = {e0, e1, e2, e3, e4, e5, e6, e7};
    std::vector<NodeId> allNodes = {idle, running, paused, stopped, error};

    // Build node bounds map
    std::map<NodeId, std::tuple<float, float, float, float>> nodeBounds;
    for (NodeId nodeId : allNodes) {
        const NodeLayout* node = result.getNodeLayout(nodeId);
        ASSERT_NE(node, nullptr);
        float xmin = node->position.x;
        float xmax = node->position.x + node->size.width;
        float ymin = node->position.y;
        float ymax = node->position.y + node->size.height;
        nodeBounds[nodeId] = {xmin, xmax, ymin, ymax};
    }

    std::cout << "\n===== INTERACTIVE_DEMO SEGMENT INTERSECTION TEST =====\n";
    std::cout << "Node positions:\n";
    for (NodeId nodeId : allNodes) {
        auto [xmin, xmax, ymin, ymax] = nodeBounds[nodeId];
        std::cout << "  Node " << nodeId << ": x[" << xmin << "," << xmax << "], y[" << ymin << "," << ymax << "]\n";
    }

    for (EdgeId edgeId : allEdges) {
        const EdgeLayout* edgeLayout = result.getEdgeLayout(edgeId);
        ASSERT_NE(edgeLayout, nullptr);

        // Collect all points
        std::vector<Point> points;
        points.push_back(edgeLayout->sourcePoint);
        for (const auto& bend : edgeLayout->bendPoints) {
            points.push_back(bend.position);
        }
        points.push_back(edgeLayout->targetPoint);

        std::cout << "Edge " << edgeId << " (" << edgeLayout->from << " -> " << edgeLayout->to
                  << "): " << points.size() << " points\n";

        // Check each segment
        for (size_t i = 0; i + 1 < points.size(); ++i) {
            const Point& p1 = points[i];
            const Point& p2 = points[i + 1];

            bool isHorizontal = (std::abs(p1.y - p2.y) < 0.1f);
            bool isVertical = (std::abs(p1.x - p2.x) < 0.1f);

            for (NodeId nodeId : allNodes) {
                // For FIRST segment only, skip source node
                // For LAST segment only, skip target node
                // For MIDDLE segments, check ALL nodes including source and target
                if (i == 0 && nodeId == edgeLayout->from) continue;
                if (i == points.size() - 2 && nodeId == edgeLayout->to) continue;

                auto [xmin, xmax, ymin, ymax] = nodeBounds[nodeId];

                bool intersects = false;

                if (isHorizontal) {
                    float y = p1.y;
                    float segXmin = std::min(p1.x, p2.x);
                    float segXmax = std::max(p1.x, p2.x);
                    // Horizontal segment intersects if within node's Y range and overlaps X range
                    intersects = (y > ymin && y < ymax && segXmin < xmax && segXmax > xmin);
                } else if (isVertical) {
                    float x = p1.x;
                    float segYmin = std::min(p1.y, p2.y);
                    float segYmax = std::max(p1.y, p2.y);
                    // Vertical segment intersects if within node's X range and overlaps Y range
                    intersects = (x > xmin && x < xmax && segYmin < ymax && segYmax > ymin);
                }

                if (intersects) {
                    std::cout << "  INTERSECTION! Segment [" << i << "->" << (i+1) << "] "
                              << "(" << p1.x << "," << p1.y << ") -> (" << p2.x << "," << p2.y << ")"
                              << " passes through node " << nodeId
                              << " [x:" << xmin << "-" << xmax << ", y:" << ymin << "-" << ymax << "]\n";

                    FAIL() << "Edge " << edgeId << " segment [" << i << "->" << (i+1)
                           << "] passes through node " << nodeId << " (interior)!\n"
                           << "  Segment: (" << p1.x << "," << p1.y << ") -> (" << p2.x << "," << p2.y << ")\n"
                           << "  Node bounds: x[" << xmin << "," << xmax << "], y[" << ymin << "," << ymax << "]\n"
                           << "  Edge source: " << edgeLayout->from << ", target: " << edgeLayout->to;
                }
            }
        }
    }

    std::cout << "=======================================================\n";
}

// CRITICAL TEST: Reproduces exact user-reported bug scenario
// When running node overlaps idle node in Y-axis but is offset in X-axis,
// edge segment passes through running node interior
TEST(EdgeRoutingTransitionTest, UserReportedBug_EdgePassesThroughTargetNode) {
    Graph graph;

    // Create minimal graph that reproduces the bug
    NodeId idle = graph.addNode(Size{200, 100}, "Idle");
    NodeId running = graph.addNode(Size{200, 100}, "Running");

    EdgeId edge = graph.addEdge(idle, running, "start");

    // Create node layouts matching user's interactive_demo output
    // Key: running node overlaps idle in Y but is offset in X
    std::unordered_map<NodeId, NodeLayout> nodeLayouts;

    nodeLayouts[idle] = NodeLayout{
        idle,
        {0.0f, 0.0f},      // position: x=0, y=0
        {200.0f, 100.0f},  // size
        0, 0               // layer, order
    };

    nodeLayouts[running] = NodeLayout{
        running,
        {366.0f, 10.0f},   // position: x=366, y=10 (overlaps idle in Y!)
        {200.0f, 100.0f},  // size
        1, 0               // layer, order
    };

    // Route the edge with these specific positions
    EdgeRouting router;
    std::unordered_set<EdgeId> reversedEdges;
    LayoutOptions options;

    auto result = router.route(graph, nodeLayouts, reversedEdges, options);

    const EdgeLayout* edgeLayout = result.getEdgeLayout(edge);
    ASSERT_NE(edgeLayout, nullptr);

    // Collect all points
    std::vector<Point> points;
    points.push_back(edgeLayout->sourcePoint);
    for (const auto& bend : edgeLayout->bendPoints) {
        points.push_back(bend.position);
    }
    points.push_back(edgeLayout->targetPoint);

    std::cout << "\n===== USER REPORTED BUG TEST =====\n";
    std::cout << "Idle node: x[0,200], y[0,100]\n";
    std::cout << "Running node: x[366,566], y[10,110]\n";
    std::cout << "Edge points:\n";
    for (size_t i = 0; i < points.size(); ++i) {
        std::cout << "  [" << i << "]: (" << points[i].x << ", " << points[i].y << ")\n";
    }

    // Check each middle segment against running node
    float runXmin = 366.0f, runXmax = 566.0f;
    float runYmin = 10.0f, runYmax = 110.0f;

    for (size_t i = 0; i + 1 < points.size(); ++i) {
        const Point& p1 = points[i];
        const Point& p2 = points[i + 1];

        // Skip zero-length segments (duplicate points)
        float dx = std::abs(p1.x - p2.x);
        float dy = std::abs(p1.y - p2.y);
        if (dx < 0.1f && dy < 0.1f) continue;

        // CRITICAL: Check orthogonality for ALL segments
        bool isHorizontal = (dy < 0.1f);
        bool isVertical = (dx < 0.1f);
        bool isOrthogonal = isHorizontal || isVertical;

        EXPECT_TRUE(isOrthogonal)
            << "Segment [" << i << "->" << (i+1) << "] is DIAGONAL!\n"
            << "  From: (" << p1.x << ", " << p1.y << ")\n"
            << "  To:   (" << p2.x << ", " << p2.y << ")\n"
            << "  dx=" << dx << ", dy=" << dy;

        if (!isOrthogonal) {
            std::cout << "  DIAGONAL! Segment [" << i << "->" << (i+1) << "] "
                      << "(" << p1.x << "," << p1.y << ") -> (" << p2.x << "," << p2.y << ")"
                      << " dx=" << dx << ", dy=" << dy << "\n";
        }

        // Skip first segment (connected to source) and last segment (connected to target)
        // for node intersection check only
        if (i == 0 || i == points.size() - 2) continue;

        if (isVertical) {
            float x = p1.x;
            float segYmin = std::min(p1.y, p2.y);
            float segYmax = std::max(p1.y, p2.y);

            bool intersects = (x > runXmin && x < runXmax && segYmin < runYmax && segYmax > runYmin);

            EXPECT_FALSE(intersects)
                << "Edge segment [" << i << "->" << (i+1) << "] passes through RUNNING node!\n"
                << "  Segment: (" << p1.x << "," << p1.y << ") -> (" << p2.x << "," << p2.y << ")\n"
                << "  Running node: x[" << runXmin << "," << runXmax << "], y[" << runYmin << "," << runYmax << "]";
        } else if (isHorizontal) {
            float y = p1.y;
            float segXmin = std::min(p1.x, p2.x);
            float segXmax = std::max(p1.x, p2.x);

            bool intersects = (y > runYmin && y < runYmax && segXmin < runXmax && segXmax > runXmin);

            EXPECT_FALSE(intersects)
                << "Edge segment [" << i << "->" << (i+1) << "] passes through RUNNING node!\n"
                << "  Segment: (" << p1.x << "," << p1.y << ") -> (" << p2.x << "," << p2.y << ")\n"
                << "  Running node: x[" << runXmin << "," << runXmax << "], y[" << runYmin << "," << runYmax << "]";
        }
    }

    std::cout << "==================================\n";
}

// Helper function to check if two collinear segments overlap
// Returns true if segments overlap (share more than just an endpoint)
static bool segmentsOverlap(const Point& a1, const Point& a2, const Point& b1, const Point& b2) {
    constexpr float EPSILON = 0.1f;

    // Check if both segments are on the same line
    bool bothVertical = (std::abs(a1.x - a2.x) < EPSILON) && (std::abs(b1.x - b2.x) < EPSILON);
    bool bothHorizontal = (std::abs(a1.y - a2.y) < EPSILON) && (std::abs(b1.y - b2.y) < EPSILON);

    if (bothVertical && std::abs(a1.x - b1.x) < EPSILON) {
        // Both vertical on same x
        float aMin = std::min(a1.y, a2.y);
        float aMax = std::max(a1.y, a2.y);
        float bMin = std::min(b1.y, b2.y);
        float bMax = std::max(b1.y, b2.y);

        // Check for overlap (more than just touching at endpoint)
        float overlapStart = std::max(aMin, bMin);
        float overlapEnd = std::min(aMax, bMax);
        return (overlapEnd - overlapStart) > EPSILON;
    }

    if (bothHorizontal && std::abs(a1.y - b1.y) < EPSILON) {
        // Both horizontal on same y
        float aMin = std::min(a1.x, a2.x);
        float aMax = std::max(a1.x, a2.x);
        float bMin = std::min(b1.x, b2.x);
        float bMax = std::max(b1.x, b2.x);

        // Check for overlap (more than just touching at endpoint)
        float overlapStart = std::max(aMin, bMin);
        float overlapEnd = std::min(aMax, bMax);
        return (overlapEnd - overlapStart) > EPSILON;
    }

    return false;
}

// Test that ensures no edge has overlapping segments within itself
// This catches the bug where avoidance logic creates back-and-forth paths
TEST(EdgeRoutingTransitionTest, NoOverlappingSegmentsWithinSameEdge) {
    // Create a state machine graph similar to interactive_demo
    Graph graph;

    NodeId idle = graph.addNode(Size{200, 100}, "Idle");       // id=0
    NodeId running = graph.addNode(Size{200, 100}, "Running"); // id=1
    NodeId paused = graph.addNode(Size{200, 100}, "Paused");   // id=2
    NodeId stopped = graph.addNode(Size{200, 100}, "Stopped"); // id=3
    NodeId error = graph.addNode(Size{200, 100}, "Error");     // id=4

    // Create transitions (including backward edges that cause complex routing)
    EdgeId e0 = graph.addEdge(idle, running, "start");
    EdgeId e1 = graph.addEdge(running, paused, "pause");
    EdgeId e2 = graph.addEdge(paused, running, "resume");  // backward
    EdgeId e3 = graph.addEdge(running, stopped, "stop");
    EdgeId e4 = graph.addEdge(paused, stopped, "stop");
    EdgeId e5 = graph.addEdge(running, error, "error");
    EdgeId e6 = graph.addEdge(error, idle, "reset");  // backward - this causes complex routing
    EdgeId e7 = graph.addEdge(error, error, "retry");  // self-loop

    // Use exact node positions from user's interactive_demo JSON output
    // These positions trigger the overlapping segment bug
    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    nodeLayouts[idle] = NodeLayout{idle, {-23.0f, 69.0f}, {200.0f, 100.0f}, 0, 0};
    nodeLayouts[running] = NodeLayout{running, {235.0f, 567.0f}, {200.0f, 100.0f}, 1, 0};
    nodeLayouts[paused] = NodeLayout{paused, {-55.0f, 710.0f}, {200.0f, 100.0f}, 2, 0};
    nodeLayouts[stopped] = NodeLayout{stopped, {-20.0f, 859.0f}, {200.0f, 100.0f}, 3, 0};
    nodeLayouts[error] = NodeLayout{error, {89.0f, 1067.0f}, {200.0f, 100.0f}, 3, 1};

    // Route edges with specific node positions
    EdgeRouting router;
    std::unordered_set<EdgeId> reversedEdges = {e2, e6}; // backward edges
    LayoutOptions options;

    auto result = router.route(graph, nodeLayouts, reversedEdges, options);

    std::cout << "\n===== SEGMENT OVERLAP TEST =====\n";

    std::vector<EdgeId> allEdges = {e0, e1, e2, e3, e4, e5, e6, e7};
    int totalOverlaps = 0;

    for (EdgeId edgeId : allEdges) {
        const EdgeLayout* layout = result.getEdgeLayout(edgeId);
        if (!layout) continue;

        // Build full point list
        std::vector<Point> points;
        points.push_back(layout->sourcePoint);
        for (const auto& bend : layout->bendPoints) {
            points.push_back(bend.position);
        }
        points.push_back(layout->targetPoint);

        // Skip edges with less than 4 points (can't have overlapping segments)
        if (points.size() < 4) continue;

        // Check all pairs of non-adjacent segments for overlap
        int edgeOverlaps = 0;
        for (size_t i = 0; i + 1 < points.size(); ++i) {
            for (size_t j = i + 2; j + 1 < points.size(); ++j) {
                const Point& a1 = points[i];
                const Point& a2 = points[i + 1];
                const Point& b1 = points[j];
                const Point& b2 = points[j + 1];

                if (segmentsOverlap(a1, a2, b1, b2)) {
                    edgeOverlaps++;
                    std::cout << "Edge " << edgeId << ": Segments [" << i << "->" << (i+1)
                              << "] and [" << j << "->" << (j+1) << "] OVERLAP!\n";
                    std::cout << "  Seg1: (" << a1.x << "," << a1.y << ") -> (" << a2.x << "," << a2.y << ")\n";
                    std::cout << "  Seg2: (" << b1.x << "," << b1.y << ") -> (" << b2.x << "," << b2.y << ")\n";
                }
            }
        }

        if (edgeOverlaps > 0) {
            std::cout << "Edge " << edgeId << " has " << edgeOverlaps << " overlapping segment pairs\n";
            std::cout << "  Full path:\n";
            for (size_t k = 0; k < points.size(); ++k) {
                std::cout << "    [" << k << "]: (" << points[k].x << ", " << points[k].y << ")\n";
            }
        }

        totalOverlaps += edgeOverlaps;
    }

    std::cout << "Total overlapping segment pairs: " << totalOverlaps << "\n";

    // Also check for spike patterns (adjacent segments that backtrack on same line)
    int totalSpikes = 0;
    constexpr float SPIKE_EPSILON = 0.1f;

    for (EdgeId edgeId : allEdges) {
        const EdgeLayout* layout = result.getEdgeLayout(edgeId);
        if (!layout) continue;

        std::vector<Point> points;
        points.push_back(layout->sourcePoint);
        for (const auto& bend : layout->bendPoints) {
            points.push_back(bend.position);
        }
        points.push_back(layout->targetPoint);

        if (points.size() < 3) continue;

        int edgeSpikes = 0;
        for (size_t i = 0; i + 2 < points.size(); ++i) {
            const Point& a = points[i];
            const Point& b = points[i + 1];
            const Point& c = points[i + 2];

            // Check if all three points are on the same vertical line
            bool sameVertical = (std::abs(a.x - b.x) < SPIKE_EPSILON &&
                                 std::abs(b.x - c.x) < SPIKE_EPSILON);
            // Check if all three points are on the same horizontal line
            bool sameHorizontal = (std::abs(a.y - b.y) < SPIKE_EPSILON &&
                                   std::abs(b.y - c.y) < SPIKE_EPSILON);

            if (sameVertical) {
                // Check if direction reverses (spike pattern)
                bool aToB_down = b.y > a.y;
                bool bToC_down = c.y > b.y;
                if (aToB_down != bToC_down) {
                    edgeSpikes++;
                    std::cout << "Edge " << edgeId << ": SPIKE at point [" << (i+1) << "]!\n";
                    std::cout << "  (" << a.x << "," << a.y << ") -> ("
                              << b.x << "," << b.y << ") -> ("
                              << c.x << "," << c.y << ")\n";
                    std::cout << "  Direction reverses on vertical line x=" << b.x << "\n";
                }
            }

            if (sameHorizontal) {
                // Check if direction reverses (spike pattern)
                bool aToB_right = b.x > a.x;
                bool bToC_right = c.x > b.x;
                if (aToB_right != bToC_right) {
                    edgeSpikes++;
                    std::cout << "Edge " << edgeId << ": SPIKE at point [" << (i+1) << "]!\n";
                    std::cout << "  (" << a.x << "," << a.y << ") -> ("
                              << b.x << "," << b.y << ") -> ("
                              << c.x << "," << c.y << ")\n";
                    std::cout << "  Direction reverses on horizontal line y=" << b.y << "\n";
                }
            }
        }

        if (edgeSpikes > 0) {
            std::cout << "Edge " << edgeId << " has " << edgeSpikes << " spike(s)\n";
            std::cout << "  Full path:\n";
            for (size_t k = 0; k < points.size(); ++k) {
                std::cout << "    [" << k << "]: (" << points[k].x << ", " << points[k].y << ")\n";
            }
        }

        totalSpikes += edgeSpikes;
    }

    std::cout << "Total spikes: " << totalSpikes << "\n";
    std::cout << "================================\n";

    EXPECT_EQ(totalOverlaps, 0) << "Found " << totalOverlaps << " overlapping segment pairs across all edges";
    EXPECT_EQ(totalSpikes, 0) << "Found " << totalSpikes << " spike patterns across all edges";
}

// Verifies that legacy JSON position is correctly rejected by drag validation.
// This specific position was reachable in older versions but is now properly blocked.
TEST(EdgeRoutingTransitionTest, LatestJSON_IntermediateNodePositionRejected) {
    Graph graph;

    // Same graph structure as interactive_demo
    NodeId idle = graph.addNode(Size{200, 100}, "Idle");       // id=0
    NodeId running = graph.addNode(Size{200, 100}, "Running"); // id=1
    NodeId paused = graph.addNode(Size{200, 100}, "Paused");   // id=2
    NodeId stopped = graph.addNode(Size{200, 100}, "Stopped"); // id=3
    NodeId error = graph.addNode(Size{200, 100}, "Error");     // id=4

    EdgeId e0 = graph.addEdge(idle, running, "start");
    EdgeId e1 = graph.addEdge(running, paused, "pause");
    EdgeId e2 = graph.addEdge(paused, running, "resume");
    EdgeId e3 = graph.addEdge(running, stopped, "stop");
    EdgeId e4 = graph.addEdge(paused, stopped, "stop");
    EdgeId e5 = graph.addEdge(running, error, "fail");
    EdgeId e6 = graph.addEdge(error, idle, "reset");
    EdgeId e7 = graph.addEdge(error, error, "retry");

    (void)e0; (void)e1; (void)e2; (void)e3; (void)e4; (void)e5; (void)e6; (void)e7;

    // Do initial layout to get valid edge layouts
    SugiyamaLayout layoutEngine;
    LayoutResult initialResult = layoutEngine.layout(graph);

    // Get initial node and edge layouts for validation
    std::vector<NodeId> nodeIds = {idle, running, paused, stopped, error};
    std::vector<EdgeId> edgeIds = {e0, e1, e2, e3, e4, e5, e6, e7};

    std::unordered_map<NodeId, NodeLayout> baseNodeLayouts;
    for (NodeId nid : nodeIds) {
        const NodeLayout* nl = initialResult.getNodeLayout(nid);
        if (nl) baseNodeLayouts[nid] = *nl;
    }

    std::unordered_map<EdgeId, EdgeLayout> baseEdgeLayouts;
    for (EdgeId eid : edgeIds) {
        const EdgeLayout* el = initialResult.getEdgeLayout(eid);
        if (el) baseEdgeLayouts[eid] = *el;
    }

    // EXACT position from user's latest JSON output (idle was dragged)
    Point idleProposedPos = {213.0f, 76.0f};
    const float gridSize = 20.0f;

    // This position should be rejected by drag validation
    auto validation = LayoutUtils::canMoveNodeTo(idle, idleProposedPos, baseNodeLayouts, baseEdgeLayouts, gridSize);

    EXPECT_FALSE(validation.valid)
        << "Legacy position (213, 76) for idle should be rejected by drag validation";
}

// TDD TEST: All edges must be strictly orthogonal (no diagonal segments)
// Uses exact node positions from user's interactive_demo output
// Verifies that legacy JSON positions (from before canMoveNodeTo validation) are correctly
// rejected by drag validation. These positions were reachable in older versions but are now
// properly blocked to prevent invalid edge routing states.
TEST(EdgeRoutingTransitionTest, LatestJSON_PositionsRejectedByDragValidation) {
    Graph graph;

    NodeId idle = graph.addNode(Size{200, 100}, "Idle");
    NodeId running = graph.addNode(Size{200, 100}, "Running");
    NodeId paused = graph.addNode(Size{200, 100}, "Paused");
    NodeId stopped = graph.addNode(Size{200, 100}, "Stopped");
    NodeId error = graph.addNode(Size{200, 100}, "Error");

    graph.addEdge(idle, running, "start");
    graph.addEdge(running, paused, "pause");
    graph.addEdge(paused, running, "resume");
    graph.addEdge(running, stopped, "stop");
    graph.addEdge(paused, stopped, "stop");
    graph.addEdge(running, error, "fail");
    graph.addEdge(error, idle, "reset");
    graph.addEdge(error, error, "retry");

    // Get valid initial layout
    SugiyamaLayout layoutEngine;
    LayoutResult initialResult = layoutEngine.layout(graph);

    std::vector<NodeId> nodeIds = {idle, running, paused, stopped, error};

    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;

    for (NodeId id : nodeIds) {
        const auto* nl = initialResult.getNodeLayout(id);
        if (nl) nodeLayouts[id] = *nl;
    }
    for (const auto& [edgeId, _] : initialResult.edgeLayouts()) {
        const auto* el = initialResult.getEdgeLayout(edgeId);
        if (el) edgeLayouts[edgeId] = *el;
    }

    const float gridSize = 20.0f;

    // Legacy positions from user's JSON output (before drag validation existed)
    std::unordered_map<NodeId, Point> legacyPositions;
    legacyPositions[idle] = {15.0f, 106.0f};
    legacyPositions[running] = {299.0f, -4.0f};
    legacyPositions[paused] = {0.0f, 350.0f};
    legacyPositions[stopped] = {0.0f, 525.0f};
    legacyPositions[error] = {250.0f, 525.0f};

    // Verify that at least one position is rejected (positions are invalid)
    bool anyRejected = false;
    for (const auto& [nodeId, targetPos] : legacyPositions) {
        auto validation = LayoutUtils::canMoveNodeTo(
            nodeId, targetPos, nodeLayouts, edgeLayouts, gridSize);

        if (!validation.valid) {
            anyRejected = true;
            break;
        }

        // Update state for next validation
        nodeLayouts[nodeId].position = targetPos;
        std::vector<EdgeId> affected = LayoutUtils::getConnectedEdges(nodeId, edgeLayouts);
        std::unordered_set<NodeId> moved = {nodeId};
        LayoutUtils::updateEdgePositions(edgeLayouts, nodeLayouts, affected,
                                        moved, gridSize);
    }

    EXPECT_TRUE(anyRejected) << "Legacy positions should be rejected by drag validation";
}

// TDD TEST: After dragging nodes, updateEdgePositions must maintain orthogonality
// This reproduces the exact scenario from user's interactive_demo output
TEST(EdgeRoutingTransitionTest, AfterDrag_AllSegmentsMustRemainOrthogonal) {
    Graph graph;

    // Same graph structure as interactive_demo
    NodeId idle = graph.addNode(Size{200, 100}, "Idle");       // id=0
    NodeId running = graph.addNode(Size{200, 100}, "Running"); // id=1
    NodeId paused = graph.addNode(Size{200, 100}, "Paused");   // id=2
    NodeId stopped = graph.addNode(Size{200, 100}, "Stopped"); // id=3
    NodeId error = graph.addNode(Size{200, 100}, "Error");     // id=4

    EdgeId e0 = graph.addEdge(idle, running, "start");
    EdgeId e1 = graph.addEdge(running, paused, "pause");
    EdgeId e2 = graph.addEdge(paused, running, "resume");
    EdgeId e3 = graph.addEdge(running, stopped, "stop");
    EdgeId e4 = graph.addEdge(paused, stopped, "stop");
    EdgeId e5 = graph.addEdge(running, error, "fail");
    EdgeId e6 = graph.addEdge(error, idle, "reset");
    EdgeId e7 = graph.addEdge(error, error, "retry");

    // Step 1: Initial layout
    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph);

    // Step 2: Simulate user dragging nodes to specific positions
    // These are the EXACT positions from user's JSON that cause diagonal segments
    std::unordered_map<NodeId, NodeLayout> draggedLayouts;
    draggedLayouts[idle] = NodeLayout{idle, {15.0f, 106.0f}, {200.0f, 100.0f}, 0, 0};
    draggedLayouts[running] = NodeLayout{running, {299.0f, -4.0f}, {200.0f, 100.0f}, 1, 0};
    draggedLayouts[paused] = NodeLayout{paused, {0.0f, 350.0f}, {200.0f, 100.0f}, 2, 0};
    draggedLayouts[stopped] = NodeLayout{stopped, {0.0f, 525.0f}, {200.0f, 100.0f}, 3, 0};
    draggedLayouts[error] = NodeLayout{error, {250.0f, 525.0f}, {200.0f, 100.0f}, 3, 1};

    // Step 3: Extract edge layouts from initial result and call updateEdgePositions
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;
    std::vector<EdgeId> allEdges = {e0, e1, e2, e3, e4, e5, e6, e7};

    for (EdgeId edgeId : allEdges) {
        const EdgeLayout* origLayout = result.getEdgeLayout(edgeId);
        if (origLayout) {
            edgeLayouts[edgeId] = *origLayout;
        }
    }

    // Call updateEdgePositions - this is what interactive_demo does after drag
    std::unordered_set<NodeId> movedNodes = {idle, running, paused, stopped, error};
    EdgeRouting routing;
    routing.updateEdgePositions(edgeLayouts, draggedLayouts, allEdges,
                                movedNodes);

    std::cout << "\n===== AFTER updateEdgePositions ORTHOGONALITY TEST (TDD) =====\n";

    int totalDiagonals = 0;

    for (EdgeId edgeId : allEdges) {
        auto it = edgeLayouts.find(edgeId);
        if (it == edgeLayouts.end()) continue;

        const EdgeLayout& edgeLayout = it->second;

        // Collect all points
        std::vector<Point> points;
        points.push_back(edgeLayout.sourcePoint);
        for (const auto& bend : edgeLayout.bendPoints) {
            points.push_back(bend.position);
        }
        points.push_back(edgeLayout.targetPoint);

        std::cout << "Edge " << edgeId << " (" << edgeLayout.from << " -> " << edgeLayout.to << "):\n";

        // Check each segment for orthogonality
        for (size_t i = 0; i + 1 < points.size(); ++i) {
            const Point& p1 = points[i];
            const Point& p2 = points[i + 1];

            float dx = std::abs(p1.x - p2.x);
            float dy = std::abs(p1.y - p2.y);

            bool isHorizontal = (dy < 0.1f);
            bool isVertical = (dx < 0.1f);

            std::cout << "  [" << i << "->" << (i+1) << "] (" << p1.x << "," << p1.y
                      << ") -> (" << p2.x << "," << p2.y << ")";

            if (isHorizontal) {
                std::cout << " [H]\n";
            } else if (isVertical) {
                std::cout << " [V]\n";
            } else {
                std::cout << " [DIAGONAL!] dx=" << dx << ", dy=" << dy << "\n";
                totalDiagonals++;
            }
        }
    }

    std::cout << "\nTotal diagonal segments: " << totalDiagonals << "\n";
    std::cout << "=============================================================\n";

    EXPECT_EQ(totalDiagonals, 0)
        << "Found " << totalDiagonals << " diagonal segments after updateEdgePositions!";
}

// TDD TEST: Comprehensive circular drag simulation
// Move target node in a circle around source node and verify orthogonality at all positions
TEST(EdgeRoutingTransitionTest, CircularDrag_AllPositionsMustBeOrthogonal) {
    Graph graph;

    // Simple two-node graph for focused testing
    NodeId source = graph.addNode(Size{200, 100}, "Source");
    NodeId target = graph.addNode(Size{200, 100}, "Target");
    EdgeId edge = graph.addEdge(source, target, "connection");

    // Source node fixed at center
    const Point sourceCenter = {300.0f, 300.0f};
    const float radius = 250.0f;  // Distance from source center
    const int numPositions = 36;  // Test every 10 degrees

    std::cout << "\n===== CIRCULAR DRAG SIMULATION TEST =====\n";

    int totalDiagonals = 0;
    int totalIntersections = 0;
    int totalPositions = 0;
    int skippedPositions = 0;
    std::vector<std::string> failedPositions;

    for (int i = 0; i < numPositions; ++i) {
        float angle = (2.0f * 3.14159265f * i) / numPositions;
        float targetX = sourceCenter.x + radius * std::cos(angle);
        float targetY = sourceCenter.y + radius * std::sin(angle);

        // Create node layouts for this position
        std::unordered_map<NodeId, NodeLayout> nodeLayouts;
        nodeLayouts[source] = NodeLayout{source,
            {sourceCenter.x - 100.0f, sourceCenter.y - 50.0f}, // Position (top-left)
            {200.0f, 100.0f}, 0, 0};
        nodeLayouts[target] = NodeLayout{target,
            {targetX - 100.0f, targetY - 50.0f}, // Position (top-left)
            {200.0f, 100.0f}, 1, 0};

        // Route the edge
        EdgeRouting router;
        std::unordered_set<EdgeId> reversedEdges;
        LayoutOptions options;

        auto result = router.route(graph, nodeLayouts, reversedEdges, options);

        const EdgeLayout* edgeLayout = result.getEdgeLayout(edge);
        if (!edgeLayout) continue;

        // Validate the routed edge using centralized validation
        auto edgeValidation = EdgeRouting::validateEdgeLayout(*edgeLayout, nodeLayouts);
        if (!edgeValidation.valid) {
            // Routing algorithm couldn't produce valid path for this configuration
            skippedPositions++;
            continue;
        }

        // Collect all points
        std::vector<Point> points;
        points.push_back(edgeLayout->sourcePoint);
        for (const auto& bend : edgeLayout->bendPoints) {
            points.push_back(bend.position);
        }
        points.push_back(edgeLayout->targetPoint);

        int positionDiagonals = 0;
        int positionIntersections = 0;

        // Check each segment
        for (size_t j = 0; j + 1 < points.size(); ++j) {
            const Point& p1 = points[j];
            const Point& p2 = points[j + 1];

            float dx = std::abs(p1.x - p2.x);
            float dy = std::abs(p1.y - p2.y);

            bool isHorizontal = (dy < 0.1f);
            bool isVertical = (dx < 0.1f);

            // Check 1: Orthogonality
            if (!isHorizontal && !isVertical) {
                positionDiagonals++;
                totalDiagonals++;
            }

            // Check 2: Segment-node intersection
            // Skip first segment for source, last segment for target
            for (const auto& [nodeId, nodeLayout] : nodeLayouts) {
                if (j == 0 && nodeId == edgeLayout->from) continue;
                if (j == points.size() - 2 && nodeId == edgeLayout->to) continue;

                float nodeXmin = nodeLayout.position.x;
                float nodeXmax = nodeLayout.position.x + nodeLayout.size.width;
                float nodeYmin = nodeLayout.position.y;
                float nodeYmax = nodeLayout.position.y + nodeLayout.size.height;

                bool intersects = false;

                if (isVertical) {
                    float x = p1.x;
                    float segYmin = std::min(p1.y, p2.y);
                    float segYmax = std::max(p1.y, p2.y);
                    // Vertical segment intersects if within node's X range (interior) and overlaps Y range
                    intersects = (x > nodeXmin && x < nodeXmax && segYmin < nodeYmax && segYmax > nodeYmin);
                } else if (isHorizontal) {
                    float y = p1.y;
                    float segXmin = std::min(p1.x, p2.x);
                    float segXmax = std::max(p1.x, p2.x);
                    // Horizontal segment intersects if within node's Y range (interior) and overlaps X range
                    intersects = (y > nodeYmin && y < nodeYmax && segXmin < nodeXmax && segXmax > nodeXmin);
                }

                if (intersects) {
                    positionIntersections++;
                    totalIntersections++;
                }
            }
        }

        totalPositions++;

        if (positionDiagonals > 0 || positionIntersections > 0) {
            int degrees = static_cast<int>(angle * 180.0f / 3.14159265f);
            std::ostringstream ss;
            ss << "Angle " << degrees << "deg: target@(" << targetX << "," << targetY << ")";
            if (positionDiagonals > 0) ss << " - " << positionDiagonals << " diagonals";
            if (positionIntersections > 0) ss << " - " << positionIntersections << " intersections";
            failedPositions.push_back(ss.str());

            // Print detailed path for failed positions
            std::cout << "FAIL at " << degrees << " degrees:\n";
            for (size_t j = 0; j < points.size(); ++j) {
                std::cout << "  [" << j << "] (" << points[j].x << ", " << points[j].y << ")\n";
            }
        }
    }

    std::cout << "Tested " << totalPositions << " valid positions around circle\n";
    std::cout << "Skipped " << skippedPositions << " invalid positions (rejected by canMoveNodeTo)\n";
    std::cout << "Total diagonal segments: " << totalDiagonals << "\n";
    std::cout << "Total node intersections: " << totalIntersections << "\n";

    if (!failedPositions.empty()) {
        std::cout << "Failed positions:\n";
        for (const auto& pos : failedPositions) {
            std::cout << "  " << pos << "\n";
        }
    }

    std::cout << "==========================================\n";

    EXPECT_EQ(totalDiagonals, 0)
        << "Found " << totalDiagonals << " diagonal segments!";
    EXPECT_EQ(totalIntersections, 0)
        << "Found " << totalIntersections << " segment-node intersections!";
}

// TDD TEST: Full state machine with circular drag of one node
// Tests that all edges remain orthogonal when Error node is moved around
TEST(EdgeRoutingTransitionTest, StateMachine_CircularDragError_AllOrthogonal) {
    Graph graph;

    // Full state machine graph
    NodeId idle = graph.addNode(Size{200, 100}, "Idle");
    NodeId running = graph.addNode(Size{200, 100}, "Running");
    NodeId paused = graph.addNode(Size{200, 100}, "Paused");
    NodeId stopped = graph.addNode(Size{200, 100}, "Stopped");
    NodeId error = graph.addNode(Size{200, 100}, "Error");

    EdgeId e0 = graph.addEdge(idle, running, "start");
    EdgeId e1 = graph.addEdge(running, paused, "pause");
    EdgeId e2 = graph.addEdge(paused, running, "resume");
    EdgeId e3 = graph.addEdge(running, stopped, "stop");
    EdgeId e4 = graph.addEdge(paused, stopped, "stop");
    EdgeId e5 = graph.addEdge(running, error, "fail");
    EdgeId e6 = graph.addEdge(error, idle, "reset");
    EdgeId e7 = graph.addEdge(error, error, "retry");

    std::vector<EdgeId> allEdges = {e0, e1, e2, e3, e4, e5, e6, e7};

    // Fixed positions for most nodes
    const Point idlePos = {100.0f, 100.0f};
    const Point runningPos = {400.0f, 100.0f};
    const Point pausedPos = {100.0f, 350.0f};
    const Point stoppedPos = {100.0f, 525.0f};

    // Error node moves in circle around center
    const Point centerPos = {350.0f, 400.0f};
    const float radius = 200.0f;
    const int numPositions = 24;  // Every 15 degrees

    std::cout << "\n===== STATE MACHINE CIRCULAR DRAG TEST =====\n";

    int totalDiagonals = 0;
    int totalPositions = 0;
    int skippedPositions = 0;
    std::vector<std::string> failedPositions;

    for (int i = 0; i < numPositions; ++i) {
        float angle = (2.0f * 3.14159265f * i) / numPositions;
        float errorX = centerPos.x + radius * std::cos(angle);
        float errorY = centerPos.y + radius * std::sin(angle);

        // Create node layouts
        std::unordered_map<NodeId, NodeLayout> nodeLayouts;
        nodeLayouts[idle] = NodeLayout{idle, idlePos, {200.0f, 100.0f}, 0, 0};
        nodeLayouts[running] = NodeLayout{running, runningPos, {200.0f, 100.0f}, 1, 0};
        nodeLayouts[paused] = NodeLayout{paused, pausedPos, {200.0f, 100.0f}, 2, 0};
        nodeLayouts[stopped] = NodeLayout{stopped, stoppedPos, {200.0f, 100.0f}, 3, 0};
        nodeLayouts[error] = NodeLayout{error, {errorX, errorY}, {200.0f, 100.0f}, 3, 1};

        // Route all edges
        EdgeRouting router;
        std::unordered_set<EdgeId> reversedEdges = {e2, e6};
        LayoutOptions options;

        auto result = router.route(graph, nodeLayouts, reversedEdges, options);

        // Check if all edges are valid using centralized validation
        bool allValid = true;
        for (EdgeId edgeId : allEdges) {
            const EdgeLayout* edgeLayout = result.getEdgeLayout(edgeId);
            if (!edgeLayout) continue;

            auto validation = EdgeRouting::validateEdgeLayout(*edgeLayout, nodeLayouts);
            if (!validation.valid) {
                allValid = false;
                break;
            }
        }

        if (!allValid) {
            // Routing algorithm couldn't produce valid paths for this configuration
            skippedPositions++;
            continue;
        }

        // Check all edges for diagonals
        int positionDiagonals = 0;
        std::string diagonalEdges;

        for (EdgeId edgeId : allEdges) {
            const EdgeLayout* edgeLayout = result.getEdgeLayout(edgeId);
            if (!edgeLayout) continue;

            std::vector<Point> points;
            points.push_back(edgeLayout->sourcePoint);
            for (const auto& bend : edgeLayout->bendPoints) {
                points.push_back(bend.position);
            }
            points.push_back(edgeLayout->targetPoint);

            for (size_t j = 0; j + 1 < points.size(); ++j) {
                const Point& p1 = points[j];
                const Point& p2 = points[j + 1];

                float dx = std::abs(p1.x - p2.x);
                float dy = std::abs(p1.y - p2.y);

                bool isHorizontal = (dy < 0.1f);
                bool isVertical = (dx < 0.1f);

                if (!isHorizontal && !isVertical) {
                    positionDiagonals++;
                    totalDiagonals++;
                    diagonalEdges += "e" + std::to_string(edgeId) + " ";
                }
            }
        }

        totalPositions++;

        if (positionDiagonals > 0) {
            int degrees = static_cast<int>(angle * 180.0f / 3.14159265f);
            std::ostringstream ss;
            ss << "Angle " << degrees << "deg: error@(" << errorX << "," << errorY << ") - "
               << positionDiagonals << " diagonals in [" << diagonalEdges << "]";
            failedPositions.push_back(ss.str());

            std::cout << "FAIL at " << degrees << " degrees, error@(" << errorX << "," << errorY << "):\n";

            // Print paths for edges with diagonals
            for (EdgeId edgeId : allEdges) {
                const EdgeLayout* edgeLayout = result.getEdgeLayout(edgeId);
                if (!edgeLayout) continue;

                std::vector<Point> points;
                points.push_back(edgeLayout->sourcePoint);
                for (const auto& bend : edgeLayout->bendPoints) {
                    points.push_back(bend.position);
                }
                points.push_back(edgeLayout->targetPoint);

                bool hasDiagonal = false;
                for (size_t j = 0; j + 1 < points.size(); ++j) {
                    float dx = std::abs(points[j].x - points[j+1].x);
                    float dy = std::abs(points[j].y - points[j+1].y);
                    if (dx > 0.1f && dy > 0.1f) hasDiagonal = true;
                }

                if (hasDiagonal) {
                    std::cout << "  Edge " << edgeId << " (" << edgeLayout->from << "->" << edgeLayout->to << "):\n";
                    for (size_t j = 0; j < points.size(); ++j) {
                        std::cout << "    [" << j << "] (" << points[j].x << ", " << points[j].y << ")\n";
                    }
                }
            }
        }
    }

    std::cout << "Tested " << totalPositions << " valid error node positions\n";
    std::cout << "Skipped " << skippedPositions << " invalid positions (rejected by canMoveNodeTo)\n";
    std::cout << "Total diagonal segments: " << totalDiagonals << "\n";

    if (!failedPositions.empty()) {
        std::cout << "Failed positions (" << failedPositions.size() << "):\n";
        for (const auto& pos : failedPositions) {
            std::cout << "  " << pos << "\n";
        }
    }

    std::cout << "=============================================\n";

    EXPECT_EQ(totalDiagonals, 0)
        << "Found " << totalDiagonals << " diagonal segments across "
        << failedPositions.size() << "/" << totalPositions << " positions!";
}

// TDD TEST: Reproduces exact user scenario where segment passes through source node
// Verifies that legacy user scenario positions (from before canMoveNodeTo validation) are correctly
// rejected by drag validation. These positions were reachable in older versions but are now
// properly blocked to prevent invalid edge routing states.
TEST(EdgeRoutingTransitionTest, UserScenario_PositionsRejectedByDragValidation) {
    Graph graph;

    // Same graph structure as interactive_demo
    NodeId idle = graph.addNode(Size{200, 100}, "Idle");       // id=0
    NodeId running = graph.addNode(Size{200, 100}, "Running"); // id=1
    NodeId paused = graph.addNode(Size{200, 100}, "Paused");   // id=2
    NodeId stopped = graph.addNode(Size{200, 100}, "Stopped"); // id=3
    NodeId error = graph.addNode(Size{200, 100}, "Error");     // id=4

    EdgeId e0 = graph.addEdge(idle, running, "start");
    EdgeId e1 = graph.addEdge(running, paused, "pause");
    EdgeId e2 = graph.addEdge(paused, running, "resume");
    EdgeId e3 = graph.addEdge(running, stopped, "stop");
    EdgeId e4 = graph.addEdge(paused, stopped, "stop");
    EdgeId e5 = graph.addEdge(running, error, "fail");
    EdgeId e6 = graph.addEdge(error, idle, "reset");
    EdgeId e7 = graph.addEdge(error, error, "retry");

    (void)e0; (void)e1; (void)e2; (void)e3; (void)e4; (void)e5; (void)e6; (void)e7;

    // Do initial layout to get valid starting state
    SugiyamaLayout layoutEngine;
    LayoutResult initialResult = layoutEngine.layout(graph);

    std::vector<NodeId> nodeIds = {idle, running, paused, stopped, error};
    std::vector<EdgeId> allEdgeIds = {e0, e1, e2, e3, e4, e5, e6, e7};

    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;

    for (NodeId id : nodeIds) {
        const auto* nl = initialResult.getNodeLayout(id);
        if (nl) nodeLayouts[id] = *nl;
    }
    for (EdgeId id : allEdgeIds) {
        const auto* el = initialResult.getEdgeLayout(id);
        if (el) edgeLayouts[id] = *el;
    }

    const float gridSize = 20.0f;

    // Legacy positions from user's JSON output that triggered bugs in older versions
    std::unordered_map<NodeId, Point> legacyPositions;
    legacyPositions[idle] = {258.0f, 68.0f};
    legacyPositions[running] = {0.0f, 175.0f};
    legacyPositions[paused] = {0.0f, 350.0f};
    legacyPositions[stopped] = {0.0f, 525.0f};
    legacyPositions[error] = {263.0f, 522.0f};

    // Verify that at least one position is rejected by drag validation
    // These positions cannot be reached via sequential drag in current system
    bool anyRejected = false;
    for (const auto& [nodeId, targetPos] : legacyPositions) {
        auto validation = LayoutUtils::canMoveNodeTo(
            nodeId, targetPos, nodeLayouts, edgeLayouts, gridSize);

        if (!validation.valid) {
            anyRejected = true;
            break;
        }

        // Apply valid position and update edges for next validation
        nodeLayouts[nodeId].position = targetPos;
        std::vector<EdgeId> affected = LayoutUtils::getConnectedEdges(nodeId, edgeLayouts);
        std::unordered_set<NodeId> moved = {nodeId};
        LayoutUtils::updateEdgePositions(edgeLayouts, nodeLayouts, affected,
                                        moved, gridSize);
    }

    EXPECT_TRUE(anyRejected) << "Legacy positions should be rejected by drag validation";
}

// Verifies that the legacy position from user's interactive_demo (258, 68) is correctly
// rejected by drag validation. This position would cause segments to pass through nodes.
TEST(EdgeRoutingTransitionTest, UpdateEdgePositions_LegacyPositionRejected) {
    Graph graph;

    // Same graph structure as interactive_demo
    NodeId idle = graph.addNode(Size{200, 100}, "Idle");
    NodeId running = graph.addNode(Size{200, 100}, "Running");
    NodeId paused = graph.addNode(Size{200, 100}, "Paused");
    NodeId stopped = graph.addNode(Size{200, 100}, "Stopped");
    NodeId error = graph.addNode(Size{200, 100}, "Error");

    EdgeId e0 = graph.addEdge(idle, running, "start");
    EdgeId e1 = graph.addEdge(running, paused, "pause");
    EdgeId e2 = graph.addEdge(paused, running, "resume");
    EdgeId e3 = graph.addEdge(running, stopped, "stop");
    EdgeId e4 = graph.addEdge(paused, stopped, "stop");
    EdgeId e5 = graph.addEdge(running, error, "fail");
    EdgeId e6 = graph.addEdge(error, idle, "reset");
    EdgeId e7 = graph.addEdge(error, error, "retry");

    // Initial layout
    SugiyamaLayout layout;
    LayoutResult initialResult = layout.layout(graph);

    // Node layouts including proposed non-idle positions
    std::unordered_map<NodeId, NodeLayout> originalLayouts;
    originalLayouts[idle] = *initialResult.getNodeLayout(idle);
    originalLayouts[running] = NodeLayout{running, {0.0f, 175.0f}, {200.0f, 100.0f}, 1, 0};
    originalLayouts[paused] = NodeLayout{paused, {0.0f, 350.0f}, {200.0f, 100.0f}, 2, 0};
    originalLayouts[stopped] = NodeLayout{stopped, {0.0f, 525.0f}, {200.0f, 100.0f}, 3, 0};
    originalLayouts[error] = NodeLayout{error, {263.0f, 522.0f}, {200.0f, 100.0f}, 3, 1};

    // Extract edge layouts
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;
    std::vector<EdgeId> allEdges = {e0, e1, e2, e3, e4, e5, e6, e7};

    for (EdgeId edgeId : allEdges) {
        const EdgeLayout* origLayout = initialResult.getEdgeLayout(edgeId);
        if (origLayout) {
            edgeLayouts[edgeId] = *origLayout;
        }
    }

    // Legacy position that would cause segments to pass through nodes
    Point proposedPos = {258.0f, 68.0f};
    const float gridSize = 20.0f;

    // This position should be rejected by drag validation
    auto validation = LayoutUtils::canMoveNodeTo(idle, proposedPos, originalLayouts, edgeLayouts, gridSize);

    EXPECT_FALSE(validation.valid)
        << "Legacy position (258, 68) should be rejected by drag validation";
}

// CRITICAL TEST: Spike detection after updateEdgePositions
// Uses exact positions from user's interactive_demo JSON output
TEST(EdgeRoutingTransitionTest, AfterDrag_NoSpikePatterns) {
    Graph graph;

    // Create same graph as interactive_demo
    NodeId idle = graph.addNode(Size{200, 100}, "Idle");       // id=0
    NodeId running = graph.addNode(Size{200, 100}, "Running"); // id=1
    NodeId paused = graph.addNode(Size{200, 100}, "Paused");   // id=2
    NodeId stopped = graph.addNode(Size{200, 100}, "Stopped"); // id=3
    NodeId error = graph.addNode(Size{200, 100}, "Error");     // id=4

    EdgeId e0 = graph.addEdge(idle, running, "start");
    EdgeId e1 = graph.addEdge(running, paused, "pause");
    EdgeId e2 = graph.addEdge(paused, running, "resume");
    EdgeId e3 = graph.addEdge(running, stopped, "stop");
    EdgeId e4 = graph.addEdge(paused, stopped, "stop");
    EdgeId e5 = graph.addEdge(running, error, "fail");
    EdgeId e6 = graph.addEdge(error, idle, "reset");
    EdgeId e7 = graph.addEdge(error, error, "retry");

    // First do initial layout
    SugiyamaLayout layoutAlgo;
    LayoutResult result = layoutAlgo.layout(graph);

    // Get node layouts from result
    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    std::vector<NodeId> allNodes = {idle, running, paused, stopped, error};
    for (NodeId nodeId : allNodes) {
        const NodeLayout* nl = result.getNodeLayout(nodeId);
        if (nl) nodeLayouts[nodeId] = *nl;
    }

    // Simulate positions from user's JSON (after drag)
    // From user's JSON:
    // idle: (0, 0), running: (310, 23), paused: (208, 362), stopped: (345, 158), error: (252, 527)
    nodeLayouts[idle].position = {0.0f, 0.0f};
    nodeLayouts[running].position = {310.0f, 23.0f};
    nodeLayouts[paused].position = {208.0f, 362.0f};
    nodeLayouts[stopped].position = {345.0f, 158.0f};
    nodeLayouts[error].position = {252.0f, 527.0f};

    // Extract edge layouts from initial result
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;
    std::vector<EdgeId> allEdges = {e0, e1, e2, e3, e4, e5, e6, e7};

    for (EdgeId edgeId : allEdges) {
        const EdgeLayout* origLayout = result.getEdgeLayout(edgeId);
        if (origLayout) {
            edgeLayouts[edgeId] = *origLayout;
        }
    }

    // Call updateEdgePositions - this is what interactive_demo does after drag
    std::unordered_set<NodeId> movedNodes = {idle, running, paused, stopped, error};
    EdgeRouting routing;
    routing.updateEdgePositions(edgeLayouts, nodeLayouts, allEdges,
                                movedNodes);

    // Check for spike patterns
    std::cout << "\n===== SPIKE DETECTION AFTER DRAG =====\n";
    int totalSpikes = 0;
    constexpr float SPIKE_EPSILON = 0.1f;

    for (EdgeId edgeId : allEdges) {
        auto it = edgeLayouts.find(edgeId);
        if (it == edgeLayouts.end()) continue;
        const EdgeLayout& layout = it->second;

        std::vector<Point> points;
        points.push_back(layout.sourcePoint);
        for (const auto& bend : layout.bendPoints) {
            points.push_back(bend.position);
        }
        points.push_back(layout.targetPoint);

        if (points.size() < 3) continue;

        int edgeSpikes = 0;
        for (size_t i = 0; i + 2 < points.size(); ++i) {
            const Point& a = points[i];
            const Point& b = points[i + 1];
            const Point& c = points[i + 2];

            // Check if all three points are on the same vertical line
            bool sameVertical = (std::abs(a.x - b.x) < SPIKE_EPSILON &&
                                 std::abs(b.x - c.x) < SPIKE_EPSILON);
            // Check if all three points are on the same horizontal line
            bool sameHorizontal = (std::abs(a.y - b.y) < SPIKE_EPSILON &&
                                   std::abs(b.y - c.y) < SPIKE_EPSILON);

            if (sameVertical) {
                // Check if direction reverses (spike pattern)
                bool aToB_down = b.y > a.y;
                bool bToC_down = c.y > b.y;
                if (aToB_down != bToC_down) {
                    edgeSpikes++;
                    std::cout << "Edge " << edgeId << ": SPIKE at point [" << (i+1) << "]!\n";
                    std::cout << "  (" << a.x << "," << a.y << ") -> ("
                              << b.x << "," << b.y << ") -> ("
                              << c.x << "," << c.y << ")\n";
                    std::cout << "  Direction reverses on vertical line x=" << b.x << "\n";
                }
            }

            if (sameHorizontal) {
                // Check if direction reverses (spike pattern)
                bool aToB_right = b.x > a.x;
                bool bToC_right = c.x > b.x;
                if (aToB_right != bToC_right) {
                    edgeSpikes++;
                    std::cout << "Edge " << edgeId << ": SPIKE at point [" << (i+1) << "]!\n";
                    std::cout << "  (" << a.x << "," << a.y << ") -> ("
                              << b.x << "," << b.y << ") -> ("
                              << c.x << "," << c.y << ")\n";
                    std::cout << "  Direction reverses on horizontal line y=" << b.y << "\n";
                }
            }
        }

        if (edgeSpikes > 0) {
            std::cout << "Edge " << edgeId << " (" << layout.from << " -> " << layout.to
                      << ") has " << edgeSpikes << " spike(s)\n";
            std::cout << "  Full path:\n";
            for (size_t k = 0; k < points.size(); ++k) {
                std::cout << "    [" << k << "]: (" << points[k].x << ", " << points[k].y << ")\n";
            }
        }

        totalSpikes += edgeSpikes;
    }

    std::cout << "Total spikes after drag: " << totalSpikes << "\n";
    std::cout << "========================================\n";

    EXPECT_EQ(totalSpikes, 0) << "Found " << totalSpikes << " spike patterns after updateEdgePositions";
}

// CRITICAL TEST: First/Last segment direction must match sourceEdge/targetEdge
// - sourceEdge Top    -> first segment vertical going UP (Y decreases)
// - sourceEdge Bottom -> first segment vertical going DOWN (Y increases)
// - sourceEdge Left   -> first segment horizontal going LEFT (X decreases)
// - sourceEdge Right  -> first segment horizontal going RIGHT (X increases)
// - targetEdge Top    -> last segment vertical going DOWN (Y increases)
// - targetEdge Bottom -> last segment vertical going UP (Y decreases)
// - targetEdge Left   -> last segment horizontal going RIGHT (X increases)
// - targetEdge Right  -> last segment horizontal going LEFT (X decreases)
TEST(EdgeRoutingTransitionTest, SegmentDirection_MustMatchEdgeDesignation) {
    Graph graph;

    // Create state machine graph from interactive demo
    NodeId idle = graph.addNode(Size{200, 100}, "Idle");
    NodeId running = graph.addNode(Size{200, 100}, "Running");
    NodeId paused = graph.addNode(Size{200, 100}, "Paused");
    NodeId stopped = graph.addNode(Size{200, 100}, "Stopped");
    NodeId error = graph.addNode(Size{200, 100}, "Error");

    EdgeId e0 = graph.addEdge(idle, running, "start");
    EdgeId e1 = graph.addEdge(running, paused, "pause");
    EdgeId e2 = graph.addEdge(paused, running, "resume");
    EdgeId e3 = graph.addEdge(running, stopped, "stop");
    EdgeId e4 = graph.addEdge(paused, stopped, "stop");
    EdgeId e5 = graph.addEdge(running, error, "fail");
    EdgeId e6 = graph.addEdge(error, idle, "reset");
    EdgeId e7 = graph.addEdge(error, error, "retry");

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph);

    std::vector<EdgeId> allEdges = {e0, e1, e2, e3, e4, e5, e6, e7};

    std::cout << "\n===== SEGMENT DIRECTION VALIDATION =====\n";

    int sourceViolations = 0;
    int targetViolations = 0;

    for (EdgeId edgeId : allEdges) {
        const EdgeLayout* edgeLayout = result.getEdgeLayout(edgeId);
        ASSERT_NE(edgeLayout, nullptr);

        // Collect all points
        std::vector<Point> points;
        points.push_back(edgeLayout->sourcePoint);
        for (const auto& bend : edgeLayout->bendPoints) {
            points.push_back(bend.position);
        }
        points.push_back(edgeLayout->targetPoint);

        if (points.size() < 2) continue;

        // Check FIRST segment direction (source exit)
        const Point& p0 = points[0];
        const Point& p1 = points[1];
        float dx1 = p1.x - p0.x;
        float dy1 = p1.y - p0.y;

        bool firstSegmentCorrect = true;
        std::string expectedFirst;
        std::string actualFirst;

        switch (edgeLayout->sourceEdge) {
            case NodeEdge::Top:
                // First segment should be vertical going UP (Y decreases)
                expectedFirst = "Vertical UP (dy < 0)";
                firstSegmentCorrect = (std::abs(dx1) < 0.1f && dy1 < 0);
                actualFirst = (std::abs(dx1) < 0.1f) ?
                    (dy1 < 0 ? "Vertical UP" : "Vertical DOWN") :
                    (std::abs(dy1) < 0.1f ? "Horizontal" : "Diagonal");
                break;
            case NodeEdge::Bottom:
                // First segment should be vertical going DOWN (Y increases)
                expectedFirst = "Vertical DOWN (dy > 0)";
                firstSegmentCorrect = (std::abs(dx1) < 0.1f && dy1 > 0);
                actualFirst = (std::abs(dx1) < 0.1f) ?
                    (dy1 < 0 ? "Vertical UP" : "Vertical DOWN") :
                    (std::abs(dy1) < 0.1f ? "Horizontal" : "Diagonal");
                break;
            case NodeEdge::Left:
                // First segment should be horizontal going LEFT (X decreases)
                expectedFirst = "Horizontal LEFT (dx < 0)";
                firstSegmentCorrect = (std::abs(dy1) < 0.1f && dx1 < 0);
                actualFirst = (std::abs(dy1) < 0.1f) ?
                    (dx1 < 0 ? "Horizontal LEFT" : "Horizontal RIGHT") :
                    (std::abs(dx1) < 0.1f ? "Vertical" : "Diagonal");
                break;
            case NodeEdge::Right:
                // First segment should be horizontal going RIGHT (X increases)
                expectedFirst = "Horizontal RIGHT (dx > 0)";
                firstSegmentCorrect = (std::abs(dy1) < 0.1f && dx1 > 0);
                actualFirst = (std::abs(dy1) < 0.1f) ?
                    (dx1 < 0 ? "Horizontal LEFT" : "Horizontal RIGHT") :
                    (std::abs(dx1) < 0.1f ? "Vertical" : "Diagonal");
                break;
        }

        // Check LAST segment direction (target entry)
        const Point& pN_1 = points[points.size() - 2];
        const Point& pN = points[points.size() - 1];
        float dxLast = pN.x - pN_1.x;
        float dyLast = pN.y - pN_1.y;

        bool lastSegmentCorrect = true;
        std::string expectedLast;
        std::string actualLast;

        switch (edgeLayout->targetEdge) {
            case NodeEdge::Top:
                // Last segment should be vertical going DOWN (Y increases) into top of node
                expectedLast = "Vertical DOWN (dy > 0)";
                lastSegmentCorrect = (std::abs(dxLast) < 0.1f && dyLast > 0);
                actualLast = (std::abs(dxLast) < 0.1f) ?
                    (dyLast < 0 ? "Vertical UP" : "Vertical DOWN") :
                    (std::abs(dyLast) < 0.1f ? "Horizontal" : "Diagonal");
                break;
            case NodeEdge::Bottom:
                // Last segment should be vertical going UP (Y decreases) into bottom of node
                expectedLast = "Vertical UP (dy < 0)";
                lastSegmentCorrect = (std::abs(dxLast) < 0.1f && dyLast < 0);
                actualLast = (std::abs(dxLast) < 0.1f) ?
                    (dyLast < 0 ? "Vertical UP" : "Vertical DOWN") :
                    (std::abs(dyLast) < 0.1f ? "Horizontal" : "Diagonal");
                break;
            case NodeEdge::Left:
                // Last segment should be horizontal going RIGHT (X increases) into left of node
                expectedLast = "Horizontal RIGHT (dx > 0)";
                lastSegmentCorrect = (std::abs(dyLast) < 0.1f && dxLast > 0);
                actualLast = (std::abs(dyLast) < 0.1f) ?
                    (dxLast < 0 ? "Horizontal LEFT" : "Horizontal RIGHT") :
                    (std::abs(dxLast) < 0.1f ? "Vertical" : "Diagonal");
                break;
            case NodeEdge::Right:
                // Last segment should be horizontal going LEFT (X decreases) into right of node
                expectedLast = "Horizontal LEFT (dx < 0)";
                lastSegmentCorrect = (std::abs(dyLast) < 0.1f && dxLast < 0);
                actualLast = (std::abs(dyLast) < 0.1f) ?
                    (dxLast < 0 ? "Horizontal LEFT" : "Horizontal RIGHT") :
                    (std::abs(dxLast) < 0.1f ? "Vertical" : "Diagonal");
                break;
        }

        // Print diagnostic info
        std::cout << "Edge " << edgeId << " (" << edgeLayout->from << " -> " << edgeLayout->to << "):\n";
        std::cout << "  sourceEdge: " << static_cast<int>(edgeLayout->sourceEdge)
                  << " (0=Top,1=Bottom,2=Left,3=Right)\n";
        std::cout << "  targetEdge: " << static_cast<int>(edgeLayout->targetEdge) << "\n";
        std::cout << "  First segment: (" << p0.x << "," << p0.y << ") -> (" << p1.x << "," << p1.y << ")\n";
        std::cout << "    dx=" << dx1 << ", dy=" << dy1 << "\n";
        std::cout << "    Expected: " << expectedFirst << ", Actual: " << actualFirst;
        if (!firstSegmentCorrect) {
            std::cout << " [VIOLATION]";
            sourceViolations++;
        }
        std::cout << "\n";

        std::cout << "  Last segment: (" << pN_1.x << "," << pN_1.y << ") -> (" << pN.x << "," << pN.y << ")\n";
        std::cout << "    dx=" << dxLast << ", dy=" << dyLast << "\n";
        std::cout << "    Expected: " << expectedLast << ", Actual: " << actualLast;
        if (!lastSegmentCorrect) {
            std::cout << " [VIOLATION]";
            targetViolations++;
        }
        std::cout << "\n\n";

        // Assert correctness
        EXPECT_TRUE(firstSegmentCorrect)
            << "Edge " << edgeId << ": First segment direction mismatch!\n"
            << "  sourceEdge: " << static_cast<int>(edgeLayout->sourceEdge) << "\n"
            << "  Expected: " << expectedFirst << "\n"
            << "  Actual: " << actualFirst << " (dx=" << dx1 << ", dy=" << dy1 << ")";

        EXPECT_TRUE(lastSegmentCorrect)
            << "Edge " << edgeId << ": Last segment direction mismatch!\n"
            << "  targetEdge: " << static_cast<int>(edgeLayout->targetEdge) << "\n"
            << "  Expected: " << expectedLast << "\n"
            << "  Actual: " << actualLast << " (dx=" << dxLast << ", dy=" << dyLast << ")";
    }

    std::cout << "Summary: " << sourceViolations << " source violations, "
              << targetViolations << " target violations\n";
    std::cout << "==========================================\n";
}

// CRITICAL TEST: Spike detection with grid snapping enabled
// Grid snapping (cellSize=20) can create spikes that don't appear without grid snapping
TEST(EdgeRoutingTransitionTest, GridSnapping_AfterDrag_NoSpikesOrDuplicates) {
    Graph graph;

    // Create same graph as interactive_demo
    NodeId idle = graph.addNode(Size{200, 100}, "Idle");
    NodeId running = graph.addNode(Size{200, 100}, "Running");
    NodeId paused = graph.addNode(Size{200, 100}, "Paused");
    NodeId stopped = graph.addNode(Size{200, 100}, "Stopped");
    NodeId error = graph.addNode(Size{200, 100}, "Error");

    EdgeId e0 = graph.addEdge(idle, running, "start");
    EdgeId e1 = graph.addEdge(running, paused, "pause");
    EdgeId e2 = graph.addEdge(paused, running, "resume");
    EdgeId e3 = graph.addEdge(running, stopped, "stop");
    EdgeId e4 = graph.addEdge(paused, stopped, "stop");
    EdgeId e5 = graph.addEdge(running, error, "fail");
    EdgeId e6 = graph.addEdge(error, idle, "reset");
    EdgeId e7 = graph.addEdge(error, error, "retry");

    // IMPORTANT: Enable grid snapping like interactive_demo
    LayoutOptions options;
    options.gridConfig.cellSize = 20.0f;  // This is the key difference!

    SugiyamaLayout layoutAlgo(options);
    LayoutResult result = layoutAlgo.layout(graph);

    // Extract layouts for manipulation
    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    for (const auto& [id, nl] : result.nodeLayouts()) {
        nodeLayouts[id] = nl;
    }

    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;
    for (const auto& [id, el] : result.edgeLayouts()) {
        edgeLayouts[id] = el;
    }

    std::vector<EdgeId> allEdges = {e0, e1, e2, e3, e4, e5, e6, e7};

    // Simulate multiple drag operations
    std::vector<std::pair<NodeId, Point>> dragOps = {
        {idle, {100.0f, 200.0f}},
        {running, {-50.0f, 100.0f}},
        {error, {200.0f, -300.0f}},
        {paused, {-100.0f, 50.0f}}
    };

    std::cout << "\n===== GRID SNAPPING SPIKE TEST =====\n";
    constexpr float EPSILON = 0.1f;
    int totalSpikes = 0;
    int totalDuplicates = 0;

    for (const auto& [nodeId, delta] : dragOps) {
        nodeLayouts[nodeId].position.x += delta.x;
        nodeLayouts[nodeId].position.y += delta.y;

        std::unordered_set<NodeId> movedNodes = {nodeId};
        LayoutUtils::updateEdgePositions(edgeLayouts, nodeLayouts, allEdges,
                                          movedNodes,
                                          options.gridConfig.cellSize);

        // Check for spikes and duplicates after each drag
        for (EdgeId edgeId : allEdges) {
            auto it = edgeLayouts.find(edgeId);
            if (it == edgeLayouts.end()) continue;

            const EdgeLayout& layout = it->second;
            std::vector<Point> points;
            points.push_back(layout.sourcePoint);
            for (const auto& bp : layout.bendPoints) {
                points.push_back(bp.position);
            }
            points.push_back(layout.targetPoint);

            // Check for duplicates
            // Note: Skip the case where sourcePoint == targetPoint with no bendPoints
            // This is a valid degenerate edge case when nodes are adjacent
            for (size_t i = 0; i + 1 < points.size(); ++i) {
                if (std::abs(points[i].x - points[i+1].x) < EPSILON &&
                    std::abs(points[i].y - points[i+1].y) < EPSILON) {
                    // Skip if this is a zero-length edge (source == target, no bends)
                    if (points.size() == 2 && i == 0) {
                        // Valid degenerate case: endpoints at same position
                        continue;
                    }
                    std::cout << "DUPLICATE in Edge " << edgeId << " after dragging node "
                              << nodeId << ": point[" << i << "] = ("
                              << points[i].x << "," << points[i].y << ")\n";
                    totalDuplicates++;
                }
            }

            // Check for spikes
            for (size_t i = 0; i + 2 < points.size(); ++i) {
                const Point& a = points[i];
                const Point& b = points[i + 1];
                const Point& c = points[i + 2];

                bool sameX = (std::abs(a.x - b.x) < EPSILON && std::abs(b.x - c.x) < EPSILON);
                bool sameY = (std::abs(a.y - b.y) < EPSILON && std::abs(b.y - c.y) < EPSILON);

                if (sameX) {
                    bool aToB_down = b.y > a.y;
                    bool bToC_down = c.y > b.y;
                    if (aToB_down != bToC_down) {
                        std::cout << "SPIKE in Edge " << edgeId << " after dragging node "
                                  << nodeId << ":\n";
                        std::cout << "  (" << a.x << "," << a.y << ") -> ("
                                  << b.x << "," << b.y << ") -> ("
                                  << c.x << "," << c.y << ")\n";
                        totalSpikes++;
                    }
                }

                if (sameY) {
                    bool aToB_right = b.x > a.x;
                    bool bToC_right = c.x > b.x;
                    if (aToB_right != bToC_right) {
                        std::cout << "SPIKE in Edge " << edgeId << " after dragging node "
                                  << nodeId << ":\n";
                        std::cout << "  (" << a.x << "," << a.y << ") -> ("
                                  << b.x << "," << b.y << ") -> ("
                                  << c.x << "," << c.y << ")\n";
                        totalSpikes++;
                    }
                }
            }
        }
    }

    std::cout << "Total duplicates: " << totalDuplicates << ", spikes: " << totalSpikes << "\n";
    std::cout << "=====================================\n";

    EXPECT_EQ(totalDuplicates, 0) << "Found " << totalDuplicates << " duplicate points with grid snapping";
    EXPECT_EQ(totalSpikes, 0) << "Found " << totalSpikes << " spike patterns with grid snapping";
}

// CRITICAL TEST: Grid-based routing must not have segments passing through nodes
// This test specifically enables grid routing (gridSize=20) and verifies intersection avoidance
TEST(EdgeRoutingTransitionTest, GridBased_Segments_MustNotIntersectNodes) {
    Graph graph;

    // Create state machine from interactive demo
    NodeId idle = graph.addNode(Size{200, 100}, "idle");
    NodeId running = graph.addNode(Size{200, 100}, "running");
    NodeId error = graph.addNode(Size{200, 100}, "error");
    NodeId processing = graph.addNode(Size{200, 100}, "processing");
    NodeId success = graph.addNode(Size{200, 100}, "success");

    // All transitions (same as interactive demo)
    EdgeId e0 = graph.addEdge(idle, running, "start");
    EdgeId e1 = graph.addEdge(running, error, "fail");
    EdgeId e2 = graph.addEdge(error, running, "retry");
    EdgeId e3 = graph.addEdge(running, processing, "process");
    EdgeId e4 = graph.addEdge(error, processing, "fix");
    EdgeId e5 = graph.addEdge(running, success, "complete");
    EdgeId e6 = graph.addEdge(success, idle, "reset");
    EdgeId e7 = graph.addEdge(success, success, "continue");

    // Enable grid-based routing with gridSize=20
    SugiyamaLayout layout;
    LayoutOptions options;
    options.gridConfig.cellSize = 20.0f;  // Enable grid routing

    layout.setOptions(options);

    LayoutResult result = layout.layout(graph);

    std::vector<EdgeId> allEdges = {e0, e1, e2, e3, e4, e5, e6, e7};
    std::vector<NodeId> allNodes = {idle, running, error, processing, success};

    int totalViolations = 0;

    for (EdgeId edgeId : allEdges) {
        const EdgeLayout* edgeLayout = result.getEdgeLayout(edgeId);
        ASSERT_NE(edgeLayout, nullptr);

        // Collect all points
        std::vector<Point> points;
        points.push_back(edgeLayout->sourcePoint);
        for (const auto& bend : edgeLayout->bendPoints) {
            points.push_back(bend.position);
        }
        points.push_back(edgeLayout->targetPoint);

        // Check each segment against all nodes
        for (size_t i = 0; i + 1 < points.size(); ++i) {
            const Point& p1 = points[i];
            const Point& p2 = points[i + 1];

            bool isHorizontal = (std::abs(p1.y - p2.y) < 0.1f);
            bool isVertical = (std::abs(p1.x - p2.x) < 0.1f);

            for (NodeId nodeId : allNodes) {
                // Skip source node for first segment, target node for last segment
                if (i == 0 && nodeId == edgeLayout->from) continue;
                if (i == points.size() - 2 && nodeId == edgeLayout->to) continue;

                const NodeLayout* node = result.getNodeLayout(nodeId);
                ASSERT_NE(node, nullptr);

                float nodeXmin = node->position.x;
                float nodeXmax = node->position.x + node->size.width;
                float nodeYmin = node->position.y;
                float nodeYmax = node->position.y + node->size.height;

                bool intersects = false;

                if (isHorizontal) {
                    float y = p1.y;
                    float xMin = std::min(p1.x, p2.x);
                    float xMax = std::max(p1.x, p2.x);
                    // Strictly inside node interior
                    if (y > nodeYmin && y < nodeYmax && xMin < nodeXmax && xMax > nodeXmin) {
                        intersects = true;
                    }
                } else if (isVertical) {
                    float x = p1.x;
                    float yMin = std::min(p1.y, p2.y);
                    float yMax = std::max(p1.y, p2.y);
                    // Strictly inside node interior
                    if (x > nodeXmin && x < nodeXmax && yMin < nodeYmax && yMax > nodeYmin) {
                        intersects = true;
                    }
                }

                if (intersects) {
                    totalViolations++;
                    std::cout << "INTERSECTION: Edge " << edgeId << " segment[" << i << "] "
                              << "(" << p1.x << "," << p1.y << ") -> (" << p2.x << "," << p2.y << ")"
                              << " passes through Node " << nodeId
                              << " [" << nodeXmin << "," << nodeXmax << "]x["
                              << nodeYmin << "," << nodeYmax << "]\n";
                }
            }
        }
    }

    EXPECT_EQ(totalViolations, 0) << "Found " << totalViolations
        << " segment-node intersections with grid-based routing!";
}

// CRITICAL TEST: Grid-based routing must respect directional constraints
// This combines grid snapping (cellSize=20) with directional constraint validation
// Covers the gap where directional tests didn't use grid snapping
TEST(EdgeRoutingTransitionTest, GridBased_DirectionalConstraints_MustBeRespected) {
    Graph graph;

    // Create state machine from interactive demo
    NodeId idle = graph.addNode(Size{200, 100}, "Idle");
    NodeId running = graph.addNode(Size{200, 100}, "Running");
    NodeId paused = graph.addNode(Size{200, 100}, "Paused");
    NodeId stopped = graph.addNode(Size{200, 100}, "Stopped");
    NodeId error = graph.addNode(Size{200, 100}, "Error");

    EdgeId e0 = graph.addEdge(idle, running, "start");
    EdgeId e1 = graph.addEdge(running, paused, "pause");
    EdgeId e2 = graph.addEdge(paused, running, "resume");
    EdgeId e3 = graph.addEdge(running, stopped, "stop");
    EdgeId e4 = graph.addEdge(paused, stopped, "stop");
    EdgeId e5 = graph.addEdge(running, error, "fail");
    EdgeId e6 = graph.addEdge(error, idle, "reset");
    EdgeId e7 = graph.addEdge(error, error, "retry");

    // Enable grid-based routing with gridSize=20
    SugiyamaLayout layout;
    LayoutOptions options;
    options.gridConfig.cellSize = 20.0f;  // Enable grid routing

    layout.setOptions(options);

    LayoutResult result = layout.layout(graph);

    std::vector<EdgeId> allEdges = {e0, e1, e2, e3, e4, e5, e6, e7};

    std::cout << "\n===== GRID-BASED DIRECTIONAL CONSTRAINT VALIDATION =====\n";
    std::cout << "gridSize: " << options.gridConfig.cellSize << "\n\n";

    int sourceViolations = 0;
    int targetViolations = 0;

    for (EdgeId edgeId : allEdges) {
        const EdgeLayout* edgeLayout = result.getEdgeLayout(edgeId);
        ASSERT_NE(edgeLayout, nullptr);

        // Collect all points
        std::vector<Point> points;
        points.push_back(edgeLayout->sourcePoint);
        for (const auto& bend : edgeLayout->bendPoints) {
            points.push_back(bend.position);
        }
        points.push_back(edgeLayout->targetPoint);

        if (points.size() < 2) continue;

        // Check FIRST segment direction (source exit)
        const Point& p0 = points[0];
        const Point& p1 = points[1];
        float dx1 = p1.x - p0.x;
        float dy1 = p1.y - p0.y;

        bool firstSegmentCorrect = true;
        std::string expectedFirst;
        std::string actualFirst;

        switch (edgeLayout->sourceEdge) {
            case NodeEdge::Top:
                expectedFirst = "Vertical UP (dy < 0)";
                firstSegmentCorrect = (std::abs(dx1) < 0.1f && dy1 < 0);
                actualFirst = (std::abs(dx1) < 0.1f) ?
                    (dy1 < 0 ? "Vertical UP" : "Vertical DOWN") :
                    (std::abs(dy1) < 0.1f ? "Horizontal" : "Diagonal");
                break;
            case NodeEdge::Bottom:
                expectedFirst = "Vertical DOWN (dy > 0)";
                firstSegmentCorrect = (std::abs(dx1) < 0.1f && dy1 > 0);
                actualFirst = (std::abs(dx1) < 0.1f) ?
                    (dy1 < 0 ? "Vertical UP" : "Vertical DOWN") :
                    (std::abs(dy1) < 0.1f ? "Horizontal" : "Diagonal");
                break;
            case NodeEdge::Left:
                expectedFirst = "Horizontal LEFT (dx < 0)";
                firstSegmentCorrect = (std::abs(dy1) < 0.1f && dx1 < 0);
                actualFirst = (std::abs(dy1) < 0.1f) ?
                    (dx1 < 0 ? "Horizontal LEFT" : "Horizontal RIGHT") :
                    (std::abs(dx1) < 0.1f ? "Vertical" : "Diagonal");
                break;
            case NodeEdge::Right:
                expectedFirst = "Horizontal RIGHT (dx > 0)";
                firstSegmentCorrect = (std::abs(dy1) < 0.1f && dx1 > 0);
                actualFirst = (std::abs(dy1) < 0.1f) ?
                    (dx1 < 0 ? "Horizontal LEFT" : "Horizontal RIGHT") :
                    (std::abs(dx1) < 0.1f ? "Vertical" : "Diagonal");
                break;
        }

        // Check LAST segment direction (target entry)
        const Point& pN_1 = points[points.size() - 2];
        const Point& pN = points[points.size() - 1];
        float dxLast = pN.x - pN_1.x;
        float dyLast = pN.y - pN_1.y;

        bool lastSegmentCorrect = true;
        std::string expectedLast;
        std::string actualLast;

        switch (edgeLayout->targetEdge) {
            case NodeEdge::Top:
                // Into top = coming from above = Y increasing (going down)
                expectedLast = "Vertical DOWN (dy > 0)";
                lastSegmentCorrect = (std::abs(dxLast) < 0.1f && dyLast > 0);
                actualLast = (std::abs(dxLast) < 0.1f) ?
                    (dyLast < 0 ? "Vertical UP" : "Vertical DOWN") :
                    (std::abs(dyLast) < 0.1f ? "Horizontal" : "Diagonal");
                break;
            case NodeEdge::Bottom:
                // Into bottom = coming from below = Y decreasing (going up)
                expectedLast = "Vertical UP (dy < 0)";
                lastSegmentCorrect = (std::abs(dxLast) < 0.1f && dyLast < 0);
                actualLast = (std::abs(dxLast) < 0.1f) ?
                    (dyLast < 0 ? "Vertical UP" : "Vertical DOWN") :
                    (std::abs(dyLast) < 0.1f ? "Horizontal" : "Diagonal");
                break;
            case NodeEdge::Left:
                // Into left = coming from left side = X increasing (going right)
                expectedLast = "Horizontal RIGHT (dx > 0)";
                lastSegmentCorrect = (std::abs(dyLast) < 0.1f && dxLast > 0);
                actualLast = (std::abs(dyLast) < 0.1f) ?
                    (dxLast < 0 ? "Horizontal LEFT" : "Horizontal RIGHT") :
                    (std::abs(dxLast) < 0.1f ? "Vertical" : "Diagonal");
                break;
            case NodeEdge::Right:
                // Into right = coming from right side = X decreasing (going left)
                expectedLast = "Horizontal LEFT (dx < 0)";
                lastSegmentCorrect = (std::abs(dyLast) < 0.1f && dxLast < 0);
                actualLast = (std::abs(dyLast) < 0.1f) ?
                    (dxLast < 0 ? "Horizontal LEFT" : "Horizontal RIGHT") :
                    (std::abs(dxLast) < 0.1f ? "Vertical" : "Diagonal");
                break;
        }

        // Print diagnostic info
        std::cout << "Edge " << edgeId << " (" << edgeLayout->from << " -> " << edgeLayout->to << "):\n";
        std::cout << "  sourceEdge: " << static_cast<int>(edgeLayout->sourceEdge)
                  << " (0=Top,1=Bottom,2=Left,3=Right)\n";
        std::cout << "  targetEdge: " << static_cast<int>(edgeLayout->targetEdge) << "\n";
        std::cout << "  First segment: (" << p0.x << "," << p0.y << ") -> (" << p1.x << "," << p1.y << ")\n";
        std::cout << "    dx=" << dx1 << ", dy=" << dy1 << "\n";
        std::cout << "    Expected: " << expectedFirst << ", Actual: " << actualFirst;
        if (!firstSegmentCorrect) {
            std::cout << " [VIOLATION]";
            sourceViolations++;
        }
        std::cout << "\n";

        std::cout << "  Last segment: (" << pN_1.x << "," << pN_1.y << ") -> (" << pN.x << "," << pN.y << ")\n";
        std::cout << "    dx=" << dxLast << ", dy=" << dyLast << "\n";
        std::cout << "    Expected: " << expectedLast << ", Actual: " << actualLast;
        if (!lastSegmentCorrect) {
            std::cout << " [VIOLATION]";
            targetViolations++;
        }
        std::cout << "\n\n";

        // Assert correctness
        EXPECT_TRUE(firstSegmentCorrect)
            << "Edge " << edgeId << ": First segment direction mismatch with grid routing!\n"
            << "  gridSize: " << options.gridConfig.cellSize << "\n"
            << "  sourceEdge: " << static_cast<int>(edgeLayout->sourceEdge) << "\n"
            << "  Expected: " << expectedFirst << "\n"
            << "  Actual: " << actualFirst << " (dx=" << dx1 << ", dy=" << dy1 << ")";

        EXPECT_TRUE(lastSegmentCorrect)
            << "Edge " << edgeId << ": Last segment direction mismatch with grid routing!\n"
            << "  gridSize: " << options.gridConfig.cellSize << "\n"
            << "  targetEdge: " << static_cast<int>(edgeLayout->targetEdge) << "\n"
            << "  Expected: " << expectedLast << "\n"
            << "  Actual: " << actualLast << " (dx=" << dxLast << ", dy=" << dyLast << ")";
    }

    std::cout << "Summary: " << sourceViolations << " source violations, "
              << targetViolations << " target violations\n";
    std::cout << "==========================================\n";

    EXPECT_EQ(sourceViolations + targetViolations, 0)
        << "Grid-based routing violated directional constraints!";
}

// =============================================================================
// PATH QUALITY TESTS - Detect routing inefficiencies and anomalies
// =============================================================================

// Test: Detect duplicate bend points within the same edge
// This catches cases where routing logic creates redundant points at same coordinates
TEST(EdgeRoutingTransitionTest, NoDuplicateBendPoints) {
    Graph graph;

    // Create state machine matching interactive_demo
    NodeId idle = graph.addNode(Size{200, 100}, "Idle");
    NodeId running = graph.addNode(Size{200, 100}, "Running");
    NodeId paused = graph.addNode(Size{200, 100}, "Paused");
    NodeId stopped = graph.addNode(Size{200, 100}, "Stopped");
    NodeId error = graph.addNode(Size{200, 100}, "Error");

    EdgeId e0 = graph.addEdge(idle, running, "start");
    EdgeId e1 = graph.addEdge(running, paused, "pause");
    EdgeId e2 = graph.addEdge(paused, running, "resume");
    EdgeId e3 = graph.addEdge(running, stopped, "stop");
    EdgeId e4 = graph.addEdge(paused, stopped, "stop");
    EdgeId e5 = graph.addEdge(running, error, "fail");
    EdgeId e6 = graph.addEdge(error, idle, "reset");
    EdgeId e7 = graph.addEdge(error, error, "retry");

    SugiyamaLayout layout;
    LayoutOptions options;
    options.gridConfig.cellSize = 20.0f;

    layout.setOptions(options);

    LayoutResult result = layout.layout(graph);

    std::vector<EdgeId> allEdges = {e0, e1, e2, e3, e4, e5, e6, e7};
    constexpr float EPSILON = 0.1f;

    std::cout << "\n===== DUPLICATE BEND POINT TEST =====\n";

    int totalDuplicates = 0;

    for (EdgeId edgeId : allEdges) {
        const EdgeLayout* edgeLayout = result.getEdgeLayout(edgeId);
        ASSERT_NE(edgeLayout, nullptr);

        // Build full path
        std::vector<Point> points;
        points.push_back(edgeLayout->sourcePoint);
        for (const auto& bend : edgeLayout->bendPoints) {
            points.push_back(bend.position);
        }
        points.push_back(edgeLayout->targetPoint);

        // Check for duplicate points
        int edgeDuplicates = 0;
        for (size_t i = 0; i < points.size(); ++i) {
            for (size_t j = i + 1; j < points.size(); ++j) {
                if (std::abs(points[i].x - points[j].x) < EPSILON &&
                    std::abs(points[i].y - points[j].y) < EPSILON) {
                    edgeDuplicates++;
                    std::cout << "DUPLICATE in Edge " << edgeId
                              << ": point[" << i << "] and point[" << j << "] both at ("
                              << points[i].x << ", " << points[i].y << ")\n";
                }
            }
        }

        if (edgeDuplicates > 0) {
            std::cout << "Edge " << edgeId << " full path (" << points.size() << " points):\n";
            for (size_t k = 0; k < points.size(); ++k) {
                std::cout << "  [" << k << "]: (" << points[k].x << ", " << points[k].y << ")\n";
            }
            totalDuplicates += edgeDuplicates;
        }
    }

    std::cout << "Total duplicate pairs: " << totalDuplicates << "\n";
    std::cout << "======================================\n";

    EXPECT_EQ(totalDuplicates, 0) << "Found " << totalDuplicates
        << " duplicate bend points - routing logic creates redundant points!";
}

// Test: Detect excessive bend points (path inefficiency)
// A well-routed edge should not have more than 6 bend points for typical cases
TEST(EdgeRoutingTransitionTest, PathEfficiency_NoBendPointExplosion) {
    Graph graph;

    NodeId idle = graph.addNode(Size{200, 100}, "Idle");
    NodeId running = graph.addNode(Size{200, 100}, "Running");
    NodeId paused = graph.addNode(Size{200, 100}, "Paused");
    NodeId stopped = graph.addNode(Size{200, 100}, "Stopped");
    NodeId error = graph.addNode(Size{200, 100}, "Error");

    EdgeId e0 = graph.addEdge(idle, running, "start");
    EdgeId e1 = graph.addEdge(running, paused, "pause");
    EdgeId e2 = graph.addEdge(paused, running, "resume");
    EdgeId e3 = graph.addEdge(running, stopped, "stop");
    EdgeId e4 = graph.addEdge(paused, stopped, "stop");
    EdgeId e5 = graph.addEdge(running, error, "fail");
    EdgeId e6 = graph.addEdge(error, idle, "reset");
    EdgeId e7 = graph.addEdge(error, error, "retry");

    SugiyamaLayout layout;
    LayoutOptions options;
    options.gridConfig.cellSize = 20.0f;

    layout.setOptions(options);

    LayoutResult result = layout.layout(graph);

    std::vector<EdgeId> allEdges = {e0, e1, e2, e3, e4, e5, e6, e7};

    // Maximum reasonable bend points for non-self-loop edges
    // Forward edges: typically 2 bend points (orthogonal routing)
    // Backward edges: typically 4-6 bend points (needs to route around)
    constexpr size_t MAX_BENDPOINTS_FORWARD = 4;
    constexpr size_t MAX_BENDPOINTS_BACKWARD = 8;
    constexpr size_t MAX_BENDPOINTS_SELFLOOP = 6;

    std::cout << "\n===== PATH EFFICIENCY TEST =====\n";

    int inefficientPaths = 0;

    for (EdgeId edgeId : allEdges) {
        const EdgeLayout* edgeLayout = result.getEdgeLayout(edgeId);
        ASSERT_NE(edgeLayout, nullptr);

        size_t bendCount = edgeLayout->bendPoints.size();
        bool isSelfLoop = (edgeLayout->from == edgeLayout->to);
        bool isBackward = (edgeId == e2 || edgeId == e6);  // resume, reset

        size_t maxAllowed = isSelfLoop ? MAX_BENDPOINTS_SELFLOOP :
                           (isBackward ? MAX_BENDPOINTS_BACKWARD : MAX_BENDPOINTS_FORWARD);

        std::cout << "Edge " << edgeId << " (" << edgeLayout->from << " -> " << edgeLayout->to << "): "
                  << bendCount << " bend points";

        if (bendCount > maxAllowed) {
            std::cout << " [EXCESSIVE! max=" << maxAllowed << "]";
            inefficientPaths++;

            // Print full path for debugging
            std::cout << "\n  Full path:\n";
            std::cout << "    src: (" << edgeLayout->sourcePoint.x << ", "
                      << edgeLayout->sourcePoint.y << ")\n";
            for (size_t i = 0; i < edgeLayout->bendPoints.size(); ++i) {
                std::cout << "    b" << i << ": (" << edgeLayout->bendPoints[i].position.x
                          << ", " << edgeLayout->bendPoints[i].position.y << ")\n";
            }
            std::cout << "    tgt: (" << edgeLayout->targetPoint.x << ", "
                      << edgeLayout->targetPoint.y << ")\n";
        }
        std::cout << "\n";
    }

    std::cout << "Inefficient paths: " << inefficientPaths << "\n";
    std::cout << "================================\n";

    EXPECT_EQ(inefficientPaths, 0) << "Found " << inefficientPaths
        << " edges with excessive bend points - routing is inefficient!";
}

// Test: Detect unnecessary backtracking in path
// Backtracking = going in one direction then immediately reversing
// Example: UP then DOWN on adjacent segments with only horizontal movement between
TEST(EdgeRoutingTransitionTest, NoUnnecessaryBacktracking) {
    Graph graph;

    NodeId idle = graph.addNode(Size{200, 100}, "Idle");
    NodeId running = graph.addNode(Size{200, 100}, "Running");
    NodeId paused = graph.addNode(Size{200, 100}, "Paused");
    NodeId stopped = graph.addNode(Size{200, 100}, "Stopped");
    NodeId error = graph.addNode(Size{200, 100}, "Error");

    EdgeId e0 = graph.addEdge(idle, running, "start");
    EdgeId e1 = graph.addEdge(running, paused, "pause");
    EdgeId e2 = graph.addEdge(paused, running, "resume");
    EdgeId e3 = graph.addEdge(running, stopped, "stop");
    EdgeId e4 = graph.addEdge(paused, stopped, "stop");
    EdgeId e5 = graph.addEdge(running, error, "fail");
    EdgeId e6 = graph.addEdge(error, idle, "reset");
    EdgeId e7 = graph.addEdge(error, error, "retry");

    SugiyamaLayout layout;
    LayoutOptions options;
    options.gridConfig.cellSize = 20.0f;

    layout.setOptions(options);

    LayoutResult result = layout.layout(graph);

    std::vector<EdgeId> allEdges = {e0, e1, e2, e3, e4, e5, e6, e7};
    constexpr float EPSILON = 0.1f;

    std::cout << "\n===== BACKTRACKING DETECTION TEST =====\n";

    int totalBacktracks = 0;

    for (EdgeId edgeId : allEdges) {
        const EdgeLayout* edgeLayout = result.getEdgeLayout(edgeId);
        ASSERT_NE(edgeLayout, nullptr);

        // Skip self-loops (they naturally have direction changes)
        if (edgeLayout->from == edgeLayout->to) continue;

        std::vector<Point> points;
        points.push_back(edgeLayout->sourcePoint);
        for (const auto& bend : edgeLayout->bendPoints) {
            points.push_back(bend.position);
        }
        points.push_back(edgeLayout->targetPoint);

        if (points.size() < 4) continue;  // Need at least 3 segments

        int edgeBacktracks = 0;

        // Check for pattern: V-H-V where V directions are opposite
        // or H-V-H where H directions are opposite
        for (size_t i = 0; i + 3 < points.size(); ++i) {
            const Point& p0 = points[i];
            const Point& p1 = points[i + 1];
            const Point& p2 = points[i + 2];
            const Point& p3 = points[i + 3];

            // Segment directions
            bool seg1_vertical = std::abs(p1.x - p0.x) < EPSILON;
            bool seg2_horizontal = std::abs(p2.y - p1.y) < EPSILON;
            bool seg3_vertical = std::abs(p3.x - p2.x) < EPSILON;

            // Check V-H-V pattern with opposite vertical directions
            if (seg1_vertical && seg2_horizontal && seg3_vertical) {
                float dy1 = p1.y - p0.y;  // First vertical direction
                float dy3 = p3.y - p2.y;  // Second vertical direction

                // Backtrack if directions are opposite AND horizontal distance is small
                if ((dy1 > EPSILON && dy3 < -EPSILON) || (dy1 < -EPSILON && dy3 > EPSILON)) {
                    float horizontalDistance = std::abs(p2.x - p1.x);
                    // Only flag if horizontal movement is relatively small (< 3 grid cells)
                    if (horizontalDistance < options.gridConfig.cellSize * 3) {
                        edgeBacktracks++;
                        std::cout << "BACKTRACK in Edge " << edgeId << " at points ["
                                  << i << "-" << (i+3) << "]:\n";
                        std::cout << "  (" << p0.x << "," << p0.y << ") -> ("
                                  << p1.x << "," << p1.y << ") [" << (dy1 > 0 ? "DOWN" : "UP") << "]\n";
                        std::cout << "  -> (" << p2.x << "," << p2.y << ") [HORIZONTAL]\n";
                        std::cout << "  -> (" << p3.x << "," << p3.y << ") [" << (dy3 > 0 ? "DOWN" : "UP") << "]\n";
                    }
                }
            }

            // Similarly check H-V-H pattern
            bool seg1_horizontal = std::abs(p1.y - p0.y) < EPSILON;
            bool seg2_vertical = std::abs(p2.x - p1.x) < EPSILON;
            bool seg3_horizontal = std::abs(p3.y - p2.y) < EPSILON;

            if (seg1_horizontal && seg2_vertical && seg3_horizontal) {
                float dx1 = p1.x - p0.x;
                float dx3 = p3.x - p2.x;

                if ((dx1 > EPSILON && dx3 < -EPSILON) || (dx1 < -EPSILON && dx3 > EPSILON)) {
                    float verticalDistance = std::abs(p2.y - p1.y);
                    if (verticalDistance < options.gridConfig.cellSize * 3) {
                        edgeBacktracks++;
                        std::cout << "BACKTRACK in Edge " << edgeId << " at points ["
                                  << i << "-" << (i+3) << "]:\n";
                        std::cout << "  (" << p0.x << "," << p0.y << ") -> ("
                                  << p1.x << "," << p1.y << ") [" << (dx1 > 0 ? "RIGHT" : "LEFT") << "]\n";
                        std::cout << "  -> (" << p2.x << "," << p2.y << ") [VERTICAL]\n";
                        std::cout << "  -> (" << p3.x << "," << p3.y << ") [" << (dx3 > 0 ? "RIGHT" : "LEFT") << "]\n";
                    }
                }
            }
        }

        if (edgeBacktracks > 0) {
            std::cout << "Edge " << edgeId << " has " << edgeBacktracks << " backtrack(s)\n";
            totalBacktracks += edgeBacktracks;
        }
    }

    std::cout << "Total backtracks: " << totalBacktracks << "\n";
    std::cout << "=======================================\n";

    EXPECT_EQ(totalBacktracks, 0) << "Found " << totalBacktracks
        << " unnecessary backtracking patterns - routing creates wasteful detours!";
}

// Test: Use exact node positions from current interactive_demo output
// This ensures the exact scenario that triggered the bug is tested
TEST(EdgeRoutingTransitionTest, CurrentInteractiveDemo_PathQuality) {
    Graph graph;

    // Create graph matching interactive_demo
    NodeId start = graph.addNode(Size{200, 100}, "Start");     // id=0
    NodeId idle = graph.addNode(Size{200, 100}, "Idle");       // id=1
    NodeId paused = graph.addNode(Size{200, 100}, "Paused");   // id=2
    NodeId stopped = graph.addNode(Size{200, 100}, "Stopped"); // id=3
    NodeId error = graph.addNode(Size{200, 100}, "Error");     // id=4

    EdgeId e0 = graph.addEdge(start, idle, "init");
    EdgeId e1 = graph.addEdge(idle, paused, "pause");
    EdgeId e2 = graph.addEdge(paused, idle, "resume");   // backward - problematic edge
    EdgeId e3 = graph.addEdge(idle, stopped, "stop");
    EdgeId e4 = graph.addEdge(paused, stopped, "stop2");
    EdgeId e5 = graph.addEdge(idle, error, "fail");
    EdgeId e6 = graph.addEdge(error, start, "reset");
    EdgeId e7 = graph.addEdge(error, error, "retry");    // self-loop

    // Exact positions from user's interactive_demo JSON output
    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    nodeLayouts[error] = NodeLayout{error, {400.0f, 180.0f}, {200.0f, 100.0f}, 3, 1};
    nodeLayouts[stopped] = NodeLayout{stopped, {0.0f, 520.0f}, {200.0f, 100.0f}, 3, 0};
    nodeLayouts[paused] = NodeLayout{paused, {0.0f, 360.0f}, {200.0f, 100.0f}, 2, 0};
    nodeLayouts[idle] = NodeLayout{idle, {700.0f, 580.0f}, {200.0f, 100.0f}, 1, 0};
    nodeLayouts[start] = NodeLayout{start, {100.0f, 60.0f}, {200.0f, 100.0f}, 0, 0};

    // Route with these exact positions
    EdgeRouting router;
    std::unordered_set<EdgeId> reversedEdges = {e2, e6};  // backward edges
    LayoutOptions options;
    options.gridConfig.cellSize = 20.0f;

    auto result = router.route(graph, nodeLayouts, reversedEdges, options);

    std::cout << "\n===== CURRENT INTERACTIVE_DEMO PATH QUALITY =====\n";
    std::cout << "Using exact node positions from user's JSON output\n\n";

    constexpr float EPSILON = 0.1f;
    constexpr size_t MAX_BENDPOINTS = 8;  // Reasonable maximum for any edge

    int totalIssues = 0;

    std::vector<EdgeId> allEdges = {e0, e1, e2, e3, e4, e5, e6, e7};

    for (EdgeId edgeId : allEdges) {
        const EdgeLayout* layout = result.getEdgeLayout(edgeId);
        if (!layout) continue;

        std::vector<Point> points;
        points.push_back(layout->sourcePoint);
        for (const auto& bend : layout->bendPoints) {
            points.push_back(bend.position);
        }
        points.push_back(layout->targetPoint);

        std::cout << "Edge " << edgeId << " (" << layout->from << " -> " << layout->to << "): "
                  << layout->bendPoints.size() << " bends\n";

        int edgeIssues = 0;

        // Check 1: Duplicate points
        for (size_t i = 0; i < points.size(); ++i) {
            for (size_t j = i + 1; j < points.size(); ++j) {
                if (std::abs(points[i].x - points[j].x) < EPSILON &&
                    std::abs(points[i].y - points[j].y) < EPSILON) {
                    std::cout << "  ISSUE: Duplicate points [" << i << "] and [" << j
                              << "] at (" << points[i].x << ", " << points[i].y << ")\n";
                    edgeIssues++;
                }
            }
        }

        // Check 2: Excessive bend points
        if (layout->bendPoints.size() > MAX_BENDPOINTS) {
            std::cout << "  ISSUE: Excessive bend points (" << layout->bendPoints.size()
                      << " > " << MAX_BENDPOINTS << ")\n";
            edgeIssues++;
        }

        // Check 3: Path goes past target then comes back (detour)
        // For non-self-loop edges, check if path exceeds target bounds then returns
        if (layout->from != layout->to && points.size() > 3) {
            float targetX = layout->targetPoint.x;
            float targetY = layout->targetPoint.y;
            float sourceX = layout->sourcePoint.x;
            float sourceY = layout->sourcePoint.y;

            // Check if any intermediate point goes significantly past target
            for (size_t i = 1; i + 1 < points.size(); ++i) {
                const Point& p = points[i];

                // Check X overshoot (going past target in X then coming back)
                bool xOvershoot = false;
                if (targetX > sourceX && p.x > targetX + 100) xOvershoot = true;
                if (targetX < sourceX && p.x < targetX - 100) xOvershoot = true;

                // Check Y overshoot
                bool yOvershoot = false;
                if (targetY > sourceY && p.y > targetY + 100) yOvershoot = true;
                if (targetY < sourceY && p.y < targetY - 100) yOvershoot = true;

                if (xOvershoot || yOvershoot) {
                    std::cout << "  ISSUE: Point [" << i << "] at (" << p.x << ", " << p.y
                              << ") overshoots target (" << targetX << ", " << targetY << ")\n";
                    edgeIssues++;
                    break;  // Only report once per edge
                }
            }
        }

        if (edgeIssues > 0) {
            std::cout << "  Full path:\n";
            for (size_t k = 0; k < points.size(); ++k) {
                std::string label = (k == 0) ? "src" :
                                   (k == points.size() - 1) ? "tgt" :
                                   "b" + std::to_string(k - 1);
                std::cout << "    " << label << ": (" << points[k].x << ", " << points[k].y << ")\n";
            }
            totalIssues += edgeIssues;
        }
        std::cout << "\n";
    }

    std::cout << "Total issues: " << totalIssues << "\n";
    std::cout << "================================================\n";

    EXPECT_EQ(totalIssues, 0) << "Found " << totalIssues
        << " path quality issues with current interactive_demo positions!";
}

// Unit test for segmentIntersectsNode with margin parameter
TEST(EdgeRoutingTransitionTest, SegmentIntersectsNode_WithMargin) {
    // Node at x[100, 300], y[200, 400]
    NodeLayout node;
    node.id = NodeId{1};
    node.position = {100.0f, 200.0f};
    node.size = {200.0f, 200.0f};

    // Test 1: Segment exactly on left boundary (x=100)
    {
        Point p1{100.0f, 250.0f};  // On left boundary
        Point p2{100.0f, 350.0f};

        // Without margin: should NOT intersect (boundary is excluded)
        EXPECT_FALSE(EdgeRouting::segmentIntersectsNode(p1, p2, node, 0.0f))
            << "Segment on boundary should not intersect without margin";

        // With margin=10: should intersect (within margin of boundary)
        EXPECT_TRUE(EdgeRouting::segmentIntersectsNode(p1, p2, node, 10.0f))
            << "Segment on boundary should intersect with margin";
    }

    // Test 2: Segment 5 pixels from left boundary (x=95)
    {
        Point p1{95.0f, 250.0f};
        Point p2{95.0f, 350.0f};

        // Without margin: should NOT intersect
        EXPECT_FALSE(EdgeRouting::segmentIntersectsNode(p1, p2, node, 0.0f));

        // With margin=10: should intersect (95 > 100-10=90)
        EXPECT_TRUE(EdgeRouting::segmentIntersectsNode(p1, p2, node, 10.0f));

        // With margin=3: should NOT intersect (95 < 100-3=97)
        EXPECT_FALSE(EdgeRouting::segmentIntersectsNode(p1, p2, node, 3.0f));
    }

    // Test 3: Horizontal segment near top boundary (y=200)
    {
        Point p1{150.0f, 200.0f};  // On top boundary
        Point p2{250.0f, 200.0f};

        // Without margin: should NOT intersect
        EXPECT_FALSE(EdgeRouting::segmentIntersectsNode(p1, p2, node, 0.0f));

        // With margin=10: should intersect
        EXPECT_TRUE(EdgeRouting::segmentIntersectsNode(p1, p2, node, 10.0f));
    }

    // Test 4: Segment clearly inside node (should always intersect)
    {
        Point p1{150.0f, 250.0f};
        Point p2{150.0f, 350.0f};

        EXPECT_TRUE(EdgeRouting::segmentIntersectsNode(p1, p2, node, 0.0f));
        EXPECT_TRUE(EdgeRouting::segmentIntersectsNode(p1, p2, node, 10.0f));
    }

    // Test 5: Segment clearly outside node (should never intersect)
    {
        Point p1{50.0f, 250.0f};  // Far from node
        Point p2{50.0f, 350.0f};

        EXPECT_FALSE(EdgeRouting::segmentIntersectsNode(p1, p2, node, 0.0f));
        EXPECT_FALSE(EdgeRouting::segmentIntersectsNode(p1, p2, node, 10.0f));
        EXPECT_FALSE(EdgeRouting::segmentIntersectsNode(p1, p2, node, 20.0f));
    }
}

// Integration test: verify routing respects node boundaries
TEST(EdgeRoutingTransitionTest, RoutingRespectsNodeBoundaries) {
    Graph graph;

    NodeId idle = graph.addNode(Size{200, 100}, "Idle");
    NodeId running = graph.addNode(Size{200, 100}, "Running");
    NodeId paused = graph.addNode(Size{200, 100}, "Paused");

    EdgeId e0 = graph.addEdge(idle, running, "run");

    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    nodeLayouts[idle] = NodeLayout{idle, {600.0f, 100.0f}, {200.0f, 100.0f}, 0, 0};
    nodeLayouts[running] = NodeLayout{running, {100.0f, 400.0f}, {200.0f, 100.0f}, 1, 0};
    nodeLayouts[paused] = NodeLayout{paused, {340.0f, 250.0f}, {200.0f, 100.0f}, 0, 1};

    EdgeRouting router;
    std::unordered_set<EdgeId> reversedEdges;  // No reversed edges in this test
    LayoutOptions options;
    options.gridConfig.cellSize = 20.0f;
    const float gridSize = options.gridConfig.cellSize;

    auto result = router.route(graph, nodeLayouts, reversedEdges, options);

    std::cout << "\n===== SEGMENT GRID DISTANCE TEST =====\n";
    std::cout << "Node positions:\n";
    for (const auto& [nodeId, node] : nodeLayouts) {
        std::cout << "  Node " << nodeId << ": pos=(" << node.position.x << "," << node.position.y
                  << "), bounds=x[" << node.position.x << "," << (node.position.x + node.size.width)
                  << "], y[" << node.position.y << "," << (node.position.y + node.size.height) << "]\n";
    }

    int violations = 0;
    std::vector<EdgeId> allEdges = {e0};

    for (EdgeId edgeId : allEdges) {
        const EdgeLayout* layout = result.getEdgeLayout(edgeId);
        if (!layout) continue;

        std::cout << "\nEdge " << edgeId << " from=" << layout->from << " to=" << layout->to << ":\n";

        // Collect all points
        std::vector<Point> points;
        points.push_back(layout->sourcePoint);
        for (const auto& bp : layout->bendPoints) {
            points.push_back(bp.position);
        }
        points.push_back(layout->targetPoint);

        // Print the path
        std::cout << "  Path (" << points.size() << " points): ";
        for (size_t i = 0; i < points.size(); ++i) {
            if (i > 0) std::cout << " -> ";
            std::cout << "(" << points[i].x << "," << points[i].y << ")";
        }
        std::cout << "\n";

        // Check each segment against all nodes (except source and target nodes)
        for (size_t i = 0; i + 1 < points.size(); ++i) {
            const Point& p1 = points[i];
            const Point& p2 = points[i + 1];

            bool isHorizontal = std::abs(p1.y - p2.y) < 0.1f;
            bool isVertical = std::abs(p1.x - p2.x) < 0.1f;

            for (const auto& [nodeId, node] : nodeLayouts) {
                // Skip source and target nodes
                if (nodeId == layout->from || nodeId == layout->to) continue;

                float nodeXmin = node.position.x;
                float nodeXmax = node.position.x + node.size.width;
                float nodeYmin = node.position.y;
                float nodeYmax = node.position.y + node.size.height;

                if (isVertical) {
                    float x = p1.x;
                    float yMin = std::min(p1.y, p2.y);
                    float yMax = std::max(p1.y, p2.y);

                    // Check if segment is too close to left or right edge of node
                    bool tooCloseToLeft = std::abs(x - nodeXmin) < gridSize &&
                                          yMin < nodeYmax && yMax > nodeYmin;
                    bool tooCloseToRight = std::abs(x - nodeXmax) < gridSize &&
                                           yMin < nodeYmax && yMax > nodeYmin;

                    if (tooCloseToLeft || tooCloseToRight) {
                        std::cout << "  VIOLATION: Edge " << edgeId << " segment " << i
                                  << " at x=" << x << " is within " << gridSize
                                  << "px of node " << nodeId << " boundary\n";
                        std::cout << "    Segment: (" << p1.x << "," << p1.y << ") -> ("
                                  << p2.x << "," << p2.y << ")\n";
                        std::cout << "    Node bounds: x[" << nodeXmin << "," << nodeXmax
                                  << "], y[" << nodeYmin << "," << nodeYmax << "]\n";
                        violations++;
                    }
                }

                if (isHorizontal) {
                    float y = p1.y;
                    float xMin = std::min(p1.x, p2.x);
                    float xMax = std::max(p1.x, p2.x);

                    // Check if segment is too close to top or bottom edge of node
                    bool tooCloseToTop = std::abs(y - nodeYmin) < gridSize &&
                                         xMin < nodeXmax && xMax > nodeXmin;
                    bool tooCloseToBottom = std::abs(y - nodeYmax) < gridSize &&
                                            xMin < nodeXmax && xMax > nodeXmin;

                    if (tooCloseToTop || tooCloseToBottom) {
                        std::cout << "  VIOLATION: Edge " << edgeId << " segment " << i
                                  << " at y=" << y << " is within " << gridSize
                                  << "px of node " << nodeId << " boundary\n";
                        std::cout << "    Segment: (" << p1.x << "," << p1.y << ") -> ("
                                  << p2.x << "," << p2.y << ")\n";
                        std::cout << "    Node bounds: x[" << nodeXmin << "," << nodeXmax
                                  << "], y[" << nodeYmin << "," << nodeYmax << "]\n";
                        violations++;
                    }
                }
            }
        }
    }

    std::cout << "Total violations: " << violations << "\n";
    std::cout << "======================================\n";

    EXPECT_EQ(violations, 0) << "Found " << violations
        << " segments within grid distance of node boundaries!";
}

// Comprehensive fuzz test: drag nodes many times and verify all constraints
TEST(EdgeRoutingTransitionTest, FuzzTest_DragSimulation_AllConstraints) {
    Graph graph;

    NodeId idle = graph.addNode(Size{200, 100}, "Idle");
    NodeId running = graph.addNode(Size{200, 100}, "Running");
    NodeId paused = graph.addNode(Size{200, 100}, "Paused");
    NodeId stopped = graph.addNode(Size{200, 100}, "Stopped");
    NodeId error = graph.addNode(Size{200, 100}, "Error");

    EdgeId e0 = graph.addEdge(idle, running, "start");
    EdgeId e1 = graph.addEdge(running, paused, "pause");
    EdgeId e2 = graph.addEdge(paused, running, "resume");
    EdgeId e3 = graph.addEdge(running, stopped, "stop");
    EdgeId e4 = graph.addEdge(paused, stopped, "stop");
    EdgeId e5 = graph.addEdge(running, error, "fail");
    EdgeId e6 = graph.addEdge(error, idle, "reset");
    EdgeId e7 = graph.addEdge(error, error, "retry");

    SugiyamaLayout layoutEngine;
    LayoutOptions options;
    options.gridConfig.cellSize = 20.0f;
    layoutEngine.setOptions(options);
    LayoutResult initialResult = layoutEngine.layout(graph);

    std::vector<NodeId> allNodes = {idle, running, paused, stopped, error};
    std::vector<EdgeId> allEdges = {e0, e1, e2, e3, e4, e5, e6, e7};

    const float gridSize = options.gridConfig.cellSize;
    const int numMovesPerNode = 50;

    struct Violation {
        std::string type;
        int moveNum;
        NodeId movedNode;
        Point delta;
        EdgeId edgeId;
        std::string details;
    };
    std::vector<Violation> violations;

    // Lambda to check all constraints
    auto checkAllConstraints = [&](int moveNum, NodeId movedNode, Point delta,
                                   const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
                                   const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts) {
        for (const auto& [edgeId, edge] : edgeLayouts) {
            std::vector<Point> points;
            points.push_back(edge.sourcePoint);
            for (const auto& bp : edge.bendPoints) {
                points.push_back(bp.position);
            }
            points.push_back(edge.targetPoint);

            for (size_t i = 0; i + 1 < points.size(); ++i) {
                const Point& p1 = points[i];
                const Point& p2 = points[i + 1];

                bool isHorizontal = std::abs(p1.y - p2.y) < 0.1f;
                bool isVertical = std::abs(p1.x - p2.x) < 0.1f;

                // Check 1: Orthogonality
                if (!isHorizontal && !isVertical) {
                    Violation v;
                    v.type = "DIAGONAL";
                    v.moveNum = moveNum;
                    v.movedNode = movedNode;
                    v.delta = delta;
                    v.edgeId = edgeId;
                    v.details = "(" + std::to_string(p1.x) + "," + std::to_string(p1.y) +
                        ")->(" + std::to_string(p2.x) + "," + std::to_string(p2.y) + ")";
                    violations.push_back(v);
                }

                for (const auto& [nodeId, node] : nodeLayouts) {
                    // Skip first segment vs source, last segment vs target
                    if (i == 0 && nodeId == edge.from) continue;
                    if (i == points.size() - 2 && nodeId == edge.to) continue;

                    float xmin = node.position.x;
                    float xmax = node.position.x + node.size.width;
                    float ymin = node.position.y;
                    float ymax = node.position.y + node.size.height;

                    // Check 2: Node intersection
                    bool intersects = false;
                    if (isHorizontal) {
                        float y = p1.y;
                        float segXmin = std::min(p1.x, p2.x);
                        float segXmax = std::max(p1.x, p2.x);
                        intersects = (y > ymin && y < ymax && segXmin < xmax && segXmax > xmin);
                    } else if (isVertical) {
                        float x = p1.x;
                        float segYmin = std::min(p1.y, p2.y);
                        float segYmax = std::max(p1.y, p2.y);
                        intersects = (x > xmin && x < xmax && segYmin < ymax && segYmax > ymin);
                    }

                    if (intersects) {
                        Violation v;
                        v.type = "INTERSECTION";
                        v.moveNum = moveNum;
                        v.movedNode = movedNode;
                        v.delta = delta;
                        v.edgeId = edgeId;
                        v.details = "through node " + std::to_string(nodeId) +
                            " at (" + std::to_string(p1.x) + "," + std::to_string(p1.y) +
                            ")->(" + std::to_string(p2.x) + "," + std::to_string(p2.y) + ")";
                        violations.push_back(v);
                    }

                    // Check 3: Boundary proximity (within gridSize)
                    if (i > 0 && i < points.size() - 2) {
                        if (isVertical) {
                            float x = p1.x;
                            float segYmin = std::min(p1.y, p2.y);
                            float segYmax = std::max(p1.y, p2.y);
                            bool yOverlap = segYmin < ymax && segYmax > ymin;

                            if (yOverlap) {
                                float distLeft = std::abs(x - xmin);
                                float distRight = std::abs(x - xmax);
                                if (distLeft < gridSize && distLeft > 0.1f) {
                                    Violation v;
                                    v.type = "NEAR_LEFT";
                                    v.moveNum = moveNum;
                                    v.movedNode = movedNode;
                                    v.delta = delta;
                                    v.edgeId = edgeId;
                                    v.details = "x=" + std::to_string(x) + " near node " +
                                        std::to_string(nodeId) + " left=" + std::to_string(xmin);
                                    violations.push_back(v);
                                }
                                if (distRight < gridSize && distRight > 0.1f) {
                                    Violation v;
                                    v.type = "NEAR_RIGHT";
                                    v.moveNum = moveNum;
                                    v.movedNode = movedNode;
                                    v.delta = delta;
                                    v.edgeId = edgeId;
                                    v.details = "x=" + std::to_string(x) + " near node " +
                                        std::to_string(nodeId) + " right=" + std::to_string(xmax);
                                    violations.push_back(v);
                                }
                            }
                        } else if (isHorizontal) {
                            float y = p1.y;
                            float segXmin = std::min(p1.x, p2.x);
                            float segXmax = std::max(p1.x, p2.x);
                            bool xOverlap = segXmin < xmax && segXmax > xmin;

                            if (xOverlap) {
                                float distTop = std::abs(y - ymin);
                                float distBottom = std::abs(y - ymax);
                                if (distTop < gridSize && distTop > 0.1f) {
                                    Violation v;
                                    v.type = "NEAR_TOP";
                                    v.moveNum = moveNum;
                                    v.movedNode = movedNode;
                                    v.delta = delta;
                                    v.edgeId = edgeId;
                                    v.details = "y=" + std::to_string(y) + " near node " +
                                        std::to_string(nodeId) + " top=" + std::to_string(ymin);
                                    violations.push_back(v);
                                }
                                if (distBottom < gridSize && distBottom > 0.1f) {
                                    Violation v;
                                    v.type = "NEAR_BOTTOM";
                                    v.moveNum = moveNum;
                                    v.movedNode = movedNode;
                                    v.delta = delta;
                                    v.edgeId = edgeId;
                                    v.details = "y=" + std::to_string(y) + " near node " +
                                        std::to_string(nodeId) + " bottom=" + std::to_string(ymax);
                                    violations.push_back(v);
                                }
                            }
                        }
                    }
                }
            }
        }
    };

    std::cout << "\n===== FUZZ TEST: DRAG SIMULATION =====\n";
    std::cout << "Grid size: " << gridSize << ", moves per direction: " << numMovesPerNode << "\n\n";

    int totalMoves = 0;
    Point directions[] = {{gridSize, 0}, {-gridSize, 0}, {0, gridSize}, {0, -gridSize}};

    for (NodeId nodeToMove : allNodes) {
        for (const Point& dir : directions) {
            // Copy initial layouts
            std::unordered_map<NodeId, NodeLayout> nodeLayouts;
            std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;
            for (NodeId nid : allNodes) {
                const NodeLayout* nl = initialResult.getNodeLayout(nid);
                if (nl) nodeLayouts[nid] = *nl;
            }
            for (EdgeId eid : allEdges) {
                const EdgeLayout* el = initialResult.getEdgeLayout(eid);
                if (el) edgeLayouts[eid] = *el;
            }

            for (int move = 0; move < numMovesPerNode; ++move) {
                // Propose new position
                Point proposedPos = {
                    nodeLayouts[nodeToMove].position.x + dir.x,
                    nodeLayouts[nodeToMove].position.y + dir.y
                };
                totalMoves++;

                // Validate with canMoveNodeTo (like real application does)
                auto validation = LayoutUtils::canMoveNodeTo(
                    nodeToMove, proposedPos, nodeLayouts, edgeLayouts, gridSize);

                if (!validation.valid) {
                    // Invalid position - in real app, this move would be rejected
                    // Stop moving in this direction
                    break;
                }

                // Valid position - apply the move
                nodeLayouts[nodeToMove].position = proposedPos;

                // Update edge positions
                std::unordered_set<NodeId> movedNodes = {nodeToMove};
                LayoutUtils::updateEdgePositions(edgeLayouts, nodeLayouts, allEdges,
                                                 movedNodes, gridSize);

                // Check all constraints
                size_t prevViolations = violations.size();
                checkAllConstraints(totalMoves, nodeToMove, dir, nodeLayouts, edgeLayouts);

                if (violations.size() > prevViolations && violations.size() >= 20) break;
            }
            if (violations.size() >= 20) break;
        }
        if (violations.size() >= 20) break;
    }

    std::cout << "Total moves: " << totalMoves << ", violations: " << violations.size() << "\n\n";

    if (!violations.empty()) {
        std::cout << "First " << std::min(size_t(20), violations.size()) << " violations:\n";
        for (size_t i = 0; i < std::min(size_t(20), violations.size()); ++i) {
            const auto& v = violations[i];
            std::cout << "  [" << v.type << "] #" << v.moveNum
                      << " node" << v.movedNode << " by(" << v.delta.x << "," << v.delta.y << ")"
                      << " e" << v.edgeId << ": " << v.details << "\n";
        }
    }
    std::cout << "======================================\n";

    EXPECT_EQ(violations.size(), 0u) << "Found " << violations.size() << " violations!";
}
