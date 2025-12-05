#include <gtest/gtest.h>
#include "../../../src/layout/sugiyama/SnapIndexManager.h"

using namespace arborvia;
using namespace arborvia;

// =============================================================================
// Index Conversion Tests
// =============================================================================

class SnapIndexManagerTest : public ::testing::Test {
protected:
    void SetUp() override {}
};

TEST_F(SnapIndexManagerTest, UnifiedToLocal_ZeroOffset_ReturnsOriginal) {
    // Incoming edges with offset 0
    EXPECT_EQ(SnapIndexManager::unifiedToLocal(0, 0, 3), 0);
    EXPECT_EQ(SnapIndexManager::unifiedToLocal(1, 0, 3), 1);
    EXPECT_EQ(SnapIndexManager::unifiedToLocal(2, 0, 3), 2);
}

TEST_F(SnapIndexManagerTest, UnifiedToLocal_WithOffset_SubtractsOffset) {
    // Outgoing edges with offset = inCount = 2
    EXPECT_EQ(SnapIndexManager::unifiedToLocal(2, 2, 3), 0);  // unified 2 -> local 0
    EXPECT_EQ(SnapIndexManager::unifiedToLocal(3, 2, 3), 1);  // unified 3 -> local 1
    EXPECT_EQ(SnapIndexManager::unifiedToLocal(4, 2, 3), 2);  // unified 4 -> local 2
}

TEST_F(SnapIndexManagerTest, UnifiedToLocal_NegativeResult_ClampsToZero) {
    // Index smaller than offset should clamp to 0
    EXPECT_EQ(SnapIndexManager::unifiedToLocal(0, 2, 3), 0);
    EXPECT_EQ(SnapIndexManager::unifiedToLocal(1, 2, 3), 0);
}

TEST_F(SnapIndexManagerTest, UnifiedToLocal_ExceedsCount_ClampsToMax) {
    // Index larger than count-1 should clamp to count-1
    EXPECT_EQ(SnapIndexManager::unifiedToLocal(5, 2, 3), 2);  // 5-2=3, clamp to 2
    EXPECT_EQ(SnapIndexManager::unifiedToLocal(10, 2, 3), 2);
}

TEST_F(SnapIndexManagerTest, UnifiedToLocal_ZeroCount_ReturnsZero) {
    EXPECT_EQ(SnapIndexManager::unifiedToLocal(0, 0, 0), 0);
    EXPECT_EQ(SnapIndexManager::unifiedToLocal(5, 2, 0), 0);
}

TEST_F(SnapIndexManagerTest, LocalToUnified_AddsOffset) {
    EXPECT_EQ(SnapIndexManager::localToUnified(0, 0), 0);
    EXPECT_EQ(SnapIndexManager::localToUnified(0, 2), 2);
    EXPECT_EQ(SnapIndexManager::localToUnified(1, 2), 3);
    EXPECT_EQ(SnapIndexManager::localToUnified(2, 2), 4);
}

// =============================================================================
// Validation Tests
// =============================================================================

TEST_F(SnapIndexManagerTest, Validate_ValidIndex_ReturnsOk) {
    auto result = SnapIndexManager::validate(0, 3);
    EXPECT_TRUE(result.valid);
    EXPECT_EQ(result.error, SnapIndexError::None);
    EXPECT_EQ(result.correctedIndex, 0);
}

TEST_F(SnapIndexManagerTest, Validate_NegativeIndex_ReturnsError) {
    auto result = SnapIndexManager::validate(-1, 3);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.error, SnapIndexError::InvalidIndex);
    EXPECT_EQ(result.correctedIndex, 0);
}

TEST_F(SnapIndexManagerTest, Validate_IndexOutOfRange_ReturnsError) {
    auto result = SnapIndexManager::validate(5, 3);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.error, SnapIndexError::IndexOutOfRange);
    EXPECT_EQ(result.correctedIndex, 2);  // count - 1
}

TEST_F(SnapIndexManagerTest, Validate_ZeroCount_ReturnsError) {
    auto result = SnapIndexManager::validate(0, 0);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.error, SnapIndexError::InvalidCount);
}

TEST_F(SnapIndexManagerTest, Clamp_ValidRange_ReturnsOriginal) {
    EXPECT_EQ(SnapIndexManager::clamp(0, 3), 0);
    EXPECT_EQ(SnapIndexManager::clamp(1, 3), 1);
    EXPECT_EQ(SnapIndexManager::clamp(2, 3), 2);
}

TEST_F(SnapIndexManagerTest, Clamp_Negative_ReturnsZero) {
    EXPECT_EQ(SnapIndexManager::clamp(-1, 3), 0);
    EXPECT_EQ(SnapIndexManager::clamp(-100, 3), 0);
}

TEST_F(SnapIndexManagerTest, Clamp_ExceedsMax_ReturnsMaxMinusOne) {
    EXPECT_EQ(SnapIndexManager::clamp(3, 3), 2);
    EXPECT_EQ(SnapIndexManager::clamp(100, 3), 2);
}

// =============================================================================
// Range Calculation Tests
// =============================================================================

TEST_F(SnapIndexManagerTest, CalculatePosition_SingleItem_ReturnsMidpoint) {
    SnapRange range{0.0f, 1.0f};
    float pos = SnapIndexManager::calculatePosition(0, 1, range);
    EXPECT_FLOAT_EQ(pos, 0.5f);  // (0+1)/(1+1) = 0.5
}

TEST_F(SnapIndexManagerTest, CalculatePosition_TwoItems_DistributesEvenly) {
    SnapRange range{0.0f, 1.0f};
    float pos0 = SnapIndexManager::calculatePosition(0, 2, range);
    float pos1 = SnapIndexManager::calculatePosition(1, 2, range);

    EXPECT_FLOAT_EQ(pos0, 1.0f/3.0f);   // (0+1)/(2+1) = 1/3
    EXPECT_FLOAT_EQ(pos1, 2.0f/3.0f);   // (1+1)/(2+1) = 2/3
}

TEST_F(SnapIndexManagerTest, CalculatePosition_HalfRange_ScalesCorrectly) {
    SnapRange range{0.0f, 0.5f};
    float pos = SnapIndexManager::calculatePosition(0, 1, range);
    EXPECT_FLOAT_EQ(pos, 0.25f);  // midpoint of [0, 0.5]
}

TEST_F(SnapIndexManagerTest, CalculatePosition_ZeroCount_ReturnsMidpoint) {
    SnapRange range{0.0f, 1.0f};
    float pos = SnapIndexManager::calculatePosition(0, 0, range);
    EXPECT_FLOAT_EQ(pos, 0.5f);  // fallback to midpoint
}

// =============================================================================
// Connection Analysis Tests
// =============================================================================

TEST_F(SnapIndexManagerTest, GetConnections_EmptyLayouts_ReturnsEmpty) {
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;
    auto conn = SnapIndexManager::getConnections(edgeLayouts, NodeId{1}, NodeEdge::Bottom);

    EXPECT_EQ(conn.incomingCount(), 0);
    EXPECT_EQ(conn.outgoingCount(), 0);
}

TEST_F(SnapIndexManagerTest, GetConnections_WithEdges_CountsCorrectly) {
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;

    // Edge 1: from node 1 (source) to node 2
    EdgeLayout e1;
    e1.id = EdgeId{1};
    e1.from = NodeId{1};
    e1.to = NodeId{2};
    e1.sourceEdge = NodeEdge::Bottom;
    e1.targetEdge = NodeEdge::Top;
    edgeLayouts[EdgeId{1}] = e1;

    // Edge 2: from node 1 (source) to node 3
    EdgeLayout e2;
    e2.id = EdgeId{2};
    e2.from = NodeId{1};
    e2.to = NodeId{3};
    e2.sourceEdge = NodeEdge::Bottom;
    e2.targetEdge = NodeEdge::Top;
    edgeLayouts[EdgeId{2}] = e2;

    // Edge 3: from node 4 to node 1 (target)
    EdgeLayout e3;
    e3.id = EdgeId{3};
    e3.from = NodeId{4};
    e3.to = NodeId{1};
    e3.sourceEdge = NodeEdge::Bottom;
    e3.targetEdge = NodeEdge::Bottom;
    edgeLayouts[EdgeId{3}] = e3;

    auto conn = SnapIndexManager::getConnections(edgeLayouts, NodeId{1}, NodeEdge::Bottom);

    EXPECT_EQ(conn.outgoingCount(), 2);  // Edges 1 and 2
    EXPECT_EQ(conn.incomingCount(), 1);  // Edge 3
    EXPECT_EQ(conn.totalCount(), 3);
}

TEST_F(SnapIndexManagerTest, NodeEdgeConnections_Helpers) {
    NodeEdgeConnections conn;

    // Empty
    EXPECT_FALSE(conn.hasOnlyIncoming());
    EXPECT_FALSE(conn.hasOnlyOutgoing());
    EXPECT_FALSE(conn.hasBoth());

    // Only incoming
    conn.incoming = {EdgeId{1}};
    EXPECT_TRUE(conn.hasOnlyIncoming());
    EXPECT_FALSE(conn.hasOnlyOutgoing());
    EXPECT_FALSE(conn.hasBoth());

    // Both
    conn.outgoing = {EdgeId{2}};
    EXPECT_FALSE(conn.hasOnlyIncoming());
    EXPECT_FALSE(conn.hasOnlyOutgoing());
    EXPECT_TRUE(conn.hasBoth());

    // Only outgoing
    conn.incoming.clear();
    EXPECT_FALSE(conn.hasOnlyIncoming());
    EXPECT_TRUE(conn.hasOnlyOutgoing());
    EXPECT_FALSE(conn.hasBoth());
}

TEST_F(SnapIndexManagerTest, BuildConnectionMap_CreatesCorrectKeys) {
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;

    EdgeLayout e1;
    e1.id = EdgeId{1};
    e1.from = NodeId{1};
    e1.to = NodeId{2};
    e1.sourceEdge = NodeEdge::Bottom;
    e1.targetEdge = NodeEdge::Top;
    edgeLayouts[EdgeId{1}] = e1;

    auto map = SnapIndexManager::buildConnectionMap(edgeLayouts);

    // Should have 2 entries: (node 1, Bottom) and (node 2, Top)
    EXPECT_EQ(map.size(), 2u);

    auto key1 = std::make_pair(NodeId{1}, NodeEdge::Bottom);
    auto key2 = std::make_pair(NodeId{2}, NodeEdge::Top);

    EXPECT_TRUE(map.count(key1) > 0);
    EXPECT_TRUE(map.count(key2) > 0);

    // Check contents
    EXPECT_EQ(map[key1].outgoingCount(), 1);
    EXPECT_EQ(map[key2].incomingCount(), 1);
}

// =============================================================================
// Debug Helper Tests
// =============================================================================

TEST_F(SnapIndexManagerTest, FormatIndexInfo_ContainsAllFields) {
    std::string info = SnapIndexManager::formatIndexInfo(3, 1, 2, 4);

    EXPECT_NE(info.find("unified=3"), std::string::npos);
    EXPECT_NE(info.find("local=1"), std::string::npos);
    EXPECT_NE(info.find("offset=2"), std::string::npos);
    EXPECT_NE(info.find("count=4"), std::string::npos);
}

// =============================================================================
// Integration Tests - Simulating Real Scenarios
// =============================================================================

TEST_F(SnapIndexManagerTest, Integration_UnifiedModeDistribution) {
    // Simulates the demo graph: node "Running" with 5 connections total
    NodeEdgeConnections conn;
    conn.incoming = {EdgeId{2}, EdgeId{4}};  // 2 incoming
    conn.outgoing = {EdgeId{1}, EdgeId{3}, EdgeId{5}};  // 3 outgoing

    // In unified mode, all connections share the full range [0, 1]
    SnapRange fullRange{0.0f, 1.0f};
    int totalCount = conn.totalCount();  // 5

    EXPECT_EQ(totalCount, 5);
    EXPECT_TRUE(conn.hasBoth());

    // Calculate positions for all 5 connections (indices 0-4)
    std::vector<float> positions;
    for (int i = 0; i < totalCount; ++i) {
        positions.push_back(SnapIndexManager::calculatePosition(i, totalCount, fullRange));
    }

    // All positions should be distinct and evenly distributed
    for (size_t i = 0; i < positions.size(); ++i) {
        for (size_t j = i + 1; j < positions.size(); ++j) {
            EXPECT_NE(positions[i], positions[j]);
        }
    }

    // Positions should be in increasing order
    for (size_t i = 0; i + 1 < positions.size(); ++i) {
        EXPECT_LT(positions[i], positions[i + 1]);
    }

    // All positions should be within range
    for (float pos : positions) {
        EXPECT_GT(pos, 0.0f);
        EXPECT_LT(pos, 1.0f);
    }
}

// =============================================================================
// Bug Reproduction: Same Snap Index on Same Node Edge
// =============================================================================

TEST_F(SnapIndexManagerTest, BugRepro_TwoEdgesSameNodeEdge_ConnectionCounting) {
    // Verifies that getConnections correctly counts both incoming and outgoing
    // connections on the same node edge.
    //
    // Setup: Node 1 right edge has two connections:
    //   - Edge 2 incoming (target on right edge)
    //   - Edge 5 outgoing (source on right edge)

    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;

    // Edge 2: from node 2 to node 1 (incoming to node 1's right edge)
    EdgeLayout e2;
    e2.id = EdgeId{2};
    e2.from = NodeId{2};
    e2.to = NodeId{1};
    e2.sourceEdge = NodeEdge::Left;
    e2.targetEdge = NodeEdge::Right;  // connects to node 1's RIGHT edge
    e2.sourceSnapIndex = 0;
    e2.targetSnapIndex = 0;
    edgeLayouts[EdgeId{2}] = e2;

    // Edge 5: from node 1 to node 4 (outgoing from node 1's right edge)
    EdgeLayout e5;
    e5.id = EdgeId{5};
    e5.from = NodeId{1};
    e5.to = NodeId{4};
    e5.sourceEdge = NodeEdge::Right;  // connects from node 1's RIGHT edge
    e5.targetEdge = NodeEdge::Bottom;
    e5.sourceSnapIndex = 0;
    e5.targetSnapIndex = 0;
    edgeLayouts[EdgeId{5}] = e5;

    // Check connections on node 1's right edge
    auto conn = SnapIndexManager::getConnections(edgeLayouts, NodeId{1}, NodeEdge::Right);

    // Should count both incoming and outgoing
    EXPECT_EQ(conn.incomingCount(), 1);   // Edge 2 target
    EXPECT_EQ(conn.outgoingCount(), 1);   // Edge 5 source
    EXPECT_EQ(conn.totalCount(), 2);
    EXPECT_TRUE(conn.hasBoth());
}
