#include <gtest/gtest.h>
#include "../../../src/layout/sugiyama/SnapIndexManager.h"

using namespace arborvia;
using namespace arborvia::algorithms;

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

TEST_F(SnapIndexManagerTest, GetSeparatedRanges_BothExist_SplitsRange) {
    NodeEdgeConnections conn;
    conn.incoming = {EdgeId{1}};
    conn.outgoing = {EdgeId{2}};
    
    auto [inRange, outRange] = SnapIndexManager::getSeparatedRanges(conn);
    
    EXPECT_FLOAT_EQ(inRange.start, 0.0f);
    EXPECT_FLOAT_EQ(inRange.end, 0.5f);
    EXPECT_FLOAT_EQ(outRange.start, 0.5f);
    EXPECT_FLOAT_EQ(outRange.end, 1.0f);
}

TEST_F(SnapIndexManagerTest, GetSeparatedRanges_OnlyIncoming_UsesFullRange) {
    NodeEdgeConnections conn;
    conn.incoming = {EdgeId{1}, EdgeId{2}};
    
    auto [inRange, outRange] = SnapIndexManager::getSeparatedRanges(conn);
    
    // Incoming should use full range [0, 1]
    EXPECT_FLOAT_EQ(inRange.start, 0.0f);
    EXPECT_FLOAT_EQ(inRange.end, 1.0f);
    EXPECT_TRUE(inRange.isFullRange());
    
    // Outgoing should be empty range {0, 0}
    EXPECT_FLOAT_EQ(outRange.start, 0.0f);
    EXPECT_FLOAT_EQ(outRange.end, 0.0f);
    EXPECT_FLOAT_EQ(outRange.width(), 0.0f);
}

TEST_F(SnapIndexManagerTest, GetSeparatedRanges_OnlyOutgoing_UsesFullRange) {
    NodeEdgeConnections conn;
    conn.outgoing = {EdgeId{1}, EdgeId{2}};
    
    auto [inRange, outRange] = SnapIndexManager::getSeparatedRanges(conn);
    
    // Incoming should be empty range {0, 0}
    EXPECT_FLOAT_EQ(inRange.start, 0.0f);
    EXPECT_FLOAT_EQ(inRange.end, 0.0f);
    EXPECT_FLOAT_EQ(inRange.width(), 0.0f);
    
    // Outgoing should use full range [0, 1]
    EXPECT_FLOAT_EQ(outRange.start, 0.0f);
    EXPECT_FLOAT_EQ(outRange.end, 1.0f);
    EXPECT_TRUE(outRange.isFullRange());
}

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

TEST_F(SnapIndexManagerTest, ModeName_ReturnsCorrectStrings) {
    EXPECT_STREQ(SnapIndexManager::modeName(SnapDistribution::Unified), "Unified");
    EXPECT_STREQ(SnapIndexManager::modeName(SnapDistribution::Separated), "Separated");
}

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

TEST_F(SnapIndexManagerTest, Integration_DemoGraphScenario) {
    // Simulates the demo graph from the bug: node "Running" with 2 incoming, 3 outgoing
    NodeEdgeConnections conn;
    conn.incoming = {EdgeId{2}, EdgeId{4}};  // 2 incoming
    conn.outgoing = {EdgeId{1}, EdgeId{3}, EdgeId{5}};  // 3 outgoing
    
    auto [inRange, outRange] = SnapIndexManager::getSeparatedRanges(conn);
    
    // Both exist, so ranges should be split
    EXPECT_TRUE(conn.hasBoth());
    
    // Calculate positions for incoming (unified indices 0, 1)
    float inPos0 = SnapIndexManager::calculatePosition(
        SnapIndexManager::unifiedToLocal(0, 0, 2), 2, inRange);
    float inPos1 = SnapIndexManager::calculatePosition(
        SnapIndexManager::unifiedToLocal(1, 0, 2), 2, inRange);
    
    // Calculate positions for outgoing (unified indices 2, 3, 4)
    float outPos0 = SnapIndexManager::calculatePosition(
        SnapIndexManager::unifiedToLocal(2, 2, 3), 3, outRange);
    float outPos1 = SnapIndexManager::calculatePosition(
        SnapIndexManager::unifiedToLocal(3, 2, 3), 3, outRange);
    float outPos2 = SnapIndexManager::calculatePosition(
        SnapIndexManager::unifiedToLocal(4, 2, 3), 3, outRange);
    
    // All positions should be distinct
    EXPECT_NE(inPos0, inPos1);
    EXPECT_NE(outPos0, outPos1);
    EXPECT_NE(outPos1, outPos2);
    EXPECT_NE(outPos0, outPos2);
    
    // Incoming positions should be in first half [0, 0.5)
    EXPECT_LT(inPos0, 0.5f);
    EXPECT_LT(inPos1, 0.5f);
    
    // Outgoing positions should be in second half (0.5, 1.0]
    EXPECT_GT(outPos0, 0.5f);
    EXPECT_GT(outPos1, 0.5f);
    EXPECT_GT(outPos2, 0.5f);
}
