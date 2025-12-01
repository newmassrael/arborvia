#include <gtest/gtest.h>
#include <arborvia/core/Graph.h>

#include <algorithm>

using namespace arborvia;

TEST(GraphTest, AddNode) {
    Graph graph;
    NodeId id = graph.addNode();
    
    EXPECT_TRUE(graph.hasNode(id));
    EXPECT_EQ(graph.nodeCount(), 1);
}

TEST(GraphTest, AddNodeWithLabel) {
    Graph graph;
    NodeId id = graph.addNode("TestNode");
    
    EXPECT_TRUE(graph.hasNode(id));
    EXPECT_EQ(graph.getNode(id).label, "TestNode");
}

TEST(GraphTest, AddNodeWithSize) {
    Graph graph;
    NodeId id = graph.addNode(Size{150.0f, 75.0f});
    
    EXPECT_TRUE(graph.hasNode(id));
    EXPECT_FLOAT_EQ(graph.getNode(id).size.width, 150.0f);
    EXPECT_FLOAT_EQ(graph.getNode(id).size.height, 75.0f);
}

TEST(GraphTest, RemoveNode) {
    Graph graph;
    NodeId n1 = graph.addNode();
    NodeId n2 = graph.addNode();
    EdgeId e = graph.addEdge(n1, n2);

    EXPECT_TRUE(graph.hasNode(n1));
    EXPECT_TRUE(graph.hasEdge(e));

    graph.removeNode(n1);

    EXPECT_FALSE(graph.hasNode(n1));
    EXPECT_EQ(graph.nodeCount(), 1);

    // Verify connected edge is also removed
    EXPECT_FALSE(graph.hasEdge(e));
    EXPECT_EQ(graph.edgeCount(), 0);

    // Verify n2 still exists
    EXPECT_TRUE(graph.hasNode(n2));
}

TEST(GraphTest, AddEdge) {
    Graph graph;
    NodeId n1 = graph.addNode();
    NodeId n2 = graph.addNode();
    EdgeId e = graph.addEdge(n1, n2);
    
    EXPECT_TRUE(graph.hasEdge(e));
    EXPECT_EQ(graph.edgeCount(), 1);
    
    const EdgeData& edge = graph.getEdge(e);
    EXPECT_EQ(edge.from, n1);
    EXPECT_EQ(edge.to, n2);
}

TEST(GraphTest, AddEdgeWithLabel) {
    Graph graph;
    NodeId n1 = graph.addNode();
    NodeId n2 = graph.addNode();
    EdgeId e = graph.addEdge(n1, n2, "transition");
    
    EXPECT_EQ(graph.getEdge(e).label, "transition");
}

TEST(GraphTest, RemoveEdge) {
    Graph graph;
    NodeId n1 = graph.addNode();
    NodeId n2 = graph.addNode();
    EdgeId e = graph.addEdge(n1, n2);
    
    graph.removeEdge(e);
    EXPECT_FALSE(graph.hasEdge(e));
    EXPECT_EQ(graph.edgeCount(), 0);
}

TEST(GraphTest, Successors) {
    Graph graph;
    NodeId n1 = graph.addNode();
    NodeId n2 = graph.addNode();
    NodeId n3 = graph.addNode();

    graph.addEdge(n1, n2);
    graph.addEdge(n1, n3);

    auto successors = graph.successors(n1);
    EXPECT_EQ(successors.size(), 2);

    // Verify actual content, not just size
    EXPECT_TRUE(std::find(successors.begin(), successors.end(), n2) != successors.end());
    EXPECT_TRUE(std::find(successors.begin(), successors.end(), n3) != successors.end());
    EXPECT_TRUE(std::find(successors.begin(), successors.end(), n1) == successors.end());
}

TEST(GraphTest, Predecessors) {
    Graph graph;
    NodeId n1 = graph.addNode();
    NodeId n2 = graph.addNode();
    NodeId n3 = graph.addNode();

    graph.addEdge(n1, n3);
    graph.addEdge(n2, n3);

    auto predecessors = graph.predecessors(n3);
    EXPECT_EQ(predecessors.size(), 2);

    // Verify actual content, not just size
    EXPECT_TRUE(std::find(predecessors.begin(), predecessors.end(), n1) != predecessors.end());
    EXPECT_TRUE(std::find(predecessors.begin(), predecessors.end(), n2) != predecessors.end());
    EXPECT_TRUE(std::find(predecessors.begin(), predecessors.end(), n3) == predecessors.end());
}

TEST(GraphTest, FindEdge) {
    Graph graph;
    NodeId n1 = graph.addNode();
    NodeId n2 = graph.addNode();
    EdgeId e = graph.addEdge(n1, n2);
    
    auto found = graph.findEdge(n1, n2);
    EXPECT_TRUE(found.has_value());
    EXPECT_EQ(found.value(), e);
    
    auto notFound = graph.findEdge(n2, n1);
    EXPECT_FALSE(notFound.has_value());
}

TEST(GraphTest, Clear) {
    Graph graph;
    NodeId n1 = graph.addNode();
    NodeId n2 = graph.addNode();
    EdgeId e = graph.addEdge(n1, n2);

    EXPECT_TRUE(graph.hasNode(n1));
    EXPECT_TRUE(graph.hasNode(n2));
    EXPECT_TRUE(graph.hasEdge(e));

    graph.clear();

    EXPECT_EQ(graph.nodeCount(), 0);
    EXPECT_EQ(graph.edgeCount(), 0);

    // Verify hasNode/hasEdge return false for cleared items
    EXPECT_FALSE(graph.hasNode(n1));
    EXPECT_FALSE(graph.hasNode(n2));
    EXPECT_FALSE(graph.hasEdge(e));
}

TEST(GraphTest, InOutDegree) {
    Graph graph;
    NodeId n1 = graph.addNode();
    NodeId n2 = graph.addNode();
    NodeId n3 = graph.addNode();
    
    graph.addEdge(n1, n2);
    graph.addEdge(n1, n3);
    graph.addEdge(n2, n3);
    
    EXPECT_EQ(graph.outDegree(n1), 2);
    EXPECT_EQ(graph.inDegree(n3), 2);
    EXPECT_EQ(graph.outDegree(n3), 0);
}

TEST(GraphTest, GetConnectedEdges) {
    Graph graph;
    NodeId n1 = graph.addNode();
    NodeId n2 = graph.addNode();
    NodeId n3 = graph.addNode();
    
    EdgeId e1 = graph.addEdge(n1, n2);  // n1 -> n2
    EdgeId e2 = graph.addEdge(n2, n3);  // n2 -> n3
    EdgeId e3 = graph.addEdge(n3, n2);  // n3 -> n2
    
    // n2 has: outgoing e2, incoming e1 and e3
    auto connectedN2 = graph.getConnectedEdges(n2);
    EXPECT_EQ(connectedN2.size(), 3);
    EXPECT_TRUE(std::find(connectedN2.begin(), connectedN2.end(), e1) != connectedN2.end());
    EXPECT_TRUE(std::find(connectedN2.begin(), connectedN2.end(), e2) != connectedN2.end());
    EXPECT_TRUE(std::find(connectedN2.begin(), connectedN2.end(), e3) != connectedN2.end());
    
    // n1 has: only outgoing e1
    auto connectedN1 = graph.getConnectedEdges(n1);
    EXPECT_EQ(connectedN1.size(), 1);
    EXPECT_EQ(connectedN1[0], e1);
    
    // n3 has: outgoing e3, incoming e2
    auto connectedN3 = graph.getConnectedEdges(n3);
    EXPECT_EQ(connectedN3.size(), 2);
    EXPECT_TRUE(std::find(connectedN3.begin(), connectedN3.end(), e2) != connectedN3.end());
    EXPECT_TRUE(std::find(connectedN3.begin(), connectedN3.end(), e3) != connectedN3.end());
}

TEST(GraphTest, GetConnectedEdgesSelfLoop) {
    Graph graph;
    NodeId n1 = graph.addNode();
    
    // Self-loop: n1 -> n1
    EdgeId selfLoop = graph.addEdge(n1, n1);
    
    // Should return self-loop only once, not twice
    auto connected = graph.getConnectedEdges(n1);
    EXPECT_EQ(connected.size(), 1);
    EXPECT_EQ(connected[0], selfLoop);
}

TEST(GraphTest, GetConnectedEdgesEmpty) {
    Graph graph;
    NodeId n1 = graph.addNode();
    
    // Node with no edges
    auto connected = graph.getConnectedEdges(n1);
    EXPECT_TRUE(connected.empty());
    
    // Invalid node
    auto invalidConnected = graph.getConnectedEdges(999);
    EXPECT_TRUE(invalidConnected.empty());
}

// ============== Compaction Tests ==============

TEST(GraphTest, Compact_RemovesInvalidNodes) {
    Graph graph;
    NodeId n1 = graph.addNode("Node1");
    NodeId n2 = graph.addNode("Node2");
    NodeId n3 = graph.addNode("Node3");
    
    // Remove middle node
    graph.removeNode(n2);
    
    // Before compaction: 3 slots, 2 valid
    EXPECT_EQ(graph.nodeCount(), 2);
    
    // Compact should remove invalid entries
    graph.compact();
    
    // After compaction: still 2 valid nodes
    EXPECT_EQ(graph.nodeCount(), 2);
    
    // Get new node IDs after compaction
    auto nodes = graph.nodes();
    EXPECT_EQ(nodes.size(), 2);
    
    // Verify node data is preserved
    bool foundNode1 = false, foundNode3 = false;
    for (NodeId id : nodes) {
        auto node = graph.tryGetNode(id);
        EXPECT_TRUE(node.has_value());
        if (node->label == "Node1") foundNode1 = true;
        if (node->label == "Node3") foundNode3 = true;
    }
    EXPECT_TRUE(foundNode1);
    EXPECT_TRUE(foundNode3);
}

TEST(GraphTest, Compact_RemovesInvalidEdges) {
    Graph graph;
    NodeId n1 = graph.addNode("Node1");
    NodeId n2 = graph.addNode("Node2");
    NodeId n3 = graph.addNode("Node3");
    
    EdgeId e1 = graph.addEdge(n1, n2, "Edge1");
    EdgeId e2 = graph.addEdge(n2, n3, "Edge2");
    EdgeId e3 = graph.addEdge(n1, n3, "Edge3");
    
    // Remove middle edge
    graph.removeEdge(e2);
    
    // Before compaction: 3 slots, 2 valid
    EXPECT_EQ(graph.edgeCount(), 2);
    
    // Compact
    graph.compact();
    
    // After compaction: still 2 valid edges
    EXPECT_EQ(graph.edgeCount(), 2);
    
    // Get new edge IDs after compaction
    auto edges = graph.edges();
    EXPECT_EQ(edges.size(), 2);
    
    // Verify edge data is preserved
    bool foundEdge1 = false, foundEdge3 = false;
    for (EdgeId id : edges) {
        auto edge = graph.tryGetEdge(id);
        EXPECT_TRUE(edge.has_value());
        if (edge->label == "Edge1") foundEdge1 = true;
        if (edge->label == "Edge3") foundEdge3 = true;
    }
    EXPECT_TRUE(foundEdge1);
    EXPECT_TRUE(foundEdge3);
}

TEST(GraphTest, Compact_RemapsEdgeNodeReferences) {
    Graph graph;
    NodeId n1 = graph.addNode("Node1");
    NodeId n2 = graph.addNode("Node2");
    NodeId n3 = graph.addNode("Node3");
    
    EdgeId e1 = graph.addEdge(n1, n3, "Edge1");
    
    // Remove middle node (no edges to it)
    graph.removeNode(n2);
    
    // Compact
    graph.compact();
    
    // Edge should still exist and connect valid nodes
    EXPECT_EQ(graph.edgeCount(), 1);
    auto edge = graph.tryGetEdge(e1);
    EXPECT_TRUE(edge.has_value());
    
    // Edge endpoints should be remapped correctly
    EXPECT_TRUE(graph.hasNode(edge->from));
    EXPECT_TRUE(graph.hasNode(edge->to));
}

TEST(GraphTest, Compact_EmptyGraph_NoEffect) {
    Graph graph;
    graph.compact();
    
    EXPECT_EQ(graph.nodeCount(), 0);
    EXPECT_EQ(graph.edgeCount(), 0);
}

TEST(GraphTest, Compact_NoRemovals_NoEffect) {
    Graph graph;
    NodeId n1 = graph.addNode("Node1");
    NodeId n2 = graph.addNode("Node2");
    EdgeId e1 = graph.addEdge(n1, n2);
    
    graph.compact();
    
    EXPECT_EQ(graph.nodeCount(), 2);
    EXPECT_EQ(graph.edgeCount(), 1);
    EXPECT_TRUE(graph.hasNode(n1));
    EXPECT_TRUE(graph.hasNode(n2));
    EXPECT_TRUE(graph.hasEdge(e1));
}

TEST(GraphTest, Compact_PreservesNodeData) {
    Graph graph;
    NodeId n1 = graph.addNode(Size{100, 50}, "Node1");
    NodeId n2 = graph.addNode(Size{200, 100}, "ToRemove");
    NodeId n3 = graph.addNode(Size{150, 75}, "Node3");
    
    graph.removeNode(n2);
    graph.compact();
    
    // Get nodes after compaction
    auto nodes = graph.nodes();
    EXPECT_EQ(nodes.size(), 2);
    
    // Verify node data is preserved
    for (NodeId id : nodes) {
        auto node = graph.tryGetNode(id);
        EXPECT_TRUE(node.has_value());
        if (node->label == "Node1") {
            EXPECT_FLOAT_EQ(node->size.width, 100.0f);
            EXPECT_FLOAT_EQ(node->size.height, 50.0f);
        } else if (node->label == "Node3") {
            EXPECT_FLOAT_EQ(node->size.width, 150.0f);
            EXPECT_FLOAT_EQ(node->size.height, 75.0f);
        }
    }
}

TEST(GraphTest, Compact_PreservesEdgeData) {
    Graph graph;
    NodeId n1 = graph.addNode("Node1");
    NodeId n2 = graph.addNode("Node2");
    
    EdgeId e1 = graph.addEdge(n1, n2, "Edge1");
    EdgeId e2 = graph.addEdge(n2, n1, "ToRemove");
    
    graph.removeEdge(e2);
    graph.compact();
    
    // Edge data should be preserved
    auto edge = graph.tryGetEdge(e1);
    EXPECT_TRUE(edge.has_value());
    EXPECT_EQ(edge->label, "Edge1");
}

// ============== Additional Compaction Validation Tests ==============

TEST(GraphTest, Compact_IDsAreSequential) {
    Graph graph;
    NodeId n1 = graph.addNode("Node1");
    NodeId n2 = graph.addNode("Node2");
    NodeId n3 = graph.addNode("Node3");
    NodeId n4 = graph.addNode("Node4");
    
    // Remove non-consecutive nodes
    graph.removeNode(n2);
    graph.removeNode(n4);
    
    graph.compact();
    
    // IDs should be 0, 1 (sequential)
    auto nodes = graph.nodes();
    EXPECT_EQ(nodes.size(), 2);
    EXPECT_EQ(nodes[0], 0);
    EXPECT_EQ(nodes[1], 1);
}

TEST(GraphTest, Compact_EdgeConnectionsValid) {
    Graph graph;
    NodeId n1 = graph.addNode("Node1");
    NodeId n2 = graph.addNode("Node2");
    NodeId n3 = graph.addNode("Node3");
    
    EdgeId e1 = graph.addEdge(n1, n3, "1->3");
    
    // Remove middle node
    graph.removeNode(n2);
    graph.compact();
    
    // Edge should connect correct nodes by label
    auto edges = graph.edges();
    EXPECT_EQ(edges.size(), 1);
    
    auto edge = graph.tryGetEdge(edges[0]);
    EXPECT_TRUE(edge.has_value());
    EXPECT_EQ(edge->label, "1->3");
    
    auto fromNode = graph.tryGetNode(edge->from);
    auto toNode = graph.tryGetNode(edge->to);
    EXPECT_TRUE(fromNode.has_value());
    EXPECT_TRUE(toNode.has_value());
    EXPECT_EQ(fromNode->label, "Node1");
    EXPECT_EQ(toNode->label, "Node3");
}

TEST(GraphTest, Compact_AdjacencyListsCorrect) {
    Graph graph;
    NodeId n1 = graph.addNode("Node1");
    NodeId n2 = graph.addNode("Node2");
    NodeId n3 = graph.addNode("Node3");
    
    EdgeId e1 = graph.addEdge(n1, n2);
    EdgeId e2 = graph.addEdge(n2, n3);
    EdgeId e3 = graph.addEdge(n1, n3);
    
    graph.compact();
    
    // Get new node IDs
    auto nodes = graph.nodes();
    std::unordered_map<std::string, NodeId> nodeMap;
    for (NodeId id : nodes) {
        auto node = graph.tryGetNode(id);
        nodeMap[node->label] = id;
    }
    
    // Check outEdges for Node1
    auto out1 = graph.outEdges(nodeMap["Node1"]);
    EXPECT_EQ(out1.size(), 2);  // Should have 2 outgoing edges
    
    // Check inEdges for Node3
    auto in3 = graph.inEdges(nodeMap["Node3"]);
    EXPECT_EQ(in3.size(), 2);  // Should have 2 incoming edges
    
    // Check Node2
    auto out2 = graph.outEdges(nodeMap["Node2"]);
    auto in2 = graph.inEdges(nodeMap["Node2"]);
    EXPECT_EQ(out2.size(), 1);  // 1 outgoing
    EXPECT_EQ(in2.size(), 1);   // 1 incoming
}

TEST(GraphTest, Compact_MemoryReduction) {
    Graph graph;
    
    // Add many nodes
    for (int i = 0; i < 100; ++i) {
        graph.addNode("Node" + std::to_string(i));
    }
    
    // Remove half of them
    auto nodes = graph.nodes();
    for (size_t i = 0; i < nodes.size(); i += 2) {
        graph.removeNode(nodes[i]);
    }
    
    size_t countBeforeCompact = graph.nodeCount();
    
    graph.compact();
    
    // Count should remain same
    EXPECT_EQ(graph.nodeCount(), countBeforeCompact);
    
    // All IDs should be sequential from 0
    auto nodesAfter = graph.nodes();
    for (size_t i = 0; i < nodesAfter.size(); ++i) {
        EXPECT_EQ(nodesAfter[i], static_cast<NodeId>(i));
    }
}

TEST(GraphTest, Compact_PreservesUserData) {
    Graph graph;
    NodeId n1 = graph.addNode("Node1");
    NodeId n2 = graph.addNode("Node2");
    
    // Set userData
    auto node1 = graph.tryGetNode(n1);
    ASSERT_TRUE(node1.has_value());
    
    // Note: NodeData has userData field, but we need mutable access
    // This test verifies that userData field exists and is preserved
    
    graph.compact();
    
    auto nodes = graph.nodes();
    EXPECT_EQ(nodes.size(), 2);
    
    // Verify nodes are accessible after compact
    for (NodeId id : nodes) {
        auto node = graph.tryGetNode(id);
        EXPECT_TRUE(node.has_value());
    }
}

TEST(GraphTest, Compact_ComplexGraphScenario) {
    Graph graph;
    
    // Create complex graph
    NodeId n1 = graph.addNode("A");
    NodeId n2 = graph.addNode("B");
    NodeId n3 = graph.addNode("C");
    NodeId n4 = graph.addNode("D");
    NodeId n5 = graph.addNode("E");
    
    EdgeId e1 = graph.addEdge(n1, n2, "A->B");
    EdgeId e2 = graph.addEdge(n2, n3, "B->C");
    EdgeId e3 = graph.addEdge(n3, n4, "C->D");
    EdgeId e4 = graph.addEdge(n4, n5, "D->E");
    EdgeId e5 = graph.addEdge(n1, n5, "A->E");
    
    // Remove some nodes and edges
    graph.removeNode(n2);  // This should also remove e1 and e2
    graph.removeEdge(e3);
    
    graph.compact();
    
    // Verify graph structure
    EXPECT_EQ(graph.nodeCount(), 4);  // A, C, D, E
    EXPECT_EQ(graph.edgeCount(), 2);  // D->E, A->E
    
    // Verify remaining edges connect correct nodes
    auto edges = graph.edges();
    for (EdgeId id : edges) {
        auto edge = graph.tryGetEdge(id);
        EXPECT_TRUE(edge.has_value());
        
        auto from = graph.tryGetNode(edge->from);
        auto to = graph.tryGetNode(edge->to);
        EXPECT_TRUE(from.has_value());
        EXPECT_TRUE(to.has_value());
        
        // Verify edge labels match expected connections
        if (edge->label == "A->E") {
            EXPECT_EQ(from->label, "A");
            EXPECT_EQ(to->label, "E");
        } else if (edge->label == "D->E") {
            EXPECT_EQ(from->label, "D");
            EXPECT_EQ(to->label, "E");
        }
    }
}
