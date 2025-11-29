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
