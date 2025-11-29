#include <gtest/gtest.h>
#include <arborvia/core/CompoundGraph.h>

#include <algorithm>

using namespace arborvia;

TEST(CompoundGraphTest, AddCompoundNode) {
    CompoundGraph graph;
    NodeId id = graph.addCompoundNode(CompoundType::Compound);

    EXPECT_TRUE(graph.hasNode(id));
    EXPECT_TRUE(graph.isCompound(id));

    // Verify mutually exclusive type flags
    EXPECT_FALSE(graph.isParallel(id));
    EXPECT_FALSE(graph.isAtomic(id));
}

TEST(CompoundGraphTest, AddParallelNode) {
    CompoundGraph graph;
    NodeId id = graph.addCompoundNode(CompoundType::Parallel);

    EXPECT_TRUE(graph.hasNode(id));
    EXPECT_TRUE(graph.isParallel(id));

    // Verify mutually exclusive type flags
    EXPECT_FALSE(graph.isCompound(id));
    EXPECT_FALSE(graph.isAtomic(id));
}

TEST(CompoundGraphTest, SetParent) {
    CompoundGraph graph;
    NodeId parent = graph.addCompoundNode(CompoundType::Compound);
    NodeId child = graph.addNode("child");
    
    graph.setParent(child, parent);
    
    auto p = graph.getParent(child);
    EXPECT_TRUE(p.has_value());
    EXPECT_EQ(p.value(), parent);
    
    auto children = graph.getChildren(parent);
    EXPECT_EQ(children.size(), 1);
    EXPECT_EQ(children[0], child);
}

TEST(CompoundGraphTest, SetParentCycleDetection) {
    CompoundGraph graph;
    NodeId n1 = graph.addNode("n1");
    NodeId n2 = graph.addNode("n2");
    NodeId n3 = graph.addNode("n3");
    
    graph.setParent(n2, n1);
    graph.setParent(n3, n2);
    
    // n1 is ancestor of n3, so setting n1's parent to n3 should throw
    EXPECT_THROW(graph.setParent(n1, n3), std::invalid_argument);
}

TEST(CompoundGraphTest, SetParentSelfReference) {
    CompoundGraph graph;
    NodeId n1 = graph.addNode("n1");
    
    EXPECT_THROW(graph.setParent(n1, n1), std::invalid_argument);
}

TEST(CompoundGraphTest, RemoveFromParent) {
    CompoundGraph graph;
    NodeId parent = graph.addCompoundNode(CompoundType::Compound);
    NodeId child = graph.addNode("child");
    
    graph.setParent(child, parent);
    graph.removeFromParent(child);
    
    EXPECT_FALSE(graph.getParent(child).has_value());
    EXPECT_TRUE(graph.getChildren(parent).empty());
}

TEST(CompoundGraphTest, GetDescendants) {
    CompoundGraph graph;
    NodeId root = graph.addCompoundNode(CompoundType::Compound);
    NodeId child1 = graph.addNode("child1");
    NodeId child2 = graph.addNode("child2");
    NodeId grandchild = graph.addNode("grandchild");

    graph.setParent(child1, root);
    graph.setParent(child2, root);
    graph.setParent(grandchild, child1);

    auto descendants = graph.getDescendants(root);
    EXPECT_EQ(descendants.size(), 3);

    // Verify actual content, not just size
    EXPECT_TRUE(std::find(descendants.begin(), descendants.end(), child1) != descendants.end());
    EXPECT_TRUE(std::find(descendants.begin(), descendants.end(), child2) != descendants.end());
    EXPECT_TRUE(std::find(descendants.begin(), descendants.end(), grandchild) != descendants.end());
    EXPECT_TRUE(std::find(descendants.begin(), descendants.end(), root) == descendants.end());
}

TEST(CompoundGraphTest, GetAncestors) {
    CompoundGraph graph;
    NodeId root = graph.addCompoundNode(CompoundType::Compound);
    NodeId child = graph.addNode("child");
    NodeId grandchild = graph.addNode("grandchild");
    
    graph.setParent(child, root);
    graph.setParent(grandchild, child);
    
    auto ancestors = graph.getAncestors(grandchild);
    EXPECT_EQ(ancestors.size(), 2);
    EXPECT_EQ(ancestors[0], child);
    EXPECT_EQ(ancestors[1], root);
}

TEST(CompoundGraphTest, IsRoot) {
    CompoundGraph graph;
    NodeId root = graph.addNode("root");
    NodeId child = graph.addNode("child");
    
    graph.setParent(child, root);
    
    EXPECT_TRUE(graph.isRoot(root));
    EXPECT_FALSE(graph.isRoot(child));
}

TEST(CompoundGraphTest, GetDepth) {
    CompoundGraph graph;
    NodeId root = graph.addNode("root");
    NodeId child = graph.addNode("child");
    NodeId grandchild = graph.addNode("grandchild");
    
    graph.setParent(child, root);
    graph.setParent(grandchild, child);
    
    EXPECT_EQ(graph.getDepth(root), 0);
    EXPECT_EQ(graph.getDepth(child), 1);
    EXPECT_EQ(graph.getDepth(grandchild), 2);
}

TEST(CompoundGraphTest, Collapse) {
    CompoundGraph graph;
    NodeId parent = graph.addCompoundNode(CompoundType::Compound);
    NodeId child = graph.addNode("child");
    
    graph.setParent(child, parent);
    
    EXPECT_TRUE(graph.isVisible(child));
    
    graph.setCollapsed(parent, true);
    EXPECT_TRUE(graph.isCollapsed(parent));
    EXPECT_FALSE(graph.isVisible(child));
    
    graph.setCollapsed(parent, false);
    EXPECT_TRUE(graph.isVisible(child));
}

TEST(CompoundGraphTest, RootNodes) {
    CompoundGraph graph;
    NodeId root1 = graph.addNode("root1");
    NodeId root2 = graph.addNode("root2");
    NodeId child = graph.addNode("child");

    graph.setParent(child, root1);

    auto roots = graph.rootNodes();
    EXPECT_EQ(roots.size(), 2);

    // Verify actual content, not just size
    EXPECT_TRUE(std::find(roots.begin(), roots.end(), root1) != roots.end());
    EXPECT_TRUE(std::find(roots.begin(), roots.end(), root2) != roots.end());
    EXPECT_TRUE(std::find(roots.begin(), roots.end(), child) == roots.end());
}

TEST(CompoundGraphTest, LowestCommonAncestor) {
    CompoundGraph graph;
    NodeId root = graph.addNode("root");
    NodeId child1 = graph.addNode("child1");
    NodeId child2 = graph.addNode("child2");
    NodeId grandchild1 = graph.addNode("grandchild1");
    NodeId grandchild2 = graph.addNode("grandchild2");
    
    graph.setParent(child1, root);
    graph.setParent(child2, root);
    graph.setParent(grandchild1, child1);
    graph.setParent(grandchild2, child2);
    
    auto lca = graph.lowestCommonAncestor(grandchild1, grandchild2);
    EXPECT_TRUE(lca.has_value());
    EXPECT_EQ(lca.value(), root);
    
    auto lca2 = graph.lowestCommonAncestor(grandchild1, child1);
    EXPECT_TRUE(lca2.has_value());
    EXPECT_EQ(lca2.value(), child1);
}

TEST(CompoundGraphTest, IsInternalEdge) {
    CompoundGraph graph;
    NodeId parent = graph.addCompoundNode(CompoundType::Compound);
    NodeId child1 = graph.addNode("child1");
    NodeId child2 = graph.addNode("child2");
    NodeId external = graph.addNode("external");
    
    graph.setParent(child1, parent);
    graph.setParent(child2, parent);
    
    EdgeId internalEdge = graph.addEdge(child1, child2);
    EdgeId externalEdge = graph.addEdge(child1, external);
    
    EXPECT_TRUE(graph.isInternalEdge(internalEdge));
    EXPECT_FALSE(graph.isInternalEdge(externalEdge));
    EXPECT_TRUE(graph.isExternalEdge(externalEdge));
}
