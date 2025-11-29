#include <arborvia/arborvia.h>
#include <iostream>
#include <iomanip>

using namespace arborvia;

// Simple edge re-routing (until routing module is implemented)
EdgeLayout rerouteEdge(const EdgeData& edge, 
                       const NodeLayout& fromNode, 
                       const NodeLayout& toNode) {
    EdgeLayout layout;
    layout.id = edge.id;
    layout.from = edge.from;
    layout.to = edge.to;
    
    Point fromCenter = fromNode.center();
    Point toCenter = toNode.center();
    
    // Connect bottom of source to top of target (TopToBottom direction)
    if (fromCenter.y < toCenter.y) {
        layout.sourcePoint = {fromCenter.x, fromNode.position.y + fromNode.size.height};
        layout.targetPoint = {toCenter.x, toNode.position.y};
    } else {
        layout.sourcePoint = {fromCenter.x, fromNode.position.y};
        layout.targetPoint = {toCenter.x, toNode.position.y + toNode.size.height};
    }
    
    // Add orthogonal bend points if needed
    if (std::abs(layout.sourcePoint.x - layout.targetPoint.x) > 1.0f) {
        float midY = (layout.sourcePoint.y + layout.targetPoint.y) / 2.0f;
        layout.bendPoints.push_back({{layout.sourcePoint.x, midY}});
        layout.bendPoints.push_back({{layout.targetPoint.x, midY}});
    }
    
    return layout;
}

void printEdgeLayout(const EdgeLayout& edge, const std::string& label = "") {
    std::cout << "  " << label << "Edge " << edge.id 
              << " (" << edge.from << " -> " << edge.to << "): ";
    auto points = edge.allPoints();
    for (size_t i = 0; i < points.size(); ++i) {
        if (i > 0) std::cout << " -> ";
        std::cout << "(" << std::fixed << std::setprecision(1) 
                  << points[i].x << "," << points[i].y << ")";
    }
    std::cout << "\n";
}

int main() {
    std::cout << "=== Drag Test: Edge Re-routing on Node Move ===\n\n";
    
    // 1. Create graph
    Graph graph;
    auto nodeA = graph.addNode(Size{80, 40}, "A");
    auto nodeB = graph.addNode(Size{80, 40}, "B");
    auto nodeC = graph.addNode(Size{80, 40}, "C");
    auto nodeD = graph.addNode(Size{80, 40}, "D");
    
    graph.addEdge(nodeA, nodeB);  // A -> B
    graph.addEdge(nodeA, nodeC);  // A -> C
    graph.addEdge(nodeB, nodeD);  // B -> D
    graph.addEdge(nodeC, nodeD);  // C -> D
    
    // 2. Initial layout
    SugiyamaLayout layoutEngine;
    LayoutResult result = layoutEngine.layout(graph);
    
    // Store layouts in mutable map for simulation
    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;
    
    for (const auto& [id, layout] : result.nodeLayouts()) {
        nodeLayouts[id] = layout;
    }
    for (const auto& [id, layout] : result.edgeLayouts()) {
        edgeLayouts[id] = layout;
    }
    
    // 3. Print initial state
    std::cout << "[Initial Layout]\n";
    std::cout << "Nodes:\n";
    for (const auto& [id, layout] : nodeLayouts) {
        std::cout << "  Node " << id << " (" << graph.getNode(id).label << "): "
                  << "pos=(" << layout.position.x << "," << layout.position.y << ")\n";
    }
    std::cout << "Edges:\n";
    for (const auto& [id, layout] : edgeLayouts) {
        printEdgeLayout(layout);
    }
    
    // 4. Export initial SVG
    SvgExport svg;
    svg.exportToFile(graph, result, "drag_before.svg");
    std::cout << "\nSaved: drag_before.svg\n";
    
    // ========================================
    // 5. SIMULATE DRAG: Move node B
    // ========================================
    std::cout << "\n[Simulating Drag: Moving Node B by (+100, +30)]\n";
    
    NodeId draggedNode = nodeB;
    Point dragDelta = {100.0f, 30.0f};
    
    // Step 1: Update node position
    nodeLayouts[draggedNode].position.x += dragDelta.x;
    nodeLayouts[draggedNode].position.y += dragDelta.y;
    
    std::cout << "Node " << draggedNode << " new position: ("
              << nodeLayouts[draggedNode].position.x << ","
              << nodeLayouts[draggedNode].position.y << ")\n";
    
    // Step 2: Get connected edges (THIS IS THE KEY API)
    auto connectedEdges = graph.getConnectedEdges(draggedNode);
    
    std::cout << "Connected edges to re-route: " << connectedEdges.size() << "\n";
    for (EdgeId edgeId : connectedEdges) {
        const EdgeData& edge = graph.getEdge(edgeId);
        std::cout << "  - Edge " << edgeId << " (" << edge.from << " -> " << edge.to << ")\n";
    }
    
    // Step 3: Re-route only connected edges
    std::cout << "\n[Re-routing affected edges]\n";
    for (EdgeId edgeId : connectedEdges) {
        const EdgeData& edge = graph.getEdge(edgeId);
        const NodeLayout& fromNode = nodeLayouts[edge.from];
        const NodeLayout& toNode = nodeLayouts[edge.to];
        
        EdgeLayout oldLayout = edgeLayouts[edgeId];
        EdgeLayout newLayout = rerouteEdge(edge, fromNode, toNode);
        
        std::cout << "Edge " << edgeId << ":\n";
        printEdgeLayout(oldLayout, "Before: ");
        printEdgeLayout(newLayout, "After:  ");
        
        edgeLayouts[edgeId] = newLayout;
    }
    
    // 6. Update result and export
    LayoutResult updatedResult;
    updatedResult.setLayerCount(result.layerCount());
    for (const auto& [id, layout] : nodeLayouts) {
        updatedResult.setNodeLayout(id, layout);
    }
    for (const auto& [id, layout] : edgeLayouts) {
        updatedResult.setEdgeLayout(id, layout);
    }
    
    svg.exportToFile(graph, updatedResult, "drag_after.svg");
    std::cout << "\nSaved: drag_after.svg\n";
    
    // 7. Summary
    std::cout << "\n=== Summary ===\n";
    std::cout << "Total edges: " << graph.edgeCount() << "\n";
    std::cout << "Re-routed edges: " << connectedEdges.size() << "\n";
    std::cout << "Unchanged edges: " << (graph.edgeCount() - connectedEdges.size()) << "\n";
    std::cout << "\nCompare drag_before.svg and drag_after.svg to see the difference.\n";
    
    return 0;
}
