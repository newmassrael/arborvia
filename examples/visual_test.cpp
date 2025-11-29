#include <arborvia/arborvia.h>
#include <iostream>

int main() {
    using namespace arborvia;
    
    // 1. Simple graph
    {
        Graph graph;
        auto start = graph.addNode(Size{80, 40}, "Start");
        auto process1 = graph.addNode(Size{100, 40}, "Process A");
        auto process2 = graph.addNode(Size{100, 40}, "Process B");
        auto end = graph.addNode(Size{80, 40}, "End");
        
        graph.addEdge(start, process1, "init");
        graph.addEdge(start, process2, "init");
        graph.addEdge(process1, end, "done");
        graph.addEdge(process2, end, "done");
        
        SugiyamaLayout layout;
        LayoutResult result = layout.layout(graph);
        
        SvgExport svg;
        svg.exportToFile(graph, result, "simple_graph.svg");
        std::cout << "Generated: simple_graph.svg\n";
    }
    
    // 2. Compound graph (state machine style)
    {
        CompoundGraph graph;
        
        // Root states
        auto idle = graph.addNode(Size{100, 50}, "Idle");
        auto running = graph.addCompoundNode("Running", CompoundType::Compound);
        auto finished = graph.addNode(Size{100, 50}, "Finished");
        
        // Child states inside "Running"
        auto init = graph.addNode(Size{80, 40}, "Init");
        auto execute = graph.addNode(Size{80, 40}, "Execute");
        auto cleanup = graph.addNode(Size{80, 40}, "Cleanup");
        
        graph.setParent(init, running);
        graph.setParent(execute, running);
        graph.setParent(cleanup, running);
        
        // Transitions
        graph.addEdge(idle, running, "start");
        graph.addEdge(running, finished, "complete");
        graph.addEdge(init, execute);
        graph.addEdge(execute, cleanup);
        
        SugiyamaLayout layout;
        LayoutResult result = layout.layout(graph);
        
        SvgExport svg;
        svg.exportToFile(graph, result, "compound_graph.svg");
        std::cout << "Generated: compound_graph.svg\n";
    }
    
    // 3. Complex graph with cycle
    {
        Graph graph;
        auto a = graph.addNode(Size{60, 40}, "A");
        auto b = graph.addNode(Size{60, 40}, "B");
        auto c = graph.addNode(Size{60, 40}, "C");
        auto d = graph.addNode(Size{60, 40}, "D");
        auto e = graph.addNode(Size{60, 40}, "E");
        
        graph.addEdge(a, b);
        graph.addEdge(a, c);
        graph.addEdge(b, d);
        graph.addEdge(c, d);
        graph.addEdge(d, e);
        graph.addEdge(e, a);  // cycle back
        
        SugiyamaLayout layout;
        LayoutResult result = layout.layout(graph);
        
        std::cout << "Cycle test - reversed edges: " 
                  << layout.lastStats().reversedEdges << "\n";
        
        SvgExport svg;
        svg.exportToFile(graph, result, "cycle_graph.svg");
        std::cout << "Generated: cycle_graph.svg\n";
    }
    
    std::cout << "\nOpen SVG files in browser to view.\n";
    return 0;
}
