# ArborVia

Lightweight C++ library for hierarchical graph layout, designed for state machine editors and workflow diagrams.

## Features

- **Zero dependencies** - Pure C++ standard library only
- **MIT License** - Free for commercial use
- **Sugiyama layout** - Classic hierarchical graph drawing
- **Compound graphs** - Nested and parallel state support
- **SVG export** - Built-in visualization output

## Build

```bash
mkdir build && cd build
cmake ..
cmake --build .
```

## Quick Start

```cpp
#include <arborvia/arborvia.h>

// Create a graph
arborvia::Graph graph;
auto n1 = graph.addNode("Start");
auto n2 = graph.addNode("End");
graph.addEdge(n1, n2, "transition");

// Compute layout
arborvia::SugiyamaLayout layout;
arborvia::LayoutResult result = layout.layout(graph);

// Use positions for rendering
for (const auto& [id, nodeLayout] : result.nodeLayouts()) {
    // nodeLayout.position, nodeLayout.size
}
```

## Compound Graph Example

```cpp
arborvia::CompoundGraph graph;

// Create compound (parent) node
auto parent = graph.addCompoundNode(arborvia::CompoundType::Compound);

// Create child nodes
auto child1 = graph.addNode("State1");
auto child2 = graph.addNode("State2");

// Set hierarchy
graph.setParent(child1, parent);
graph.setParent(child2, parent);

// Layout handles nesting automatically
arborvia::SugiyamaLayout layout;
auto result = layout.layout(graph);
```

## Architecture

```
arborvia/
├── core/      - Graph data structures (Graph, CompoundGraph)
├── layout/    - Layout algorithms (SugiyamaLayout)
└── export/    - Output formats (SvgExport)
```

See [docs/KR/ARCHITECTURE_KR.md](docs/KR/ARCHITECTURE_KR.md) for detailed architecture.

## License

MIT License
