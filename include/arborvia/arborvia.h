#pragma once

/// @file arborvia.h
/// @brief Main header for the ArborVia graph layout library
///
/// ArborVia is a lightweight C++ library for hierarchical graph layout,
/// designed for use in state machine editors and similar applications.
///
/// Example usage:
/// @code
/// #include <arborvia/arborvia.h>
///
/// arborvia::CompoundGraph graph;
/// auto s1 = graph.addNode("State1");
/// auto s2 = graph.addNode("State2");
/// graph.addEdge(s1, s2, "transition");
///
/// arborvia::SugiyamaLayout layout;
/// auto result = layout.layout(graph);
///
/// arborvia::SvgExport svg;
/// svg.exportToFile(graph, result, "output.svg");
/// @endcode

// Core module - Graph data structures
#include "core/Types.h"
#include "core/Graph.h"
#include "core/CompoundGraph.h"

// Layout module - Layout algorithms and results
#include "layout/config/LayoutOptions.h"
#include "layout/config/LayoutResult.h"
#include "layout/api/ILayout.h"
#include "layout/SugiyamaLayout.h"
#include "layout/config/LayoutEnums.h"
#include "layout/config/ManualLayoutState.h"
#include "layout/config/LayoutTypes.h"
#include "layout/interactive/UserLayoutController.h"
#include "layout/util/LayoutUtils.h"
#include "layout/util/LayoutSerializer.h"

// Export module - Output formats
#include "export/IExporter.h"
#ifdef ARBORVIA_ENABLE_SVG_EXPORT
#include "export/SvgExport.h"
#endif

namespace arborvia {

/// Library version
constexpr int VERSION_MAJOR = 0;
constexpr int VERSION_MINOR = 1;
constexpr int VERSION_PATCH = 0;

/// Get version as string (computed from constants)
inline std::string versionString() {
    return std::to_string(VERSION_MAJOR) + "." + 
           std::to_string(VERSION_MINOR) + "." + 
           std::to_string(VERSION_PATCH);
}

}  // namespace arborvia
