#pragma once

#include "../core/Types.h"
#include "LayoutResult.h"
#include "LayoutOptions.h"

namespace arborvia {

// Forward declarations
class Graph;
class CompoundGraph;

/// Abstract interface for graph layout algorithms
///
/// All layout algorithms should implement this interface to enable
/// polymorphic usage and easy swapping of layout strategies.
class ILayout {
public:
    virtual ~ILayout() = default;

    /// Set layout options
    virtual void setOptions(const LayoutOptions& options) = 0;
    
    /// Get current layout options
    virtual const LayoutOptions& options() const = 0;

    /// Perform layout on a simple graph
    virtual LayoutResult layout(const Graph& graph) = 0;

    /// Perform layout on a compound graph (with hierarchy)
    virtual LayoutResult layout(const CompoundGraph& graph) = 0;
};

}  // namespace arborvia
