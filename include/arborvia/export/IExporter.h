#pragma once

#include <ostream>
#include <string>

namespace arborvia {

// Forward declarations
class Graph;
class CompoundGraph;
class LayoutResult;

/// Abstract interface for graph exporters
///
/// All export formats (SVG, PNG, DOT, etc.) should implement this interface
/// to enable polymorphic usage and easy format switching.
class IExporter {
public:
    virtual ~IExporter() = default;

    /// Export a simple graph layout to string
    virtual std::string exportToString(const Graph& graph, const LayoutResult& layout) = 0;
    
    /// Export a compound graph layout to string
    virtual std::string exportToString(const CompoundGraph& graph, const LayoutResult& layout) = 0;
    
    /// Export to an output stream
    virtual void exportToStream(const Graph& graph, const LayoutResult& layout, std::ostream& out) = 0;
    virtual void exportToStream(const CompoundGraph& graph, const LayoutResult& layout, std::ostream& out) = 0;
    
    /// Export to a file
    virtual bool exportToFile(const Graph& graph, const LayoutResult& layout, const std::string& filename) = 0;
    virtual bool exportToFile(const CompoundGraph& graph, const LayoutResult& layout, const std::string& filename) = 0;
    
    /// Get the file extension for this export format (e.g., "svg", "png", "dot")
    virtual std::string fileExtension() const = 0;
    
    /// Get the MIME type for this export format (e.g., "image/svg+xml")
    virtual std::string mimeType() const = 0;
};

}  // namespace arborvia
