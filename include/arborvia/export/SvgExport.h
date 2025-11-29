#pragma once

#include "../core/CompoundGraph.h"
#include "../layout/LayoutResult.h"
#include "IExporter.h"

#include <ostream>
#include <string>

namespace arborvia {

/// Options for SVG export
struct SvgExportOptions {
    // Canvas settings
    float padding = 20.0f;
    std::string backgroundColor = "white";
    
    // Node styling
    std::string nodeFill = "#e0e0e0";
    std::string nodeStroke = "#333333";
    float nodeStrokeWidth = 1.5f;
    float nodeCornerRadius = 5.0f;
    
    // Compound node styling
    std::string compoundFill = "#f5f5f5";
    std::string compoundStroke = "#666666";
    float compoundStrokeWidth = 1.0f;
    
    // Parallel node styling (dashed border)
    std::string parallelFill = "#f0f8ff";
    std::string parallelStroke = "#4a90d9";
    std::string parallelStrokeDasharray = "5,3";
    
    // Edge styling
    std::string edgeStroke = "#333333";
    float edgeStrokeWidth = 1.5f;
    std::string edgeMarkerEnd = "url(#arrowhead)";
    
    // Text styling
    std::string textFill = "#000000";
    std::string fontFamily = "Arial, sans-serif";
    float fontSize = 12.0f;
    
    // Show labels
    bool showNodeLabels = true;
    bool showEdgeLabels = true;
    
    // Include CSS styling
    bool embedStyles = true;
};

/// Exports layout results to SVG format
class SvgExport : public IExporter {
public:
    SvgExport() = default;
    explicit SvgExport(const SvgExportOptions& options);
    ~SvgExport() override = default;

    /// Export a simple graph layout to SVG
    std::string exportToString(const Graph& graph, const LayoutResult& layout) override;

    /// Export a compound graph layout to SVG
    std::string exportToString(const CompoundGraph& graph, const LayoutResult& layout) override;

    /// Export to an output stream
    void exportToStream(const Graph& graph, const LayoutResult& layout, std::ostream& out) override;
    void exportToStream(const CompoundGraph& graph, const LayoutResult& layout, std::ostream& out) override;

    /// Export to a file
    bool exportToFile(const Graph& graph, const LayoutResult& layout, const std::string& filename) override;
    bool exportToFile(const CompoundGraph& graph, const LayoutResult& layout, const std::string& filename) override;

    /// Get file extension
    std::string fileExtension() const override { return "svg"; }

    /// Get MIME type
    std::string mimeType() const override { return "image/svg+xml"; }

    /// Set export options
    void setOptions(const SvgExportOptions& options) { options_ = options; }
    const SvgExportOptions& options() const { return options_; }

private:
    SvgExportOptions options_;
    
    void writeHeader(std::ostream& out, const Rect& bounds);
    void writeStyles(std::ostream& out);
    void writeMarkers(std::ostream& out);
    void writeFooter(std::ostream& out);
    
    void writeNode(std::ostream& out, const Graph& graph, const NodeLayout& layout);
    void writeCompoundNode(std::ostream& out, const CompoundGraph& graph, const NodeLayout& layout);
    void writeEdge(std::ostream& out, const Graph& graph, const EdgeLayout& layout);
    
    std::string escapeXml(const std::string& text);
};

}  // namespace arborvia
