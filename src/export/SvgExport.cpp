#include "arborvia/export/SvgExport.h"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>

namespace arborvia {

SvgExport::SvgExport(const SvgExportOptions& options)
    : options_(options) {}

std::string SvgExport::exportToString(const Graph& graph, const LayoutResult& layout) {
    std::ostringstream out;
    exportToStream(graph, layout, out);
    return out.str();
}

std::string SvgExport::exportToString(const CompoundGraph& graph, const LayoutResult& layout) {
    std::ostringstream out;
    exportToStream(graph, layout, out);
    return out.str();
}

void SvgExport::exportToStream(const Graph& graph, const LayoutResult& layout, std::ostream& out) {
    Rect bounds = layout.computeBounds(options_.padding);
    
    writeHeader(out, bounds);
    writeStyles(out);
    writeMarkers(out);
    
    // Write edges first (so nodes are drawn on top)
    for (const auto& [id, edgeLayout] : layout.edgeLayouts()) {
        writeEdge(out, graph, edgeLayout);
    }
    
    // Write nodes
    for (const auto& [id, nodeLayout] : layout.nodeLayouts()) {
        writeNode(out, graph, nodeLayout);
    }
    
    writeFooter(out);
}

void SvgExport::exportToStream(const CompoundGraph& graph, const LayoutResult& layout, std::ostream& out) {
    Rect bounds = layout.computeBounds(options_.padding);
    
    writeHeader(out, bounds);
    writeStyles(out);
    writeMarkers(out);
    
    // Write edges first
    for (const auto& [id, edgeLayout] : layout.edgeLayouts()) {
        writeEdge(out, graph, edgeLayout);
    }
    
    // Write compound nodes (largest first, so children are drawn on top)
    std::vector<std::pair<float, NodeId>> nodesByArea;
    for (const auto& [id, nodeLayout] : layout.nodeLayouts()) {
        float area = nodeLayout.size.width * nodeLayout.size.height;
        nodesByArea.emplace_back(area, id);
    }
    std::sort(nodesByArea.rbegin(), nodesByArea.rend());
    
    for (const auto& [area, id] : nodesByArea) {
        const NodeLayout* nodeLayout = layout.getNodeLayout(id);
        if (nodeLayout) {
            writeCompoundNode(out, graph, *nodeLayout);
        }
    }
    
    writeFooter(out);
}

bool SvgExport::exportToFile(const Graph& graph, const LayoutResult& layout, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    exportToStream(graph, layout, file);
    return true;
}

bool SvgExport::exportToFile(const CompoundGraph& graph, const LayoutResult& layout, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    exportToStream(graph, layout, file);
    return true;
}

void SvgExport::writeHeader(std::ostream& out, const Rect& bounds) {
    out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
        << "width=\"" << bounds.width << "\" "
        << "height=\"" << bounds.height << "\" "
        << "viewBox=\"" << bounds.x << " " << bounds.y << " " 
        << bounds.width << " " << bounds.height << "\">\n";
    
    // Background
    out << "  <rect x=\"" << bounds.x << "\" y=\"" << bounds.y << "\" "
        << "width=\"" << bounds.width << "\" height=\"" << bounds.height << "\" "
        << "fill=\"" << options_.backgroundColor << "\"/>\n";
}

void SvgExport::writeStyles(std::ostream& out) {
    if (!options_.embedStyles) return;
    
    out << "  <style>\n";
    out << "    .node { fill: " << options_.nodeFill << "; "
        << "stroke: " << options_.nodeStroke << "; "
        << "stroke-width: " << options_.nodeStrokeWidth << "; }\n";
    out << "    .compound { fill: " << options_.compoundFill << "; "
        << "stroke: " << options_.compoundStroke << "; "
        << "stroke-width: " << options_.compoundStrokeWidth << "; }\n";
    out << "    .parallel { fill: " << options_.parallelFill << "; "
        << "stroke: " << options_.parallelStroke << "; "
        << "stroke-width: " << options_.compoundStrokeWidth << "; "
        << "stroke-dasharray: " << options_.parallelStrokeDasharray << "; }\n";
    out << "    .edge { fill: none; "
        << "stroke: " << options_.edgeStroke << "; "
        << "stroke-width: " << options_.edgeStrokeWidth << "; }\n";
    out << "    .label { fill: " << options_.textFill << "; "
        << "font-family: " << options_.fontFamily << "; "
        << "font-size: " << options_.fontSize << "px; "
        << "text-anchor: middle; dominant-baseline: central; }\n";
    out << "  </style>\n";
}

void SvgExport::writeMarkers(std::ostream& out) {
    // Arrowhead marker for edges
    out << "  <defs>\n";
    out << "    <marker id=\"arrowhead\" markerWidth=\"10\" markerHeight=\"7\" "
        << "refX=\"9\" refY=\"3.5\" orient=\"auto\">\n";
    out << "      <polygon points=\"0 0, 10 3.5, 0 7\" fill=\"" 
        << options_.edgeStroke << "\"/>\n";
    out << "    </marker>\n";
    out << "  </defs>\n";
}

void SvgExport::writeFooter(std::ostream& out) {
    out << "</svg>\n";
}

void SvgExport::writeNode(std::ostream& out, const Graph& graph, const NodeLayout& layout) {
    out << "  <rect class=\"node\" "
        << "x=\"" << layout.position.x << "\" "
        << "y=\"" << layout.position.y << "\" "
        << "width=\"" << layout.size.width << "\" "
        << "height=\"" << layout.size.height << "\" "
        << "rx=\"" << options_.nodeCornerRadius << "\"/>\n";
    
    // Node label
    if (options_.showNodeLabels && graph.hasNode(layout.id)) {
        const NodeData node = graph.getNode(layout.id);
        if (!node.label.empty()) {
            Point center = layout.center();
            out << "  <text class=\"label\" "
                << "x=\"" << center.x << "\" "
                << "y=\"" << center.y << "\">"
                << escapeXml(node.label) << "</text>\n";
        }
    }
}

void SvgExport::writeCompoundNode(std::ostream& out, const CompoundGraph& graph, const NodeLayout& layout) {
    std::string nodeClass = "node";
    
    if (graph.isParallel(layout.id)) {
        nodeClass = "parallel";
    } else if (graph.isCompound(layout.id)) {
        nodeClass = "compound";
    }
    
    out << "  <rect class=\"" << nodeClass << "\" "
        << "x=\"" << layout.position.x << "\" "
        << "y=\"" << layout.position.y << "\" "
        << "width=\"" << layout.size.width << "\" "
        << "height=\"" << layout.size.height << "\" "
        << "rx=\"" << options_.nodeCornerRadius << "\"/>\n";
    
    // Node label
    if (options_.showNodeLabels && graph.hasNode(layout.id)) {
        const NodeData node = graph.getNode(layout.id);
        if (!node.label.empty()) {
            // For compound nodes, put label at top
            float labelY = layout.position.y + options_.fontSize + 5;
            if (graph.isAtomic(layout.id)) {
                labelY = layout.center().y;
            }
            
            out << "  <text class=\"label\" "
                << "x=\"" << layout.center().x << "\" "
                << "y=\"" << labelY << "\">"
                << escapeXml(node.label) << "</text>\n";
        }
    }
}

void SvgExport::writeEdge(std::ostream& out, const Graph& graph, const EdgeLayout& layout) {
    std::vector<Point> points = layout.allPoints();
    
    if (points.size() < 2) return;
    
    out << "  <path class=\"edge\" d=\"";
    out << "M " << points[0].x << " " << points[0].y;
    
    for (size_t i = 1; i < points.size(); ++i) {
        out << " L " << points[i].x << " " << points[i].y;
    }
    
    out << "\" marker-end=\"" << options_.edgeMarkerEnd << "\"/>\n";
    
    // Edge label
    if (options_.showEdgeLabels && graph.hasEdge(layout.id)) {
        const EdgeData edge = graph.getEdge(layout.id);
        if (!edge.label.empty()) {
            // Use pre-computed label position
            out << "  <text class=\"label\" "
                << "x=\"" << layout.labelPosition.x << "\" "
                << "y=\"" << layout.labelPosition.y - 5 << "\">"
                << escapeXml(edge.label) << "</text>\n";
        }
    }
}

std::string SvgExport::escapeXml(const std::string& text) {
    std::string result;
    result.reserve(text.size());
    
    for (char c : text) {
        switch (c) {
            case '&': result += "&amp;"; break;
            case '<': result += "&lt;"; break;
            case '>': result += "&gt;"; break;
            case '"': result += "&quot;"; break;
            case '\'': result += "&apos;"; break;
            default: result += c;
        }
    }
    
    return result;
}

}  // namespace arborvia
