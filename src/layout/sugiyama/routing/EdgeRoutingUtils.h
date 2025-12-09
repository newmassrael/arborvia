#pragma once

#include "arborvia/layout/config/LayoutResult.h"
#include <unordered_map>
#include <cmath>

namespace arborvia {

/**
 * @brief Result of evaluating a single edge routing combination.
 *
 * Shared by AStarEdgeOptimizer and GeometricEdgeOptimizer for
 * consistent edge combination evaluation results.
 */
struct EdgeCombinationResult {
    NodeEdge sourceEdge;       ///< Source edge of the node
    NodeEdge targetEdge;       ///< Target edge of the node
    int score;                 ///< Penalty score (lower is better)
    EdgeLayout layout;         ///< The resulting edge layout
    bool valid = true;         ///< False if no valid path exists
};

/**
 * @brief Shared utility functions for edge routing operations.
 *
 * This class consolidates common utility functions used across multiple
 * edge routing components to eliminate code duplication.
 */
class EdgeRoutingUtils {
public:
    /**
     * @brief Check if an EdgeLayout has fresh (non-stale) bendPoints.
     *
     * Stale bendPoints occur when sourcePoint was updated (e.g., due to node drag)
     * but bendPoints still reflect the old position, creating a diagonal path.
     *
     * @param layout The edge layout to check
     * @param gridSize Grid size for diagonal detection threshold
     * @return true if bendPoints are fresh and consistent with sourcePoint
     */
    static bool hasFreshBendPoints(const EdgeLayout& layout, float gridSize) {
        if (layout.bendPoints.empty()) {
            return true;  // No bendPoints = direct connection, considered fresh
        }

        // Check source side: sourcePoint -> firstBend
        const Point& src = layout.sourcePoint;
        const Point& firstBend = layout.bendPoints[0].position;
        float dx_src = std::abs(src.x - firstBend.x);
        float dy_src = std::abs(src.y - firstBend.y);
        bool sourceDiagonal = (dx_src > gridSize && dy_src > gridSize);

        // Check target side: lastBend -> targetPoint
        const Point& lastBend = layout.bendPoints.back().position;
        const Point& tgt = layout.targetPoint;
        float dx_tgt = std::abs(lastBend.x - tgt.x);
        float dy_tgt = std::abs(lastBend.y - tgt.y);
        bool targetDiagonal = (dx_tgt > gridSize && dy_tgt > gridSize);

        // If either side has diagonal, bendPoints are stale
        return !(sourceDiagonal || targetDiagonal);
    }

    /**
     * @brief Check if an orthogonal segment passes through a node's INTERIOR.
     *
     * Returns true only if the segment enters the interior region, not just touches edges.
     * Uses a small margin (1px) to avoid false positives from boundary-touching segments.
     *
     * @param p1 First point of the segment
     * @param p2 Second point of the segment
     * @param node The node to check against
     * @return true if segment passes through node interior
     */
    static bool segmentPenetratesNodeInterior(const Point& p1, const Point& p2, const NodeLayout& node) {
        constexpr float MARGIN = 1.0f;

        float left = node.position.x;
        float right = node.position.x + node.size.width;
        float top = node.position.y;
        float bottom = node.position.y + node.size.height;

        // Check vertical segment (same X coordinate)
        if (std::abs(p1.x - p2.x) < 0.1f) {
            float x = p1.x;
            float minY = std::min(p1.y, p2.y);
            float maxY = std::max(p1.y, p2.y);

            if (x > left + MARGIN && x < right - MARGIN) {
                if (minY < bottom - MARGIN && maxY > top + MARGIN) {
                    return true;
                }
            }
        }
        // Check horizontal segment (same Y coordinate)
        else if (std::abs(p1.y - p2.y) < 0.1f) {
            float y = p1.y;
            float minX = std::min(p1.x, p2.x);
            float maxX = std::max(p1.x, p2.x);

            if (y > top + MARGIN && y < bottom - MARGIN) {
                if (minX < right - MARGIN && maxX > left + MARGIN) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * @brief Check if shifting a segment would cause it to penetrate any node's interior.
     *
     * @param p1 First point of segment (will be shifted)
     * @param p2 Second point of segment (will be shifted)
     * @param isVertical Whether this is a vertical segment (shift in X direction)
     * @param shift The shift amount to apply
     * @param nodeLayouts All node layouts to check against
     * @param excludeFrom Node ID to exclude (source node)
     * @param excludeTo Node ID to exclude (target node)
     * @return true if shifted segment would penetrate any node
     */
    static bool shiftWouldPenetrateNode(
        const Point& p1, const Point& p2,
        bool isVertical, float shift,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        NodeId excludeFrom, NodeId excludeTo) {

        Point shiftedP1 = p1;
        Point shiftedP2 = p2;

        if (isVertical) {
            shiftedP1.x += shift;
            shiftedP2.x += shift;
        } else {
            shiftedP1.y += shift;
            shiftedP2.y += shift;
        }

        for (const auto& [nodeId, nodeLayout] : nodeLayouts) {
            if (nodeId == excludeFrom || nodeId == excludeTo) continue;

            if (segmentPenetratesNodeInterior(shiftedP1, shiftedP2, nodeLayout)) {
                return true;
            }
        }
        return false;
    }

    /**
     * @brief Count how many edges are connected to a specific NodeEdge.
     *
     * Used for proper snap index allocation when switching NodeEdges.
     *
     * @param edgeLayouts All edge layouts
     * @param nodeId The node to check
     * @param edge Which edge of the node
     * @param excludeId Edge ID to exclude from counting (the edge being routed)
     * @return Number of edges connected to this NodeEdge
     */
    static int countEdgesOnNodeEdge(
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        NodeId nodeId,
        NodeEdge edge,
        EdgeId excludeId) {

        int count = 0;
        for (const auto& [id, layout] : edgeLayouts) {
            if (id == excludeId) continue;

            if (layout.from == nodeId && layout.sourceEdge == edge) {
                count++;
            }
            if (layout.to == nodeId && layout.targetEdge == edge) {
                count++;
            }
        }
        return count;
    }

    /**
     * @brief Calculate the center point of a node edge.
     *
     * Returns the midpoint of the specified edge of the node.
     * Used by optimizers to determine edge attachment points.
     *
     * @param node The node layout
     * @param edge Which edge of the node (Top, Bottom, Left, Right)
     * @return Center point of the specified edge
     */
    static Point calculateEdgeCenter(const NodeLayout& node, NodeEdge edge) {
        switch (edge) {
            case NodeEdge::Top:
                return {
                    node.position.x + node.size.width * 0.5f,
                    node.position.y
                };
            case NodeEdge::Bottom:
                return {
                    node.position.x + node.size.width * 0.5f,
                    node.position.y + node.size.height
                };
            case NodeEdge::Left:
                return {
                    node.position.x,
                    node.position.y + node.size.height * 0.5f
                };
            case NodeEdge::Right:
                return {
                    node.position.x + node.size.width,
                    node.position.y + node.size.height * 0.5f
                };
        }
        return node.center();
    }

    /**
     * @brief Hash function for std::pair types.
     */
    struct PairHash {
        template <typename T1, typename T2>
        std::size_t operator()(const std::pair<T1, T2>& p) const {
            auto h1 = std::hash<T1>{}(p.first);
            auto h2 = std::hash<T2>{}(p.second);
            return h1 ^ (h2 << 1);
        }
    };
};

} // namespace arborvia
