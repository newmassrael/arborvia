#pragma once

#include "../sugiyama/routing/EdgeRouting.h"
#include <unordered_map>
#include <functional>

namespace arborvia {

/**
 * @brief Distributes snap points across node edges for initial layout.
 *
 * This class handles the distribution of connection points (snap points)
 * along node edges during the initial layout phase. It ensures:
 * 1. Connections are evenly distributed along each node edge
 * 2. Snap points are sorted to minimize edge crossings
 * 3. Self-loops get corner positions
 * 4. Bend points are calculated for each edge
 */
class SnapDistributor {
public:
    /// Function type for recalculating bend points
    using RecalcBendPointsFunc = std::function<void(
        EdgeLayout& layout,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize)>;

    /**
     * @brief Construct a SnapDistributor.
     * @param recalcFunc Function to call for bend point recalculation.
     */
    explicit SnapDistributor(RecalcBendPointsFunc recalcFunc);

    /**
     * @brief Distribute snap points across all edges in the result.
     *
     * @param result Layout result containing edge layouts to update.
     * @param nodeLayouts All node layouts.
     * @param gridSize Grid size for snapping (0 = disabled).
     * @param sortSnapPoints If true, sort snap points to minimize crossings.
     */
    void distribute(
        EdgeRouting::Result& result,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize,
        bool sortSnapPoints = true);

private:
    /**
     * @brief Ensure first bend has proper clearance from source node.
     */
    void ensureFirstBendClearance(
        EdgeLayout& layout,
        float effectiveGridSize);

    RecalcBendPointsFunc recalcFunc_;
};

} // namespace arborvia
