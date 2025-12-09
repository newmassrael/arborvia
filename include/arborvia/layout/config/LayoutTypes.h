#pragma once

/// @file LayoutTypes.h
/// @brief Convenience header that includes all layout type definitions
/// @note For finer-grained includes, use LayoutEnums.h and ManualLayoutState.h directly

#include "LayoutEnums.h"
#include "ManualLayoutState.h"
#include "../../core/Types.h"

namespace arborvia {

/// Forbidden zone representing a region where a node cannot be placed
struct ForbiddenZone {
    Rect bounds;       ///< Bounding rectangle of the forbidden region
    NodeId blockedBy;  ///< Node ID that creates this forbidden zone
};

}  // namespace arborvia
