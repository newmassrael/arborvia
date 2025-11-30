#pragma once

#include "LayoutEnums.h"
#include <string>

namespace arborvia {

// Forward declarations
class ManualLayoutManager;
class LayoutResult;

/// Handles JSON serialization and file I/O for layout state
/// Separates serialization concerns from ManualLayoutManager and LayoutResult
class LayoutSerializer {
public:
    // === ManualLayoutManager serialization ===

    /// Serialize manual layout state to JSON string
    /// @param manager The ManualLayoutManager to serialize
    /// @return JSON string representation
    static std::string toJson(const ManualLayoutManager& manager);

    /// Deserialize JSON string to manual layout state
    /// @param manager The ManualLayoutManager to populate
    /// @param json JSON string to parse
    /// @return true if parsing succeeded
    static bool fromJson(ManualLayoutManager& manager, const std::string& json);

    /// Save manual layout state to file
    /// @param manager The ManualLayoutManager to save
    /// @param path File path to write
    /// @return true if save succeeded
    static bool saveToFile(const ManualLayoutManager& manager, const std::string& path);

    /// Load manual layout state from file
    /// @param manager The ManualLayoutManager to populate
    /// @param path File path to read
    /// @return true if load succeeded
    static bool loadFromFile(ManualLayoutManager& manager, const std::string& path);

    // === LayoutResult serialization ===

    /// Serialize layout result to JSON string
    /// @param result The LayoutResult to serialize
    /// @return JSON string representation
    static std::string toJson(const LayoutResult& result);

    /// Deserialize JSON string to layout result
    /// @param json JSON string to parse
    /// @return Parsed LayoutResult
    /// @throws std::runtime_error if parsing fails
    static LayoutResult layoutResultFromJson(const std::string& json);

private:
    // JSON helper functions
    static std::string nodeEdgeToString(NodeEdge edge);
    static NodeEdge stringToNodeEdge(const std::string& str);
};

}  // namespace arborvia
