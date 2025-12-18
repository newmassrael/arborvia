#pragma once

#include "../config/LayoutEnums.h"
#include <string>

namespace arborvia {

// Forward declarations
class UserLayoutController;
class LayoutResult;

/// Handles JSON serialization and file I/O for layout state
/// Separates serialization concerns from UserLayoutController and LayoutResult
class LayoutSerializer {
public:
    // === UserLayoutController serialization ===

    /// Serialize user layout state to JSON string
    /// @param controller The UserLayoutController to serialize
    /// @return JSON string representation
    static std::string toJson(const UserLayoutController& controller);

    /// Deserialize JSON string to user layout state
    /// @param controller The UserLayoutController to populate
    /// @param json JSON string to parse
    /// @return true if parsing succeeded
    static bool fromJson(UserLayoutController& controller, const std::string& json);

    /// Save user layout state to file
    /// @param controller The UserLayoutController to save
    /// @param path File path to write
    /// @return true if save succeeded
    static bool saveToFile(const UserLayoutController& controller, const std::string& path);

    /// Load user layout state from file
    /// @param controller The UserLayoutController to populate
    /// @param path File path to read
    /// @return true if load succeeded
    static bool loadFromFile(UserLayoutController& controller, const std::string& path);

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
