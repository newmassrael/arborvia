#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace arborvia {

// Forward declarations
class IConstraintValidator;
struct LayoutOptions;

/// Configuration for a single constraint
struct SingleConstraintConfig {
    std::string type;                    ///< Constraint type name
    bool enabled = true;                 ///< Whether this constraint is active
    
    // Type-specific parameters
    std::optional<float> gridDistance;   ///< For margin/distance constraints
    std::optional<float> margin;         ///< For overlap constraints
    std::optional<float> gridSize;       ///< For path-finding constraints
    
    /// Create DirectionAwareMargin constraint config
    static SingleConstraintConfig directionAwareMargin(float gridUnits = 5.0f) {
        SingleConstraintConfig config;
        config.type = "DirectionAwareMargin";
        config.gridDistance = gridUnits;
        return config;
    }
    
    /// Create NoOverlap constraint config
    static SingleConstraintConfig noOverlap(float margin = 0.0f) {
        SingleConstraintConfig config;
        config.type = "NoOverlap";
        config.margin = margin;
        return config;
    }
    
    /// Create EdgePathValidity constraint config
    static SingleConstraintConfig edgePathValidity(float gridSize = 10.0f) {
        SingleConstraintConfig config;
        config.type = "EdgePathValidity";
        config.gridSize = gridSize;
        return config;
    }
    
    /// Create Boundary constraint config (auto-detect mode)
    static SingleConstraintConfig boundary(float margin = 50.0f) {
        SingleConstraintConfig config;
        config.type = "Boundary";
        config.margin = margin;
        return config;
    }
    
    /// Create Boundary constraint config (explicit bounds)
    static SingleConstraintConfig boundaryExplicit(float minX, float minY, float maxX, float maxY, float margin = 0.0f) {
        SingleConstraintConfig config;
        config.type = "BoundaryExplicit";
        config.margin = margin;
        // Store bounds in unused fields (or we could add more fields)
        config.minX = minX;
        config.minY = minY;
        config.maxX = maxX;
        config.maxY = maxY;
        return config;
    }
    
    // Additional fields for boundary constraints
    std::optional<float> minX;
    std::optional<float> minY;
    std::optional<float> maxX;
    std::optional<float> maxY;
};

/// Configuration for drag constraints
///
/// This struct provides a declarative way to configure which constraints
/// are active during drag operations. It can be serialized to/from JSON
/// and used to create a ConstraintManager.
///
/// Example usage:
/// @code
/// ConstraintConfig config = ConstraintConfig::createDefault();
/// auto manager = ConstraintFactory::create(config);
/// @endcode
struct ConstraintConfig {
    std::vector<SingleConstraintConfig> constraints;
    
    /// Create default constraint configuration
    /// Includes: DirectionAwareMargin, EdgePathValidity (unless HideUntilDrop mode)
    /// @param options Layout options to determine which constraints to enable (nullptr for default)
    /// @note EdgePathValidity is skipped when DragAlgorithm::HideUntilDrop is used
    static ConstraintConfig createDefault(const LayoutOptions* options = nullptr);
    
    /// Create empty configuration (no constraints)
    static ConstraintConfig createEmpty() { return {}; }
    
    /// Check if configuration has any constraints
    bool empty() const { return constraints.empty(); }
    
    /// Builder: add DirectionAwareMargin constraint
    ConstraintConfig& addDirectionAwareMargin(float gridUnits = 5.0f) {
        constraints.push_back(SingleConstraintConfig::directionAwareMargin(gridUnits));
        return *this;
    }
    
    /// Builder: add NoOverlap constraint
    ConstraintConfig& addNoOverlap(float margin = 0.0f) {
        constraints.push_back(SingleConstraintConfig::noOverlap(margin));
        return *this;
    }
    
    /// Builder: add EdgePathValidity constraint
    ConstraintConfig& addEdgePathValidity(float gridSize = 10.0f) {
        constraints.push_back(SingleConstraintConfig::edgePathValidity(gridSize));
        return *this;
    }
    
    /// Builder: add Boundary constraint (auto-detect mode)
    ConstraintConfig& addBoundary(float margin = 50.0f) {
        constraints.push_back(SingleConstraintConfig::boundary(margin));
        return *this;
    }
    
    /// Builder: add Boundary constraint (explicit bounds)
    ConstraintConfig& addBoundaryExplicit(float minX, float minY, float maxX, float maxY, float margin = 0.0f) {
        constraints.push_back(SingleConstraintConfig::boundaryExplicit(minX, minY, maxX, maxY, margin));
        return *this;
    }

    /// Remove constraint by type
    ConstraintConfig& remove(const std::string& type);
    
    /// Check if constraint type is present
    bool has(const std::string& type) const;
    
    /// Get constraint config by type (nullptr if not found)
    const SingleConstraintConfig* get(const std::string& type) const;
};

/// Factory for creating IConstraintValidator from configuration
class ConstraintFactory {
public:
    /// Create a constraint validator from configuration
    /// @param config The constraint configuration
    /// @return Configured IConstraintValidator (ConstraintManager implementation)
    static std::unique_ptr<IConstraintValidator> create(const ConstraintConfig& config);
    
    /// Register a custom constraint creator
    /// @param type The constraint type name
    /// @param creator Function that creates the constraint from config
    using ConstraintCreator = std::function<std::unique_ptr<class IDragConstraint>(const SingleConstraintConfig&)>;
    static void registerType(const std::string& type, ConstraintCreator creator);
    
    /// Check if a constraint type is registered
    static bool isTypeRegistered(const std::string& type);
    
    /// Get list of registered constraint types
    static std::vector<std::string> registeredTypes();
};

/// JSON serialization for ConstraintConfig
///
/// JSON format:
/// @code
/// {
///   "constraints": [
///     {"type": "MinDistance", "enabled": true, "minGridDistance": 5.0}
///   ]
/// }
/// @endcode
class ConstraintConfigSerializer {
public:
    /// Serialize ConstraintConfig to JSON string
    static std::string toJson(const ConstraintConfig& config);
    
    /// Deserialize ConstraintConfig from JSON string
    static ConstraintConfig fromJson(const std::string& json);
};

}  // namespace arborvia
