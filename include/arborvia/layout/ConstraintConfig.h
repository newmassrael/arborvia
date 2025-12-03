#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace arborvia {

// Forward declaration
class ConstraintManager;

/// Configuration for a single constraint
struct SingleConstraintConfig {
    std::string type;                    ///< Constraint type name (e.g., "MinDistance", "EdgeValidity")
    bool enabled = true;                 ///< Whether this constraint is active
    
    // Type-specific parameters
    std::optional<float> minGridDistance;  ///< For MinDistance constraint
    
    /// Create MinDistance constraint config
    static SingleConstraintConfig minDistance(float gridUnits = 5.0f) {
        SingleConstraintConfig config;
        config.type = "MinDistance";
        config.minGridDistance = gridUnits;
        return config;
    }
    
    /// Create EdgeValidity constraint config
    static SingleConstraintConfig edgeValidity() {
        SingleConstraintConfig config;
        config.type = "EdgeValidity";
        return config;
    }
};

/// Configuration for drag constraints
/// 
/// This struct provides a declarative way to configure which constraints
/// are active during drag operations. It can be serialized to/from JSON
/// and used to create a ConstraintManager.
///
/// Example usage:
/// @code
/// ConstraintConfig config;
/// config.constraints.push_back(SingleConstraintConfig::minDistance(3.0f));
/// config.constraints.push_back(SingleConstraintConfig::edgeValidity());
/// 
/// auto manager = ConstraintFactory::create(config);
/// @endcode
struct ConstraintConfig {
    std::vector<SingleConstraintConfig> constraints;
    
    /// Create default constraint configuration
    /// Includes MinDistance (5 grid units) and EdgeValidity
    static ConstraintConfig createDefault();
    
    /// Create empty configuration (no constraints)
    static ConstraintConfig createEmpty() { return {}; }
    
    /// Check if configuration has any constraints
    bool empty() const { return constraints.empty(); }
    
    /// Builder: add MinDistance constraint
    ConstraintConfig& addMinDistance(float gridUnits = 5.0f) {
        constraints.push_back(SingleConstraintConfig::minDistance(gridUnits));
        return *this;
    }
    
    /// Builder: add EdgeValidity constraint
    ConstraintConfig& addEdgeValidity() {
        constraints.push_back(SingleConstraintConfig::edgeValidity());
        return *this;
    }
    
    /// Remove constraint by type
    ConstraintConfig& remove(const std::string& type);
    
    /// Check if constraint type is present
    bool has(const std::string& type) const;
    
    /// Get constraint config by type (nullptr if not found)
    const SingleConstraintConfig* get(const std::string& type) const;
};

/// Factory for creating ConstraintManager from configuration
class ConstraintFactory {
public:
    /// Create a ConstraintManager from configuration
    /// @param config The constraint configuration
    /// @return Configured ConstraintManager
    static std::unique_ptr<ConstraintManager> create(const ConstraintConfig& config);
    
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
///     {"type": "MinDistance", "enabled": true, "minGridDistance": 5.0},
///     {"type": "EdgeValidity", "enabled": true}
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
