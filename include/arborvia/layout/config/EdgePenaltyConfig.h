#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace arborvia {

// Forward declarations
class EdgePenaltySystem;
class IEdgePenalty;

/// Configuration for a single edge penalty
struct SinglePenaltyConfig {
    std::string type;                    ///< Penalty type name
    bool enabled = true;                 ///< Whether this penalty is active
    
    // Type-specific parameters
    std::optional<int> weight;           ///< Custom weight (overrides default)
    std::optional<float> minDistance;    ///< For distance-based penalties
    std::optional<float> tolerance;      ///< For tolerance-based penalties
    
    /// Create SegmentOverlap penalty config
    static SinglePenaltyConfig segmentOverlap() {
        SinglePenaltyConfig config;
        config.type = "SegmentOverlap";
        return config;
    }
    
    /// Create Direction penalty config
    static SinglePenaltyConfig direction() {
        SinglePenaltyConfig config;
        config.type = "Direction";
        return config;
    }
    
    /// Create NodeCollision penalty config
    static SinglePenaltyConfig nodeCollision() {
        SinglePenaltyConfig config;
        config.type = "NodeCollision";
        return config;
    }
    
    /// Create TooCloseSnap penalty config
    static SinglePenaltyConfig tooCloseSnap(float minDist = 60.0f) {
        SinglePenaltyConfig config;
        config.type = "TooCloseSnap";
        config.minDistance = minDist;
        return config;
    }
    
    /// Create SnapPointOverlap penalty config
    static SinglePenaltyConfig snapPointOverlap(float tol = 1.0f) {
        SinglePenaltyConfig config;
        config.type = "SnapPointOverlap";
        config.tolerance = tol;
        return config;
    }
    
    /// Create SelfOverlap penalty config
    static SinglePenaltyConfig selfOverlap(float minSegLen = 20.0f) {
        SinglePenaltyConfig config;
        config.type = "SelfOverlap";
        config.minDistance = minSegLen;
        return config;
    }
    
    /// Create ForbiddenZone penalty config
    static SinglePenaltyConfig forbiddenZone() {
        SinglePenaltyConfig config;
        config.type = "ForbiddenZone";
        return config;
    }
    
    /// Create PathIntersection penalty config
    static SinglePenaltyConfig pathIntersection(int wgt = 1000) {
        SinglePenaltyConfig config;
        config.type = "PathIntersection";
        config.weight = wgt;
        return config;
    }
    
    /// Create Orthogonality penalty config
    static SinglePenaltyConfig orthogonality() {
        SinglePenaltyConfig config;
        config.type = "Orthogonality";
        return config;
    }
    
    /// Create FixedEndpoint penalty config
    static SinglePenaltyConfig fixedEndpoint() {
        SinglePenaltyConfig config;
        config.type = "FixedEndpoint";
        return config;
    }
};

/// Configuration for edge penalty system
///
/// This struct provides a declarative way to configure which penalties
/// are active during edge routing optimization. It can be serialized to/from JSON.
///
/// Example usage:
/// @code
/// EdgePenaltyConfig config = EdgePenaltyConfig::createDefault();
/// auto system = EdgePenaltyFactory::create(config);
/// @endcode
struct EdgePenaltyConfig {
    std::vector<SinglePenaltyConfig> penalties;
    
    /// Create default configuration with all penalties
    static EdgePenaltyConfig createDefault();
    
    /// Create minimal configuration (only critical hard constraints)
    static EdgePenaltyConfig createMinimal();
    
    /// Create strict configuration (higher weights)
    static EdgePenaltyConfig createStrict();
    
    /// Create empty configuration (no penalties)
    static EdgePenaltyConfig createEmpty() { return {}; }
    
    /// Check if configuration has any penalties
    bool empty() const { return penalties.empty(); }
    
    // Builder pattern methods
    EdgePenaltyConfig& addSegmentOverlap() {
        penalties.push_back(SinglePenaltyConfig::segmentOverlap());
        return *this;
    }
    
    EdgePenaltyConfig& addDirection() {
        penalties.push_back(SinglePenaltyConfig::direction());
        return *this;
    }
    
    EdgePenaltyConfig& addNodeCollision() {
        penalties.push_back(SinglePenaltyConfig::nodeCollision());
        return *this;
    }
    
    EdgePenaltyConfig& addTooCloseSnap(float minDist = 60.0f) {
        penalties.push_back(SinglePenaltyConfig::tooCloseSnap(minDist));
        return *this;
    }
    
    EdgePenaltyConfig& addSnapPointOverlap(float tol = 1.0f) {
        penalties.push_back(SinglePenaltyConfig::snapPointOverlap(tol));
        return *this;
    }
    
    EdgePenaltyConfig& addSelfOverlap(float minSegLen = 20.0f) {
        penalties.push_back(SinglePenaltyConfig::selfOverlap(minSegLen));
        return *this;
    }
    
    EdgePenaltyConfig& addForbiddenZone() {
        penalties.push_back(SinglePenaltyConfig::forbiddenZone());
        return *this;
    }
    
    EdgePenaltyConfig& addPathIntersection(int wgt = 1000) {
        penalties.push_back(SinglePenaltyConfig::pathIntersection(wgt));
        return *this;
    }
    
    EdgePenaltyConfig& addOrthogonality() {
        penalties.push_back(SinglePenaltyConfig::orthogonality());
        return *this;
    }
    
    EdgePenaltyConfig& addFixedEndpoint() {
        penalties.push_back(SinglePenaltyConfig::fixedEndpoint());
        return *this;
    }
    
    /// Remove penalty by type
    EdgePenaltyConfig& remove(const std::string& type);
    
    /// Check if penalty type is present
    bool has(const std::string& type) const;
    
    /// Get penalty config by type (nullptr if not found)
    const SinglePenaltyConfig* get(const std::string& type) const;
};

/// Factory for creating EdgePenaltySystem from configuration
class EdgePenaltyFactory {
public:
    /// Create an EdgePenaltySystem from configuration
    /// @param config The penalty configuration
    /// @return Configured EdgePenaltySystem
    static std::shared_ptr<EdgePenaltySystem> create(const EdgePenaltyConfig& config);
    
    /// Register a custom penalty creator
    using PenaltyCreator = std::function<std::unique_ptr<IEdgePenalty>(const SinglePenaltyConfig&)>;
    static void registerType(const std::string& type, PenaltyCreator creator);
    
    /// Check if a penalty type is registered
    static bool isTypeRegistered(const std::string& type);
    
    /// Get list of registered penalty types
    static std::vector<std::string> registeredTypes();
};

/// JSON serialization for EdgePenaltyConfig
class EdgePenaltyConfigSerializer {
public:
    static std::string toJson(const EdgePenaltyConfig& config);
    static EdgePenaltyConfig fromJson(const std::string& json);
};

}  // namespace arborvia
