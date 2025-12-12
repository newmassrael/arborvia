#include "arborvia/layout/config/EdgePenaltyConfig.h"
#include "arborvia/layout/api/EdgePenaltySystem.h"
#include "arborvia/layout/constraints/ConstraintPenaltyAdapter.h"
#include "arborvia/layout/constraints/builtins/OrthogonalityConstraint.h"
#include "arborvia/layout/constraints/builtins/DirectionalPenetrationConstraint.h"
#include "optimization/BuiltinPenalties.h"

#include <nlohmann/json.hpp>
#include <algorithm>
#include <unordered_map>

using json = nlohmann::json;

namespace arborvia {

// Static registry for custom penalty types
namespace {
    std::unordered_map<std::string, EdgePenaltyFactory::PenaltyCreator>& getPenaltyRegistry() {
        static std::unordered_map<std::string, EdgePenaltyFactory::PenaltyCreator> registry;
        return registry;
    }
    
    // Initialize built-in penalty types
    struct BuiltinPenaltyInitializer {
        BuiltinPenaltyInitializer() {
            EdgePenaltyFactory::registerType("SegmentOverlap",
                [](const SinglePenaltyConfig&) -> std::unique_ptr<IEdgePenalty> {
                    return std::make_unique<SegmentOverlapPenalty>();
                });
            
            EdgePenaltyFactory::registerType("Direction",
                [](const SinglePenaltyConfig&) -> std::unique_ptr<IEdgePenalty> {
                    return std::make_unique<DirectionPenalty>();
                });
            
            EdgePenaltyFactory::registerType("NodeCollision",
                [](const SinglePenaltyConfig&) -> std::unique_ptr<IEdgePenalty> {
                    return std::make_unique<NodeCollisionPenalty>();
                });
            
            EdgePenaltyFactory::registerType("TooCloseSnap",
                [](const SinglePenaltyConfig& config) -> std::unique_ptr<IEdgePenalty> {
                    float minDist = config.minDistance.value_or(60.0f);
                    return std::make_unique<TooCloseSnapPenalty>(minDist);
                });
            
            EdgePenaltyFactory::registerType("SnapPointOverlap",
                [](const SinglePenaltyConfig& config) -> std::unique_ptr<IEdgePenalty> {
                    float tol = config.tolerance.value_or(1.0f);
                    return std::make_unique<SnapPointOverlapPenalty>(tol);
                });
            
            EdgePenaltyFactory::registerType("SelfOverlap",
                [](const SinglePenaltyConfig& config) -> std::unique_ptr<IEdgePenalty> {
                    float minSegLen = config.minDistance.value_or(20.0f);
                    return std::make_unique<SelfOverlapPenalty>(minSegLen);
                });
            
            EdgePenaltyFactory::registerType("ForbiddenZone",
                [](const SinglePenaltyConfig&) -> std::unique_ptr<IEdgePenalty> {
                    return std::make_unique<ForbiddenZonePenalty>();
                });
            
            EdgePenaltyFactory::registerType("PathIntersection",
                [](const SinglePenaltyConfig& config) -> std::unique_ptr<IEdgePenalty> {
                    int weight = config.weight.value_or(1000);
                    return std::make_unique<PathIntersectionPenalty>(weight);
                });
            
            EdgePenaltyFactory::registerType("Orthogonality",
                [](const SinglePenaltyConfig&) -> std::unique_ptr<IEdgePenalty> {
                    return std::make_unique<ConstraintPenaltyAdapter>(
                        std::make_shared<OrthogonalityConstraint>());
                });
            
            EdgePenaltyFactory::registerType("DirectionalSourcePenetration",
                [](const SinglePenaltyConfig&) -> std::unique_ptr<IEdgePenalty> {
                    return std::make_unique<ConstraintPenaltyAdapter>(
                        std::make_shared<DirectionalSourcePenetrationConstraint>());
                });
            
            EdgePenaltyFactory::registerType("DirectionalTargetPenetration",
                [](const SinglePenaltyConfig&) -> std::unique_ptr<IEdgePenalty> {
                    return std::make_unique<ConstraintPenaltyAdapter>(
                        std::make_shared<DirectionalTargetPenetrationConstraint>());
                });
            
            EdgePenaltyFactory::registerType("FixedEndpoint",
                [](const SinglePenaltyConfig&) -> std::unique_ptr<IEdgePenalty> {
                    return std::make_unique<FixedEndpointPenalty>();
                });
        }
    };
    
    static BuiltinPenaltyInitializer s_builtinPenaltyInit;
}

// ============================================================================
// EdgePenaltyConfig implementation
// ============================================================================

EdgePenaltyConfig EdgePenaltyConfig::createDefault() {
    EdgePenaltyConfig config;
    config.addSegmentOverlap()
          .addDirection()
          .addNodeCollision()
          .addTooCloseSnap()
          .addSnapPointOverlap()
          .addSelfOverlap()
          .addForbiddenZone()
          .addPathIntersection()
          .addOrthogonality()
          .addFixedEndpoint();
    return config;
}

EdgePenaltyConfig EdgePenaltyConfig::createMinimal() {
    EdgePenaltyConfig config;
    config.addNodeCollision()
          .addOrthogonality()
          .addSegmentOverlap();
    return config;
}

EdgePenaltyConfig EdgePenaltyConfig::createStrict() {
    EdgePenaltyConfig config;
    config.addSegmentOverlap()
          .addDirection()
          .addNodeCollision()
          .addTooCloseSnap()
          .addSnapPointOverlap()
          .addSelfOverlap()
          .addForbiddenZone()
          .addPathIntersection(5000)  // Higher weight for stricter enforcement
          .addOrthogonality()
          .addFixedEndpoint();
    return config;
}

EdgePenaltyConfig& EdgePenaltyConfig::remove(const std::string& type) {
    penalties.erase(
        std::remove_if(penalties.begin(), penalties.end(),
            [&type](const SinglePenaltyConfig& c) { return c.type == type; }),
        penalties.end());
    return *this;
}

bool EdgePenaltyConfig::has(const std::string& type) const {
    return std::any_of(penalties.begin(), penalties.end(),
        [&type](const SinglePenaltyConfig& c) { return c.type == type && c.enabled; });
}

const SinglePenaltyConfig* EdgePenaltyConfig::get(const std::string& type) const {
    auto it = std::find_if(penalties.begin(), penalties.end(),
        [&type](const SinglePenaltyConfig& c) { return c.type == type; });
    return it != penalties.end() ? &(*it) : nullptr;
}

// ============================================================================
// EdgePenaltyFactory implementation
// ============================================================================

std::shared_ptr<EdgePenaltySystem> EdgePenaltyFactory::create(const EdgePenaltyConfig& config) {
    auto system = std::make_shared<EdgePenaltySystem>();
    
    for (const auto& penaltyConfig : config.penalties) {
        if (!penaltyConfig.enabled) continue;
        
        auto& registry = getPenaltyRegistry();
        auto it = registry.find(penaltyConfig.type);
        if (it != registry.end()) {
            auto penalty = it->second(penaltyConfig);
            if (penalty) {
                system->addPenalty(std::move(penalty));
            }
        }
    }
    
    return system;
}

void EdgePenaltyFactory::registerType(const std::string& type, PenaltyCreator creator) {
    getPenaltyRegistry()[type] = std::move(creator);
}

bool EdgePenaltyFactory::isTypeRegistered(const std::string& type) {
    return getPenaltyRegistry().find(type) != getPenaltyRegistry().end();
}

std::vector<std::string> EdgePenaltyFactory::registeredTypes() {
    std::vector<std::string> types;
    for (const auto& [type, _] : getPenaltyRegistry()) {
        types.push_back(type);
    }
    return types;
}

// ============================================================================
// JSON Serialization
// ============================================================================

std::string EdgePenaltyConfigSerializer::toJson(const EdgePenaltyConfig& config) {
    json j;
    j["penalties"] = json::array();
    
    for (const auto& penalty : config.penalties) {
        json pj;
        pj["type"] = penalty.type;
        pj["enabled"] = penalty.enabled;
        
        if (penalty.weight.has_value()) {
            pj["weight"] = penalty.weight.value();
        }
        if (penalty.minDistance.has_value()) {
            pj["minDistance"] = penalty.minDistance.value();
        }
        if (penalty.tolerance.has_value()) {
            pj["tolerance"] = penalty.tolerance.value();
        }
        
        j["penalties"].push_back(pj);
    }
    
    return j.dump(2);
}

EdgePenaltyConfig EdgePenaltyConfigSerializer::fromJson(const std::string& jsonStr) {
    EdgePenaltyConfig config;
    
    try {
        json j = json::parse(jsonStr);
        
        if (j.contains("penalties") && j["penalties"].is_array()) {
            for (const auto& pj : j["penalties"]) {
                SinglePenaltyConfig penalty;
                
                if (pj.contains("type") && pj["type"].is_string()) {
                    penalty.type = pj["type"].get<std::string>();
                }
                
                if (pj.contains("enabled") && pj["enabled"].is_boolean()) {
                    penalty.enabled = pj["enabled"].get<bool>();
                }
                
                if (pj.contains("weight") && pj["weight"].is_number()) {
                    penalty.weight = pj["weight"].get<int>();
                }
                if (pj.contains("minDistance") && pj["minDistance"].is_number()) {
                    penalty.minDistance = pj["minDistance"].get<float>();
                }
                if (pj.contains("tolerance") && pj["tolerance"].is_number()) {
                    penalty.tolerance = pj["tolerance"].get<float>();
                }
                
                config.penalties.push_back(penalty);
            }
        }
    } catch (const json::exception&) {
        return {};
    }
    
    return config;
}

}  // namespace arborvia
