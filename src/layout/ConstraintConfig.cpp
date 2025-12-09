#include "arborvia/layout/config/ConstraintConfig.h"
#include "layout/interactive/ConstraintManager.h"
#include "layout/constraints/MinDistanceConstraint.h"
#include "arborvia/core/GeometryUtils.h"

#include <nlohmann/json.hpp>
#include <algorithm>
#include <unordered_map>

using json = nlohmann::json;

namespace arborvia {

// Static registry for custom constraint types
namespace {
    std::unordered_map<std::string, ConstraintFactory::ConstraintCreator>& getRegistry() {
        static std::unordered_map<std::string, ConstraintFactory::ConstraintCreator> registry;
        return registry;
    }
    
    // Initialize built-in types
    struct BuiltinInitializer {
        BuiltinInitializer() {
            // MinDistance constraint (delegates to ValidRegionCalculator internally)
            ConstraintFactory::registerType("MinDistance",
                [](const SingleConstraintConfig& config) -> std::unique_ptr<IDragConstraint> {
                    float distance = config.minGridDistance.value_or(constants::MIN_NODE_GRID_DISTANCE);
                    return std::make_unique<MinDistanceConstraint>(distance);
                });
        }
    };
    
    // Ensure built-in types are registered on startup
    static BuiltinInitializer s_builtinInit;
}

ConstraintConfig ConstraintConfig::createDefault() {
    ConstraintConfig config;
    // MinDistanceConstraint delegates to ValidRegionCalculator internally
    // which provides direction-aware margin calculation
    config.addMinDistance(constants::MIN_NODE_GRID_DISTANCE);
    return config;
}

ConstraintConfig& ConstraintConfig::remove(const std::string& type) {
    constraints.erase(
        std::remove_if(constraints.begin(), constraints.end(),
            [&type](const SingleConstraintConfig& c) { return c.type == type; }),
        constraints.end());
    return *this;
}

bool ConstraintConfig::has(const std::string& type) const {
    return std::any_of(constraints.begin(), constraints.end(),
        [&type](const SingleConstraintConfig& c) { return c.type == type && c.enabled; });
}

const SingleConstraintConfig* ConstraintConfig::get(const std::string& type) const {
    auto it = std::find_if(constraints.begin(), constraints.end(),
        [&type](const SingleConstraintConfig& c) { return c.type == type; });
    return it != constraints.end() ? &(*it) : nullptr;
}

std::unique_ptr<ConstraintManager> ConstraintFactory::create(const ConstraintConfig& config) {
    auto manager = std::make_unique<ConstraintManager>();
    
    for (const auto& constraintConfig : config.constraints) {
        if (!constraintConfig.enabled) continue;
        
        auto& registry = getRegistry();
        auto it = registry.find(constraintConfig.type);
        if (it != registry.end()) {
            auto constraint = it->second(constraintConfig);
            if (constraint) {
                manager->addConstraint(std::move(constraint));
            }
        }
    }
    
    return manager;
}

void ConstraintFactory::registerType(const std::string& type, ConstraintCreator creator) {
    getRegistry()[type] = std::move(creator);
}

bool ConstraintFactory::isTypeRegistered(const std::string& type) {
    return getRegistry().find(type) != getRegistry().end();
}

std::vector<std::string> ConstraintFactory::registeredTypes() {
    std::vector<std::string> types;
    for (const auto& [type, _] : getRegistry()) {
        types.push_back(type);
    }
    return types;
}

// JSON Serialization

std::string ConstraintConfigSerializer::toJson(const ConstraintConfig& config) {
    json j;
    j["constraints"] = json::array();
    
    for (const auto& constraint : config.constraints) {
        json cj;
        cj["type"] = constraint.type;
        cj["enabled"] = constraint.enabled;
        
        // Type-specific parameters
        if (constraint.minGridDistance.has_value()) {
            cj["minGridDistance"] = constraint.minGridDistance.value();
        }
        
        j["constraints"].push_back(cj);
    }
    
    return j.dump(2);
}

ConstraintConfig ConstraintConfigSerializer::fromJson(const std::string& jsonStr) {
    ConstraintConfig config;
    
    try {
        json j = json::parse(jsonStr);
        
        if (j.contains("constraints") && j["constraints"].is_array()) {
            for (const auto& cj : j["constraints"]) {
                SingleConstraintConfig constraint;
                
                if (cj.contains("type") && cj["type"].is_string()) {
                    constraint.type = cj["type"].get<std::string>();
                }
                
                if (cj.contains("enabled") && cj["enabled"].is_boolean()) {
                    constraint.enabled = cj["enabled"].get<bool>();
                }
                
                // Type-specific parameters
                if (cj.contains("minGridDistance") && cj["minGridDistance"].is_number()) {
                    constraint.minGridDistance = cj["minGridDistance"].get<float>();
                }
                
                config.constraints.push_back(constraint);
            }
        }
    } catch (const json::exception&) {
        // Return empty config on parse error
        return {};
    }
    
    return config;
}

}  // namespace arborvia
