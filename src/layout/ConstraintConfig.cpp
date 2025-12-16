#include "arborvia/layout/config/ConstraintConfig.h"
#include "arborvia/common/Logger.h"
#include "layout/interactive/ConstraintManager.h"
#include "layout/constraints/DirectionAwareMarginConstraint.h"
#include "layout/constraints/NoOverlapConstraint.h"
#include "layout/constraints/EdgePathValidityConstraint.h"
#include "layout/constraints/BoundaryConstraint.h"
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
            // DirectionAwareMargin: considers edge connections for margin calculation
            ConstraintFactory::registerType("DirectionAwareMargin",
                [](const SingleConstraintConfig& config) -> std::unique_ptr<IDragConstraint> {
                    float distance = config.gridDistance.value_or(constants::MIN_NODE_GRID_DISTANCE);
                    return std::make_unique<DirectionAwareMarginConstraint>(distance);
                });
            
            // NoOverlap: simple AABB collision check
            ConstraintFactory::registerType("NoOverlap",
                [](const SingleConstraintConfig& config) -> std::unique_ptr<IDragConstraint> {
                    float margin = config.margin.value_or(0.0f);
                    return std::make_unique<NoOverlapConstraint>(margin);
                });
            
            // EdgePathValidity: A* pathfinding validation
            ConstraintFactory::registerType("EdgePathValidity",
                [](const SingleConstraintConfig& config) -> std::unique_ptr<IDragConstraint> {
                    float gridSize = config.gridSize.value_or(10.0f);
                    return std::make_unique<EdgePathValidityConstraint>(gridSize);
                });
            
            // Boundary: ensures nodes stay within bounds (Medium tier)
            ConstraintFactory::registerType("Boundary",
                [](const SingleConstraintConfig& config) -> std::unique_ptr<IDragConstraint> {
                    float margin = config.margin.value_or(50.0f);
                    return std::make_unique<BoundaryConstraint>(margin);
                });
            
            // BoundaryExplicit: explicit bounds version
            ConstraintFactory::registerType("BoundaryExplicit",
                [](const SingleConstraintConfig& config) -> std::unique_ptr<IDragConstraint> {
                    float minX = config.minX.value_or(0.0f);
                    float minY = config.minY.value_or(0.0f);
                    float maxX = config.maxX.value_or(1000.0f);
                    float maxY = config.maxY.value_or(1000.0f);
                    float margin = config.margin.value_or(0.0f);
                    return std::make_unique<BoundaryConstraint>(minX, minY, maxX, maxY, margin);
                });
        }
    };
    
    // Ensure built-in types are registered on startup
    static BuiltinInitializer s_builtinInit;
}

ConstraintConfig ConstraintConfig::createDefault(const LayoutOptions* options) {
    ConstraintConfig config;
    config.addDirectionAwareMargin(constants::MIN_NODE_GRID_DISTANCE);

    // ROOT CAUSE ANALYSIS LOG: Compare constraint margin with layout spacing
    // DirectionAwareMargin requires: MIN_NODE_GRID_DISTANCE * gridSize = 5.0 * gridSize
    // CoordinateAssignment uses: minLayerSpacing = gridSize * 2.0
    // If 5.0 * gridSize > gridSize * 2.0 (always true when MIN_NODE_GRID_DISTANCE > 2),
    // then initial layout WILL violate constraints!
    if (options) {
        float gridSize = options->gridConfig.cellSize;
        float constraintMargin = constants::MIN_NODE_GRID_DISTANCE * gridSize;
        float layoutSpacing = gridSize * 2.0f;  // CoordinateAssignment's minLayerSpacing
        LOG_DEBUG("[ConstraintConfig::createDefault] ROOT CAUSE ANALYSIS:");
        LOG_DEBUG("[ConstraintConfig::createDefault]   MIN_NODE_GRID_DISTANCE={} gridSize={}",
                  constants::MIN_NODE_GRID_DISTANCE, gridSize);
        LOG_DEBUG("[ConstraintConfig::createDefault]   constraintMargin (MIN_NODE_GRID_DISTANCE * gridSize) = {} px",
                  constraintMargin);
        LOG_DEBUG("[ConstraintConfig::createDefault]   layoutSpacing (CoordinateAssignment minLayerSpacing) = {} px",
                  layoutSpacing);
        if (constraintMargin > layoutSpacing) {
            LOG_DEBUG("[ConstraintConfig::createDefault]   WARNING: constraintMargin ({}) > layoutSpacing ({})!",
                      constraintMargin, layoutSpacing);
            LOG_DEBUG("[ConstraintConfig::createDefault]   Initial layout WILL violate constraints by {} px",
                      constraintMargin - layoutSpacing);
            LOG_DEBUG("[ConstraintConfig::createDefault]   FIX: Either increase nodeSpacingVertical >= {} or decrease MIN_NODE_GRID_DISTANCE <= 2",
                      constraintMargin);
        } else {
            LOG_DEBUG("[ConstraintConfig::createDefault]   OK: constraintMargin ({}) <= layoutSpacing ({})",
                      constraintMargin, layoutSpacing);
        }
    }

    // EdgePathValidity: A* path validation during drag
    // Use DragAlgorithmTraits (Single Source of Truth) to determine if validation is needed
    bool needsPathValidation = options &&
        DragAlgorithmTraits::requiresPathValidationDuringDrag(options->optimizationOptions.dragAlgorithm);

    const char* algoName = "Unknown";
    if (options) {
        switch (options->optimizationOptions.dragAlgorithm) {
            case DragAlgorithm::None: algoName = "None"; break;
            case DragAlgorithm::Geometric: algoName = "Geometric"; break;
            case DragAlgorithm::AStar: algoName = "AStar"; break;
            case DragAlgorithm::HideUntilDrop: algoName = "HideUntilDrop"; break;
        }
    }
    LOG_DEBUG("[ConstraintConfig::createDefault] options={} dragAlgorithm={}({}) needsPathValidation={}",
              options ? "provided" : "null",
              options ? static_cast<int>(options->optimizationOptions.dragAlgorithm) : -1,
              algoName,
              needsPathValidation);
    LOG_DEBUG("[ConstraintConfig::createDefault] NOTE: EdgePathValidity only added for AStar mode. "
              "In Geometric/HideUntilDrop mode, edge constraints (penetration, overlap) are NOT validated during drag.");

    if (needsPathValidation) {
        LOG_DEBUG("[ConstraintConfig::createDefault] ADDING EdgePathValidityConstraint (A* validation)");
        config.addEdgePathValidity(10.0f);
    } else {
        LOG_DEBUG("[ConstraintConfig::createDefault] SKIPPING EdgePathValidityConstraint");
    }

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

std::unique_ptr<IConstraintValidator> ConstraintFactory::create(const ConstraintConfig& config) {
    LOG_DEBUG("[ConstraintFactory::create] Creating manager from config with {} constraints", config.constraints.size());
    auto manager = std::make_unique<ConstraintManager>();

    for (const auto& constraintConfig : config.constraints) {
        LOG_DEBUG("[ConstraintFactory::create] Processing constraint: type={} enabled={}", constraintConfig.type, constraintConfig.enabled);
        if (!constraintConfig.enabled) continue;

        auto& registry = getRegistry();
        auto it = registry.find(constraintConfig.type);
        if (it != registry.end()) {
            auto constraint = it->second(constraintConfig);
            if (constraint) {
                LOG_DEBUG("[ConstraintFactory::create] Created constraint: {} tier={}", constraint->name(), static_cast<int>(constraint->tier()));
                manager->addConstraint(std::move(constraint));
            }
        } else {
            LOG_DEBUG("[ConstraintFactory::create] Constraint type not found in registry: {}", constraintConfig.type);
        }
    }

    LOG_DEBUG("[ConstraintFactory::create] Manager created with {} total constraints", manager->constraintCount());
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
        if (constraint.gridDistance.has_value()) {
            cj["gridDistance"] = constraint.gridDistance.value();
        }
        if (constraint.margin.has_value()) {
            cj["margin"] = constraint.margin.value();
        }
        if (constraint.gridSize.has_value()) {
            cj["gridSize"] = constraint.gridSize.value();
        }
        if (constraint.minX.has_value()) {
            cj["minX"] = constraint.minX.value();
        }
        if (constraint.minY.has_value()) {
            cj["minY"] = constraint.minY.value();
        }
        if (constraint.maxX.has_value()) {
            cj["maxX"] = constraint.maxX.value();
        }
        if (constraint.maxY.has_value()) {
            cj["maxY"] = constraint.maxY.value();
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
                if (cj.contains("gridDistance") && cj["gridDistance"].is_number()) {
                    constraint.gridDistance = cj["gridDistance"].get<float>();
                }
                if (cj.contains("margin") && cj["margin"].is_number()) {
                    constraint.margin = cj["margin"].get<float>();
                }
                if (cj.contains("gridSize") && cj["gridSize"].is_number()) {
                    constraint.gridSize = cj["gridSize"].get<float>();
                }
                if (cj.contains("minX") && cj["minX"].is_number()) {
                    constraint.minX = cj["minX"].get<float>();
                }
                if (cj.contains("minY") && cj["minY"].is_number()) {
                    constraint.minY = cj["minY"].get<float>();
                }
                if (cj.contains("maxX") && cj["maxX"].is_number()) {
                    constraint.maxX = cj["maxX"].get<float>();
                }
                if (cj.contains("maxY") && cj["maxY"].is_number()) {
                    constraint.maxY = cj["maxY"].get<float>();
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
