#include "arborvia/layout/OptimizerRegistry.h"
#include "arborvia/layout/IEdgeOptimizer.h"
#include "sugiyama/AStarOptimizerFactory.h"
#include "sugiyama/GeometricOptimizerFactory.h"

#include <stdexcept>

namespace arborvia {

OptimizerRegistry& OptimizerRegistry::instance() {
    static OptimizerRegistry instance;
    if (!instance.builtinsRegistered_) {
        instance.registerBuiltinFactories();
        instance.builtinsRegistered_ = true;
    }
    return instance;
}

OptimizerRegistry::OptimizerRegistry() = default;

void OptimizerRegistry::registerBuiltinFactories() {
    // Register builtin optimizer factories
    registerFactory(std::make_unique<AStarOptimizerFactory>());
    registerFactory(std::make_unique<GeometricOptimizerFactory>());
}

void OptimizerRegistry::registerFactory(std::unique_ptr<IEdgeOptimizerFactory> factory) {
    if (!factory) {
        throw std::runtime_error("Cannot register null factory");
    }

    const std::string name = factory->name();
    if (factories_.find(name) != factories_.end()) {
        throw std::runtime_error("Factory with name '" + name + "' already registered");
    }

    factories_[name] = std::move(factory);
}

bool OptimizerRegistry::unregisterFactory(const std::string& name) {
    return factories_.erase(name) > 0;
}

std::unique_ptr<IEdgeOptimizer> OptimizerRegistry::create(
    const std::string& name,
    const OptimizerConfig& config) {

    auto it = factories_.find(name);
    if (it == factories_.end()) {
        return nullptr;
    }
    return it->second->create(config);
}

bool OptimizerRegistry::hasFactory(const std::string& name) const {
    return factories_.find(name) != factories_.end();
}

std::vector<std::string> OptimizerRegistry::availableOptimizers() const {
    std::vector<std::string> names;
    names.reserve(factories_.size());
    for (const auto& [name, _] : factories_) {
        names.push_back(name);
    }
    return names;
}

const IEdgeOptimizerFactory* OptimizerRegistry::getFactory(const std::string& name) const {
    auto it = factories_.find(name);
    if (it == factories_.end()) {
        return nullptr;
    }
    return it->second.get();
}

}  // namespace arborvia
