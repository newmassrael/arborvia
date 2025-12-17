#include "SCXMLManager.h"
#include <iostream>

namespace arborvia {

using test::scxml::SCXMLTestLoader;
using test::scxml::SCXMLGraph;
using test::scxml::SCXMLNodeType;
using test::scxml::TestInfo;

SCXMLManager::SCXMLManager(DemoState::SCXMLState& scxmlState)
    : scxmlState_(scxmlState) {
}

bool SCXMLManager::init(const std::string& basePath) {
    loader_ = std::make_unique<SCXMLTestLoader>(basePath);
    if (!loader_->loadIndex()) {
        std::cerr << "Failed to load SCXML test index from " << basePath << std::endl;
        loader_.reset();
        return false;
    }
    std::cout << "Loaded " << loader_->getTestCount() << " SCXML tests from " << basePath << std::endl;
    return true;
}

bool SCXMLManager::loadTest(size_t index) {
    if (!loader_ || index >= loader_->getTestCount()) {
        return false;
    }

    auto newGraph = loader_->loadGraph(index);
    if (!newGraph) {
        std::cerr << "Failed to load SCXML test " << index << std::endl;
        return false;
    }

    const auto* testInfo = loader_->getTest(index);
    std::cout << "Loading SCXML test " << testInfo->id << ": " << testInfo->file << std::endl;

    // Update state
    graph_ = std::move(newGraph);
    scxmlState_.currentTestIndex = static_cast<int>(index);
    scxmlState_.modeActive = true;

    // Notify listener to update graph and layout
    if (onTestLoadedCallback_) {
        onTestLoadedCallback_(*graph_);
    }

    return true;
}

void SCXMLManager::nextTest() {
    if (!loader_ || loader_->getTestCount() == 0) return;
    size_t next = (scxmlState_.currentTestIndex < 0) ? 0 :
                  static_cast<size_t>((scxmlState_.currentTestIndex + 1) % static_cast<int>(loader_->getTestCount()));
    loadTest(next);
}

void SCXMLManager::prevTest() {
    if (!loader_ || loader_->getTestCount() == 0) return;
    size_t prev = (scxmlState_.currentTestIndex <= 0) ?
                  loader_->getTestCount() - 1 :
                  static_cast<size_t>(scxmlState_.currentTestIndex - 1);
    loadTest(prev);
}

void SCXMLManager::exitMode() {
    scxmlState_.modeActive = false;
    graph_.reset();
    scxmlState_.currentTestIndex = -1;

    // Notify listener to restore default graph
    if (onModeExitCallback_) {
        onModeExitCallback_();
    }
}

bool SCXMLManager::isModeActive() const {
    return scxmlState_.modeActive;
}

size_t SCXMLManager::getTestCount() const {
    return loader_ ? loader_->getTestCount() : 0;
}

const TestInfo* SCXMLManager::getCurrentTestInfo() const {
    if (!loader_ || scxmlState_.currentTestIndex < 0) return nullptr;
    return loader_->getTest(static_cast<size_t>(scxmlState_.currentTestIndex));
}

SCXMLTestLoader* SCXMLManager::getLoader() {
    return loader_.get();
}

SCXMLGraph* SCXMLManager::getCurrentGraph() {
    return graph_.get();
}

bool SCXMLManager::canConvertToPoint(NodeId nodeId) const {
    if (!graph_) {
        return false;  // Non-SCXML graphs: no point conversion allowed
    }

    auto nodeType = graph_->getNodeType(nodeId);
    // Only Final can toggle between Point and Regular
    return nodeType == SCXMLNodeType::Final;
}

bool SCXMLManager::canConvertToRegular(NodeId nodeId) const {
    if (!graph_) {
        return true;  // Non-SCXML graphs: allow conversion
    }

    auto nodeType = graph_->getNodeType(nodeId);
    // Only Final can toggle between Point and Regular
    // Initial/History must remain as Point
    return nodeType == SCXMLNodeType::Final;
}

std::string SCXMLManager::getScxmlId(NodeId nodeId) const {
    if (!graph_) {
        return "";
    }
    return graph_->getScxmlId(nodeId);
}

}  // namespace arborvia
