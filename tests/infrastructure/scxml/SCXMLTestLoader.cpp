#include "SCXMLTestLoader.h"

#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

namespace arborvia::test::scxml {

SCXMLTestLoader::SCXMLTestLoader(const std::filesystem::path& basePath)
    : basePath_(basePath) {}

bool SCXMLTestLoader::loadIndex() {
    tests_.clear();
    lastError_.clear();
    
    auto indexPath = basePath_ / "index.json";
    if (!std::filesystem::exists(indexPath)) {
        lastError_ = "Index file not found: " + indexPath.string();
        return false;
    }
    
    try {
        std::ifstream file(indexPath);
        if (!file.is_open()) {
            lastError_ = "Failed to open index file: " + indexPath.string();
            return false;
        }
        
        nlohmann::json index;
        file >> index;
        
        if (!index.contains("tests") || !index["tests"].is_array()) {
            lastError_ = "Invalid index format: missing 'tests' array";
            return false;
        }
        
        for (const auto& testJson : index["tests"]) {
            TestInfo info;
            info.id = testJson.value("id", "");
            info.file = testJson.value("file", "");
            info.description = testJson.value("description", "");
            info.fullPath = basePath_ / info.id / info.file;
            
            if (!info.id.empty() && !info.file.empty()) {
                tests_.push_back(std::move(info));
            }
        }
        
        return true;
    } catch (const std::exception& e) {
        lastError_ = std::string("Failed to parse index: ") + e.what();
        return false;
    }
}

const TestInfo* SCXMLTestLoader::getTest(size_t index) const {
    if (index >= tests_.size()) {
        return nullptr;
    }
    return &tests_[index];
}

const TestInfo* SCXMLTestLoader::getTestById(const std::string& id) const {
    for (const auto& test : tests_) {
        if (test.id == id) {
            return &test;
        }
    }
    return nullptr;
}

std::optional<size_t> SCXMLTestLoader::findTestIndex(const std::string& id) const {
    for (size_t i = 0; i < tests_.size(); ++i) {
        if (tests_[i].id == id) {
            return i;
        }
    }
    return std::nullopt;
}

std::unique_ptr<SCXMLGraph> SCXMLTestLoader::loadGraph(const TestInfo& test,
                                                        const ConvertOptions& opts) {
    lastError_.clear();
    
    if (!std::filesystem::exists(test.fullPath)) {
        lastError_ = "SCXML file not found: " + test.fullPath.string();
        return nullptr;
    }
    
    auto graph = converter_.fromFile(test.fullPath.string(), opts);
    if (!graph) {
        auto err = converter_.getLastError();
        lastError_ = err ? err->toString() : "Unknown parse error";
    }
    
    return graph;
}

std::unique_ptr<SCXMLGraph> SCXMLTestLoader::loadGraph(size_t index,
                                                        const ConvertOptions& opts) {
    auto* test = getTest(index);
    if (!test) {
        lastError_ = "Invalid test index: " + std::to_string(index);
        return nullptr;
    }
    return loadGraph(*test, opts);
}

std::unique_ptr<SCXMLGraph> SCXMLTestLoader::loadGraphById(const std::string& id,
                                                            const ConvertOptions& opts) {
    auto* test = getTestById(id);
    if (!test) {
        lastError_ = "Test not found: " + id;
        return nullptr;
    }
    return loadGraph(*test, opts);
}

}  // namespace arborvia::test::scxml
