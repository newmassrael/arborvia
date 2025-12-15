#pragma once

#include "SCXMLConverter.h"
#include "SCXMLGraph.h"

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace arborvia::test::scxml {

/// Information about a W3C SCXML test case
struct TestInfo {
    std::string id;           ///< Test ID (e.g., "144")
    std::string file;         ///< Main SCXML file name
    std::string description;  ///< Test description from metadata
    std::filesystem::path fullPath;  ///< Full path to SCXML file
};

/// Loader for W3C SCXML test suite
class SCXMLTestLoader {
public:
    /// Construct with base path to resources/scxml directory
    explicit SCXMLTestLoader(const std::filesystem::path& basePath);
    
    /// Load test index from index.json
    bool loadIndex();
    
    /// Get all available tests
    const std::vector<TestInfo>& getTests() const { return tests_; }
    
    /// Get test count
    size_t getTestCount() const { return tests_.size(); }
    
    /// Get test by index (0-based)
    const TestInfo* getTest(size_t index) const;
    
    /// Get test by ID (e.g., "144")
    const TestInfo* getTestById(const std::string& id) const;
    
    /// Find test index by ID
    std::optional<size_t> findTestIndex(const std::string& id) const;
    
    /// Load SCXML graph for a test
    std::unique_ptr<SCXMLGraph> loadGraph(const TestInfo& test, 
                                           const ConvertOptions& opts = {});
    
    /// Load SCXML graph by index
    std::unique_ptr<SCXMLGraph> loadGraph(size_t index,
                                           const ConvertOptions& opts = {});
    
    /// Load SCXML graph by ID
    std::unique_ptr<SCXMLGraph> loadGraphById(const std::string& id,
                                               const ConvertOptions& opts = {});
    
    /// Get last error message
    const std::string& getLastError() const { return lastError_; }

private:
    std::filesystem::path basePath_;
    std::vector<TestInfo> tests_;
    SCXMLConverter converter_;
    std::string lastError_;
};

}  // namespace arborvia::test::scxml
