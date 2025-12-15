#pragma once

#include "SCXMLGraph.h"

#include <memory>
#include <optional>
#include <string>

namespace arborvia::test::scxml {

/// Options for SCXML to Graph conversion
struct ConvertOptions {
    /// Create visual initial pseudo-state nodes for 'initial' attributes
    bool createInitialPseudoStates = true;
    
    /// Parse onentry/onexit actions
    bool includeActions = false;
    
    /// Default size for state nodes
    float defaultStateWidth = 120.0f;
    float defaultStateHeight = 60.0f;
    
    /// Size for final states
    float finalStateSize = 40.0f;
    
    /// Size for initial pseudo-states
    float initialPseudoSize = 20.0f;
    
    /// Size for history states
    float historyStateSize = 30.0f;
};

/// Parse error information
struct ParseError {
    int line = 0;
    int column = 0;
    std::string message;
    
    std::string toString() const {
        return "Line " + std::to_string(line) + ", Col " + std::to_string(column) 
               + ": " + message;
    }
};

/// Converter from SCXML XML to SCXMLGraph
class SCXMLConverter {
public:
    SCXMLConverter() = default;
    ~SCXMLConverter() = default;
    
    // Non-copyable
    SCXMLConverter(const SCXMLConverter&) = delete;
    SCXMLConverter& operator=(const SCXMLConverter&) = delete;

    /// Convert SCXML from file path
    /// @param path Path to .scxml file
    /// @param opts Conversion options
    /// @return SCXMLGraph or nullptr on error
    std::unique_ptr<SCXMLGraph> fromFile(const std::string& path,
                                          const ConvertOptions& opts = {});
    
    /// Convert SCXML from string
    /// @param xml SCXML XML content
    /// @param opts Conversion options
    /// @return SCXMLGraph or nullptr on error
    std::unique_ptr<SCXMLGraph> fromString(const std::string& xml,
                                            const ConvertOptions& opts = {});
    
    /// Get last parse error (if fromFile/fromString returned nullptr)
    std::optional<ParseError> getLastError() const { return lastError_; }

private:
    std::optional<ParseError> lastError_;
    
    void setError(int line, int col, const std::string& msg);
    void clearError();
};

}  // namespace arborvia::test::scxml
