#pragma once

#include "SCXMLGraph.h"
#include <arborvia/layout/config/LayoutEnums.h>

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
    
    /// Grid size for node sizing (all sizes are multiples of this)
    float gridSize = 20.0f;
    
    /// Default size for state nodes (in grid units)
    int stateWidthGrids = 6;   // 6 * 20 = 120
    int stateHeightGrids = 3;  // 3 * 20 = 60
    
    /// Node type for final states (Point or Regular)
    NodeType finalNodeType = NodeType::Regular;
    
    /// Size for final states in grid units (when finalNodeType is Regular)
    int finalWidthGrids = 6;   // 6 * 20 = 120
    int finalHeightGrids = 3;  // 3 * 20 = 60
    
    // Helper methods to get actual pixel sizes
    float stateWidth() const { return stateWidthGrids * gridSize; }
    float stateHeight() const { return stateHeightGrids * gridSize; }
    float finalWidth() const { return finalWidthGrids * gridSize; }
    float finalHeight() const { return finalHeightGrids * gridSize; }
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
