#pragma once

#include "../config/LayoutResult.h"
#include <utility>

namespace arborvia {

// Forward declaration
class ConstraintGateway;

/// Wrapper type that guarantees EdgeLayout has passed constraint validation.
/// 
/// This type enforces compile-time validation by:
/// 1. Private constructor - only ConstraintGateway can create instances
/// 2. Required at exit points - LayoutResult::setEdgeLayout requires this type
/// 3. trustUnchecked is private - only friend classes can bypass validation
/// 
/// Usage pattern:
/// ```cpp
/// ConstraintGateway gateway;
/// auto validated = gateway.validateAndWrap(layout, ctx);
/// if (validated) {
///     result.setEdgeLayout(id, std::move(*validated));
/// }
/// ```
/// 
/// For internal trusted sources (e.g., deserialization), friend classes use:
/// ```cpp
/// auto validated = ValidatedEdgeLayout::trustUnchecked(std::move(layout));
/// ```
/// Helper class for test/demo code to create ValidatedEdgeLayout
/// This is NOT for production use - it exists to allow demos and tests
/// to work with the validated type system while bypassing actual validation.
class InternalTestAccess;

class ValidatedEdgeLayout {
    // ========== Friend Classes ==========
    // These classes can create ValidatedEdgeLayout via trustUnchecked.
    // ConstraintGateway: Creates validated layouts after constraint checking
    // LayoutSerializer: Deserializes trusted data from files
    // InternalTestAccess: Test/demo code only
    // 
    // NOTE: SugiyamaLayout is NOT a friend - it MUST use validated path
    friend class ConstraintGateway;
    friend class LayoutSerializer;
    friend class InternalTestAccess;  // For test/demo code only
    
public:
    /// Access the underlying layout (const reference)
    const EdgeLayout& get() const { return layout_; }
    
    /// Implicit conversion for read access
    operator const EdgeLayout&() const { return layout_; }
    
    /// Move semantics to extract layout (consumes the validation)
    /// Use this when you need to modify the layout or transfer ownership
    EdgeLayout extract() && { return std::move(layout_); }
    
    // Move semantics
    ValidatedEdgeLayout(ValidatedEdgeLayout&&) = default;
    ValidatedEdgeLayout& operator=(ValidatedEdgeLayout&&) = default;
    
    // Copy semantics
    ValidatedEdgeLayout(const ValidatedEdgeLayout&) = default;
    ValidatedEdgeLayout& operator=(const ValidatedEdgeLayout&) = default;
    
private:
    EdgeLayout layout_;
    
    /// Private constructor - only friend classes can create
    explicit ValidatedEdgeLayout(EdgeLayout layout) : layout_(std::move(layout)) {}
    
    /// Create ValidatedEdgeLayout bypassing validation (INTERNAL USE ONLY)
    /// This is private and only accessible to friend classes.
    /// 
    /// Use this ONLY for:
    /// - Layouts from serialized/persisted data (LayoutSerializer)
    /// - Internal trusted processing
    /// 
    /// @param layout The edge layout to wrap
    /// @return ValidatedEdgeLayout wrapping the layout
    static ValidatedEdgeLayout trustUnchecked(EdgeLayout layout) {
        return ValidatedEdgeLayout(std::move(layout));
    }
};

/// Helper class for test/demo code to access ValidatedEdgeLayout::trustUnchecked
/// 
/// WARNING: This class is for TEST and DEMO code ONLY!
/// Production code MUST use ConstraintGateway::validateAndWrap().
/// 
/// Usage in test/demo:
/// ```cpp
/// #include <arborvia/layout/constraints/ValidatedEdgeLayout.h>
/// 
/// // In test/demo code only:
/// auto validated = InternalTestAccess::trustUnchecked(layout);
/// result.setEdgeLayout(id, std::move(validated));
/// ```
class InternalTestAccess {
public:
    /// Create ValidatedEdgeLayout bypassing validation (TEST/DEMO ONLY)
    /// 
    /// @param layout The edge layout to wrap
    /// @return ValidatedEdgeLayout wrapping the layout
    static ValidatedEdgeLayout trustUnchecked(EdgeLayout layout) {
        return ValidatedEdgeLayout::trustUnchecked(std::move(layout));
    }
};

}  // namespace arborvia
