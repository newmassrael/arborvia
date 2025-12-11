#!/bin/bash
# Check for std::cout usage in source files
# Use LOG_* macros from arborvia/common/Logger.h instead

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_ROOT"

echo "Checking for std::cout usage in src/ and include/..."

# Search for cout usage, excluding:
# - backends/ (DefaultBackend legitimately uses cout for output)
# - third-party and build directories
# - Documentation comments (lines starting with ///)
if grep -rn --include="*.cpp" --include="*.h" --include="*.hpp" \
    --exclude-dir="backends" \
    -E "(std::cout|cout\s*<<|std::cerr|cerr\s*<<)" \
    src/ include/ 2>/dev/null | grep -v "^\s*///" | grep -v "^[^:]*:[0-9]*:\s*///"; then
    echo ""
    echo "ERROR: Found std::cout/std::cerr usage!"
    echo "Please use LOG_* macros instead:"
    echo "  #include \"arborvia/common/Logger.h\""
    echo "  LOG_INFO(\"message {}\", value);"
    echo "  LOG_DEBUG(\"debug info\");"
    echo "  LOG_ERROR(\"error: {}\", error_msg);"
    exit 1
fi

echo "OK: No cout/cerr usage found in source files"
exit 0
