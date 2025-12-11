# ArborVia Common Module
# Logging and utility infrastructure

set(ARBORVIA_COMMON_SOURCES
    src/common/Logger.cpp
)

set(ARBORVIA_COMMON_HEADERS
    include/arborvia/common/ILoggerBackend.h
    include/arborvia/common/Logger.h
)

# Backend sources (conditionally compiled)
set(ARBORVIA_DEFAULT_BACKEND_SOURCES
    src/backends/DefaultBackend.cpp
)

set(ARBORVIA_SPDLOG_BACKEND_SOURCES
    src/backends/SpdlogBackend.cpp
)

set(ARBORVIA_BACKEND_HEADERS
    include/arborvia/backends/DefaultBackend.h
    include/arborvia/backends/SpdlogBackend.h
)
