add_library(cooling)

target_sources(cooling
PRIVATE
    cooling.cpp
PUBLIC
    include/cooling.hpp
)

target_include_directories(cooling
PRIVATE
    ${INCLUDE_DIRS}
PUBLIC
    include/
)

target_link_libraries(cooling
PUBLIC
    ${COMMON_LIBS}
    fan_ssr
    error
    debug_
)
