add_library(indicator)

target_sources(indicator
PRIVATE
    indicator.cpp
PUBLIC
    include/indicator.hpp
)

target_include_directories(indicator
PRIVATE
    ${INCLUDE_DIRS}
PUBLIC
    include/
)

target_link_libraries(indicator
PUBLIC
    ${COMMON_LIBS}
    gpio
    error
    debug_
)
