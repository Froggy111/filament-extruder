add_library(fan_ssr)

target_sources(fan_ssr
PRIVATE
    fan_ssr.cpp
PUBLIC
    include/fan_ssr.hpp
)

target_include_directories(fan_ssr
PRIVATE
    ${INCLUDE_DIRS}
PUBLIC
    include/
)

target_link_libraries(fan_ssr
PUBLIC
    ${COMMON_LIBS}
    gpio
    error
    debug_
)
