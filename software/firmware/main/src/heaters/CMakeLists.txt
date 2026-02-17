add_library(heaters)

target_sources(heaters
PRIVATE
    heaters.cpp
    PID.cpp
    PID.hpp
PUBLIC
    include/heaters.hpp
)

target_include_directories(heaters
PRIVATE
    ${INCLUDE_DIRS}
PUBLIC
    include/
)

target_link_libraries(heaters
PUBLIC
    ${COMMON_LIBS}
    error
    debug_
    fan_ssr
    thermistor
)
