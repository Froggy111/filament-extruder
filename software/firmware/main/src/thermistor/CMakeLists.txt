add_library(thermistor)

target_sources(thermistor
PRIVATE
    thermistor.cpp
PUBLIC
    include/thermistor.hpp
)

target_include_directories(thermistor
PRIVATE
    ${INCLUDE_DIRS}
PUBLIC
    include/
)

target_link_libraries(thermistor
PUBLIC
    ${COMMON_LIBS}
    gpio
    error
    debug_
)
