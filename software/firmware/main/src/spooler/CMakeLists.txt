add_library(spooler)

target_sources(spooler
PRIVATE
    spooler.cpp
PUBLIC
    include/spooler.hpp
)

target_include_directories(spooler
PRIVATE
    ${INCLUDE_DIRS}
PUBLIC
    include/
)

target_link_libraries(spooler
PUBLIC
    ${COMMON_LIBS}
    gpio
    stepper
    motor
    encoder
    error
    debug_
)
