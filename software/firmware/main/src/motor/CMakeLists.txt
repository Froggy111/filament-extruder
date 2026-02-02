add_library(motor)

target_sources(motor
PRIVATE
    motor.cpp
PUBLIC
    include/motor.hpp
)

target_include_directories(motor
PRIVATE
    ${INCLUDE_DIRS}
PUBLIC
    include/
)

target_link_libraries(motor
PUBLIC
    ${COMMON_LIBS}
    gpio
    error
    debug_
)
