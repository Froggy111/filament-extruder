add_library(stepper)

target_sources(stepper
PRIVATE
    stepper.cpp
PUBLIC
    include/stepper.hpp
)

target_include_directories(stepper
PRIVATE
    ${INCLUDE_DIRS}
PUBLIC
    include/
)

target_link_libraries(stepper
PUBLIC
    ${COMMON_LIBS}
    gpio
    error
    debug_
)
