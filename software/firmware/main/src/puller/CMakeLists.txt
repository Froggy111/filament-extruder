add_library(puller)

target_sources(puller
PRIVATE
    puller.cpp
PUBLIC
    include/puller.hpp
)

target_include_directories(puller
PRIVATE
    ${INCLUDE_DIRS}
PUBLIC
    include/
)

target_link_libraries(puller
PUBLIC
    ${COMMON_LIBS}
    stepper
    indicator
    error
    debug_
)
