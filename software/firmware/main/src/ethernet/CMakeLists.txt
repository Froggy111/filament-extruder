add_library(ethernet)

target_sources(ethernet
PRIVATE
   ethernet.cpp
   ethernetif.c
   webpage.hpp
   webpage.cpp
PUBLIC
    include/ethernet.hpp
    include/ethernetif.h
)

target_include_directories(ethernet
PRIVATE
    ${INCLUDE_DIRS}
PUBLIC
    include/
)

target_link_libraries(ethernet
PUBLIC
    ${COMMON_LIBS}
    error
    debug
    LWIP
    thermistor
    gpio
    heaters
    puller
    spooler
)
