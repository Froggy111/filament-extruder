cmake_minimum_required(VERSION 3.16)

set(LWIP_DIR ${CMAKE_CURRENT_SOURCE_DIR})
include(src/Filelists.cmake)

add_library(LWIP STATIC)

target_sources(LWIP
PRIVATE
    ${lwipcore_SRCS}
    ${lwipcore4_SRCS}
    ${lwipcore6_SRCS}
    ${lwipapi_SRCS}
    ${lwiphttp_SRCS}
    ${lwipnetif_SRCS}
    system/sys_arch.c
)

target_include_directories(LWIP
PRIVATE
    ${INCLUDE_DIRS}
PUBLIC
    src/include
    src/include/apps
    system/include
)

target_link_libraries(LWIP
PUBLIC
    ${COMMON_LIBS}
)
