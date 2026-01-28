add_library(mpu)
target_sources(mpu
PUBLIC
    include/mpu.hpp
PRIVATE
    mpu.cpp
)

target_include_directories(mpu
PUBLIC
    include
PRIVATE
    ${INCLUDE_DIRS}
)

target_link_libraries(mpu
PUBLIC
    ${COMMON_LIBS}
)
