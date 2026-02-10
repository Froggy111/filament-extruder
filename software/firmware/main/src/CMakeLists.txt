set(CUSTOM_LIBS
error
clock
gpio
usb
debug_
mpu
fan_ssr
stepper
encoder
motor
indicator
thermistor
spooler
ethernet
)

add_subdirectory(configs)
list(APPEND COMMON_LIBS configs)

message(STATUS "COMMON_LIBS: ${COMMON_LIBS}")
message(STATUS "INCLUDE_DIRS: ${INCLUDE_DIRS}")
message(STATUS "CUSTOM_LIBS: ${CUSTOM_LIBS}")

add_subdirectory(clock)
add_subdirectory(debug)
add_subdirectory(error)
add_subdirectory(fan_ssr)
add_subdirectory(stepper)
add_subdirectory(peripherals)
add_subdirectory(encoder)
add_subdirectory(motor)
add_subdirectory(indicator)
add_subdirectory(thermistor)
add_subdirectory(spooler)
add_subdirectory(ethernet)

set(TARGET_NAME "${PROJECT_NAME}-${MCU_SERIES}")
add_executable(${TARGET_NAME})

target_sources(${TARGET_NAME}
PUBLIC
    include/main.hpp
PRIVATE
    main.cpp
    core_interrupts.cpp
    syscalls.cpp
)

target_include_directories(${TARGET_NAME}
PRIVATE
    ${INCLUDE_DIRS}
    include/
)

message(STATUS "COMMON_LIBS: ${COMMON_LIBS}")

target_link_libraries(${TARGET_NAME}
PRIVATE
    ${COMMON_LIBS}
    ${CUSTOM_LIBS}
)

stm32_add_linker_script(${TARGET_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR}/linker_scripts/${MCU_SUBSERIES}_${MCU_CORE}.ld)

target_print_size(${TARGET_NAME})

stm32_generate_binary_file(${TARGET_NAME})
stm32_generate_hex_file(${TARGET_NAME})
generate_flash_target(
    TARGET ${TARGET_NAME}
    INTERFACE ${OPENOCD_INTERFACE_CFG}
    MCU ${OPENOCD_MCU_CFG}
)
