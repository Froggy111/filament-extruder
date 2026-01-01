cmake_minimum_required(VERSION 4.0)

include(../../cmake/repo-common.cmake)
include(../../cmake/generate-flash-target.cmake)
include(../../cmake/print-size.cmake)

set(CMAKE_TOOLCHAIN_FILE "${STM32_CMAKE_PATH}/cmake/stm32_gcc.cmake")


set(PROJECT_NAME filament-extruder-fw)

# microcontroller info
set(MCU_SERIES H7)
string(TOLOWER "${MCU_SERIES}" MCU_SERIES_LOWER)
set(MCU_CORE M7)
set(MCU_TYPE H723)
set(MCU_SUBSERIES H723ZE)
# flashing and debugging
set(OPENOCD_INTERFACE_CFG "interface/stlink.cfg")
set(OPENOCD_MCU_CFG "target/stm32${MCU_SERIES_LOWER}x.cfg")

project(${PROJECT_NAME} C CXX ASM)
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(STM32_CUBE_H7_PATH "${CMAKE_CURRENT_LIST_DIR}/STM32CubeH7")
set(FREERTOS_PATH "${CMAKE_CURRENT_LIST_DIR}/FreeRTOS")
message(STATUS "STM32_CUBE_H7_PATH is ${STM32_CUBE_H7_PATH}")

# RWX error suppression
add_link_options(
    "-Wl,--no-warn-rwx-segments"
    "-save-temps"
)

add_compile_options(
    "-mfloat-abi=hard"
    "-ffunction-sections"
    "-fdata-sections"
    "-save-temps"
)

# packages
set(HAL_COMPONENTS
    RCC
    CORTEX
    GPIO
    PWR
    FLASH
    PCD
    LL_USB
    TIM
    ADC
    SPI
    DMA
    UART
)
set(HAL_EXTENSIONS
    RCCEx
    PWREx
    PCDEx
    TIMEx
    ADCEx
    UARTEx
)

set(HAL_FINDPACKAGE_COMPONENTS
    ${HAL_COMPONENTS}
    STM32${MCU_SUBSERIES}${MCU_CORE}
)
set(CMSIS_COMPONENTS
    STM32${MCU_TYPE}_${MCU_CORE}
    STM32${MCU_SERIES}_${MCU_CORE}
)
set(FREERTOS_COMPONENTS
    ARM_C${MCU_CORE}
)

find_package(HAL COMPONENTS ${HAL_FINDPACKAGE_COMPONENTS} REQUIRED)
find_package(CMSIS 2 COMPONENTS ${CMSIS_COMPONENTS} REQUIRED)
find_package(FreeRTOS COMPONENTS ${FREERTOS_COMPONENTS} REQUIRED)

set(FREERTOS_NAMESPACE FreeRTOS)
set(HAL_NAMESPACE HAL::STM32::${MCU_SERIES}::${MCU_CORE})

set(COMMON_LIBS
${HAL_NAMESPACE}
${FREERTOS_NAMESPACE}::Timers
${FREERTOS_NAMESPACE}::Heap::4
${FREERTOS_NAMESPACE}::ARM_C${MCU_CORE}
CMSIS::STM32::${MCU_SUBSERIES}::${MCU_CORE}
STM32::NoSys
)

foreach(HAL_COMPONENT ${HAL_COMPONENTS})
    list(APPEND COMMON_LIBS "${HAL_NAMESPACE}::${HAL_COMPONENT}")
endforeach()
foreach(HAL_EXTENSION ${HAL_EXTENSIONS})
    list(APPEND COMMON_LIBS "${HAL_NAMESPACE}::${HAL_EXTENSION}")
endforeach()

set(INCLUDE_DIRS config)
get_filename_component(INCLUDE_DIRS
    "${INCLUDE_DIRS}" ABSOLUTE)

add_subdirectory(middlewares)
add_subdirectory(src)
