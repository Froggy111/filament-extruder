add_library(configs)

target_sources(configs
PUBLIC
    config.hpp

    configs/clock.hpp
    configs/debug.hpp
    configs/encoder.hpp
    configs/fan_ssr_.hpp
    configs/stepper_.hpp
    configs/peripherals.hpp
    configs/motor.hpp
    configs/indicator.hpp
    configs/thermistor_.hpp
    configs/spooler.hpp
    configs/ethernet.hpp
    configs/heaters.hpp
    configs/puller.hpp
    configs/cooling.hpp

    configs/encoder.hpp
    configs/encoder/ABZ.hpp

    configs/peripherals.hpp
    configs/peripherals/usb.hpp
)

target_include_directories(configs
PRIVATE
    ${INCLUDE_DIRS}
PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(configs
PUBLIC
    ${COMMON_LIBS}
    gpio
    stepper
    fan_ssr
    thermistor
)

set_target_properties(configs PROPERTIES LINKER_LANGUAGE CXX)
