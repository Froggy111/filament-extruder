add_library(configs)

target_sources(configs
PUBLIC
    config.hpp

    configs/clock.hpp
    configs/debug.hpp
    configs/encoder.hpp
    configs/fan_ssr.hpp
    configs/stepper_.hpp
    configs/peripherals.hpp
    configs/motor.hpp
    configs/indicator.hpp
    configs/thermistor.hpp
    configs/spooler.hpp

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
)

set_target_properties(configs PROPERTIES LINKER_LANGUAGE CXX)
