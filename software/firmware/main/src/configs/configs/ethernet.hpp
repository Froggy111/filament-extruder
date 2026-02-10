#pragma once

#include "gpio.hpp"

const gpio::PinConfig RMII_REF_CLK_PIN = {GPIOA, gpio::Pin::PIN1,
                                          gpio::AF::AF11_ETH};
const gpio::PinConfig RMII_CRS_DV_PIN = {GPIOA, gpio::Pin::PIN7,
                                         gpio::AF::AF11_ETH};
const gpio::PinConfig RMII_TX_EN_PIN = {GPIOB, gpio::Pin::PIN11,
                                        gpio::AF::AF11_ETH};
const gpio::PinConfig RMII_TXD0_PIN = {GPIOB, gpio::Pin::PIN12,
                                       gpio::AF::AF11_ETH};
const gpio::PinConfig RMII_TXD1_PIN = {GPIOB, gpio::Pin::PIN13,
                                       gpio::AF::AF11_ETH};
const gpio::PinConfig RMII_RXD0_PIN = {GPIOC, gpio::Pin::PIN4,
                                       gpio::AF::AF11_ETH};
const gpio::PinConfig RMII_RXD1_PIN = {GPIOC, gpio::Pin::PIN5,
                                       gpio::AF::AF11_ETH};
const gpio::PinConfig SMI_MDIO_PIN = {GPIOA, gpio::Pin::PIN2,
                                      gpio::AF::AF11_ETH};
const gpio::PinConfig SMI_MDC_PIN = {GPIOC, gpio::Pin::PIN1,
                                     gpio::AF::AF11_ETH};

const uint16_t PHY_BSR = 0x01;
const uint16_t PHY_LINK_STATUS = 0x0004;

const uint8_t IPV4_ADDR[4] = {192, 168, 1, 100};
const uint8_t IPV4_MASK[4] = {255, 255, 255, 0};
const uint8_t IPV4_GATEWAY[4] = {192, 168, 1, 1};
