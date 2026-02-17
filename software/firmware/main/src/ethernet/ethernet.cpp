#include "ethernet.hpp"

#include <FreeRTOS.h>
#include <lwip/api.h>
#include <lwip/netif.h>
#include <lwip/tcpip.h>
#include <stm32h7xx_hal.h>
#include <string.h>
#include <task.h>

#include "config.hpp"
#include "debug.hpp"
#include "error.hpp"
#include "ethernetif.h"
#include "gpio.hpp"
#include "thermistor.hpp"
#include "webpage.hpp"

extern "C" {
ETH_HandleTypeDef eth_handle;
}
static uint8_t MAC_addr[6] = {0x00, 0x80, 0xE1, 0x00, 0x00, 0x00};

// netif app_netif __attribute__((section(".Rx_PoolSection")));
netif app_netif;

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT]
    __attribute__((section(".RxDescripSection"), aligned(32)));
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT]
    __attribute__((section(".TxDescripSection"), aligned(32)));

void network_task(void *args);

bool ethernet::init(void) {
    __HAL_RCC_ETH1MAC_CLK_ENABLE();
    __HAL_RCC_ETH1TX_CLK_ENABLE();
    __HAL_RCC_ETH1RX_CLK_ENABLE();
    gpio::init(RMII_REF_CLK_PIN, gpio::Mode::AF_PP, gpio::Pull::NOPULL,
               gpio::Speed::VERY_HIGH);
    gpio::init(RMII_CRS_DV_PIN, gpio::Mode::AF_PP, gpio::Pull::NOPULL,
               gpio::Speed::VERY_HIGH);
    gpio::init(RMII_TX_EN_PIN, gpio::Mode::AF_PP, gpio::Pull::NOPULL,
               gpio::Speed::VERY_HIGH);
    gpio::init(RMII_TXD0_PIN, gpio::Mode::AF_PP, gpio::Pull::NOPULL,
               gpio::Speed::VERY_HIGH);
    gpio::init(RMII_TXD1_PIN, gpio::Mode::AF_PP, gpio::Pull::NOPULL,
               gpio::Speed::VERY_HIGH);
    gpio::init(RMII_RXD0_PIN, gpio::Mode::AF_PP, gpio::Pull::NOPULL,
               gpio::Speed::VERY_HIGH);
    gpio::init(RMII_RXD1_PIN, gpio::Mode::AF_PP, gpio::Pull::NOPULL,
               gpio::Speed::VERY_HIGH);
    gpio::init(SMI_MDIO_PIN, gpio::Mode::AF_PP, gpio::Pull::NOPULL,
               gpio::Speed::VERY_HIGH);
    gpio::init(SMI_MDC_PIN, gpio::Mode::AF_PP, gpio::Pull::NOPULL,
               gpio::Speed::VERY_HIGH);

    eth_handle.Instance = ETH;
    eth_handle.Init.MACAddr = MAC_addr;
    eth_handle.Init.MediaInterface = HAL_ETH_RMII_MODE;
    eth_handle.Init.RxDesc = DMARxDscrTab;  // SRAM1
    eth_handle.Init.TxDesc = DMATxDscrTab;
    eth_handle.Init.RxBuffLen = 1536;

    if (HAL_ETH_Init(&eth_handle) != HAL_OK) {
        return false;
    }

    HAL_NVIC_SetPriority(ETH_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(ETH_IRQn);

    debug::debug("creating network task");
    BaseType_t status =
        xTaskCreate(network_task, "network task", 4096, NULL, 5, NULL);
    if (status != pdPASS) {
        debug::error("network task creation failed");
        return false;
    }

    return true;
}

void ethernet::irq_handler(void) {
    HAL_ETH_IRQHandler(&eth_handle);
    return;
}

void network_task([[maybe_unused]] void *args) {
    tcpip_init(NULL, NULL);

    debug::debug("Waiting for Ethernet PHY link...");
    uint32_t phy_status = 0;
    int link_retries = 0;
    while (link_retries < 100) {  // Timeout after ~10 seconds
        HAL_ETH_ReadPHYRegister(&eth_handle, 0, PHY_BSR, &phy_status);
        if (phy_status & PHY_LINK_STATUS) {
            debug::debug("PHY Link UP.");
            vTaskDelay(100);
            break;
        }
        link_retries++;
    }
    if (!(phy_status & PHY_LINK_STATUS)) {
        debug::error("PHY Link DOWN. Check cable and PHY connection.");
        vTaskDelay(5000);
        vTaskDelete(NULL);  // Abort task
        return;
    }

    ip4_addr_t ip, mask, gateway;
    IP4_ADDR(&ip, IPV4_ADDR[0], IPV4_ADDR[1], IPV4_ADDR[2], IPV4_ADDR[3]);
    IP4_ADDR(&mask, IPV4_MASK[0], IPV4_MASK[1], IPV4_MASK[2], IPV4_MASK[3]);
    IP4_ADDR(&gateway, IPV4_GATEWAY[0], IPV4_GATEWAY[1], IPV4_GATEWAY[2],
             IPV4_GATEWAY[3]);
    debug::debug("adding netif");
    netif_add(&app_netif, &ip, &mask, &gateway, NULL, &ethernetif_init,
              &tcpip_input);
    debug::debug("setting default netif");
    netif_set_default(&app_netif);
    debug::debug("setting netif up");
    netif_set_up(&app_netif);
    debug::debug("network setup done");
    uint32_t mac_conf = eth_handle.Instance->MACCR;
    if (mac_conf & ETH_MACCR_FES) {
        debug::debug("MAC is configured for 100 MBit");
    } else {
        debug::error("MAC is configured for 10 MBit! <--- FOUND THE BUG");
    }

    if (mac_conf & ETH_MACCR_DM) {
        debug::debug("MAC is in Full Duplex");
    } else {
        debug::error("MAC is in Half Duplex!");
    }

    vTaskDelay(100);
    BaseType_t status =
        xTaskCreate(http_server_task, "http server", 8192, NULL, 1, NULL);
    if (status != pdPASS) {
        debug::error("http task creation failed");
        vTaskDelay(5000);
        error::handler();
    }
    debug::debug("created http task");

    for (;;) {
        vTaskDelay(1000);
    }
}
