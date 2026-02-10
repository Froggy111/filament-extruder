#include "ethernetif.h"

#include <string.h>

#include "FreeRTOS.h"
#include "lwip/snmp.h"
#include "lwip/stats.h"
#include "lwip/tcpip.h"
#include "lwip/timeouts.h"
#include "netif/etharp.h"
#include "netif/ethernet.h"
#include "semphr.h"
#include "stm32h7xx_hal.h"
#include "task.h"

/* Network Interface Name */
#define IFNAME0 's'
#define IFNAME1 't'

#define ETH_RX_BUFFER_SIZE 1536
#define ETH_RX_BUFFER_CNT 32
#define ETH_DMA_TRANSMIT_TIMEOUT (20U)

/* PHY Registers */
#define PHY_BSR ((uint16_t)0x01)
#define PHY_LINK_STATUS ((uint16_t)0x0004)

/*
 * CRITICAL ALIGNMENT FIX
 * 1. `buff` is aligned to 32 bytes. This ensures DMA writes don't share a cache
 * line with `pbuf_custom`.
 * 2. The struct size is padded to a multiple of 32 bytes so the array stays
 * aligned.
 */
typedef struct {
    struct pbuf_custom pbuf_custom;
    /* Force 'buff' to start at a 32-byte offset inside the struct */
    uint8_t buff[ETH_RX_BUFFER_SIZE] __attribute__((aligned(32)));
} __attribute__((aligned(32))) RxBuff_t;

/* RX Buffer Pool - PLACED IN RAM_D1 (via Linker) */
LWIP_MEMPOOL_DECLARE(RX_POOL, ETH_RX_BUFFER_CNT, sizeof(RxBuff_t),
                     "Zero-copy RX PBUF pool")
__attribute__((
    section(".Rx_PoolSection"))) extern u8_t memp_memory_RX_POOL_base[];

/* Globals */
extern ETH_HandleTypeDef eth_handle;

/* Descriptors - PLACED IN RAM_D2 (via Linker) - UNCACHED MPU REGION */
ETH_TxPacketConfig TxConfig __attribute__((section(".TxDescripSection")));
ETH_BufferTypeDef Txbuffer[ETH_TX_DESC_CNT]
    __attribute__((section(".TxDescripSection")));

/* FreeRTOS Objects */
SemaphoreHandle_t RxPktSemaphore = NULL;
SemaphoreHandle_t TxPktSemaphore = NULL;
TaskHandle_t EthIfThreadHandle = NULL;

/* Prototypes */
void ethernetif_input(void *argument);
void pbuf_free_custom(struct pbuf *p);

/*******************************************************************************
                       Low Level Init
*******************************************************************************/
static void low_level_init(struct netif *netif) {
    uint32_t regvalue = 0;

    HAL_ETH_ReadPHYRegister(&eth_handle, 0, PHY_BSR, &regvalue);

    netif->hwaddr_len = ETH_HWADDR_LEN;
    memcpy(netif->hwaddr, eth_handle.Init.MACAddr, 6);
    netif->mtu = 1500;
    netif->flags |=
        NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;

    LWIP_MEMPOOL_INIT(RX_POOL);

    memset(&TxConfig, 0, sizeof(ETH_TxPacketConfig));
    TxConfig.Attributes =
        ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
    TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
    TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;

    RxPktSemaphore = xSemaphoreCreateBinary();
    TxPktSemaphore = xSemaphoreCreateBinary();

    /* Very High Priority Task to drain the 128-slot ring buffer */
    xTaskCreate(ethernetif_input, "EthInput", 1024, netif,
                configMAX_PRIORITIES - 1, &EthIfThreadHandle);

    HAL_ETH_Start_IT(&eth_handle);
}

/*******************************************************************************
                       Low Level Output
*******************************************************************************/
static err_t low_level_output(struct netif *netif, struct pbuf *p) {
    uint32_t i = 0U;
    struct pbuf *q = NULL;
    err_t errval = ERR_OK;

    memset(Txbuffer, 0, ETH_TX_DESC_CNT * sizeof(ETH_BufferTypeDef));

    for (q = p; q != NULL; q = q->next) {
        if (i >= ETH_TX_DESC_CNT) return ERR_IF;

        Txbuffer[i].buffer = q->payload;
        Txbuffer[i].len = q->len;

        if (i > 0) {
            Txbuffer[i - 1].next = &Txbuffer[i];
        }

        if (q->next == NULL) {
            Txbuffer[i].next = NULL;
        }

        i++;
    }

    TxConfig.Length = p->tot_len;
    TxConfig.TxBuffer = Txbuffer;
    TxConfig.pData = p;

    pbuf_ref(p);

    do {
        if (HAL_ETH_Transmit_IT(&eth_handle, &TxConfig) == HAL_OK) {
            errval = ERR_OK;
        } else {
            if (HAL_ETH_GetError(&eth_handle) & HAL_ETH_ERROR_BUSY) {
                /* Wait for descriptors to become available */
                xSemaphoreTake(TxPktSemaphore,
                               pdMS_TO_TICKS(ETH_DMA_TRANSMIT_TIMEOUT));
                HAL_ETH_ReleaseTxPacket(&eth_handle);
                errval = ERR_BUF;
            } else {
                /* Other error */
                pbuf_free(p);
                errval = ERR_IF;
            }
        }
    } while (errval == ERR_BUF);

    return errval;
}

/*******************************************************************************
                       Low Level Input
*******************************************************************************/
static struct pbuf *low_level_input(struct netif *netif) {
    struct pbuf *p = NULL;
    /*
     * HAL_ETH_ReadData checks if the current descriptor is owned by CPU.
     * It extracts the buffer, calls RxAllocateCallback for a new one,
     * swaps them, and gives the descriptor back to DMA.
     */
    HAL_ETH_ReadData(&eth_handle, (void **)&p);
    return p;
}

/*******************************************************************************
                       Input Task
*******************************************************************************/
void ethernetif_input(void *argument) {
    struct pbuf *p = NULL;
    struct netif *netif = (struct netif *)argument;

    for (;;) {
        if (xSemaphoreTake(RxPktSemaphore, portMAX_DELAY) == pdTRUE) {
            do {
                p = low_level_input(netif);
                if (p != NULL) {
                    if (netif->input(p, netif) != ERR_OK) {
                        pbuf_free(p);
                    }
                }
            } while (p != NULL);
        }
    }
}

/*******************************************************************************
                       Callbacks
*******************************************************************************/
err_t ethernetif_init(struct netif *netif) {
    LWIP_ASSERT("netif != NULL", (netif != NULL));
    netif->hostname = "filament-extruder";
    netif->name[0] = IFNAME0;
    netif->name[1] = IFNAME1;
    netif->output = etharp_output;
    netif->linkoutput = low_level_output;
    low_level_init(netif);
    return ERR_OK;
}

void HAL_ETH_RxCpltCallback([[maybe_unused]] ETH_HandleTypeDef *eth_handle) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(RxPktSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_ETH_TxCpltCallback([[maybe_unused]] ETH_HandleTypeDef *eth_handle) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(TxPktSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_ETH_RxAllocateCallback(uint8_t **buff) {
    /* Allocates a new pbuf to give to the DMA descriptor */
    struct pbuf_custom *p = LWIP_MEMPOOL_ALLOC(RX_POOL);
    if (p) {
        *buff = (uint8_t *)p + offsetof(RxBuff_t, buff);
        p->custom_free_function = pbuf_free_custom;
        pbuf_alloced_custom(PBUF_RAW, 0, PBUF_REF, p, *buff,
                            ETH_RX_BUFFER_SIZE);
    } else {
        *buff = NULL;
    }
}

void HAL_ETH_RxLinkCallback(void **pStart, void **pEnd, uint8_t *buff,
                            uint16_t Length) {
    struct pbuf **ppStart = (struct pbuf **)pStart;
    struct pbuf **ppEnd = (struct pbuf **)pEnd;
    struct pbuf *p = NULL;

    p = (struct pbuf *)(buff - offsetof(RxBuff_t, buff));
    p->next = NULL;
    p->tot_len = 0;
    p->len = Length;

    if (!*ppStart)
        *ppStart = p;
    else
        (*ppEnd)->next = p;
    *ppEnd = p;

    for (p = *ppStart; p != NULL; p = p->next) p->tot_len += Length;

    /*
     * INVALIDATE D-CACHE
     * Because 'buff' is now guaranteed to be 32-byte aligned by the struct
     * definition, this Invalidate will NOT corrupt the 'pbuf_custom' header
     * that precedes it.
     */
    SCB_InvalidateDCache_by_Addr((uint32_t *)buff, ETH_RX_BUFFER_SIZE);
}

void HAL_ETH_TxFreeCallback(uint32_t *buff) { pbuf_free((struct pbuf *)buff); }

void pbuf_free_custom(struct pbuf *p) {
    struct pbuf_custom *custom_pbuf = (struct pbuf_custom *)p;
    LWIP_MEMPOOL_FREE(RX_POOL, custom_pbuf);
}
