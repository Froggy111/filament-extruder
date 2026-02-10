#ifndef __ETHERNETIF_H__
#define __ETHERNETIF_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "lwip/err.h"
#include "lwip/netif.h"
#include "stm32h7xx_hal.h"

/* Exported functions */
err_t ethernetif_init(struct netif *netif);

#ifdef __cplusplus
}
#endif

#endif
