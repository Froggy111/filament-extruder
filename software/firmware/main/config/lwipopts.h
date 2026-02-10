#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

#include "FreeRTOSConfig.h"
#include "stm32h7xx_hal.h"  // Needed to detect cache settings if used dynamically

/**
 * NO_SYS==1: Provides VERY minimal functionality.
 * NO_SYS==0: Use lwIP facilities (RTOS mode).
 */
#define NO_SYS 0
#define SYS_LIGHTWEIGHT_PROT 1

/* ---------- Memory options ---------- */
#define MEM_ALIGNMENT 4

/*
 * MEM_SIZE: The size of the LwIP internal heap.
 * Used for UDP/TCP control blocks (PCBs) and miscellaneous data.
 * Since we have plenty of RAM, we increase this to ensure we don't run out
 * of connections.
 * NOTE: This usually lives in DTCM (.bss) for maximum CPU speed.
 */
#define MEM_SIZE (32 * 1024)

/*
 * REMOVED: LWIP_RAM_HEAP_POINTER
 * Reason: We rely on the Linker Script to place .bss in DTCM (Fastest)
 * and .Rx_PoolSection in RAM_D1 (DMA Access).
 * Hardcoding addresses here is dangerous with the new memory map.
 */

/* ---------- Connection options ---------- */
/* MEMP_NUM_TCP_PCB: Number of simultaneous active TCP connections */
#define MEMP_NUM_TCP_PCB 16
/* MEMP_NUM_UDP_PCB: Number of simultaneous UDP sockets */
#define MEMP_NUM_UDP_PCB 12
/* MEMP_NUM_TCP_SEG: Simultaneous queued TCP segments */
#define MEMP_NUM_TCP_SEG 64

/* ---------- Pbuf options (The Critical Performance Section) ---------- */

/*
 * PBUF_POOL_SIZE: The number of buffers in the RX pool.
 * Each buffer is ~1.5KB.
 * 64 buffers * 1536 bytes = ~96KB.
 * We place this in RAM_D1 (320KB available) via Linker Script
 * (.Rx_PoolSection). A larger pool prevents packet drops during high load.
 */
#define PBUF_POOL_SIZE 64

/* PBUF_POOL_BUFSIZE: Size of each pbuf. Matches Ethernet MTU + overhead */
#define PBUF_POOL_BUFSIZE 1536

/* LWIP_SUPPORT_CUSTOM_PBUF == 1: Zero-copy RX support */
#define LWIP_SUPPORT_CUSTOM_PBUF 1

/* ---------- IPv4 options ---------- */
#define LWIP_IPV4 1

/* ---------- TCP options ---------- */
#define LWIP_TCP 1
#define TCP_TTL 255

/* TCP Maximum segment size. */
#define TCP_MSS (1500 - 40)

/*
 * TCP Performance Tuning
 * For high throughput on H7, Window and Send Buffer must be large.
 * 16 * MSS is approx 23KB.
 */
#define TCP_SND_BUF (16 * TCP_MSS)
#define TCP_WND (16 * TCP_MSS)

/* TCP_SND_QUEUELEN: Must be at least (2 * TCP_SND_BUF/TCP_MSS) for efficiency
 */
#define TCP_SND_QUEUELEN (4 * TCP_SND_BUF / TCP_MSS)

/* ---------- ICMP options ---------- */
#define LWIP_ICMP 1

/* ---------- DHCP options ---------- */
#define LWIP_DHCP 1

/* ---------- UDP options ---------- */
#define LWIP_UDP 1
#define UDP_TTL 255

/* ---------- Statistics options ---------- */
#define LWIP_STATS 0

/* ---------- Link Callback ---------- */
#define LWIP_NETIF_LINK_CALLBACK 1

/*
   --------------------------------------
   ---------- Checksum options ----------
   --------------------------------------
   STM32H7 Hardware Checksum Acceleration.
   0 = Use Hardware (Disable Software Calculation).
   1 = Use Software.
*/
#define CHECKSUM_BY_HARDWARE

#ifdef CHECKSUM_BY_HARDWARE
/* Output Checksums (TX) - Let Hardware do it */
#define CHECKSUM_GEN_IP 0
#define CHECKSUM_GEN_UDP 0
#define CHECKSUM_GEN_TCP 0
#define CHECKSUM_GEN_ICMP 0 /* H7 MAC usually supports ICMP TX checksum too */

/* Input Checksums (RX) - Let Hardware check it */
#define CHECKSUM_CHECK_IP 0
#define CHECKSUM_CHECK_UDP 0
#define CHECKSUM_CHECK_TCP 0
#define CHECKSUM_CHECK_ICMP 0
#else
/* Software Fallback */
#define CHECKSUM_GEN_IP 1
#define CHECKSUM_GEN_UDP 1
#define CHECKSUM_GEN_TCP 1
#define CHECKSUM_GEN_ICMP 1
#define CHECKSUM_CHECK_IP 1
#define CHECKSUM_CHECK_UDP 1
#define CHECKSUM_CHECK_TCP 1
#define CHECKSUM_CHECK_ICMP 1
#endif

/*
   ----------------------------------------------
   ---------- Sequential layer options ----------
   ----------------------------------------------
*/
#define LWIP_NETCONN 1

/*
   ------------------------------------
   ---------- Socket options ----------
   ------------------------------------
*/
#define LWIP_SOCKET 0

/*
   ------------------------------------
   ---------- LWIP_NETIF_API options ----------
   ------------------------------------
*/
#define LWIP_NETIF_API 1
#define LWIP_NETIF_HOSTNAME 1

/*
   ------------------------------------
   ---------- httpd options ----------
   ------------------------------------
*/
#define LWIP_HTTPD 1
#define LWIP_HTTPD_CGI 1
#define LWIP_HTTPD_SSI 1
#define HTTPD_USE_CUSTOM_FSDATA 0

/* Debugging off for performance */
#define MEMP_DEBUG LWIP_DBG_OFF
#define MEM_DEBUG LWIP_DBG_OFF

/*
   ---------------------------------
   ---------- OS options ----------
   ---------------------------------
   Increased Mailbox sizes to prevent thread starvation during bursts.
*/

#define TCPIP_THREAD_NAME "TCP/IP"
#define TCPIP_THREAD_STACKSIZE 2048

/*
 * MBOX SIZES:
 * Increased from 6 to 32.
 * If the IP task is busy processing a packet, the ISR needs space
 * to queue the next ones. 6 is too small for 100Mbit bursts.
 */
#define TCPIP_MBOX_SIZE 32
#define DEFAULT_UDP_RECVMBOX_SIZE 32
#define DEFAULT_TCP_RECVMBOX_SIZE 32
#define DEFAULT_ACCEPTMBOX_SIZE 32

#define DEFAULT_THREAD_STACKSIZE 1024
#define TCPIP_THREAD_PRIO (configMAX_PRIORITIES - 2)

#endif /* __LWIPOPTS_H__ */
