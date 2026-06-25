/*
 * Minimal lwIP configuration for bare-metal static IPv4 ping, UDP, and TCP
 * echo testing.
 */

#pragma once

#include "ethernet_config.h"

#include <stdint.h>

#define NO_SYS                          1
#define SYS_LIGHTWEIGHT_PROT            0

#define LWIP_IPV4                       1
#define LWIP_IPV6                       0
#define LWIP_ARP                        1
#define LWIP_ETHERNET                   1
#define LWIP_ICMP                       1
#define LWIP_RAW                        1
#define LWIP_UDP                        1
#define LWIP_TCP                        1
#define LWIP_DHCP                       0
#define LWIP_DNS                        0
#define LWIP_AUTOIP                     0
#define LWIP_IGMP                       0

#define LWIP_NETCONN                    0
#define LWIP_SOCKET                     0
#define LWIP_STATS                      0
#define LWIP_NETIF_STATUS_CALLBACK      0
#define LWIP_NETIF_LINK_CALLBACK        0

#define MEM_LIBC_MALLOC                 0
#define MEM_SIZE                        (10 * 1024)
#define MEMP_MEM_MALLOC                 1
#define MEMP_NUM_UDP_PCB                2
#define MEMP_NUM_TCP_PCB                2
#define MEMP_NUM_TCP_PCB_LISTEN         1
#define MEMP_NUM_TCP_SEG                16
#define MEM_ALIGNMENT                   4

#define PBUF_POOL_SIZE                  8
#define PBUF_POOL_BUFSIZE               1536
#define LWIP_NETIF_TX_SINGLE_PBUF       0

#define TCP_MSS                         1460
#define TCP_WND                         (4 * TCP_MSS)
#define TCP_SND_BUF                     (4 * TCP_MSS)
#define TCP_SND_QUEUELEN                16
#define TCP_QUEUE_OOSEQ                 0

#define IP_FORWARD                      0
#define IP_REASSEMBLY                   0
#define IP_FRAG                         0
#define LWIP_BROADCAST_PING             0
#define LWIP_MULTICAST_PING             0

#if ETH_TX_CHECKSUM_OFFLOAD_ENABLE
#define CHECKSUM_GEN_IP                 0
#define CHECKSUM_GEN_UDP                0
#define CHECKSUM_GEN_TCP                0
#define CHECKSUM_GEN_ICMP               0
#else
#define CHECKSUM_GEN_IP                 1
#define CHECKSUM_GEN_UDP                1
#define CHECKSUM_GEN_TCP                1
#define CHECKSUM_GEN_ICMP               1
#endif
#define CHECKSUM_CHECK_IP               1
#define CHECKSUM_CHECK_UDP              1
#define CHECKSUM_CHECK_TCP              1
#define CHECKSUM_CHECK_ICMP             1

#define LWIP_PLATFORM_DIAG(x)           do { } while (0)

#ifdef __cplusplus
extern "C" {
#endif
extern unsigned char __lwip_heap_start[];
#ifdef __cplusplus
}
#endif

#define LWIP_RAM_HEAP_POINTER           ((void *)__lwip_heap_start)
