#pragma once

#include "lwip/netif.h"

#ifdef __cplusplus
extern "C" {
#endif

err_t modm_lwip_netif_init(struct netif *netif);
void modm_lwip_netif_input(struct netif *netif);

#ifdef __cplusplus
}
#endif
