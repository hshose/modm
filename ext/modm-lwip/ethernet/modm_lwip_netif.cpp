#include "modm_lwip_netif.hpp"

#include "modm_lwip_ethernet.hpp"

#include "lwip/etharp.h"
#include "lwip/pbuf.h"
#include "lwip/prot/ethernet.h"

#include <cstring>

namespace
{

err_t
lowLevelOutput(struct netif *, struct pbuf *p)
{
	if (modm::lwip::ethernet::transmitPbuf(p)) {
		return ERR_OK;
	}

	return ERR_MEM;
}

struct pbuf *
lowLevelInput()
{
	return modm::lwip::ethernet::receivePbuf();
}

void
lowLevelInit(struct netif *netif)
{
	netif->hwaddr_len = ETH_HWADDR_LEN;
	std::memcpy(netif->hwaddr, modm::lwip::ethernet::macAddress(), ETH_HWADDR_LEN);
	netif->mtu = 1500;
	netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;
	if (modm::lwip::ethernet::linkIsUp()) {
		netif->flags |= NETIF_FLAG_LINK_UP;
	}
}

}

extern "C" err_t
modm_lwip_netif_init(struct netif *netif)
{
	netif->name[0] = 'e';
	netif->name[1] = 'n';
	netif->output = etharp_output;
	netif->linkoutput = lowLevelOutput;

	lowLevelInit(netif);
	return ERR_OK;
}

extern "C" void
modm_lwip_netif_input(struct netif *netif)
{
	while (true) {
		struct pbuf *p = lowLevelInput();
		if (p == nullptr)
			return;

		if (netif->input(p, netif) != ERR_OK) {
			modm::lwip::ethernet::recordRxInputError();
			pbuf_free(p);
		}
	}
}
