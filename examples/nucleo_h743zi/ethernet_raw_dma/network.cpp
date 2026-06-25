#include "network.hpp"

#include "ethernet_dma.hpp"
#include "ethernetif.h"
#include "tcp_echo.h"
#include "udp_echo.h"
#include "udp_speedtest.h"

#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "netif/ethernet.h"

#include <modm/processing.hpp>

namespace
{
struct netif Netif;
}

namespace network
{

DiagnosticsData Diagnostics {};

void
initialize()
{
	lwip_init();

	ip4_addr_t ip;
	ip4_addr_t netmask;
	ip4_addr_t gateway;

	IP4_ADDR(&ip, StaticConfig.ip[0], StaticConfig.ip[1], StaticConfig.ip[2], StaticConfig.ip[3]);
	IP4_ADDR(&netmask, StaticConfig.netmask[0], StaticConfig.netmask[1],
			StaticConfig.netmask[2], StaticConfig.netmask[3]);
	IP4_ADDR(&gateway, StaticConfig.gateway[0], StaticConfig.gateway[1],
			StaticConfig.gateway[2], StaticConfig.gateway[3]);

	netif_add(&Netif, &ip, &netmask, &gateway, nullptr, ethernetif_init, ethernet_input);
	netif_set_default(&Netif);
	netif_set_up(&Netif);

	if (ethernet_dma::linkIsUp()) {
		netif_set_link_up(&Netif);
		Diagnostics.linkUp = true;
	}
	else {
		netif_set_link_down(&Netif);
		Diagnostics.linkUp = false;
	}

	udp_echo_init();
	udp_speedtest_init();
	tcp_echo_init();
}

void
poll()
{
	const bool linkUp = ethernet_dma::linkIsUp();
	if (linkUp != Diagnostics.linkUp) {
		Diagnostics.linkUp = linkUp;
		if (linkUp)
			netif_set_link_up(&Netif);
		else
			netif_set_link_down(&Netif);
	}

	ethernet_dma::reclaimTxDescriptors();
	ethernetif_input(&Netif);
	sys_check_timeouts();
	udp_speedtest_poll();
}

}

extern "C" uint32_t
sys_now()
{
	return modm::Clock::now().time_since_epoch().count();
}
