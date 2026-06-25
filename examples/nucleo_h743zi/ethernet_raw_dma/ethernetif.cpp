#include "ethernetif.h"

#include "eth_bench.h"
#include "ethernet_dma.hpp"
#include "network.hpp"

#include "lwip/etharp.h"
#include "lwip/pbuf.h"
#include "lwip/prot/etharp.h"
#include "lwip/prot/ethernet.h"
#include "lwip/prot/icmp.h"
#include "lwip/prot/ip.h"
#include "lwip/prot/ip4.h"

#include <cstring>

namespace
{

alignas(32) uint8_t RxFrame[ethernet_dma::MaxFrameSize];
alignas(32) uint8_t TxFrame[ethernet_dma::MaxFrameSize];

void
countFrame(const uint8_t *frame, std::size_t length)
{
	if (length < SIZEOF_ETH_HDR)
		return;

	const auto *eth = reinterpret_cast<const struct eth_hdr *>(frame);
	const uint16_t type = lwip_ntohs(eth->type);
	if (type == ETHTYPE_ARP) {
		network::Diagnostics.arpFrames++;
	}
	else if (type == ETHTYPE_IP && length >= SIZEOF_ETH_HDR + IP_HLEN) {
		const auto *ipHeader = reinterpret_cast<const struct ip_hdr *>(frame + SIZEOF_ETH_HDR);
		if (IPH_PROTO(ipHeader) == IP_PROTO_ICMP) {
			network::Diagnostics.icmpFrames++;
		}
	}
}

err_t
lowLevelOutputCopy(struct netif *, struct pbuf *p)
{
	if (p->tot_len > sizeof(TxFrame)) {
		network::Diagnostics.txErrors++;
		return ERR_BUF;
	}

	std::size_t offset = 0;
	for (const struct pbuf *q = p; q != nullptr; q = q->next) {
		std::memcpy(TxFrame + offset, q->payload, q->len);
		offset += q->len;
	}

	countFrame(TxFrame, offset);
	if (not ethernet_dma::transmitFrame(TxFrame, offset)) {
		network::Diagnostics.txErrors++;
		return ERR_IF;
	}

	network::Diagnostics.txFrames++;
	return ERR_OK;
}

err_t
lowLevelOutputZeroCopy(struct netif *, struct pbuf *p)
{
	countFrame(static_cast<const uint8_t *>(p->payload), p->len);
	if (ethernet_dma::transmitPbuf(p)) {
		network::Diagnostics.txFrames++;
		return ERR_OK;
	}

	return ERR_MEM;
}

err_t
lowLevelOutput(struct netif *netif, struct pbuf *p)
{
	BENCH_TIME_BEGIN(output_start);
#if ETH_TX_ZERO_COPY_ENABLE
	const err_t result = lowLevelOutputZeroCopy(netif, p);
	if (result == ERR_OK) {
		BENCH_TIME_END(eth_bench_eth_low_level_output, output_start);
		return ERR_OK;
	}

#if ETH_TX_COPY_FALLBACK_ENABLE
	const err_t copyResult = lowLevelOutputCopy(netif, p);
	BENCH_TIME_END(eth_bench_eth_low_level_output, output_start);
	return copyResult;
#else
	network::Diagnostics.txErrors++;
	BENCH_TIME_END(eth_bench_eth_low_level_output, output_start);
	return result;
#endif

#else
	const err_t result = lowLevelOutputCopy(netif, p);
	BENCH_TIME_END(eth_bench_eth_low_level_output, output_start);
	return result;
#endif
}

struct pbuf *
lowLevelInput()
{
	std::size_t frameLength = 0;
	if (not ethernet_dma::receiveFrame(RxFrame, sizeof(RxFrame), frameLength))
		return nullptr;

	countFrame(RxFrame, frameLength);

	struct pbuf *p = pbuf_alloc(PBUF_RAW, static_cast<u16_t>(frameLength), PBUF_POOL);
	if (p == nullptr) {
		network::Diagnostics.rxAllocFailures++;
		network::Diagnostics.droppedFrames++;
		return nullptr;
	}

	std::size_t offset = 0;
	for (struct pbuf *q = p; q != nullptr; q = q->next) {
		std::memcpy(q->payload, RxFrame + offset, q->len);
		offset += q->len;
	}

	network::Diagnostics.rxFrames++;
	return p;
}

void
lowLevelInit(struct netif *netif)
{
	netif->hwaddr_len = ETH_HWADDR_LEN;
	std::memcpy(netif->hwaddr, ethernet_dma::LocalMac, ETH_HWADDR_LEN);
	netif->mtu = 1500;
	netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;
	if (ethernet_dma::linkIsUp()) {
		netif->flags |= NETIF_FLAG_LINK_UP;
		network::Diagnostics.linkUp = true;
	}
}

}

extern "C" err_t
ethernetif_init(struct netif *netif)
{
	netif->name[0] = 'e';
	netif->name[1] = 'n';
	netif->output = etharp_output;
	netif->linkoutput = lowLevelOutput;

	lowLevelInit(netif);
	return ERR_OK;
}

extern "C" void
ethernetif_input(struct netif *netif)
{
	while (true) {
		struct pbuf *p = lowLevelInput();
		if (p == nullptr)
			return;

		if (netif->input(p, netif) != ERR_OK) {
			pbuf_free(p);
			network::Diagnostics.droppedFrames++;
		}
	}
}
