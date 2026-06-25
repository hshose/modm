#include "udp_echo.h"

#include "lwip/err.h"
#include "lwip/ip_addr.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

static struct udp_pcb *echo_pcb;
static uint32_t rx_count;
static uint32_t tx_count;
static uint32_t send_error_count;
static uint32_t bind_error_count;
static uint32_t rx_bytes;
static uint32_t tx_bytes;

static void
udp_echo_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p,
		const ip_addr_t *addr, u16_t port)
{
	(void)arg;

	if (p == NULL) {
		send_error_count++;
		return;
	}

	rx_count++;
	rx_bytes += p->tot_len;

	const err_t result = udp_sendto(pcb, p, addr, port);
	if (result == ERR_OK) {
		tx_count++;
		tx_bytes += p->tot_len;
	}
	else {
		send_error_count++;
	}

	pbuf_free(p);
}

void
udp_echo_init(void)
{
	if (echo_pcb != NULL)
		return;

	echo_pcb = udp_new();
	if (echo_pcb == NULL) {
		bind_error_count++;
		return;
	}

	const err_t result = udp_bind(echo_pcb, IP_ANY_TYPE, UDP_ECHO_PORT);
	if (result != ERR_OK) {
		udp_remove(echo_pcb);
		echo_pcb = NULL;
		bind_error_count++;
		return;
	}

	udp_recv(echo_pcb, udp_echo_recv, NULL);
}

uint32_t
udp_echo_get_rx_count(void)
{
	return rx_count;
}

uint32_t
udp_echo_get_tx_count(void)
{
	return tx_count;
}

uint32_t
udp_echo_get_error_count(void)
{
	return send_error_count;
}

uint32_t
udp_echo_get_bind_error_count(void)
{
	return bind_error_count;
}

uint32_t
udp_echo_get_rx_bytes(void)
{
	return rx_bytes;
}

uint32_t
udp_echo_get_tx_bytes(void)
{
	return tx_bytes;
}
