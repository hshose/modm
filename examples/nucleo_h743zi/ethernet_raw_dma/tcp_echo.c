#include "tcp_echo.h"

#include "lwip/err.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define TCP_ECHO_MAX_CONNECTIONS 2U

typedef struct
{
	struct tcp_pcb *pcb;
	bool in_use;
} TcpEchoState;

static struct tcp_pcb *listen_pcb;
static TcpEchoState states[TCP_ECHO_MAX_CONNECTIONS];

static uint32_t listen_errors;
static uint32_t connection_count;
static uint32_t active_connections;
static uint32_t closed_connections;
static uint32_t aborted_connections;
static uint32_t rx_bytes;
static uint32_t tx_bytes;
static uint32_t send_limited_events;
static uint32_t tcp_errors;

static TcpEchoState *
allocate_state(void)
{
	for (size_t index = 0; index < TCP_ECHO_MAX_CONNECTIONS; ++index) {
		if (!states[index].in_use) {
			memset(&states[index], 0, sizeof(states[index]));
			states[index].in_use = true;
			active_connections++;
			return &states[index];
		}
	}
	return NULL;
}

static void
free_state(TcpEchoState *state)
{
	if (state == NULL || !state->in_use)
		return;

	state->pcb = NULL;
	state->in_use = false;
	if (active_connections > 0)
		active_connections--;
}

static void
clear_callbacks(struct tcp_pcb *pcb)
{
	tcp_arg(pcb, NULL);
	tcp_recv(pcb, NULL);
	tcp_sent(pcb, NULL);
	tcp_err(pcb, NULL);
	tcp_poll(pcb, NULL, 0);
}

static err_t
close_connection(struct tcp_pcb *pcb, TcpEchoState *state)
{
	clear_callbacks(pcb);
	free_state(state);

	const err_t result = tcp_close(pcb);
	if (result == ERR_OK) {
		closed_connections++;
		return ERR_OK;
	}

	tcp_abort(pcb);
	aborted_connections++;
	tcp_errors++;
	return ERR_ABRT;
}

static err_t
tcp_echo_sent(void *arg, struct tcp_pcb *pcb, u16_t len)
{
	(void)arg;
	(void)pcb;
	(void)len;
	return ERR_OK;
}

static void
tcp_echo_err(void *arg, err_t err)
{
	(void)err;
	TcpEchoState *state = (TcpEchoState *)arg;
	free_state(state);
	aborted_connections++;
	tcp_errors++;
}

static err_t
queue_echo_bytes(struct tcp_pcb *pcb, const uint8_t *data, uint16_t length)
{
	uint16_t offset = 0;
	while (offset < length) {
		const u16_t available = tcp_sndbuf(pcb);
		if (available == 0) {
			send_limited_events++;
			return ERR_MEM;
		}

		uint16_t chunk = length - offset;
		if (chunk > available)
			chunk = available;

		const err_t result = tcp_write(pcb, data + offset, chunk, TCP_WRITE_FLAG_COPY);
		if (result != ERR_OK) {
			send_limited_events++;
			return result;
		}

		tx_bytes += chunk;
		offset += chunk;
	}

	return ERR_OK;
}

static err_t
tcp_echo_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
	TcpEchoState *state = (TcpEchoState *)arg;

	if (err != ERR_OK) {
		if (p != NULL)
			pbuf_free(p);
		tcp_errors++;
		return err;
	}

	if (p == NULL)
		return close_connection(pcb, state);

	rx_bytes += p->tot_len;
	tcp_recved(pcb, p->tot_len);

	err_t result = ERR_OK;
	for (struct pbuf *q = p; q != NULL; q = q->next) {
		result = queue_echo_bytes(pcb, (const uint8_t *)q->payload, q->len);
		if (result != ERR_OK)
			break;
	}

	if (result == ERR_OK)
		result = tcp_output(pcb);
	else
		tcp_output(pcb);

	pbuf_free(p);
	return result == ERR_MEM ? ERR_OK : result;
}

static err_t
tcp_echo_poll(void *arg, struct tcp_pcb *pcb)
{
	TcpEchoState *state = (TcpEchoState *)arg;
	if (state == NULL || state->pcb != pcb)
		return ERR_OK;

	(void)tcp_output(pcb);
	return ERR_OK;
}

static err_t
tcp_echo_accept(void *arg, struct tcp_pcb *new_pcb, err_t err)
{
	(void)arg;

	if (err != ERR_OK || new_pcb == NULL) {
		tcp_errors++;
		return ERR_VAL;
	}

	TcpEchoState *state = allocate_state();
	if (state == NULL) {
		tcp_abort(new_pcb);
		aborted_connections++;
		tcp_errors++;
		return ERR_ABRT;
	}

	state->pcb = new_pcb;
	connection_count++;

	tcp_arg(new_pcb, state);
	tcp_recv(new_pcb, tcp_echo_recv);
	tcp_sent(new_pcb, tcp_echo_sent);
	tcp_err(new_pcb, tcp_echo_err);
	tcp_poll(new_pcb, tcp_echo_poll, 4);
	return ERR_OK;
}

void
tcp_echo_init(void)
{
	if (listen_pcb != NULL)
		return;

	struct tcp_pcb *pcb = tcp_new();
	if (pcb == NULL) {
		listen_errors++;
		return;
	}

	err_t result = tcp_bind(pcb, IP_ANY_TYPE, TCP_ECHO_PORT);
	if (result != ERR_OK) {
		tcp_abort(pcb);
		listen_errors++;
		return;
	}

	listen_pcb = tcp_listen(pcb);
	if (listen_pcb == NULL) {
		tcp_abort(pcb);
		listen_errors++;
		return;
	}

	tcp_accept(listen_pcb, tcp_echo_accept);
}

uint32_t
tcp_echo_get_connection_count(void)
{
	return connection_count;
}

uint32_t
tcp_echo_get_active_connection_count(void)
{
	return active_connections;
}

uint32_t
tcp_echo_get_rx_byte_count(void)
{
	return rx_bytes;
}

uint32_t
tcp_echo_get_tx_byte_count(void)
{
	return tx_bytes;
}

uint32_t
tcp_echo_get_error_count(void)
{
	return listen_errors + tcp_errors;
}

uint32_t
tcp_echo_get_send_limited_count(void)
{
	return send_limited_events;
}
