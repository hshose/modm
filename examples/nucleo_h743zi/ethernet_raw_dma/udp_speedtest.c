#include "udp_speedtest.h"

#include "ethernet_dma_diag.h"
#include "eth_bench.h"

#include "lwip/err.h"
#include "lwip/ip_addr.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

#include <string.h>

#define UDP_SPEEDTEST_HEADER_SIZE 20U

typedef struct __attribute__((packed))
{
	uint32_t magic;
	uint16_t version;
	uint16_t type;
	uint32_t seq;
	uint32_t payload_len;
	uint32_t timestamp_us;
} UdpSpeedtestHeader;

typedef struct __attribute__((packed))
{
	UdpSpeedtestHeader header;
	uint32_t rx_packets;
	uint32_t rx_bytes;
	uint32_t missing_packets;
	uint32_t malformed_packets;
	uint32_t out_of_order_packets;
	uint32_t elapsed_us;
	uint32_t bytes_per_second;
} UdpSpeedtestRxStats;

typedef struct __attribute__((packed))
{
	UdpSpeedtestHeader header;
	uint32_t packet_count;
	uint32_t payload_size;
	uint32_t inter_packet_delay_us;
	uint32_t tx_budget_per_poll;
	uint32_t fill_mode;
} UdpSpeedtestTxStart;

typedef struct __attribute__((packed))
{
	UdpSpeedtestHeader header;
	uint32_t packet_count;
	uint32_t payload_size;
	uint32_t inter_packet_delay_us;
	uint32_t tx_budget_per_poll;
} UdpSpeedtestTxStartLegacy;

typedef struct __attribute__((packed))
{
	UdpSpeedtestHeader header;
	uint32_t packet_count;
	uint32_t payload_size;
	uint32_t inter_packet_delay_us;
} UdpSpeedtestTxStartVeryLegacy;

typedef struct __attribute__((packed))
{
	UdpSpeedtestHeader header;
	uint32_t attempted_packets;
	uint32_t sent_packets;
	uint32_t send_errors;
	uint32_t sent_bytes;
	uint32_t elapsed_us;
	uint32_t bytes_per_second;
	uint32_t active_tx_budget_per_poll;
	uint32_t udp_sendto_calls;
	uint32_t udp_sendto_errors;
	uint32_t tx_blocked_events;
	uint32_t tx_poll_calls;
	uint32_t tx_max_packets_per_poll;
	uint32_t active_fill_mode;
	EthernetDmaDiagnosticsSnapshot eth;
	EthBenchTimingSnapshot timing;
} UdpSpeedtestTxDone;

extern uint32_t sys_now(void);

static struct udp_pcb *speedtest_pcb;

static uint32_t rx_packets;
static uint32_t rx_bytes;
static uint32_t rx_missing_packets;
static uint32_t rx_malformed_packets;
static uint32_t rx_out_of_order_packets;
static uint32_t rx_expected_seq;
static uint32_t rx_first_time_us;
static uint32_t rx_last_time_us;
static bool rx_started;

static bool tx_active;
static ip_addr_t tx_addr;
static uint16_t tx_port;
static uint32_t tx_requested_packets;
static uint32_t tx_payload_size;
static uint32_t tx_inter_packet_delay_us;
static uint32_t tx_next_seq;
static uint32_t tx_attempted_packets;
static uint32_t tx_sent_packets;
static uint32_t tx_send_errors;
static uint32_t tx_sent_bytes;
static uint32_t tx_start_time_us;
static uint32_t tx_last_send_time_us;
static uint32_t tx_budget_per_poll = UDP_SPEEDTEST_TX_BUDGET_DEFAULT;
static uint32_t tx_sendto_calls;
static uint32_t tx_sendto_errors;
static uint32_t tx_blocked_events;
static uint32_t tx_poll_calls;
static uint32_t tx_current_poll_packets;
static uint32_t tx_max_packets_per_poll;
static UdpSpeedtestFillMode tx_fill_mode = UDP_SPEEDTEST_FILL_FULL_PATTERN;

static uint8_t tx_payload[UDP_SPEEDTEST_MAX_PAYLOAD_SIZE];

static uint32_t
udp_speedtest_time_us(void)
{
	return sys_now() * 1000UL;
}

static uint32_t
elapsed_us(uint32_t start, uint32_t end)
{
	return end - start;
}

static uint32_t
bytes_per_second(uint32_t bytes, uint32_t elapsed)
{
	if (elapsed == 0)
		return 0;
	return (uint32_t)(((uint64_t)bytes * 1000000ULL) / elapsed);
}

static bool
copy_header(struct pbuf *p, UdpSpeedtestHeader *header)
{
	if (p == NULL || p->tot_len < sizeof(*header))
		return false;

	pbuf_copy_partial(p, header, sizeof(*header), 0);
	return header->magic == UDP_SPEEDTEST_MAGIC &&
			header->version == UDP_SPEEDTEST_VERSION &&
			header->payload_len == p->tot_len;
}

static void
fill_header(UdpSpeedtestHeader *header, uint16_t type, uint32_t seq, uint32_t payload_len)
{
	header->magic = UDP_SPEEDTEST_MAGIC;
	header->version = UDP_SPEEDTEST_VERSION;
	header->type = type;
	header->seq = seq;
	header->payload_len = payload_len;
	header->timestamp_us = udp_speedtest_time_us();
}

static void
reset_rx_stats(void)
{
	rx_packets = 0;
	rx_bytes = 0;
	rx_missing_packets = 0;
	rx_malformed_packets = 0;
	rx_out_of_order_packets = 0;
	rx_expected_seq = 0;
	rx_first_time_us = 0;
	rx_last_time_us = 0;
	rx_started = false;
}

static void
stop_tx_test(void)
{
	tx_active = false;
	tx_requested_packets = 0;
	tx_payload_size = 0;
	tx_inter_packet_delay_us = 0;
	tx_next_seq = 0;
}

static void
send_rx_stats(const ip_addr_t *addr, uint16_t port)
{
	UdpSpeedtestRxStats stats;
	const uint32_t duration = rx_started ? elapsed_us(rx_first_time_us, rx_last_time_us) : 0;

	fill_header(&stats.header, UDP_SPEEDTEST_RX_STATS, 0, sizeof(stats));
	stats.rx_packets = rx_packets;
	stats.rx_bytes = rx_bytes;
	stats.missing_packets = rx_missing_packets;
	stats.malformed_packets = rx_malformed_packets;
	stats.out_of_order_packets = rx_out_of_order_packets;
	stats.elapsed_us = duration;
	stats.bytes_per_second = bytes_per_second(rx_bytes, duration);

	struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, sizeof(stats), PBUF_RAM);
	if (p == NULL)
		return;

	if (pbuf_take(p, &stats, sizeof(stats)) == ERR_OK)
		(void) udp_sendto(speedtest_pcb, p, addr, port);

	pbuf_free(p);
}

static void
send_tx_done(void)
{
	UdpSpeedtestTxDone done;
	const uint32_t now = udp_speedtest_time_us();
	const uint32_t duration = elapsed_us(tx_start_time_us, now);

	fill_header(&done.header, UDP_SPEEDTEST_TX_DONE, tx_next_seq, sizeof(done));
	done.attempted_packets = tx_attempted_packets;
	done.sent_packets = tx_sent_packets;
	done.send_errors = tx_send_errors;
	done.sent_bytes = tx_sent_bytes;
	done.elapsed_us = duration;
	done.bytes_per_second = bytes_per_second(tx_sent_bytes, duration);
	done.active_tx_budget_per_poll = tx_budget_per_poll;
	done.udp_sendto_calls = tx_sendto_calls;
	done.udp_sendto_errors = tx_sendto_errors;
	done.tx_blocked_events = tx_blocked_events;
	done.tx_poll_calls = tx_poll_calls;
	done.tx_max_packets_per_poll = tx_max_packets_per_poll;
	done.active_fill_mode = (uint32_t)tx_fill_mode;
	EthernetDmaDiagnosticsSnapshot eth;
	ethernet_dma_get_diagnostics(&eth);
	memcpy(&done.eth, &eth, sizeof(eth));
	EthBenchTimingSnapshot timing;
	eth_bench_get_snapshot(&timing);
	memcpy(&done.timing, &timing, sizeof(timing));

	struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, sizeof(done), PBUF_RAM);
	if (p == NULL) {
		tx_send_errors++;
		stop_tx_test();
		return;
	}

	if (pbuf_take(p, &done, sizeof(done)) == ERR_OK) {
		if (udp_sendto(speedtest_pcb, p, &tx_addr, tx_port) != ERR_OK)
			tx_send_errors++;
	}
	else {
		tx_send_errors++;
	}

	pbuf_free(p);
	stop_tx_test();
}

static void
handle_rx_data(const UdpSpeedtestHeader *header, uint16_t total_len)
{
	const uint32_t now = udp_speedtest_time_us();

	if (!rx_started) {
		rx_started = true;
		rx_first_time_us = now;
		rx_expected_seq = header->seq + 1;
	}
	else if (header->seq == rx_expected_seq) {
		rx_expected_seq++;
	}
	else if ((int32_t)(header->seq - rx_expected_seq) > 0) {
		rx_missing_packets += header->seq - rx_expected_seq;
		rx_expected_seq = header->seq + 1;
	}
	else {
		rx_out_of_order_packets++;
	}

	rx_last_time_us = now;
	rx_packets++;
	rx_bytes += total_len;
}

static void
handle_tx_start(struct pbuf *p, const ip_addr_t *addr, uint16_t port)
{
	UdpSpeedtestTxStart command;
	if (p->tot_len < sizeof(UdpSpeedtestTxStartVeryLegacy)) {
		rx_malformed_packets++;
		return;
	}

	memset(&command, 0, sizeof(command));
	const uint16_t command_size = (p->tot_len < sizeof(command)) ? p->tot_len : sizeof(command);
	pbuf_copy_partial(p, &command, command_size, 0);
	if (command.packet_count == 0 ||
			command.payload_size < sizeof(UdpSpeedtestHeader) ||
			command.payload_size > UDP_SPEEDTEST_MAX_PAYLOAD_SIZE) {
		rx_malformed_packets++;
		return;
	}

	tx_addr = *addr;
	tx_port = port;
	tx_requested_packets = command.packet_count;
	tx_payload_size = command.payload_size;
	tx_inter_packet_delay_us = command.inter_packet_delay_us;
	tx_budget_per_poll = command.tx_budget_per_poll;
	if (tx_budget_per_poll == 0)
		tx_budget_per_poll = UDP_SPEEDTEST_TX_BUDGET_DEFAULT;
	if (tx_budget_per_poll > UDP_SPEEDTEST_TX_BUDGET_MAX)
		tx_budget_per_poll = UDP_SPEEDTEST_TX_BUDGET_MAX;
	tx_fill_mode = (UdpSpeedtestFillMode)command.fill_mode;
	if (tx_fill_mode != UDP_SPEEDTEST_FILL_FULL_PATTERN &&
			tx_fill_mode != UDP_SPEEDTEST_FILL_HEADER_ONLY) {
		tx_fill_mode = UDP_SPEEDTEST_FILL_FULL_PATTERN;
	}
	tx_next_seq = 0;
	tx_attempted_packets = 0;
	tx_sent_packets = 0;
	tx_send_errors = 0;
	tx_sent_bytes = 0;
	tx_sendto_calls = 0;
	tx_sendto_errors = 0;
	tx_blocked_events = 0;
	tx_poll_calls = 0;
	tx_current_poll_packets = 0;
	tx_max_packets_per_poll = 0;
	tx_start_time_us = udp_speedtest_time_us();
	tx_last_send_time_us = 0;
	eth_bench_reset();
	tx_active = true;
}

static bool
send_tx_packet(void)
{
	BENCH_TIME_BEGIN(send_one_start);
	BENCH_TIME_BEGIN(alloc_start);
	struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, (u16_t)tx_payload_size, PBUF_RAM);
	BENCH_TIME_END(eth_bench_udp_pbuf_alloc, alloc_start);

	if (p == NULL) {
		tx_blocked_events++;
		BENCH_TIME_END(eth_bench_udp_send_one, send_one_start);
		return false;
	}

	tx_attempted_packets++;

	UdpSpeedtestHeader header;
	err_t result = ERR_OK;
	BENCH_TIME_BEGIN(fill_start);
	fill_header(&header, UDP_SPEEDTEST_TX_DATA, tx_next_seq, tx_payload_size);
	if (tx_fill_mode == UDP_SPEEDTEST_FILL_HEADER_ONLY) {
		result = pbuf_take(p, &header, sizeof(header));
	}
	else {
		memcpy(tx_payload, &header, sizeof(header));
		for (uint32_t index = sizeof(UdpSpeedtestHeader); index < tx_payload_size; ++index)
			tx_payload[index] = (uint8_t)(tx_next_seq + index);
		result = pbuf_take(p, tx_payload, (u16_t)tx_payload_size);
	}
	BENCH_TIME_END(eth_bench_udp_payload_fill, fill_start);

	if (result == ERR_OK) {
		tx_sendto_calls++;
		BENCH_TIME_BEGIN(sendto_start);
		result = udp_sendto(speedtest_pcb, p, &tx_addr, tx_port);
		BENCH_TIME_END(eth_bench_udp_sendto, sendto_start);
	}

	if (result == ERR_OK) {
		tx_sent_packets++;
		tx_sent_bytes += tx_payload_size;
		tx_last_send_time_us = udp_speedtest_time_us();
		tx_next_seq++;
		tx_current_poll_packets++;
		if (tx_current_poll_packets > tx_max_packets_per_poll)
			tx_max_packets_per_poll = tx_current_poll_packets;
	}
	else {
		tx_send_errors++;
		if (result == ERR_MEM)
			tx_blocked_events++;
		if (result != ERR_OK)
			tx_sendto_errors++;
	}

	BENCH_TIME_BEGIN(free_start);
	pbuf_free(p);
	BENCH_TIME_END(eth_bench_udp_pbuf_free, free_start);
	BENCH_TIME_END(eth_bench_udp_send_one, send_one_start);
	return result == ERR_OK;
}

static void
udp_speedtest_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p,
		const ip_addr_t *addr, u16_t port)
{
	(void)arg;
	(void)pcb;

	if (p == NULL) {
		rx_malformed_packets++;
		return;
	}

	UdpSpeedtestHeader header;
	if (!copy_header(p, &header)) {
		rx_malformed_packets++;
		pbuf_free(p);
		return;
	}

	switch (header.type) {
	case UDP_SPEEDTEST_RX_DATA:
		handle_rx_data(&header, p->tot_len);
		break;
	case UDP_SPEEDTEST_RX_DONE:
		send_rx_stats(addr, port);
		break;
	case UDP_SPEEDTEST_TX_START:
		handle_tx_start(p, addr, port);
		break;
	case UDP_SPEEDTEST_RESET:
		reset_rx_stats();
		stop_tx_test();
		break;
	default:
		rx_malformed_packets++;
		break;
	}

	pbuf_free(p);
}

void
udp_speedtest_init(void)
{
	eth_bench_init();

	if (speedtest_pcb != NULL)
		return;

	speedtest_pcb = udp_new();
	if (speedtest_pcb == NULL) {
		rx_malformed_packets++;
		return;
	}

	if (udp_bind(speedtest_pcb, IP_ANY_TYPE, UDP_SPEEDTEST_PORT) != ERR_OK) {
		udp_remove(speedtest_pcb);
		speedtest_pcb = NULL;
		rx_malformed_packets++;
		return;
	}

	udp_recv(speedtest_pcb, udp_speedtest_recv, NULL);
}

void
udp_speedtest_poll(void)
{
	if (!tx_active || speedtest_pcb == NULL)
		return;

	BENCH_TIME_BEGIN(poll_start);
	ethernet_dma_reclaim_tx_descriptors();

	tx_poll_calls++;
	tx_current_poll_packets = 0;
	uint32_t budget = tx_budget_per_poll;
	while (tx_active && budget-- > 0) {
		const uint32_t now = udp_speedtest_time_us();
		if (tx_inter_packet_delay_us != 0 &&
				tx_last_send_time_us != 0 &&
				elapsed_us(tx_last_send_time_us, now) < tx_inter_packet_delay_us) {
			BENCH_TIME_END(eth_bench_udp_poll, poll_start);
			return;
		}

		if (tx_next_seq >= tx_requested_packets) {
			BENCH_TIME_END(eth_bench_udp_poll, poll_start);
			send_tx_done();
			return;
		}

		if (!send_tx_packet())
			break;
	}

	ethernet_dma_reclaim_tx_descriptors();
	BENCH_TIME_END(eth_bench_udp_poll, poll_start);
}

uint32_t
udp_speedtest_get_rx_packet_count(void)
{
	return rx_packets;
}

uint32_t
udp_speedtest_get_rx_byte_count(void)
{
	return rx_bytes;
}

uint32_t
udp_speedtest_get_rx_drop_count(void)
{
	return rx_missing_packets;
}

uint32_t
udp_speedtest_get_rx_malformed_count(void)
{
	return rx_malformed_packets;
}

uint32_t
udp_speedtest_get_tx_packet_count(void)
{
	return tx_sent_packets;
}

uint32_t
udp_speedtest_get_tx_byte_count(void)
{
	return tx_sent_bytes;
}

uint32_t
udp_speedtest_get_tx_error_count(void)
{
	return tx_send_errors;
}

bool
udp_speedtest_tx_active(void)
{
	return tx_active;
}
