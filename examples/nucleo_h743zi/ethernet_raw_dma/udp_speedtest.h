#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UDP_SPEEDTEST_PORT 5006U
#define UDP_SPEEDTEST_MAGIC 0x53504454UL
#define UDP_SPEEDTEST_VERSION 1U
#define UDP_SPEEDTEST_MAX_PAYLOAD_SIZE 1472U
#define UDP_SPEEDTEST_TX_BUDGET_PER_POLL 4U

enum
{
	UDP_SPEEDTEST_RX_DATA = 1,
	UDP_SPEEDTEST_RX_DONE = 2,
	UDP_SPEEDTEST_RX_STATS = 3,
	UDP_SPEEDTEST_TX_START = 4,
	UDP_SPEEDTEST_TX_DATA = 5,
	UDP_SPEEDTEST_TX_DONE = 6,
	UDP_SPEEDTEST_RESET = 7,
	UDP_SPEEDTEST_ERROR = 8,
};

void udp_speedtest_init(void);
void udp_speedtest_poll(void);

uint32_t udp_speedtest_get_rx_packet_count(void);
uint32_t udp_speedtest_get_rx_byte_count(void);
uint32_t udp_speedtest_get_rx_drop_count(void);
uint32_t udp_speedtest_get_rx_malformed_count(void);
uint32_t udp_speedtest_get_tx_packet_count(void);
uint32_t udp_speedtest_get_tx_byte_count(void);
uint32_t udp_speedtest_get_tx_error_count(void);
bool udp_speedtest_tx_active(void);

#ifdef __cplusplus
}
#endif
