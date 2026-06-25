#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UDP_ECHO_PORT 5005U

void udp_echo_init(void);

uint32_t udp_echo_get_rx_count(void);
uint32_t udp_echo_get_tx_count(void);
uint32_t udp_echo_get_error_count(void);
uint32_t udp_echo_get_bind_error_count(void);
uint32_t udp_echo_get_rx_bytes(void);
uint32_t udp_echo_get_tx_bytes(void);

#ifdef __cplusplus
}
#endif
