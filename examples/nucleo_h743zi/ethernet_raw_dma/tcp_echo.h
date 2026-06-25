#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TCP_ECHO_PORT 5007U

void tcp_echo_init(void);

uint32_t tcp_echo_get_connection_count(void);
uint32_t tcp_echo_get_active_connection_count(void);
uint32_t tcp_echo_get_rx_byte_count(void);
uint32_t tcp_echo_get_tx_byte_count(void);
uint32_t tcp_echo_get_error_count(void);
uint32_t tcp_echo_get_send_limited_count(void);

#ifdef __cplusplus
}
#endif
