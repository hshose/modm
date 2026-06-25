#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void encoder_udp_streamer_initialize(void);
void encoder_udp_streamer_poll(void);

uint32_t encoder_udp_streamer_samples_sent(void);
uint32_t encoder_udp_streamer_packets_sent(void);
uint32_t encoder_udp_streamer_send_errors(void);
bool encoder_udp_streamer_is_enabled(void);

#ifdef __cplusplus
}
#endif
