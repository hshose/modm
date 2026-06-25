#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef UDP_SPEEDTEST_TX_TIMING_ENABLE
#define UDP_SPEEDTEST_TX_TIMING_ENABLE 1
#endif

typedef struct
{
	uint64_t total_cycles;
	uint32_t calls;
	uint32_t max_cycles;
	uint32_t min_cycles;
} BenchTimingCounter;

typedef struct __attribute__((packed))
{
	uint64_t total_cycles;
	uint32_t calls;
	uint32_t max_cycles;
	uint32_t min_cycles;
} BenchTimingCounterSnapshot;

typedef struct __attribute__((packed))
{
	uint32_t enabled;
	uint32_t cycle_hz;
	BenchTimingCounterSnapshot udp_pbuf_alloc;
	BenchTimingCounterSnapshot udp_payload_fill;
	BenchTimingCounterSnapshot udp_sendto;
	BenchTimingCounterSnapshot udp_pbuf_free;
	BenchTimingCounterSnapshot udp_send_one;
	BenchTimingCounterSnapshot udp_poll;
	BenchTimingCounterSnapshot eth_low_level_output;
	BenchTimingCounterSnapshot eth_descriptor_setup;
	BenchTimingCounterSnapshot eth_cache_clean;
	BenchTimingCounterSnapshot eth_reclaim;
} EthBenchTimingSnapshot;

void eth_bench_init(void);
void eth_bench_reset(void);
uint32_t eth_bench_cycles_now(void);
uint32_t eth_bench_cycles_to_us(uint64_t cycles);
void eth_bench_timing_add(BenchTimingCounter *counter, uint32_t cycles);
void eth_bench_get_snapshot(EthBenchTimingSnapshot *snapshot);

extern BenchTimingCounter eth_bench_udp_pbuf_alloc;
extern BenchTimingCounter eth_bench_udp_payload_fill;
extern BenchTimingCounter eth_bench_udp_sendto;
extern BenchTimingCounter eth_bench_udp_pbuf_free;
extern BenchTimingCounter eth_bench_udp_send_one;
extern BenchTimingCounter eth_bench_udp_poll;
extern BenchTimingCounter eth_bench_eth_low_level_output;
extern BenchTimingCounter eth_bench_eth_descriptor_setup;
extern BenchTimingCounter eth_bench_eth_cache_clean;
extern BenchTimingCounter eth_bench_eth_reclaim;

#if UDP_SPEEDTEST_TX_TIMING_ENABLE
#define BENCH_TIME_BEGIN(var) uint32_t var = eth_bench_cycles_now()
#define BENCH_TIME_END(counter, start) \
	eth_bench_timing_add(&(counter), eth_bench_cycles_now() - (start))
#else
#define BENCH_TIME_BEGIN(var) (void)0
#define BENCH_TIME_END(counter, start) (void)0
#endif

#ifdef __cplusplus
}
#endif
