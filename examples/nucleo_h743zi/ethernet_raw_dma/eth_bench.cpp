#include "eth_bench.h"

#include <modm/board.hpp>

#include <cstring>

BenchTimingCounter eth_bench_udp_pbuf_alloc {};
BenchTimingCounter eth_bench_udp_payload_fill {};
BenchTimingCounter eth_bench_udp_sendto {};
BenchTimingCounter eth_bench_udp_pbuf_free {};
BenchTimingCounter eth_bench_udp_send_one {};
BenchTimingCounter eth_bench_udp_poll {};
BenchTimingCounter eth_bench_eth_low_level_output {};
BenchTimingCounter eth_bench_eth_descriptor_setup {};
BenchTimingCounter eth_bench_eth_cache_clean {};
BenchTimingCounter eth_bench_eth_reclaim {};

namespace
{

constexpr uint32_t CycleHz { Board::SystemClock::Frequency };

void
copyCounter(BenchTimingCounterSnapshot &snapshot, const BenchTimingCounter &counter)
{
	snapshot.total_cycles = counter.total_cycles;
	snapshot.calls = counter.calls;
	snapshot.max_cycles = counter.max_cycles;
	snapshot.min_cycles = counter.min_cycles;
}

}

extern "C" void
eth_bench_init(void)
{
#if UDP_SPEEDTEST_TX_TIMING_ENABLE
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
}

extern "C" void
eth_bench_reset(void)
{
	eth_bench_init();
	eth_bench_udp_pbuf_alloc = {};
	eth_bench_udp_payload_fill = {};
	eth_bench_udp_sendto = {};
	eth_bench_udp_pbuf_free = {};
	eth_bench_udp_send_one = {};
	eth_bench_udp_poll = {};
	eth_bench_eth_low_level_output = {};
	eth_bench_eth_descriptor_setup = {};
	eth_bench_eth_cache_clean = {};
	eth_bench_eth_reclaim = {};
}

extern "C" uint32_t
eth_bench_cycles_now(void)
{
#if UDP_SPEEDTEST_TX_TIMING_ENABLE
	return DWT->CYCCNT;
#else
	return 0;
#endif
}

extern "C" uint32_t
eth_bench_cycles_to_us(uint64_t cycles)
{
	return static_cast<uint32_t>((cycles * 1000000ULL) / CycleHz);
}

extern "C" void
eth_bench_timing_add(BenchTimingCounter *counter, uint32_t cycles)
{
#if UDP_SPEEDTEST_TX_TIMING_ENABLE
	if (counter == nullptr)
		return;

	counter->total_cycles += cycles;
	counter->calls++;
	if (cycles > counter->max_cycles)
		counter->max_cycles = cycles;
	if (counter->min_cycles == 0 || cycles < counter->min_cycles)
		counter->min_cycles = cycles;
#else
	(void) counter;
	(void) cycles;
#endif
}

extern "C" void
eth_bench_get_snapshot(EthBenchTimingSnapshot *snapshot)
{
	if (snapshot == nullptr)
		return;

	std::memset(snapshot, 0, sizeof(*snapshot));
#if UDP_SPEEDTEST_TX_TIMING_ENABLE
	snapshot->enabled = 1;
#endif
	snapshot->cycle_hz = CycleHz;
	copyCounter(snapshot->udp_pbuf_alloc, eth_bench_udp_pbuf_alloc);
	copyCounter(snapshot->udp_payload_fill, eth_bench_udp_payload_fill);
	copyCounter(snapshot->udp_sendto, eth_bench_udp_sendto);
	copyCounter(snapshot->udp_pbuf_free, eth_bench_udp_pbuf_free);
	copyCounter(snapshot->udp_send_one, eth_bench_udp_send_one);
	copyCounter(snapshot->udp_poll, eth_bench_udp_poll);
	copyCounter(snapshot->eth_low_level_output, eth_bench_eth_low_level_output);
	copyCounter(snapshot->eth_descriptor_setup, eth_bench_eth_descriptor_setup);
	copyCounter(snapshot->eth_cache_clean, eth_bench_eth_cache_clean);
	copyCounter(snapshot->eth_reclaim, eth_bench_eth_reclaim);
}
