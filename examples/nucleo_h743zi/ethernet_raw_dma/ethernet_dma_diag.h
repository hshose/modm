#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	uint32_t tx_zero_copy_frames;
	uint32_t tx_zero_copy_bytes;
	uint32_t tx_copy_frames;
	uint32_t tx_copy_bytes;
	uint32_t tx_descriptor_starvation;
	uint32_t tx_ring_full;
	uint32_t tx_reclaim_calls;
	uint32_t tx_completed_descriptors;
	uint32_t tx_completed_frames;
	uint32_t tx_pbuf_refs_acquired;
	uint32_t tx_pbufs_released;
	uint32_t tx_descriptors_in_use;
	uint32_t tx_max_descriptors_in_use;
	uint32_t tx_min_free_descriptors;
	uint32_t tx_max_pbuf_chain_length;
	uint32_t tx_max_descriptors_per_frame;
	uint32_t tx_errors;
	uint32_t tx_busy;
} EthernetDmaDiagnosticsSnapshot;

void ethernet_dma_get_diagnostics(EthernetDmaDiagnosticsSnapshot *snapshot);
void ethernet_dma_reclaim_tx_descriptors(void);

#ifdef __cplusplus
}
#endif
