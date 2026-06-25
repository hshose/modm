/*
 * Copyright (c) 2026, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include "ethernet_dma.hpp"
#include "eth_bench.h"
#include "ethernet_dma_diag.h"

#include <modm/board.hpp>
#include <modm/driver/ethernet/lan8742a.hpp>

#include <lwip/pbuf.h>

#include <cstring>

using namespace Board;
using EMAC = modm::platform::Eth<modm::Lan8742a>;

namespace
{

namespace Ethernet
{
	using RMII_Ref_Clk = GpioInputA1;
	using RMII_Mdio = GpioA2;
	using RMII_Crs_Dv = GpioInputA7;
	using RMII_Tx_En = GpioOutputG11;
	using RMII_Tx_D0 = GpioOutputG13;
	using RMII_Tx_D1 = GpioOutputB13;
	using RMII_Mdc = GpioOutputC1;
	using RMII_Rx_D0 = GpioInputC4;
	using RMII_Rx_D1 = GpioInputC5;
	using Port = EMAC;
}

constexpr uint32_t EthernetRamBase { 0x3002'0000 };
constexpr uint32_t EthernetRamSizeBytes { 128 * 1024 };

struct DmaDescriptor
{
	__IO uint32_t DESC0;
	__IO uint32_t DESC1;
	__IO uint32_t DESC2;
	__IO uint32_t DESC3;
	uint32_t BackupAddr0;
	uint32_t BackupAddr1;
};

constexpr uint32_t RxDescriptorTableAddress { EthernetRamBase };
constexpr std::size_t DescriptorCount { 32 };
constexpr std::size_t RxBufferSize { ethernet_dma::MaxFrameSize };
constexpr std::size_t TxBufferSize { ethernet_dma::MaxFrameSize };
constexpr std::size_t LwipHeapSize { 10 * 1024 };

constexpr std::size_t DmaDescriptorSize { sizeof(DmaDescriptor) };
constexpr uint32_t TxDescriptorTableAddress {
		RxDescriptorTableAddress + DescriptorCount * DmaDescriptorSize };
constexpr uint32_t RxBufferAddress {
		TxDescriptorTableAddress + DescriptorCount * DmaDescriptorSize };
constexpr uint32_t TxBufferAddress {
		RxBufferAddress + DescriptorCount * RxBufferSize };
constexpr uint32_t LwipHeapAddress {
		TxBufferAddress + DescriptorCount * TxBufferSize };

constexpr uint32_t TxDesc3Own { modm::Bit31 };
constexpr uint32_t TxDesc3Fd { modm::Bit29 };
constexpr uint32_t TxDesc3Ld { modm::Bit28 };
constexpr uint32_t TxDesc3ChecksumFull { modm::Bit17 | modm::Bit16 };
constexpr uint32_t TxDesc3FrameLengthMask { 0x0000'7fff };

constexpr uint32_t RxDesc3Own { modm::Bit31 };
constexpr uint32_t RxDesc3Fd { modm::Bit29 };
constexpr uint32_t RxDesc3Ld { modm::Bit28 };
constexpr uint32_t RxDesc3Buf1Valid { modm::Bit24 };
constexpr uint32_t RxDesc3ErrorSummary { modm::Bit15 };
constexpr uint32_t RxDesc3FrameLengthMask { 0x0000'7fff };

static_assert(DmaDescriptorSize == 24);
static_assert(DescriptorCount > 0);
static_assert(RxDescriptorTableAddress >= EthernetRamBase);
static_assert((RxDescriptorTableAddress % 32) == 0);
static_assert((TxDescriptorTableAddress % 32) == 0);
static_assert((RxBufferAddress % 32) == 0);
static_assert((TxBufferAddress % 32) == 0);
static_assert((LwipHeapAddress % 32) == 0);
static_assert(LwipHeapAddress + LwipHeapSize <= EthernetRamBase + EthernetRamSizeBytes);

auto * const RxDescriptors = reinterpret_cast<DmaDescriptor *>(RxDescriptorTableAddress);
auto * const TxDescriptors = reinterpret_cast<DmaDescriptor *>(TxDescriptorTableAddress);
auto * const RxBuffers = reinterpret_cast<uint8_t *>(RxBufferAddress);
auto * const TxBuffers = reinterpret_cast<uint8_t *>(TxBufferAddress);
std::size_t RxDescriptorIndex { 0 };
std::size_t TxDescriptorIndex { 0 };
ethernet_dma::Diagnostics DmaDiagnostics {};

struct TxDescriptorContext
{
	struct pbuf *frame;
	bool inUse;
	bool isFrameEnd;
};

TxDescriptorContext TxContexts[DescriptorCount] {};

void
clearEthernetRam()
{
	std::memset(reinterpret_cast<void *>(RxDescriptorTableAddress), 0, EthernetRamSizeBytes);
	__DMB();
}

void
initializeRxDescriptors()
{
	ETH->DMACRCR =
			(ETH->DMACRCR & ~ETH_DMACRCR_RBSZ) |
			((uint32_t(RxBufferSize) << ETH_DMACRCR_RBSZ_Pos) & ETH_DMACRCR_RBSZ);

	for (std::size_t index = 0; index < DescriptorCount; ++index) {
		const uint32_t bufferAddress = RxBufferAddress + index * RxBufferSize;
		auto &descriptor = RxDescriptors[index];
		descriptor.DESC0 = bufferAddress;
		descriptor.DESC1 = 0;
		descriptor.DESC2 = 0;
		descriptor.BackupAddr0 = bufferAddress;
		descriptor.BackupAddr1 = 0;
		__DMB();
		descriptor.DESC3 = RxDesc3Own | RxDesc3Buf1Valid;
	}

	EMAC::setDmaRxDescriptorTable(RxDescriptorTableAddress, DescriptorCount);
	__DMB();
	ETH->DMACRDTPR = uint32_t(&RxDescriptors[DescriptorCount - 1]);
}

void
initializeTxDescriptors()
{
	for (std::size_t index = 0; index < DescriptorCount; ++index) {
		TxDescriptors[index] = {};
		TxDescriptors[index].BackupAddr0 = TxBufferAddress + index * TxBufferSize;
		TxContexts[index] = {};
	}

	EMAC::setDmaTxDescriptorTable(TxDescriptorTableAddress, DescriptorCount);
	__DMB();
	ETH->DMACTDTPR = TxDescriptorTableAddress;
}

void
releaseRxDescriptor(DmaDescriptor &descriptor)
{
	const uint32_t bufferAddress = descriptor.BackupAddr0;
	descriptor.DESC0 = bufferAddress;
	descriptor.DESC1 = 0;
	descriptor.DESC2 = 0;
	__DMB();
	descriptor.DESC3 = RxDesc3Own | RxDesc3Buf1Valid;
	__DMB();
	ETH->DMACRDTPR = uint32_t(&descriptor);
}

void
disableEthernetInterrupts()
{
	ETH->DMACIER = 0;
	NVIC_DisableIRQ(ETH_IRQn);
	ETH->DMACSR = ETH->DMACSR;
}

bool
isTxDescriptorFree(std::size_t index)
{
	return (TxDescriptors[index].DESC3 & TxDesc3Own) == 0 && !TxContexts[index].inUse;
}

std::size_t
countFreeTxDescriptors()
{
	std::size_t count = 0;
	for (std::size_t index = 0; index < DescriptorCount; ++index) {
		if (isTxDescriptorFree(index))
			count++;
	}
	if (DmaDiagnostics.txMinFreeDescriptors == 0 ||
			count < DmaDiagnostics.txMinFreeDescriptors) {
		DmaDiagnostics.txMinFreeDescriptors = count;
	}
	if (count == 0)
		DmaDiagnostics.txRingFull++;
	return count;
}

void
updateTxDescriptorUseStats()
{
	std::size_t inUse = 0;
	for (const auto &context : TxContexts) {
		if (context.inUse)
			inUse++;
	}

	DmaDiagnostics.txDescriptorsInUse = inUse;
	if (inUse > DmaDiagnostics.txMaxDescriptorsInUse)
		DmaDiagnostics.txMaxDescriptorsInUse = inUse;
}

void
cleanTxPayloadCache(const void *payload, std::size_t length)
{
	BENCH_TIME_BEGIN(cache_clean_start);
	if (payload == nullptr || length == 0) {
		BENCH_TIME_END(eth_bench_eth_cache_clean, cache_clean_start);
		return;
	}

	const auto address = reinterpret_cast<uintptr_t>(payload);
	const bool inEthernetLwipRegion =
			address >= EthernetRamBase &&
			address + length <= EthernetRamBase + EthernetRamSizeBytes;

	// The Ethernet/lwIP MPU region is configured as non-cacheable, so pbufs
	// allocated from the lwIP heap do not need cache cleaning before TX DMA.
	if (inEthernetLwipRegion) {
		BENCH_TIME_END(eth_bench_eth_cache_clean, cache_clean_start);
		return;
	}

	if ((SCB->CCR & SCB_CCR_DC_Msk) == 0) {
		BENCH_TIME_END(eth_bench_eth_cache_clean, cache_clean_start);
		return;
	}

	constexpr uintptr_t CacheLineSize { 32 };
	const uintptr_t start = address & ~(CacheLineSize - 1);
	const uintptr_t end = (address + length + CacheLineSize - 1) & ~(CacheLineSize - 1);
	SCB_CleanDCache_by_Addr(reinterpret_cast<uint32_t *>(start),
			static_cast<int32_t>(end - start));
	BENCH_TIME_END(eth_bench_eth_cache_clean, cache_clean_start);
}

}

namespace ethernet_dma
{

void
configureMpuRegion()
{
	constexpr uint32_t regionNumber { 7 };
	constexpr uint32_t size128k { 16 };

	__DMB();
	MPU->CTRL &= ~MPU_CTRL_ENABLE_Msk;
	MPU->RNR = regionNumber;
	MPU->RBAR = EthernetRamBase & MPU_RBAR_ADDR_Msk;
	MPU->RASR =
			(1UL << MPU_RASR_XN_Pos) |
			(3UL << MPU_RASR_AP_Pos) |
			(1UL << MPU_RASR_TEX_Pos) |
			(1UL << MPU_RASR_S_Pos) |
			(size128k << MPU_RASR_SIZE_Pos) |
			MPU_RASR_ENABLE_Msk;
	MPU->CTRL = MPU_CTRL_PRIVDEFENA_Msk | MPU_CTRL_ENABLE_Msk;
	__DSB();
	__ISB();
}

void
initialize()
{
	Ethernet::RMII_Ref_Clk::setInput();
	Ethernet::Port::connect<Ethernet::RMII_Ref_Clk::Refclk,
		Ethernet::RMII_Mdc::Mdc,
		Ethernet::RMII_Mdio::Mdio,
		Ethernet::RMII_Crs_Dv::Rcccrsdv,
		Ethernet::RMII_Tx_En::Txen,
		Ethernet::RMII_Tx_D0::Txd0,
		Ethernet::RMII_Tx_D1::Txd1,
		Ethernet::RMII_Rx_D0::Rxd0,
		Ethernet::RMII_Rx_D1::Rxd1>();

	clearEthernetRam();
	EMAC::setMacAddress(EMAC::MacAddressIndex::Index0, LocalMac);

	if (not EMAC::initialize<modm::platform::eth::MediaInterface::RMII>()) {
		MODM_LOG_WARNING << "ETH init returned false, continuing with forced 100M full duplex"
				<< modm::endl;
	}

	initializeTxDescriptors();
	initializeRxDescriptors();

	const bool autoNegotiationFailed = not EMAC::phyStartAutoNegotiation();
	EMAC::configureMac(autoNegotiationFailed);
	EMAC::start();
	disableEthernetInterrupts();
	DmaDiagnostics.linkUp = linkIsUp();
}

bool
transmitFrame(const uint8_t *frame, std::size_t length)
{
	reclaimTxDescriptors();

	if (frame == nullptr || length == 0 || length > TxBufferSize) {
		DmaDiagnostics.txErrors++;
		return false;
	}

	auto &descriptor = TxDescriptors[TxDescriptorIndex];
	if (!isTxDescriptorFree(TxDescriptorIndex)) {
		DmaDiagnostics.txBusy++;
		DmaDiagnostics.txDescriptorStarvation++;
		return false;
	}

	uint8_t *buffer = TxBuffers + TxDescriptorIndex * TxBufferSize;
	const std::size_t frameLength = length < 60 ? 60 : length;
	std::memcpy(buffer, frame, length);
	if (frameLength > length)
		std::memset(buffer + length, 0, frameLength - length);

	descriptor.DESC0 = uint32_t(buffer);
	descriptor.DESC1 = 0;
	descriptor.DESC2 = uint32_t(frameLength);
	descriptor.DESC3 = TxDesc3Fd | TxDesc3Ld | (uint32_t(frameLength) & TxDesc3FrameLengthMask);
#if ETH_TX_CHECKSUM_OFFLOAD_ENABLE
	descriptor.DESC3 |= TxDesc3ChecksumFull;
#endif

	__DMB();
	descriptor.DESC3 |= TxDesc3Own;
	__DSB();

	TxDescriptorIndex = (TxDescriptorIndex + 1) % DescriptorCount;
	ETH->DMACTDTPR = uint32_t(&TxDescriptors[TxDescriptorIndex]);
	DmaDiagnostics.txFrames++;
	DmaDiagnostics.txCopyFrames++;
	DmaDiagnostics.txCopyBytes += frameLength;

	return true;
}

bool
transmitPbuf(struct pbuf *p)
{
	reclaimTxDescriptors();

	if (p == nullptr || p->tot_len == 0) {
		DmaDiagnostics.txErrors++;
		return false;
	}

	std::size_t descriptorCount = 0;
	for (const struct pbuf *q = p; q != nullptr; q = q->next) {
		if (q->len == 0)
			continue;
		descriptorCount++;
	}

	if (descriptorCount == 0 || p->tot_len > MaxFrameSize) {
		DmaDiagnostics.txErrors++;
		return false;
	}

	if (descriptorCount > DmaDiagnostics.txMaxPbufChainLength)
		DmaDiagnostics.txMaxPbufChainLength = descriptorCount;
	if (descriptorCount > DmaDiagnostics.txMaxDescriptorsPerFrame)
		DmaDiagnostics.txMaxDescriptorsPerFrame = descriptorCount;

	if (descriptorCount > DescriptorCount) {
		DmaDiagnostics.txPbufChainTooLong++;
		return false;
	}

	if (countFreeTxDescriptors() < descriptorCount) {
		DmaDiagnostics.txDescriptorStarvation++;
		return false;
	}

	for (std::size_t offset = 0; offset < descriptorCount; ++offset) {
		const std::size_t index = (TxDescriptorIndex + offset) % DescriptorCount;
		if (!isTxDescriptorFree(index)) {
			DmaDiagnostics.txDescriptorStarvation++;
			return false;
		}
	}

	pbuf_ref(p);
	DmaDiagnostics.txPbufRefsAcquired++;

	BENCH_TIME_BEGIN(descriptor_setup_start);
	std::size_t descriptorOffset = 0;
	for (const struct pbuf *q = p; q != nullptr; q = q->next) {
		if (q->len == 0)
			continue;

		const std::size_t index = (TxDescriptorIndex + descriptorOffset) % DescriptorCount;
		auto &descriptor = TxDescriptors[index];
		auto &context = TxContexts[index];
		const bool first = descriptorOffset == 0;
		const bool last = descriptorOffset == descriptorCount - 1;

		cleanTxPayloadCache(q->payload, q->len);

		descriptor.DESC0 = reinterpret_cast<uint32_t>(q->payload);
		descriptor.DESC1 = 0;
		descriptor.DESC2 = uint32_t(q->len);
		descriptor.DESC3 =
				(first ? TxDesc3Fd : 0) |
				(last ? TxDesc3Ld : 0) |
				(first ? (uint32_t(p->tot_len) & TxDesc3FrameLengthMask) : 0)
#if ETH_TX_CHECKSUM_OFFLOAD_ENABLE
				| (first ? TxDesc3ChecksumFull : 0)
#endif
				;

		context.frame = last ? p : nullptr;
		context.inUse = true;
		context.isFrameEnd = last;
		descriptorOffset++;
	}

	__DMB();
	for (std::size_t offset = descriptorCount; offset > 0; --offset) {
		const std::size_t index = (TxDescriptorIndex + offset - 1) % DescriptorCount;
		TxDescriptors[index].DESC3 |= TxDesc3Own;
	}
	__DSB();

	TxDescriptorIndex = (TxDescriptorIndex + descriptorCount) % DescriptorCount;
	ETH->DMACTDTPR = uint32_t(&TxDescriptors[TxDescriptorIndex]);

	DmaDiagnostics.txFrames++;
	DmaDiagnostics.txZeroCopyFrames++;
	DmaDiagnostics.txZeroCopyBytes += p->tot_len;
	updateTxDescriptorUseStats();
	BENCH_TIME_END(eth_bench_eth_descriptor_setup, descriptor_setup_start);
	return true;
}

bool
receiveFrame(uint8_t *frame, std::size_t capacity, std::size_t &length)
{
	for (std::size_t checked = 0; checked < DescriptorCount; ++checked) {
		auto &descriptor = RxDescriptors[RxDescriptorIndex];
		const uint32_t status = descriptor.DESC3;
		if ((status & RxDesc3Own) != 0)
			return false;

		RxDescriptorIndex = (RxDescriptorIndex + 1) % DescriptorCount;
		const std::size_t frameLength = status & RxDesc3FrameLengthMask;
		const bool wholeFrame = (status & (RxDesc3Fd | RxDesc3Ld)) == (RxDesc3Fd | RxDesc3Ld);
		const bool hasError = (status & RxDesc3ErrorSummary) != 0;

		if (not wholeFrame || hasError || frameLength == 0 || frameLength > capacity) {
			if (hasError)
				DmaDiagnostics.rxErrors++;
			DmaDiagnostics.droppedFrames++;
			releaseRxDescriptor(descriptor);
			continue;
		}

		std::memcpy(frame, reinterpret_cast<const void *>(descriptor.BackupAddr0), frameLength);
		length = frameLength;
		releaseRxDescriptor(descriptor);
		DmaDiagnostics.rxFrames++;
		return true;
	}

	return false;
}

bool
linkIsUp()
{
	return EMAC::phyReadLinkStatus() == modm::platform::eth::LinkStatus::Up;
}

void
pollDmaStatus()
{
	reclaimTxDescriptors();

	const uint32_t status = ETH->DMACSR;
	if (status != 0) {
		MODM_LOG_INFO << "DMACSR=0x" << modm::hex << status << modm::ascii << modm::endl;
		ETH->DMACSR = status;
	}
	DmaDiagnostics.linkUp = linkIsUp();
}

void
reclaimTxDescriptors()
{
	BENCH_TIME_BEGIN(reclaim_start);
	DmaDiagnostics.txReclaimCalls++;

	for (std::size_t index = 0; index < DescriptorCount; ++index) {
		auto &context = TxContexts[index];
		if (!context.inUse)
			continue;

		if ((TxDescriptors[index].DESC3 & TxDesc3Own) != 0)
			continue;

		DmaDiagnostics.txCompletedDescriptors++;
		if (context.isFrameEnd) {
			DmaDiagnostics.txCompletedFrames++;
			if (context.frame != nullptr) {
				pbuf_free(context.frame);
				DmaDiagnostics.txPbufsReleased++;
			}
		}

		context = {};
	}

	updateTxDescriptorUseStats();
	BENCH_TIME_END(eth_bench_eth_reclaim, reclaim_start);
}

void
printMemoryLayout()
{
	MODM_LOG_INFO << "ETH/lwIP memory: rxdesc=0x" << modm::hex << RxDescriptorTableAddress
			<< " txdesc=0x" << TxDescriptorTableAddress
			<< " rxbuf=0x" << RxBufferAddress
			<< " txbuf=0x" << TxBufferAddress
			<< " lwip_heap=0x" << LwipHeapAddress
			<< modm::ascii
			<< " descriptors=" << uint32_t(DescriptorCount)
			<< " rxbufsize=" << uint32_t(RxBufferSize)
			<< " txbufsize=" << uint32_t(TxBufferSize)
			<< " heap=" << uint32_t(LwipHeapSize)
			<< modm::endl;
}

const Diagnostics &
diagnostics()
{
	return DmaDiagnostics;
}

}

extern "C" void
ethernet_dma_get_diagnostics(EthernetDmaDiagnosticsSnapshot *snapshot)
{
	if (snapshot == nullptr)
		return;

	snapshot->tx_zero_copy_frames = DmaDiagnostics.txZeroCopyFrames;
	snapshot->tx_zero_copy_bytes = DmaDiagnostics.txZeroCopyBytes;
	snapshot->tx_copy_frames = DmaDiagnostics.txCopyFrames;
	snapshot->tx_copy_bytes = DmaDiagnostics.txCopyBytes;
	snapshot->tx_descriptor_starvation = DmaDiagnostics.txDescriptorStarvation;
	snapshot->tx_ring_full = DmaDiagnostics.txRingFull;
	snapshot->tx_reclaim_calls = DmaDiagnostics.txReclaimCalls;
	snapshot->tx_completed_descriptors = DmaDiagnostics.txCompletedDescriptors;
	snapshot->tx_completed_frames = DmaDiagnostics.txCompletedFrames;
	snapshot->tx_pbuf_refs_acquired = DmaDiagnostics.txPbufRefsAcquired;
	snapshot->tx_pbufs_released = DmaDiagnostics.txPbufsReleased;
	snapshot->tx_descriptors_in_use = DmaDiagnostics.txDescriptorsInUse;
	snapshot->tx_max_descriptors_in_use = DmaDiagnostics.txMaxDescriptorsInUse;
	snapshot->tx_min_free_descriptors = DmaDiagnostics.txMinFreeDescriptors;
	snapshot->tx_max_pbuf_chain_length = DmaDiagnostics.txMaxPbufChainLength;
	snapshot->tx_max_descriptors_per_frame = DmaDiagnostics.txMaxDescriptorsPerFrame;
	snapshot->tx_errors = DmaDiagnostics.txErrors;
	snapshot->tx_busy = DmaDiagnostics.txBusy;
}

extern "C" void
ethernet_dma_reclaim_tx_descriptors(void)
{
	ethernet_dma::reclaimTxDescriptors();
}
