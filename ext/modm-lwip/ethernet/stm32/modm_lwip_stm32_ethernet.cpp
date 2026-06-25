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

#include "modm_lwip_stm32_ethernet.hpp"

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
constexpr std::size_t RxBufferSize { modm::lwip::ethernet::MaxFrameSize };
constexpr std::size_t LwipHeapSize { 10 * 1024 };

constexpr std::size_t DmaDescriptorSize { sizeof(DmaDescriptor) };
constexpr uint32_t TxDescriptorTableAddress {
		RxDescriptorTableAddress + DescriptorCount * DmaDescriptorSize };
constexpr uint32_t RxBufferAddress {
		TxDescriptorTableAddress + DescriptorCount * DmaDescriptorSize };
constexpr uint32_t LwipHeapAddress {
		RxBufferAddress + DescriptorCount * RxBufferSize };

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
static_assert((LwipHeapAddress % 32) == 0);
static_assert(LwipHeapAddress + LwipHeapSize <= EthernetRamBase + EthernetRamSizeBytes);

auto * const RxDescriptors = reinterpret_cast<DmaDescriptor *>(RxDescriptorTableAddress);
auto * const TxDescriptors = reinterpret_cast<DmaDescriptor *>(TxDescriptorTableAddress);
std::size_t RxDescriptorIndex { 0 };
std::size_t TxDescriptorIndex { 0 };

enum class RxDescriptorState : uint8_t
{
	DmaOwned,
	ReadyForLwip,
	LwipOwned,
};

struct RxPbufContext
{
	struct pbuf_custom custom;
	std::size_t descriptorIndex;
	uint16_t length;
	RxDescriptorState state;
};

struct TxDescriptorContext
{
	struct pbuf *frame;
	bool inUse;
	bool isFrameEnd;
};

modm::lwip::ethernet::Diagnostics EthernetDiagnostics {};
uint8_t LocalMac[6] {
	modm::lwip::ethernet::DefaultMac[0],
	modm::lwip::ethernet::DefaultMac[1],
	modm::lwip::ethernet::DefaultMac[2],
	modm::lwip::ethernet::DefaultMac[3],
	modm::lwip::ethernet::DefaultMac[4],
	modm::lwip::ethernet::DefaultMac[5],
};
RxPbufContext RxContexts[DescriptorCount] {};
TxDescriptorContext TxContexts[DescriptorCount] {};

std::size_t RxLwipOwnedDescriptorCount { 0 };

void
clearEthernetRam()
{
	std::memset(reinterpret_cast<void *>(RxDescriptorTableAddress), 0, EthernetRamSizeBytes);
	EthernetDiagnostics = {};
	EthernetDiagnostics.rxMinFreeDescriptors = DescriptorCount;
	RxLwipOwnedDescriptorCount = 0;
	__DMB();
}

std::size_t
countFreeRxDescriptors()
{
	return DescriptorCount - RxLwipOwnedDescriptorCount;
}

void
updateRxFreeDescriptorLowWatermark()
{
	const std::size_t freeDescriptors = countFreeRxDescriptors();
	if (freeDescriptors < EthernetDiagnostics.rxMinFreeDescriptors) {
		EthernetDiagnostics.rxMinFreeDescriptors = freeDescriptors;
	}
}

void releaseRxDescriptor(std::size_t index);

void
freeRxCustomPbuf(struct pbuf *p)
{
	auto *custom = reinterpret_cast<struct pbuf_custom *>(p);
	auto *context = reinterpret_cast<RxPbufContext *>(custom);
	const std::size_t index = context->descriptorIndex;

	EthernetDiagnostics.rxCustomFreeCallbacks++;
	if (index >= DescriptorCount || context->state != RxDescriptorState::LwipOwned) {
		EthernetDiagnostics.rxErrors++;
		return;
	}

	context->length = 0;
	context->state = RxDescriptorState::DmaOwned;
	if (RxLwipOwnedDescriptorCount > 0) {
		RxLwipOwnedDescriptorCount--;
	}
	EthernetDiagnostics.rxLwipOwnedDescriptors = static_cast<uint32_t>(RxLwipOwnedDescriptorCount);

	releaseRxDescriptor(index);
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
		auto &context = RxContexts[index];

		context = {};
		context.custom.custom_free_function = freeRxCustomPbuf;
		context.descriptorIndex = index;
		context.length = 0;
		context.state = RxDescriptorState::DmaOwned;

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
		TxContexts[index] = {};
	}

	EMAC::setDmaTxDescriptorTable(TxDescriptorTableAddress, DescriptorCount);
	__DMB();
	ETH->DMACTDTPR = TxDescriptorTableAddress;
}

void
releaseRxDescriptor(std::size_t index)
{
	auto &descriptor = RxDescriptors[index];
	const uint32_t bufferAddress = descriptor.BackupAddr0;
	descriptor.DESC0 = bufferAddress;
	descriptor.DESC1 = 0;
	descriptor.DESC2 = 0;
	__DMB();
	descriptor.DESC3 = RxDesc3Own | RxDesc3Buf1Valid;
	__DMB();
	ETH->DMACRDTPR = uint32_t(&descriptor);
	EthernetDiagnostics.rxDescriptorsReturnedToDma++;
	updateRxFreeDescriptorLowWatermark();
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
	return count;
}

void
cleanTxPayloadCache(const void *payload, std::size_t length)
{
	if (payload == nullptr || length == 0) {
		return;
	}

	const auto address = reinterpret_cast<uintptr_t>(payload);
	const bool inEthernetLwipRegion =
			address >= EthernetRamBase &&
			address + length <= EthernetRamBase + EthernetRamSizeBytes;

	// The Ethernet/lwIP MPU region is configured as non-cacheable, so pbufs
	// allocated from the lwIP heap do not need cache cleaning before TX DMA.
	if (inEthernetLwipRegion) {
		return;
	}

	if ((SCB->CCR & SCB_CCR_DC_Msk) == 0) {
		return;
	}

	constexpr uintptr_t CacheLineSize { 32 };
	const uintptr_t start = address & ~(CacheLineSize - 1);
	const uintptr_t end = (address + length + CacheLineSize - 1) & ~(CacheLineSize - 1);
	SCB_CleanDCache_by_Addr(reinterpret_cast<uint32_t *>(start),
			static_cast<int32_t>(end - start));
}

void
invalidateRxPayloadCache(const void *payload, std::size_t length)
{
	if (payload == nullptr || length == 0) {
		return;
	}

	const auto address = reinterpret_cast<uintptr_t>(payload);
	const bool inEthernetLwipRegion =
			address >= EthernetRamBase &&
			address + length <= EthernetRamBase + EthernetRamSizeBytes;

	// The Ethernet/lwIP MPU region is configured as non-cacheable. Keep this
	// hook here so cacheable external RX buffers can be supported without
	// changing the RX descriptor ownership path.
	if (inEthernetLwipRegion) {
		return;
	}

	if ((SCB->CCR & SCB_CCR_DC_Msk) == 0) {
		return;
	}

	constexpr uintptr_t CacheLineSize { 32 };
	const uintptr_t start = address & ~(CacheLineSize - 1);
	const uintptr_t end = (address + length + CacheLineSize - 1) & ~(CacheLineSize - 1);
	SCB_InvalidateDCache_by_Addr(reinterpret_cast<uint32_t *>(start),
			static_cast<int32_t>(end - start));
	EthernetDiagnostics.rxCacheInvalidateCalls++;
	EthernetDiagnostics.rxCacheInvalidateBytes += static_cast<uint32_t>(end - start);
}

}

namespace modm::lwip::ethernet
{

void
configureMemory()
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
}

void
setMacAddress(const uint8_t *address)
{
	if (address == nullptr) {
		return;
	}

	std::memcpy(LocalMac, address, sizeof(LocalMac));
}

bool
transmitPbuf(struct pbuf *p)
{
	reclaimTxDescriptors();

	if (p == nullptr || p->tot_len == 0) {
		EthernetDiagnostics.txInvalidPbufs++;
		EthernetDiagnostics.txErrors++;
		return false;
	}

	std::size_t descriptorCount = 0;
	for (const struct pbuf *q = p; q != nullptr; q = q->next) {
		if (q->len == 0)
			continue;
		descriptorCount++;
	}

	if (descriptorCount == 0 || p->tot_len > MaxFrameSize) {
		EthernetDiagnostics.txInvalidPbufs++;
		EthernetDiagnostics.txErrors++;
		return false;
	}


	if (descriptorCount > DescriptorCount) {
		EthernetDiagnostics.txPbufChainTooLong++;
		EthernetDiagnostics.txErrors++;
		return false;
	}

	if (countFreeTxDescriptors() < descriptorCount) {
		EthernetDiagnostics.txDescriptorStarvation++;
		EthernetDiagnostics.txErrors++;
		return false;
	}

	for (std::size_t offset = 0; offset < descriptorCount; ++offset) {
		const std::size_t index = (TxDescriptorIndex + offset) % DescriptorCount;
		if (!isTxDescriptorFree(index)) {
			EthernetDiagnostics.txDescriptorStarvation++;
			EthernetDiagnostics.txErrors++;
			return false;
		}
	}

	pbuf_ref(p);
	EthernetDiagnostics.txPbufRefsAcquired++;
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
				(first ? (uint32_t(p->tot_len) & TxDesc3FrameLengthMask) : 0) |
				(first ? TxDesc3ChecksumFull : 0);

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
	EthernetDiagnostics.txFrames++;
	return true;
}

struct pbuf *
receivePbuf()
{
	for (std::size_t checked = 0; checked < DescriptorCount; ++checked) {
		const std::size_t index = RxDescriptorIndex;
		auto &descriptor = RxDescriptors[RxDescriptorIndex];
		const uint32_t status = descriptor.DESC3;
		if ((status & RxDesc3Own) != 0) {
			if (RxLwipOwnedDescriptorCount == DescriptorCount) {
				EthernetDiagnostics.rxDescriptorStarvation++;
			}
			return nullptr;
		}

		RxDescriptorIndex = (RxDescriptorIndex + 1) % DescriptorCount;
		const std::size_t frameLength = status & RxDesc3FrameLengthMask;
		const bool wholeFrame = (status & (RxDesc3Fd | RxDesc3Ld)) == (RxDesc3Fd | RxDesc3Ld);
		const bool hasError = (status & RxDesc3ErrorSummary) != 0;
		auto &context = RxContexts[index];

		if (context.state != RxDescriptorState::DmaOwned) {
			EthernetDiagnostics.rxErrors++;
			EthernetDiagnostics.rxDescriptorStarvation++;
			return nullptr;
		}

		if (not wholeFrame || hasError || frameLength == 0 || frameLength > RxBufferSize) {
			if (hasError) {
				EthernetDiagnostics.rxDmaErrorFrames++;
			}
			EthernetDiagnostics.rxInvalidFrames++;
			EthernetDiagnostics.rxErrors++;
			releaseRxDescriptor(index);
			continue;
		}

		context.length = static_cast<uint16_t>(frameLength);
		context.state = RxDescriptorState::ReadyForLwip;
		context.custom.custom_free_function = freeRxCustomPbuf;

		auto * const payload = reinterpret_cast<void *>(descriptor.BackupAddr0);
		invalidateRxPayloadCache(payload, frameLength);

		struct pbuf *p = pbuf_alloced_custom(PBUF_RAW,
				static_cast<u16_t>(frameLength),
				PBUF_REF,
				&context.custom,
				payload,
				RxBufferSize);
		if (p == nullptr) {
			context.length = 0;
			context.state = RxDescriptorState::DmaOwned;
			EthernetDiagnostics.rxCustomPbufAllocationFailures++;
			EthernetDiagnostics.rxErrors++;
			releaseRxDescriptor(index);
			continue;
		}

		context.state = RxDescriptorState::LwipOwned;
		RxLwipOwnedDescriptorCount++;
		EthernetDiagnostics.rxLwipOwnedDescriptors = static_cast<uint32_t>(RxLwipOwnedDescriptorCount);
		if (RxLwipOwnedDescriptorCount > EthernetDiagnostics.rxMaxLwipOwnedDescriptors) {
			EthernetDiagnostics.rxMaxLwipOwnedDescriptors =
					static_cast<uint32_t>(RxLwipOwnedDescriptorCount);
		}
		updateRxFreeDescriptorLowWatermark();
		EthernetDiagnostics.rxFrames++;
		return p;
	}

	return nullptr;
}

void
recordRxInputError()
{
	EthernetDiagnostics.rxInputErrors++;
	EthernetDiagnostics.rxErrors++;
}

bool
linkIsUp()
{
	return EMAC::phyReadLinkStatus() == modm::platform::eth::LinkStatus::Up;
}

const uint8_t *
macAddress()
{
	return LocalMac;
}

void
pollStatus()
{
	reclaimTxDescriptors();

	const uint32_t status = ETH->DMACSR;
	if (status != 0) {
		ETH->DMACSR = status;
	}
}

void
reclaimTxDescriptors()
{

	for (std::size_t index = 0; index < DescriptorCount; ++index) {
		auto &context = TxContexts[index];
		if (!context.inUse)
			continue;

		if ((TxDescriptors[index].DESC3 & TxDesc3Own) != 0)
			continue;

		if (context.isFrameEnd) {
			if (context.frame != nullptr) {
				pbuf_free(context.frame);
				EthernetDiagnostics.txPbufsReleased++;
			}
		}

		context = {};
	}
}

const Diagnostics &
diagnostics()
{
	return EthernetDiagnostics;
}

}
