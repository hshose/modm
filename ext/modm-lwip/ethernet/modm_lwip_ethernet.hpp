#pragma once

#include <cstddef>
#include <cstdint>

struct pbuf;

namespace modm::lwip::ethernet
{

constexpr std::size_t MaxFrameSize { 1536 };

struct Diagnostics
{
	uint32_t txFrames;
	uint32_t txErrors;
	uint32_t txInvalidPbufs;
	uint32_t txDescriptorStarvation;
	uint32_t txPbufChainTooLong;
	uint32_t txPbufRefsAcquired;
	uint32_t txPbufsReleased;

	uint32_t rxFrames;
	uint32_t rxErrors;
	uint32_t rxDmaErrorFrames;
	uint32_t rxInvalidFrames;
	uint32_t rxCustomPbufAllocationFailures;
	uint32_t rxLwipOwnedDescriptors;
	uint32_t rxMaxLwipOwnedDescriptors;
	uint32_t rxMinFreeDescriptors;
	uint32_t rxDescriptorStarvation;
	uint32_t rxInputErrors;
	uint32_t rxCustomFreeCallbacks;
	uint32_t rxDescriptorsReturnedToDma;
	uint32_t rxCacheInvalidateCalls;
	uint32_t rxCacheInvalidateBytes;
};

void configureMemory();
void initialize();
void pollStatus();
void reclaimTxDescriptors();

void setMacAddress(const uint8_t *address);
bool transmitPbuf(struct pbuf *p);
struct pbuf *receivePbuf();
void recordRxInputError();
bool linkIsUp();

const uint8_t *macAddress();
const Diagnostics &diagnostics();

}
