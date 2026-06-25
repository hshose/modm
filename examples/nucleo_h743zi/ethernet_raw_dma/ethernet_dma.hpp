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

#pragma once

#include "ethernet_config.h"

#include <cstddef>
#include <cstdint>

struct pbuf;

namespace ethernet_dma
{

constexpr uint8_t LocalMac[] { 0x02, 0x00, 0x00, 0x00, 0x00, 0x01 };
constexpr std::size_t MaxFrameSize { 1536 };

struct Diagnostics
{
	uint32_t rxFrames;
	uint32_t txFrames;
	uint32_t rxErrors;
	uint32_t txBusy;
	uint32_t txErrors;
	uint32_t droppedFrames;
	uint32_t txZeroCopyFrames;
	uint32_t txZeroCopyBytes;
	uint32_t txCopyFrames;
	uint32_t txCopyBytes;
	uint32_t txDescriptorStarvation;
	uint32_t txRingFull;
	uint32_t txReclaimCalls;
	uint32_t txCompletedDescriptors;
	uint32_t txPbufChainTooLong;
	uint32_t txCompletedFrames;
	uint32_t txPbufRefsAcquired;
	uint32_t txPbufsReleased;
	uint32_t txDescriptorsInUse;
	uint32_t txMaxDescriptorsInUse;
	uint32_t txMinFreeDescriptors;
	uint32_t txMaxPbufChainLength;
	uint32_t txMaxDescriptorsPerFrame;
	bool linkUp;
};

void configureMpuRegion();
void initialize();
void pollDmaStatus();
void printMemoryLayout();
void reclaimTxDescriptors();

bool transmitFrame(const uint8_t *frame, std::size_t length);
bool transmitPbuf(struct pbuf *p);
bool receiveFrame(uint8_t *frame, std::size_t capacity, std::size_t &length);
bool linkIsUp();

const Diagnostics &diagnostics();

}
