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

#include <cstddef>
#include <cstdint>

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
	bool linkUp;
};

void configureMpuRegion();
void initialize();
void pollDmaStatus();
void printMemoryLayout();

bool transmitFrame(const uint8_t *frame, std::size_t length);
bool receiveFrame(uint8_t *frame, std::size_t capacity, std::size_t &length);
bool linkIsUp();

const Diagnostics &diagnostics();

}
