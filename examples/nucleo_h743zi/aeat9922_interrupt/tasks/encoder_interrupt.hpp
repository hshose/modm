/*
 * Copyright (c) 2026, Lukas Wildberger
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#pragma once

#include <atomic>
#include <cstdint>
#include <limits>

#include "../encoder.hpp"
#include "../hardware.hpp"

namespace EncoderInterrupt
{

using Hardware = Board::Encoder::Aeat9922;
using Timer = Hardware::Timer;
static_assert(Data::Encoder::Resolution == Hardware::Resolution);

// Fixed storage written directly by the bare 24-bit SPI position transfer.
inline uint32_t positionBuffer{0};

inline std::atomic<uint32_t> readCount{0};
inline std::atomic<uint32_t> benchmarkCount{0};
inline std::atomic<uint32_t> benchmarkTotalCycles{0};
inline std::atomic<uint32_t> benchmarkMinCycles{std::numeric_limits<uint32_t>::max()};
inline std::atomic<uint32_t> benchmarkMaxCycles{0};

} // namespace EncoderInterrupt

MODM_ISR(TIM2)
{
	using namespace EncoderInterrupt;

	Timer::acknowledgeInterruptFlags(Timer::InterruptFlag::Update);

	const uint32_t startCycles = DWT->CYCCNT;
	Hardware::SpiTransfer::readPosition(positionBuffer);
	updateEncoder(positionBuffer);
	const uint32_t elapsedCycles = DWT->CYCCNT - startCycles;

	benchmarkCount.fetch_add(1, std::memory_order_relaxed);
	benchmarkTotalCycles.fetch_add(elapsedCycles, std::memory_order_relaxed);
	if (elapsedCycles < benchmarkMinCycles.load(std::memory_order_relaxed))
		benchmarkMinCycles.store(elapsedCycles, std::memory_order_relaxed);
	if (elapsedCycles > benchmarkMaxCycles.load(std::memory_order_relaxed))
		benchmarkMaxCycles.store(elapsedCycles, std::memory_order_relaxed);
	readCount.fetch_add(1, std::memory_order_relaxed);
}
