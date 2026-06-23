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

#include <cstdint>
#include <limits>

#include "hardware.hpp"
#include "tasks/encoder_interrupt.hpp"

using namespace Board;
using namespace std::chrono_literals;

using EncoderHardware = Board::Encoder::Aeat9922;

constexpr uint32_t ReportDivider = EncoderHardware::SampleFrequency;
constexpr uint32_t CpuFrequencyMHz = Board::SystemClock::Frequency / 1'000'000;

static uint32_t
cyclesToNanoseconds(uint32_t cycles)
{
	return uint32_t(uint64_t(cycles) * 1000 / CpuFrequencyMHz);
}

int
main()
{
	SCB_DisableICache();
	SCB_DisableDCache();

	Board::initialize();
	Board::Leds::setOutput();

	MODM_LOG_INFO << "AEAT-9922 deterministic interrupt reads" << modm::endl;
	MODM_LOG_INFO << "SPI3: SCK=PB3, MISO=PB4, MOSI=PB5, CS=PA4" << modm::endl;
	MODM_LOG_INFO << "Sample frequency: " << EncoderHardware::SampleFrequency << " Hz" << modm::endl;

	while (!EncoderHardware::initialize())
	{
		Board::LedRed::toggle();
		modm::delay(250ms);
	}
	Board::LedRed::reset();
	Board::LedGreen::set();

	uint32_t lastReportCount = 0;
	while (true)
	{
		const uint32_t count = EncoderInterrupt::readCount.load(std::memory_order_relaxed);
		if ((count - lastReportCount) >= ReportDivider)
		{
			lastReportCount = count;
			const uint32_t benchmarkCount =
				EncoderInterrupt::benchmarkCount.exchange(0, std::memory_order_relaxed);
			const uint32_t totalCycles =
				EncoderInterrupt::benchmarkTotalCycles.exchange(0, std::memory_order_relaxed);
			const uint32_t minCycles = EncoderInterrupt::benchmarkMinCycles.exchange(
				std::numeric_limits<uint32_t>::max(), std::memory_order_relaxed);
			const uint32_t maxCycles =
				EncoderInterrupt::benchmarkMaxCycles.exchange(0, std::memory_order_relaxed);
			const uint32_t minNs = benchmarkCount ? cyclesToNanoseconds(minCycles) : 0;
			const uint32_t meanNs =
				benchmarkCount ? cyclesToNanoseconds((totalCycles + benchmarkCount / 2) / benchmarkCount) : 0;
			const uint32_t maxNs = benchmarkCount ? cyclesToNanoseconds(maxCycles) : 0;

			MODM_LOG_INFO << "raw_frame="
			              << static_cast<unsigned long>(
				                 Data::encoder.rawFrame.load(std::memory_order_relaxed))
			              << ", raw_position="
			              << static_cast<unsigned long>(
				                 Data::encoder.rawValue.load(std::memory_order_relaxed))
			              << ", angle_deg="
			              << Data::encoder.angleDegrees.load(std::memory_order_relaxed)
			              << ", reads=" << static_cast<unsigned long>(count)
			              << ", valid="
			              << static_cast<unsigned long>(
				                 Data::encoder.validSampleCount.load(std::memory_order_relaxed))
			              << ", parity/encoder errors="
			              << static_cast<unsigned long>(
				                 Data::encoder.parityErrorCount.load(std::memory_order_relaxed))
			              << "/"
			              << static_cast<unsigned long>(
			                 Data::encoder.encoderErrorCount.load(std::memory_order_relaxed))
			              << ", read+decode ns min/mean/max="
			              << static_cast<unsigned long>(minNs)
			              << "/"
			              << static_cast<unsigned long>(meanNs)
			              << "/"
			              << static_cast<unsigned long>(maxNs)
			              << ", bench_samples="
			              << static_cast<unsigned long>(benchmarkCount)
			              << modm::endl;
		}
		modm::delay(10ms);
	}
}
