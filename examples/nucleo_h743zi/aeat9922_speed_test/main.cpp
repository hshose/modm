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

#include <modm/board.hpp>
#include <modm/driver/encoder/aeat9922.hpp>

using namespace Board;
using namespace std::chrono_literals;

using SpiMaster = SpiMaster3;
using Cs = GpioOutputA4;
using Sck = GpioB3;
using Miso = GpioB4;
using Mosi = GpioB5;

constexpr uint32_t WarmupReadCount = 1'000;
constexpr uint32_t ReadCount = 10'000;

using Encoder = modm::Aeat9922<SpiMaster, Cs, 18>;

Encoder::Data data{};
Encoder encoder{data};

static uint32_t
frequencyFromNanoseconds(uint32_t ns)
{
	return (ns == 0) ? 0 : uint32_t(1'000'000'000ull / ns);
}

int
main()
{
	SCB_DisableICache();
    SCB_DisableDCache();
	
	Board::initialize();
	Board::Leds::setOutput();

	Cs::setOutput(modm::Gpio::High);

	SpiMaster::connect<Sck::Sck, Miso::Miso, Mosi::Mosi>();
	SpiMaster::initialize<Board::SystemClock, 12.5_MHz>();

	MODM_LOG_INFO << "AEAT-9922 speed test" << modm::endl;
	MODM_LOG_INFO << "SPI3: SCK=PB3, MISO=PB4, MOSI=PB5, CS=PA4" << modm::endl;
	MODM_LOG_INFO << "Warm-up reads: " << static_cast<unsigned long>(WarmupReadCount) << modm::endl;
	MODM_LOG_INFO << "Reads: " << static_cast<unsigned long>(ReadCount) << modm::endl;
	MODM_LOG_INFO << "Timing source: DWT cycle counter, 2.5 ns resolution" << modm::endl;
	// MODM_LOG_INFO.flush();

	while (!encoder.initialize())
	{
		Board::LedRed::toggle();
		modm::delay(250ms);
	}

	Board::LedRed::reset();
	Board::LedGreen::set();

	MODM_LOG_INFO << "Running warm-up..." << modm::endl;
	for (uint32_t ii = 0; ii < WarmupReadCount; ++ii)
	{
		encoder.read();
	}

	MODM_LOG_INFO << "Starting benchmark..." << modm::endl;
	// MODM_LOG_INFO.flush();
	modm::delay(250ms);

	constexpr uint32_t CpuFreqMHz = Board::SystemClock::Frequency / 1'000'000;

	uint32_t minCycles = std::numeric_limits<uint32_t>::max();
	uint32_t maxCycles = 0;
	uint64_t totalCycles = 0;
	uint32_t readErrors = 0;

	for (uint32_t ii = 0; ii < ReadCount; ++ii)
	{
		const uint32_t start = DWT->CYCCNT;
		const bool readOk = encoder.read();
		const uint32_t elapsed = DWT->CYCCNT - start;

		if (elapsed < minCycles) minCycles = elapsed;
		if (elapsed > maxCycles) maxCycles = elapsed;
		totalCycles += elapsed;
		if (!readOk) ++readErrors;
	}

	const uint32_t minNs = uint32_t(uint64_t(minCycles) * 1000 / CpuFreqMHz);
	const uint32_t maxNs = uint32_t(uint64_t(maxCycles) * 1000 / CpuFreqMHz);
	const uint32_t meanNs = uint32_t((totalCycles * 1000 / CpuFreqMHz + ReadCount / 2) / ReadCount);
	const uint32_t minFrequencyHz = frequencyFromNanoseconds(maxNs);
	const uint32_t meanFrequencyHz =
		(totalCycles == 0) ? 0 : uint32_t(uint64_t(ReadCount) * CpuFreqMHz * 1'000'000ull / totalCycles);
	const uint32_t maxFrequencyHz = frequencyFromNanoseconds(minNs);

	modm::delay(250ms);
	MODM_LOG_INFO << "\nAEAT-9922 speed test result" << modm::endl;
	modm::delay(50ms);
	MODM_LOG_INFO << "  errors:      " << static_cast<unsigned long>(readErrors) << modm::endl;
	MODM_LOG_INFO << "  min time:    " << static_cast<unsigned long>(minNs) << " ns" << modm::endl;
	MODM_LOG_INFO << "  mean time:   " << static_cast<unsigned long>(meanNs) << " ns" << modm::endl;
	MODM_LOG_INFO << "  max time:    " << static_cast<unsigned long>(maxNs) << " ns" << modm::endl;
	MODM_LOG_INFO << "  min freq:    " << static_cast<unsigned long>(minFrequencyHz) << " Hz" << modm::endl;
	MODM_LOG_INFO << "  mean freq:   " << static_cast<unsigned long>(meanFrequencyHz) << " Hz" << modm::endl;
	MODM_LOG_INFO << "  max freq:    " << static_cast<unsigned long>(maxFrequencyHz) << " Hz" << modm::endl;
	// MODM_LOG_INFO.flush();

	Board::LedBlue::set();

	while (true)
	{
		modm::delay(1s);
	}

	return 0;
}
