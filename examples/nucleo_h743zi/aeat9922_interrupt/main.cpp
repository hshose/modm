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

#include "hardware.hpp"
#include "tasks/encoder_interrupt.hpp"

using namespace Board;
using namespace std::chrono_literals;

using EncoderHardware = Board::Encoder::Aeat9922;

constexpr uint32_t ReportDivider = EncoderHardware::SampleFrequency;

int
main()
{
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
			              << modm::endl;
		}
		modm::delay(10ms);
	}
}
