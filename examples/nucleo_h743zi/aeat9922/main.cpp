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

#include <modm/board.hpp>
#include <modm/driver/encoder/aeat9922.hpp>

using namespace Board;
using namespace std::chrono_literals;

using SpiMaster = SpiMaster3;
using Cs = GpioOutputA4;
using Sck = GpioB3;
using Miso = GpioB4;
using Mosi = GpioB5;

using Encoder = modm::Aeat9922<SpiMaster, Cs, 18, modm::aeat9922::AxisMode::OffAxis>;

Encoder::Data data{0};
Encoder encoder{data};

int
main()
{
	SCB_DisableICache();
    SCB_DisableDCache();
	
	Board::initialize();
	Board::Leds::setOutput();

	Cs::setOutput(modm::Gpio::High);

	SpiMaster::connect<Sck::Sck, Miso::Miso, Mosi::Mosi>();
	SpiMaster::initialize<Board::SystemClock, 12'500'000>();

	MODM_LOG_INFO << "==========AEAT-9922 Test==========" << modm::endl;
	MODM_LOG_INFO << "SPI3: SCK=PB3, MISO=PB4, MOSI=PB5, CS=PA4" << modm::endl;

	while (!encoder.initialize())
	{
		Board::LedRed::toggle();
		MODM_LOG_INFO << "Waiting for encoder ready" << modm::endl;
		modm::delay(250ms);
	}

	Board::LedRed::reset();
	Board::LedGreen::set();
	MODM_LOG_INFO << "Encoder initialized" << modm::endl;
	MODM_LOG_INFO << "Axis mode: " << modm::endl;

	while (true)
	{
		const bool readOk = encoder.read();
		const auto status = encoder.readStatus();

		MODM_LOG_INFO << "New readout:" << modm::endl;
		MODM_LOG_INFO << "  angle degree: " << data.toDegree() << modm::endl;
		MODM_LOG_INFO << "     angle raw: " << data.data <<modm::endl;

		if (!readOk)
		{
			Board::LedRed::set();
			MODM_LOG_INFO << "  position read error flag set" << modm::endl;
		}
		else
		{
			Board::LedRed::reset();
			Board::LedBlue::toggle();
		}

		modm::delay(500ms);
	}

	return 0;
}
