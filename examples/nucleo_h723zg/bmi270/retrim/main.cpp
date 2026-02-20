/*
 * Copyright (c) 2026, Henrik Hose
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include <modm/board.hpp>
#include <modm/driver/inertial/bmi270.hpp>

using namespace Board;

using I2c = I2cMaster1;
using Scl = GpioB8;  // D15
using Sda = GpioB9;  // D14

using Int1 = GpioD15;  // D8
using Int2 = GpioF3;   // D9

using Transport = modm::Bmi270I2cTransport<I2c>;
using Imu = modm::Bmi270<Transport>;

Imu imu{static_cast<uint8_t>(0x68)};

int
main()
{
	Board::initialize();
	Leds::setOutput();
	I2c::connect<Scl::Scl, Sda::Sda>(I2c::PullUps::Internal);
	I2c::initialize<Board::SystemClock, 1_MHz, 10_pct>();

	MODM_LOG_INFO << "BMI270 CRT test (I2C)\n";

	while (!imu.initialize()) {
		MODM_LOG_ERROR << "Initialization failed, retrying...\n";
		modm::delay(500ms);
	}

	MODM_LOG_INFO << "Keep sensor stationary. Starting CRT...\n";
	if (imu.doCrt()) {
		MODM_LOG_INFO << "CRT completed successfully\n";
	}
	else {
		MODM_LOG_ERROR << "CRT failed\n";
	}

	if (const auto crt = imu.getGyroCrtConfig()) {
		MODM_LOG_INFO.printf("CRT status: running=%u ready_for_download=%u\n",
							 crt->running, crt->readyForDownload);
	}

	while (true)
	{
		Board::LedGreen::toggle();
		modm::delay(250ms);
	}

	return 0;
}
