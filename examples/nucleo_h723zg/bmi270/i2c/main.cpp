/*
 * Copyright (c) 2026, Christopher Durand
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

using Transport = modm::Bmi270I2cTransport<I2c>;
using Imu = modm::Bmi270<Transport>;

Imu imu{static_cast<uint8_t>(0x68)};

int main()
{
	Board::initialize();
	Leds::setOutput();
	I2c::connect<Scl::Scl, Sda::Sda>(I2c::PullUps::Internal);
	I2c::initialize<Board::SystemClock, 100_kHz, 10_pct>();

	MODM_LOG_INFO << "BMI270 I2C Test\n";

	while (!imu.initialize()) {
		MODM_LOG_ERROR << "Initialization failed, retrying ...\n";
		modm::delay(500ms);
	}

	while (true)
	{
		if (imu.readAccDataReady() and imu.readGyroDataReady()) {
			const auto data = imu.readData();
			if (data) {
				const modm::Vector3f acc = data->acc.getFloat();
				const modm::Vector3f gyro = data->gyro.getFloat();
				MODM_LOG_INFO.printf("Acc  [mg]\tx: %6.1f\ty: %6.1f\tz: %6.1f\n", acc[0], acc[1], acc[2]);
				MODM_LOG_INFO.printf("Gyro [deg/s]\tx: %6.2f\ty: %6.2f\tz: %6.2f\n", gyro[0], gyro[1], gyro[2]);
			}
		}

		Board::LedGreen::toggle();
		modm::delay(100ms);
	}

	return 0;
}
