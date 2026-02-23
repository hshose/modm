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

using Transport = modm::Bmi270I2cTransport<I2c>;
using Imu = modm::Bmi270<Transport>;

Imu imu{static_cast<uint8_t>(0x68)};

bool
configureDriver()
{
	while (!imu.initialize()) {
		MODM_LOG_ERROR << "Initialization failed, retrying ..." << modm::endl;
		modm::this_fiber::sleep_for(500ms);
	}

	bool ok = true;
	ok &= imu.setAccRate(Imu::AccRate::Rate800Hz_Normal);
	ok &= imu.setAccRange(Imu::AccRange::Range4g);
	ok &= imu.setGyroRate(Imu::GyroRate::Rate800Hz_Normal);
	ok &= imu.setGyroRange(Imu::GyroRange::Range2000dps);

	ok &= imu.setPowerControl(
		Imu::PowerControl::Accelerometer |
		Imu::PowerControl::Gyroscope |
		Imu::PowerControl::Temperature);
	return ok;
}

int
main()
{
	Board::initialize();
	Leds::setOutput();
	I2c::connect<Scl::Scl, Sda::Sda>(I2c::PullUps::Internal);
	I2c::initialize<Board::SystemClock, 1_MHz, 10_pct>();

	MODM_LOG_INFO << "BMI270 I2C calibration example" << modm::endl;

	if (!configureDriver()) {
		MODM_LOG_ERROR << "Configuration failed!" << modm::endl;
	}


	MODM_LOG_INFO << "BMI270 FOC test (I2C)\n";

	while (!imu.initialize()) {
		MODM_LOG_ERROR << "Initialization failed, retrying...\n";
		modm::delay(500ms);
	}

	MODM_LOG_INFO << "Keep sensor stable with +Z aligned to gravity\n";

	Imu::AccelFocTarget accelTarget{};
	accelTarget.axis = Imu::FocAxis::Z;
	accelTarget.negative = false;

	const bool accOk = imu.performAccelFoc(accelTarget);
	const bool gyroOk = imu.performGyroFoc();

	MODM_LOG_INFO.printf("FOC result: accel=%u gyro=%u\n", accOk, gyroOk);

	if (const auto accOffsets = imu.getAccOffsets()) {
		MODM_LOG_INFO.printf("Accel offsets: x=%u y=%u z=%u\n",
							 accOffsets->x, accOffsets->y, accOffsets->z);
	}
	if (const auto gyroOffsets = imu.getGyroOffsets()) {
		MODM_LOG_INFO.printf("Gyro offsets: x=%u y=%u z=%u offEn=%u gainEn=%u\n",
							 gyroOffsets->x, gyroOffsets->y, gyroOffsets->z,
							 gyroOffsets->offsetEnabled, gyroOffsets->gainEnabled);
	}

	while (true)
	{
		if (const auto data = imu.readData()) {
			const auto acc = data->acc.getFloat();
			const auto gyro = data->gyro.getFloat();
			MODM_LOG_INFO.printf("Acc [mg]\t%6.1f\t%6.1f\t%6.1f\n", acc[0], acc[1], acc[2]);
			MODM_LOG_INFO.printf("Gyr [dps]\t%6.2f\t%6.2f\t%6.2f\n", gyro[0], gyro[1], gyro[2]);
		}

		Board::LedGreen::toggle();
		modm::delay(200ms);
	}

	return 0;
}
