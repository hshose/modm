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

#include <cstddef>
#include <modm/board.hpp>
#include <modm/driver/inertial/bmi270.hpp>

using namespace Board;

using I2c = I2cMaster1;
using Scl = GpioB8;  // D15
using Sda = GpioB9;  // D14

using Transport = modm::Bmi270I2cTransport<I2c>;
using Imu = modm::Bmi270<Transport>;

Imu imu{static_cast<uint8_t>(0x68)};

namespace
{

bool
initializeImu()
{
	while (!imu.initialize())
	{
		MODM_LOG_ERROR << "Initialization failed, retrying..." << modm::endl;
		modm::delay(500ms);
	}
	return true;
}

void
printGainUpdate(const char* label, const Imu::GyroGainUpdate& gain)
{
	MODM_LOG_INFO.printf(
		"%s: x=0x%03x y=0x%03x z=0x%03x enable=%u\n",
		label,
		gain.ratioX,
		gain.ratioY,
		gain.ratioZ,
		gain.enable);
}

void
printGyroAverage(const char* label, std::size_t sampleCount = 64)
{
	float sumX = 0.f;
	float sumY = 0.f;
	float sumZ = 0.f;
	std::size_t validSamples = 0;

	for (std::size_t sample = 0; sample < sampleCount; ++sample)
	{
		modm::delay(10ms);
		const auto gyroData = imu.readGyroData();
		if (!gyroData) {
			continue;
		}

		const auto gyro = gyroData->getFloat();
		sumX += gyro[0];
		sumY += gyro[1];
		sumZ += gyro[2];
		++validSamples;
	}

	if (validSamples == 0) {
		MODM_LOG_ERROR << label << ": no valid samples" << modm::endl;
		return;
	}

	MODM_LOG_INFO.printf(
		"%s avg [deg/s] over %u samples: x=%0.6f y=%0.6f z=%0.6f\n",
		label,
		unsigned(validSamples),
		sumX / float(validSamples),
		sumY / float(validSamples),
		sumZ / float(validSamples));
}

} // namespace

int
main()
{
	Board::initialize();
	Leds::setOutput();
	I2c::connect<Scl::Scl, Sda::Sda>(I2c::PullUps::Internal);
	I2c::initialize<Board::SystemClock, 1_MHz, 10_pct>();

	MODM_LOG_INFO << "BMI270 CRT restore test (I2C)" << modm::endl;

	initializeImu();
	printGyroAverage("Before CRT");

	MODM_LOG_INFO << "Keep sensor stationary. Starting CRT..." << modm::endl;
	const auto downloadedCrtGain = imu.doCrtAndReadGainUpdate();
	if (!downloadedCrtGain) {
		MODM_LOG_ERROR << "CRT failed" << modm::endl;
	}
	else {
		MODM_LOG_INFO << "CRT completed successfully" << modm::endl;
		printGainUpdate("Downloaded CRT gain", *downloadedCrtGain);
	}

	printGyroAverage("After CRT");

	MODM_LOG_INFO << "Resetting sensor..." << modm::endl;
	if (!imu.sendCommand(Imu::Command::SoftReset)) {
		MODM_LOG_ERROR << "Soft reset command failed" << modm::endl;
	}
	modm::delay(50ms);
	initializeImu();

	printGyroAverage("After reset");

	if (downloadedCrtGain)
	{
		MODM_LOG_INFO << "Restoring CRT gain update values..." << modm::endl;
		if (!imu.setGyroGainUpdate(*downloadedCrtGain)) {
			MODM_LOG_ERROR << "setGyroGainUpdate failed" << modm::endl;
		}
		else if (!imu.applyGyroGainUpdate(true)) {
			MODM_LOG_ERROR << "applyGyroGainUpdate failed" << modm::endl;
		}
		else {
			MODM_LOG_INFO << "CRT gain values restored and applied" << modm::endl;
		}
	}

	printGyroAverage("After restored CRT gain");

	while (true)
	{
		Board::LedGreen::toggle();
		modm::delay(250ms);
	}

	return 0;
}
