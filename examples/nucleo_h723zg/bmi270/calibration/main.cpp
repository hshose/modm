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
#include <modm/processing.hpp>

using namespace Board;

using I2c = I2cMaster1;
using Scl = GpioB8;  // D15
using Sda = GpioB9;  // D14

using Transport = modm::Bmi270I2cTransport<I2c>;
using Imu = modm::Bmi270<Transport>;

Imu imu{static_cast<uint8_t>(0x68)};

namespace
{

struct CalibrationSnapshot
{
	Imu::AccOffsets accOffsets;
	Imu::GyroOffsets gyroOffsets;
	Imu::GyroGainUpdate gyroGain;
	Imu::GyroUserGain gyroUserGainOut;
};

bool
initializeImu()
{
	while (!imu.initialize()) {
		MODM_LOG_ERROR << "Initialization failed, retrying..." << modm::endl;
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

void
printAccOffsets(const char* label, const Imu::AccOffsets& offsets)
{
	MODM_LOG_INFO << label
				  << " x=" << int(offsets.x)
				  << " y=" << int(offsets.y)
				  << " z=" << int(offsets.z)
				  << modm::endl;
}

void
printGyroOffsets(const char* label, const Imu::GyroOffsets& offsets)
{
	MODM_LOG_INFO << label
				  << " x=" << offsets.x
				  << " y=" << offsets.y
				  << " z=" << offsets.z
				  << " offEn=" << offsets.offsetEnabled
				  << " gainEn=" << offsets.gainEnabled
				  << modm::endl;
}

void
printGyroGain(const char* label, const Imu::GyroGainUpdate& gain)
{
	MODM_LOG_INFO.printf(
		"%s x=0x%03x y=0x%03x z=0x%03x enable=%u\n",
		label,// you could save the calibration to reserved flash on the microcontroller now
		gain.ratioX,
		gain.ratioY,
		gain.ratioZ,
		gain.enable);
}

void
printGyroUserGain(const char* label, const Imu::GyroUserGain& gain)
{
	MODM_LOG_INFO << label
				  << " x=" << int(gain.x)
				  << " y=" << int(gain.y)
				  << " z=" << int(gain.z)
				  << modm::endl;
}

void
printGyroGainStatus(const char* label, const Imu::GyroGainStatus& status)
{
	MODM_LOG_INFO << label
				  << " satX=" << status.saturationX
				  << " satY=" << status.saturationY
				  << " satZ=" << status.saturationZ
				  << " trigStatus=" << int(status.triggerStatus)
				  << modm::endl;
}

bool
equalAccOffsets(const Imu::AccOffsets& lhs, const Imu::AccOffsets& rhs)
{
	return (lhs.x == rhs.x) and (lhs.y == rhs.y) and (lhs.z == rhs.z);
}

bool
equalGyroOffsets(const Imu::GyroOffsets& lhs, const Imu::GyroOffsets& rhs)
{
	return (lhs.x == rhs.x) and
		   (lhs.y == rhs.y) and
		   (lhs.z == rhs.z) and
		   (lhs.offsetEnabled == rhs.offsetEnabled) and
		   (lhs.gainEnabled == rhs.gainEnabled);
}

bool
equalGyroGain(const Imu::GyroGainUpdate& lhs, const Imu::GyroGainUpdate& rhs)
{
	return (lhs.ratioX == rhs.ratioX) and
		   (lhs.ratioY == rhs.ratioY) and
		   (lhs.ratioZ == rhs.ratioZ) and
		   (lhs.enable == rhs.enable);
}

bool
equalGyroUserGain(const Imu::GyroUserGain& lhs, const Imu::GyroUserGain& rhs)
{
	return (lhs.x == rhs.x) and
		   (lhs.y == rhs.y) and
		   (lhs.z == rhs.z);
}

}  // namespace

int
main()
{

	/**
	 * The BMI270 has a bug in the non-volatile memory that resets the offset
	 * compensation (foc) values to zero when saving component retrim (crt) values.
	 * As the non-volatile memory on the BMI270 has a maximum of 5 write cycles
	 * anyways, this demonstrate how to perform the foc and crt calibration and
	 * reapply the values from software after resetting the BMI270. The intent
	 * is to save calibration on the microcontroller side, e.g., to the reserved
	 * flash section.
	 */

	Board::initialize();
	Leds::setOutput();
	I2c::connect<Scl::Scl, Sda::Sda>(I2c::PullUps::Internal);
	I2c::initialize<Board::SystemClock, 1_MHz, 10_pct>();

	MODM_LOG_INFO << "BMI270 calibration example (FOC + CRT + restore)" << modm::endl;

	MODM_LOG_ERROR << "Resetting to zero out any previous calibration values." << modm::endl;
	if (!imu.reset()) {
		MODM_LOG_ERROR << "Soft reset command failed" << modm::endl;
	}

	if (!initializeImu()) {
		MODM_LOG_ERROR << "Initial IMU configuration failed" << modm::endl;
	}

	if (const auto userGain = imu.getGyroUserGain()) {
		printGyroUserGain("Pre-CRT gyro user gain out", *userGain);
	}

	MODM_LOG_INFO << "Starting component retrim (CRT)..." << modm::endl;
	const auto crtGain = imu.doCrtAndReadGainUpdate();
	if (!crtGain) {
		MODM_LOG_ERROR << "CRT failed" << modm::endl;
	}
	else {
		MODM_LOG_INFO << "CRT completed" << modm::endl;
		printGyroGain("Downloaded CRT gain", *crtGain);
	}

	if (const auto gainStatus = imu.getGyroGainStatus()) {
		printGyroGainStatus("CRT gain status", *gainStatus);
	}
	if (const auto userGain = imu.getGyroUserGain()) {
		printGyroUserGain("CRT gyro user gain out", *userGain);
	}

	const auto gainUpdate = imu.getGyroGainUpdate();
	const auto userGainOut = imu.getGyroUserGain();

	
	MODM_LOG_INFO << "FOC: Keep sensor stable with +Z aligned to gravity" << modm::endl;

	Imu::AccelFocTarget accelTarget{};
	accelTarget.axis = Imu::FocAxis::Z;
	accelTarget.negative = false;

	MODM_LOG_INFO << "Accel FOC..." << modm::endl;
	const bool accelFocOk = imu.performAccelFoc(accelTarget);
	
	MODM_LOG_INFO << "Gyro FOC..." << modm::endl;
	const bool gyroFocOk = imu.performGyroFoc();

	MODM_LOG_INFO << "FOC result: accel=" << accelFocOk
				  << " gyro=" << gyroFocOk << modm::endl;

	const auto accOffsets = imu.getAccOffsets();
	const auto gyroOffsets = imu.getGyroOffsets();

	CalibrationSnapshot savedCalibration{};
	savedCalibration.accOffsets   	 = accOffsets.value();
	savedCalibration.gyroOffsets  	 = gyroOffsets.value();
	savedCalibration.gyroGain     	 = gainUpdate.value();
	savedCalibration.gyroUserGainOut = userGainOut.value();

	printAccOffsets("Saved accel offsets", savedCalibration.accOffsets);
	printGyroOffsets("Saved gyro offsets", savedCalibration.gyroOffsets);
	printGyroGain("Saved gyro gain", savedCalibration.gyroGain);
	printGyroUserGain("Saved gyro user gain out", savedCalibration.gyroUserGainOut);
	
	
	
	// you could save the calibration to reserved flash on the microcontroller now

	MODM_LOG_INFO << "Soft resetting sensor..." << modm::endl;
	if (!imu.reset()) {
		MODM_LOG_ERROR << "Soft reset command failed" << modm::endl;
	}

	if (!initializeImu()) {
		MODM_LOG_ERROR << "Re-initialization after reset failed" << modm::endl;
	}

	if (const auto userGainBeforeRestore = imu.getGyroUserGain()) {
		printGyroUserGain("After reset, before restore gyro user gain out", *userGainBeforeRestore);
	}

	MODM_LOG_INFO << "Restoring saved calibration values..." << modm::endl;
	bool writeOk = true;
	writeOk &= imu.setAccOffsets(savedCalibration.accOffsets);
	writeOk &= imu.setGyroOffsets(savedCalibration.gyroOffsets);
	writeOk &= imu.setGyroGainUpdate(savedCalibration.gyroGain);
	writeOk &= imu.setGyroUserGain(savedCalibration.gyroUserGainOut);

	if (!writeOk) {
		MODM_LOG_ERROR << "Writing calibration values failed" << modm::endl;
	}

	const auto restoredAccOffsets = imu.getAccOffsets();
	const auto restoredGyroOffsets = imu.getGyroOffsets();
	const auto restoredGainUpdate = imu.getGyroGainUpdate();

	bool accMatch = false;
	bool gyroMatch = false;
	bool gainMatch = false;
	bool userGainOutMatch = false;

	if (restoredAccOffsets) {
		accMatch = equalAccOffsets(savedCalibration.accOffsets, *restoredAccOffsets);
	}
	if (restoredGyroOffsets) {
		gyroMatch = equalGyroOffsets(savedCalibration.gyroOffsets, *restoredGyroOffsets);
	}
	if (restoredGainUpdate) {
		gainMatch = equalGyroGain(savedCalibration.gyroGain, *restoredGainUpdate);
	}

	const auto restoredUserGain = imu.getGyroUserGain();
	if (restoredUserGain) {
		userGainOutMatch = equalGyroUserGain(savedCalibration.gyroUserGainOut, *restoredUserGain);
	}

	if (restoredAccOffsets) {
		printAccOffsets("Restored accel offsets", *restoredAccOffsets);
	}
	if (restoredGyroOffsets) {
		printGyroOffsets("Restored gyro offsets", *restoredGyroOffsets);
	}
	if (restoredGainUpdate) {
		printGyroGain("Restored gyro gain", *restoredGainUpdate);
	}
	if (restoredUserGain) {
		printGyroUserGain("Restored gyro user gain out", *restoredUserGain);
	}

	MODM_LOG_INFO << "Verification: acc=" << accMatch
				  << " gyro=" << gyroMatch
				  << " gain=" << gainMatch
				  << " userGainOut=" << userGainOutMatch
				  << modm::endl;

	const bool allMatch = writeOk and accMatch and gyroMatch and gainMatch and userGainOutMatch;
	MODM_LOG_INFO << (allMatch ? "Calibration restore PASSED" : "Calibration restore FAILED")
				  << modm::endl;

	modm::PeriodicTimer printTimer{1s};
	bool hasSample = false;
	uint32_t lastSampleTimeUs = 0;
	float rollDeg = 0.f;
	float pitchDeg = 0.f;
	float yawDeg = 0.f;
	modm::Vector3f lastAcc{};
	modm::Vector3f lastGyro{};

	while (true)
	{
		if (const auto data = imu.readData()) {
			const uint32_t nowUs = modm::PreciseClock::now().time_since_epoch().count();
			lastAcc = data->acc.getFloat();
			lastGyro = data->gyro.getFloat();

			if (hasSample) {
				const uint32_t deltaUs = nowUs - lastSampleTimeUs;
				const float dtSeconds = float(deltaUs) * 1e-6f;
				rollDeg += lastGyro[0] * dtSeconds;
				pitchDeg += lastGyro[1] * dtSeconds;
				yawDeg += lastGyro[2] * dtSeconds;
			}

			lastSampleTimeUs = nowUs;
			hasSample = true;
		}

		if (printTimer.execute()) {
			if (hasSample) {
				MODM_LOG_INFO << "Acc [mg] x=" << lastAcc[0]
							  << " y=" << lastAcc[1]
							  << " z=" << lastAcc[2]
							  << modm::endl;
				MODM_LOG_INFO << "Gyr [deg/s] x=" << lastGyro[0]
							  << " y=" << lastGyro[1]
							  << " z=" << lastGyro[2]
							  << modm::endl;
				MODM_LOG_INFO << "Ori [deg] roll=" << rollDeg
							  << " pitch=" << pitchDeg
							  << " yaw=" << yawDeg
							  << modm::endl;
			}
			else {
				MODM_LOG_ERROR << "readData failed (no valid sample yet)" << modm::endl;
			}

			if (allMatch) {
				Board::LedGreen::toggle();
			}
			else {
				Board::LedRed::toggle();
			}
		}
	}

	return 0;
}
