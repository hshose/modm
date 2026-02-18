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

void
printDriverState()
{
	if (const auto chipId = imu.getChipId()) {
		MODM_LOG_INFO.printf("Chip ID: 0x%02x\n", *chipId);
	}
	if (const auto errors = imu.getErrors()) {
		MODM_LOG_INFO.printf("Errors: fatal=%u internal=0x%02x fifo=%u aux=%u\n",
							 errors->fatalError, errors->internalError, errors->fifoError, errors->auxError);
	}
	if (const auto status = imu.getStatus()) {
		MODM_LOG_INFO.printf("Status: acc=%u gyro=%u aux=%u cmd=%u auxBusy=%u\n",
							 status->accDataReady, status->gyroDataReady, status->auxDataReady,
							 status->commandReady, status->auxBusy);
	}
	if (const auto internalStatus = imu.getInternalStatus()) {
		MODM_LOG_INFO.printf("Internal Status: msg=%u remapErr=%u odr50Err=%u\n",
							 static_cast<uint8_t>(internalStatus->message),
							 internalStatus->axesRemapError, internalStatus->odr50HzError);
	}
	if (const auto temperature = imu.getTemperature()) {
		if (temperature->valid) {
			MODM_LOG_INFO.printf("Temperature: %.2f C\n", temperature->celsius);
		}
		else {
			MODM_LOG_INFO << "Temperature: invalid\n";
		}
	}
	if (const auto interruptStatus = imu.getInterruptStatus()) {
		MODM_LOG_INFO.printf("Interrupt Status: ff=%u fwm=%u err=%u aux=%u gyro=%u acc=%u\n",
							 interruptStatus->fifoFull, interruptStatus->fifoWatermark,
							 interruptStatus->error, interruptStatus->auxDataReady,
							 interruptStatus->gyroDataReady, interruptStatus->accDataReady);
	}
	if (const auto intMapData = imu.getInterruptMapData()) {
		MODM_LOG_INFO.printf("INT map: int1(drdy=%u,err=%u) int2(drdy=%u,err=%u)\n",
							 intMapData->int1DataReady, intMapData->int1Error,
							 intMapData->int2DataReady, intMapData->int2Error);
	}
	if (const auto powerConf = imu.getPowerConfiguration()) {
		MODM_LOG_INFO.printf("Power Config: 0x%02x\n", powerConf->value);
	}
	if (const auto powerCtrl = imu.getPowerControl()) {
		MODM_LOG_INFO.printf("Power Ctrl: 0x%02x\n", powerCtrl->value);
	}
	if (const auto internalError = imu.getInternalError()) {
		MODM_LOG_INFO.printf("Internal Error: long=%u fatal=%u featDisabled=%u\n",
							 internalError->longProcessingTime,
							 internalError->fatalError,
							 internalError->featureEngineDisabled);
	}
	if (const auto pullUp = imu.getPullUpConfiguration()) {
		MODM_LOG_INFO.printf("Pull-up config: %u\n", static_cast<uint8_t>(*pullUp));
	}
	if (const auto gyroCrt = imu.getGyroCrtConfig()) {
		MODM_LOG_INFO.printf("Gyro CRT: running=%u ready=%u\n",
							 gyroCrt->running, gyroCrt->readyForDownload);
	}
	if (const auto nvmCrt = imu.getNvmCrtEnabled()) {
		MODM_LOG_INFO.printf("NVM CRT: %u\n", *nvmCrt);
	}
	if (const auto ifConf = imu.getInterfaceConfig()) {
		MODM_LOG_INFO.printf("IF conf: spi=%u oisSpi=%u ois=%u aux=%u\n",
							 static_cast<uint8_t>(ifConf->primarySpiMode),
							 static_cast<uint8_t>(ifConf->oisSpiMode),
							 ifConf->oisEnabled, ifConf->auxEnabled);
	}
	if (const auto drv = imu.getDriveConfig()) {
		MODM_LOG_INFO.printf("Drive: d1=%u b1=%u d2=%u b2=%u\n",
							 static_cast<uint8_t>(drv->ioPadDrv1), drv->ioPadI2cBoost1,
							 static_cast<uint8_t>(drv->ioPadDrv2), drv->ioPadI2cBoost2);
	}
	if (const auto accOffsets = imu.getAccOffsets()) {
		MODM_LOG_INFO.printf("Acc offsets: x=%u y=%u z=%u\n",
							 accOffsets->x, accOffsets->y, accOffsets->z);
	}
	if (const auto gyroOffsets = imu.getGyroOffsets()) {
		MODM_LOG_INFO.printf("Gyro offsets: x=%u y=%u z=%u offEn=%u gainEn=%u\n",
							 gyroOffsets->x, gyroOffsets->y, gyroOffsets->z,
							 gyroOffsets->offsetEnabled, gyroOffsets->gainEnabled);
	}
}

bool
configureDriver()
{
	while (!imu.initialize()) {
		MODM_LOG_ERROR << "Initialization failed, retrying ...\n";
		modm::delay(500ms);
	}

	bool ok = true;
	ok &= imu.setAccRate(Imu::AccRate::Rate100Hz_Normal);
	ok &= imu.setAccRange(Imu::AccRange::Range2g);
	ok &= imu.setGyroRate(Imu::GyroRate::Rate100Hz_Normal);
	ok &= imu.setGyroRange(Imu::GyroRange::Range2000dps);

	Imu::InterruptIoControl int1{};
	int1.level = Imu::InterruptOutputLevel::ActiveHigh;
	int1.outputType = Imu::InterruptOutputType::PushPull;
	int1.outputEnable = true;
	int1.inputEnable = false;
	ok &= imu.setInt1IoControl(int1);

	Imu::InterruptIoControl int2{};
	int2.level = Imu::InterruptOutputLevel::ActiveHigh;
	int2.outputType = Imu::InterruptOutputType::PushPull;
	int2.outputEnable = false;
	int2.inputEnable = false;
	ok &= imu.setInt2IoControl(int2);

	ok &= imu.setInterruptLatch(Imu::InterruptLatch::None);

	Imu::InterruptMapData intMap{};
	intMap.int1DataReady = true;
	intMap.int1Error = true;
	ok &= imu.setInterruptMapData(intMap);

	Imu::ErrorInterruptMask errMask{};
	errMask.fatalError = true;
	errMask.internalError = true;
	errMask.fifoError = true;
	errMask.auxError = true;
	ok &= imu.setErrorInterruptMask(errMask);

	ok &= imu.setPowerControl(
		Imu::PowerControl::Accelerometer |
		Imu::PowerControl::Gyroscope |
		Imu::PowerControl::Temperature);

	if (const auto powerConf = imu.getPowerConfiguration()) {
		ok &= imu.setPowerConfiguration(*powerConf);
	}
	if (const auto pullUp = imu.getPullUpConfiguration()) {
		ok &= imu.setPullUpConfiguration(*pullUp);
	}
	if (const auto gyroCrt = imu.getGyroCrtConfig()) {
		Imu::GyroCrtConfig config{};
		config.running = gyroCrt->running;
		ok &= imu.setGyroCrtConfig(config);
	}
	if (const auto nvmCrt = imu.getNvmCrtEnabled()) {
		ok &= imu.setNvmCrtEnabled(*nvmCrt);
	}
	if (const auto ifConf = imu.getInterfaceConfig()) {
		ok &= imu.setInterfaceConfig(*ifConf);
	}
	if (const auto drv = imu.getDriveConfig()) {
		ok &= imu.setDriveConfig(*drv);
	}
	if (const auto accOffsets = imu.getAccOffsets()) {
		ok &= imu.setAccOffsets(*accOffsets);
	}
	if (const auto gyroOffsets = imu.getGyroOffsets()) {
		ok &= imu.setGyroOffsets(*gyroOffsets);
	}

	ok &= imu.sendCommand(Imu::Command::FlushFifo);
	return ok;
}

int main()
{
	Board::initialize();
	Leds::setOutput();
	I2c::connect<Scl::Scl, Sda::Sda>(I2c::PullUps::Internal);
	I2c::initialize<Board::SystemClock, 100_kHz, 10_pct>();

	MODM_LOG_INFO << "BMI270 I2C Test\n";

	if (!configureDriver()) {
		MODM_LOG_ERROR << "Configuration failed!\n";
	}
	printDriverState();

	uint32_t counter = 0;
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
		if ((counter % 10u) == 0u) {
			if (const auto temperature = imu.getTemperature(); temperature and temperature->valid) {
				MODM_LOG_INFO.printf("Temp [C]: %.2f\n", temperature->celsius);
			}
			if (const auto interruptStatus = imu.getInterruptStatus(); interruptStatus) {
				MODM_LOG_INFO.printf("Int drdy(acc=%u gyro=%u) err=%u\n",
									 interruptStatus->accDataReady, interruptStatus->gyroDataReady,
									 interruptStatus->error);
			}
		}

		Board::LedGreen::toggle();
		modm::delay(100ms);
		++counter;
	}

	return 0;
}
