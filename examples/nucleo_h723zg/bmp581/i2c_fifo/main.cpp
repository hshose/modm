/*
 * Copyright (c) 2026, Joel Schulz-Andres
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include <array>
#include <modm/board.hpp>
#include <modm/driver/pressure/bmp581.hpp>

using namespace Board;

using I2c = I2cMaster1;
using Scl = GpioB8;
using Sda = GpioB9;
using BaroInt = GpioD14;

using Transport = modm::Bmp581I2cTransport<I2c>;
using Baro = modm::Bmp581<Transport>;

// BMP581 I2C address (0x46 with SDO to GND, 0x47 with SDO to VDDIO)
constexpr uint8_t BaroAddress = 0x47;
constexpr auto FifoFrameSelection = Baro::FifoFrameSelection::PressureTemperature;
constexpr uint8_t FifoWatermark = 8;

Baro baro{BaroAddress};
std::array<modm::bmp581::Data, 16> fifoFrames{};
volatile bool fifoThresholdInterrupt = false;

bool
initializeBaro()
{
	if (!baro.initialize()) {
		MODM_LOG_ERROR << "BMP581 initialization failed!\n";
		return false;
	}

	if (!baro.setOversampling(Baro::Osr::X64, Baro::Osr::X8, true)) {
		MODM_LOG_ERROR << "Failed to set oversampling!\n";
		return false;
	}

	if (!baro.setOdr(Baro::Odr::Hz50)) {
		MODM_LOG_ERROR << "Failed to set ODR!\n";
		return false;
	}

	if (!baro.setIirFilter(Baro::IirFilter::Coef1, Baro::IirFilter::Coef1)) {
		MODM_LOG_ERROR << "Failed to set IIR filter!\n";
		return false;
	}

	if (!baro.setFifoConfig(Baro::FifoMode::Streaming, FifoFrameSelection,
	                        Baro::FifoDecimation::NoDownsampling, true, true)) {
		MODM_LOG_ERROR << "Failed to configure FIFO!\n";
		return false;
	}

	if (!baro.setFifoWatermark(FifoWatermark)) {
		MODM_LOG_ERROR << "Failed to set FIFO watermark!\n";
		return false;
	}

	const auto intConfig = Baro::IntConfig::Enable | Baro::IntConfig::Polarity;
	if (!baro.setIntConfig(intConfig)) {
		MODM_LOG_ERROR << "Failed to set interrupt config!\n";
		return false;
	}

	if (!baro.setIntSource(Baro::IntSource::FifoThresholdEnable)) {
		MODM_LOG_ERROR << "Failed to set interrupt source!\n";
		return false;
	}

	if (!baro.setPowerMode(Baro::PowerMode::Normal)) {
		MODM_LOG_ERROR << "Failed to set power mode!\n";
		return false;
	}

	return true;
}

int
main()
{
	Board::initialize();
	Leds::setOutput();

	I2c::connect<Scl::Scl, Sda::Sda>(I2c::PullUps::Internal);
	I2c::initialize<Board::SystemClock, 400_kHz>();

	BaroInt::setInput(BaroInt::InputType::PullDown);
	Exti::connect<BaroInt>(Exti::Trigger::RisingEdge, [](auto) {
		fifoThresholdInterrupt = true;
		LedYellow::toggle();
	});

	MODM_LOG_INFO << "BMP581 FIFO Threshold Example\n";
	MODM_LOG_INFO << "============================\n\n";

	while (!initializeBaro()) {
		LedRed::toggle();
		MODM_LOG_ERROR << "Retrying initialization...\n";
		modm::delay(250ms);
	}

	if (const auto chipId = baro.readChipId(); chipId) {
		MODM_LOG_INFO.printf("Chip ID: 0x%02X (expected 0x%02X)\n",
		                     static_cast<unsigned int>(*chipId),
		                     static_cast<unsigned int>(Baro::ChipId));
	}

	MODM_LOG_INFO.printf("FIFO watermark: %u frames\n", static_cast<unsigned int>(FifoWatermark));
	MODM_LOG_INFO << "Starting FIFO interrupt-driven readout...\n\n";

	uint32_t burstCount = 0;
	uint32_t totalFrames = 0;
	uint32_t readErrorCount = 0;

	while (true)
	{
		if (!fifoThresholdInterrupt) {
			continue;
		}
		fifoThresholdInterrupt = false;

		const auto intStatus = baro.readIntStatus();
		if (!intStatus || !(*intStatus & Baro::IntStatus::FifoThreshold)) {
			continue;
		}

		std::size_t framesRead = 0;
		if (!baro.readFifoData(fifoFrames.data(), fifoFrames.size(), framesRead)) {
			readErrorCount++;
			if ((readErrorCount % 20u) == 0u) {
				MODM_LOG_ERROR << "Failed to read FIFO data!\n";
			}
			LedRed::set();
			continue;
		}

		if (framesRead == 0) {
			continue;
		}

		float temperatureAverage = 0.f;
		float pressureAverageHpa = 0.f;
		for (std::size_t ii = 0; ii < framesRead; ++ii)
		{
			temperatureAverage += fifoFrames[ii].getTemperature();
			pressureAverageHpa += fifoFrames[ii].getPressureHpa();
		}
		temperatureAverage /= static_cast<float>(framesRead);
		pressureAverageHpa /= static_cast<float>(framesRead);

		const auto &lastFrame = fifoFrames[framesRead - 1];
		burstCount++;
		totalFrames += static_cast<uint32_t>(framesRead);
		readErrorCount = 0;

		if ((burstCount % 5u) == 0u) {
			MODM_LOG_INFO.printf("Burst #%lu: %lu frames (total %lu)\n",
					         static_cast<unsigned long>(burstCount),
					         static_cast<unsigned long>(framesRead),
					         static_cast<unsigned long>(totalFrames));
			MODM_LOG_INFO.printf("  Avg Temp: %7.3f C\n", temperatureAverage);
			MODM_LOG_INFO.printf("  Avg Press:%8.3f hPa\n", pressureAverageHpa);
			MODM_LOG_INFO.printf("  Last:     %7.3f C, %8.3f hPa\n\n",
					         lastFrame.getTemperature(),
					         lastFrame.getPressureHpa());
		}

		LedGreen::toggle();
	}

	return 0;
}
