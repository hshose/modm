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

#ifndef MODM_BMP581_HPP
#error "Don't include this file directly, use 'bmp581.hpp' instead!"
#endif

namespace modm
{

template<Bmp581Transport Transport>
template<typename... Args>
Bmp581<Transport>::Bmp581(Args... transportArgs)
	: Transport{transportArgs...}
{
}

template<Bmp581Transport Transport>
void
Bmp581<Transport>::waitForCommandGap()
{
	modm::this_fiber::sleep_for(std::chrono::microseconds{2});
}

template<Bmp581Transport Transport>
bool
Bmp581<Transport>::initialize()
{
	if (!Transport::initialize()) {
		return false;
	}

	if (!reset()) {
		return false;
	}

	const auto chipId = readChipId();
	if (!chipId || *chipId != ChipId) {
		return false;
	}

	const auto status = readStatus();
	if (!status) {
		return false;
	}
	// Check NvmReady is set and NvmError/NvmCmdError are clear
	if (!(*status & Status::NvmReady) ||
	    (*status & Status::NvmError) ||
	    (*status & Status::NvmCmdError)) {
		return false;
	}

	const auto intStatus = readIntStatus();
	if (!intStatus || !(*intStatus & IntStatus::PowerOnReset)) {
		return false;
	}

	// Disable deep standby mode
	if (!updateRegister(Register::OdrConfig, uint8_t(OdrConfig::DeepDis), uint8_t(OdrConfig::DeepDis))) {
		return false;
	}

	return true;
}

template<Bmp581Transport Transport>
bool
Bmp581<Transport>::reset()
{
	waitForCommandGap();

	if (!writeRegister(Register::Cmd, ResetCommand)) {
		return false;
	}

	// Wait for reset to complete (datasheet: 2ms typical)
	modm::this_fiber::sleep_for(std::chrono::milliseconds{5});

	// Re-initialize transport (needed for SPI mode)
	if (!Transport::initialize()) {
		return false;
	}

	// Dummy read to prime SPI state machine after reset (Bosch recommendation)
	// First SPI transaction after reset can be unreliable; discard result
	(void)readRegister(Register::ChipId);

	modm::this_fiber::sleep_for(std::chrono::microseconds{2});
	return true;
}

template<Bmp581Transport Transport>
std::optional<uint8_t>
Bmp581<Transport>::readChipId()
{
	return readRegister(Register::ChipId);
}

template<Bmp581Transport Transport>
std::optional<bmp581::Status_t>
Bmp581<Transport>::readStatus()
{
	const auto value = readRegister(Register::Status);
	if (!value) {
		return std::nullopt;
	}
	return Status_t{*value};
}

template<Bmp581Transport Transport>
std::optional<bmp581::IntStatus_t>
Bmp581<Transport>::readIntStatus()
{
	const auto value = readRegister(Register::IntStatus);
	if (!value) {
		return std::nullopt;
	}
	return IntStatus_t{*value};
}

template<Bmp581Transport Transport>
bool
Bmp581<Transport>::setPowerMode(PowerMode mode)
{
	waitForCommandGap();

	const uint8_t mask = uint8_t(OdrConfig::Mode0) | uint8_t(OdrConfig::Mode1);
	const uint8_t targetMode = static_cast<uint8_t>(mode);
	const uint8_t standbyMode = static_cast<uint8_t>(PowerMode::Standby);

	const auto current = readRegister(Register::OdrConfig);
	if (!current) {
		return false;
	}

	// Per datasheet, active mode transitions should go through STANDBY first.
	if ((*current & mask) != standbyMode) {
		if (!updateRegister(Register::OdrConfig, mask, standbyMode)) {
			return false;
		}
		// Maximum transition time to STANDBY.
		modm::this_fiber::sleep_for(std::chrono::microseconds{2500});
	}

	if (targetMode != standbyMode) {
		if (!updateRegister(Register::OdrConfig, mask, targetMode)) {
			return false;
		}
	}

	modm::this_fiber::sleep_for(std::chrono::microseconds{2});
	return true;
}

template<Bmp581Transport Transport>
bool
Bmp581<Transport>::setOdr(Odr odr)
{
	waitForCommandGap();

	const uint8_t mask = uint8_t(OdrConfig::Odr0) | uint8_t(OdrConfig::Odr1) |
	                     uint8_t(OdrConfig::Odr2) | uint8_t(OdrConfig::Odr3) |
	                     uint8_t(OdrConfig::Odr4);
	const uint8_t value = static_cast<uint8_t>(odr) << 2;

	const bool ok = updateRegister(Register::OdrConfig, mask, value);
	modm::this_fiber::sleep_for(std::chrono::microseconds{2});
	return ok;
}

template<Bmp581Transport Transport>
bool
Bmp581<Transport>::setOversampling(Osr pressOsr, Osr tempOsr, bool enablePressure)
{
	waitForCommandGap();

	uint8_t value = (static_cast<uint8_t>(tempOsr) << 0) |
	                (static_cast<uint8_t>(pressOsr) << 3);
	if (enablePressure) {
		value |= uint8_t(OsrConfig::PressEn);
	}

	const bool ok = writeRegister(Register::OsrConfig, value);
	modm::this_fiber::sleep_for(std::chrono::microseconds{2});
	return ok;
}

template<Bmp581Transport Transport>
bool
Bmp581<Transport>::setIirFilter(IirFilter pressIir, IirFilter tempIir)
{
	// IIR writes require STANDBY mode per datasheet section 4.3.8
	// Save current ODR config, switch to standby, write IIR, restore mode
	const auto odrConfig = readRegister(Register::OdrConfig);
	if (!odrConfig) {
		return false;
	}

	// Switch to standby mode
	const uint8_t modeMask = uint8_t(OdrConfig::Mode0) | uint8_t(OdrConfig::Mode1);
	if (!writeRegister(Register::OdrConfig, (*odrConfig & ~modeMask) | uint8_t(PowerMode::Standby))) {
		return false;
	}

	// Wait for STANDBY transition (tstandby = 2.5ms max per datasheet)
	modm::this_fiber::sleep_for(std::chrono::microseconds{2500});

	// Temperature IIR at [2:0], Pressure IIR at [5:3]
	const uint8_t value = (static_cast<uint8_t>(tempIir) << 0) |
	                      (static_cast<uint8_t>(pressIir) << 3);

	const bool iirOk = writeRegister(Register::DspIir, value);

	// Restore original power mode
	const bool restoreOk = writeRegister(Register::OdrConfig, *odrConfig);

	modm::this_fiber::sleep_for(std::chrono::microseconds{2});
	return iirOk && restoreOk;
}

template<Bmp581Transport Transport>
bool
Bmp581<Transport>::setIntConfig(IntConfig_t config)
{
	waitForCommandGap();

	// Read-modify-write to preserve upper nibble (pad drive strength)
	constexpr uint8_t intConfigMask = uint8_t(IntConfig::Mode) | uint8_t(IntConfig::Polarity) |
	                                  uint8_t(IntConfig::OpenDrain) | uint8_t(IntConfig::Enable);
	const bool ok = updateRegister(Register::IntConfig, intConfigMask, config.value);

	modm::this_fiber::sleep_for(std::chrono::microseconds{2});
	return ok;
}

template<Bmp581Transport Transport>
bool
Bmp581<Transport>::setIntSource(IntSource_t sources)
{
	waitForCommandGap();
	const bool ok = writeRegister(Register::IntSource, sources.value);
	modm::this_fiber::sleep_for(std::chrono::microseconds{2});
	return ok;
}

template<Bmp581Transport Transport>
bool
Bmp581<Transport>::setFifoConfig(FifoMode mode, FifoFrameSelection frameSelection,
                                 FifoDecimation decimation,
                                 bool enablePressureIir, bool enableTemperatureIir)
{
	waitForCommandGap();

	const auto odrConfig = readRegister(Register::OdrConfig);
	if (!odrConfig) {
		return false;
	}

	const uint8_t powerModeMask = uint8_t(OdrConfig::Mode0) | uint8_t(OdrConfig::Mode1);
	const uint8_t standbyMode = static_cast<uint8_t>(PowerMode::Standby);
	const bool needsStandbyTransition = ((*odrConfig & powerModeMask) != standbyMode);

	bool standbyOk = true;
	if (needsStandbyTransition) {
		standbyOk = updateRegister(Register::OdrConfig, powerModeMask, standbyMode);
		if (standbyOk) {
			modm::this_fiber::sleep_for(std::chrono::microseconds{2500});
		}
	}

	const uint8_t fifoIirMask = uint8_t(DspConfig::FifoSelIir_T) | uint8_t(DspConfig::FifoSelIir_P);
	const uint8_t fifoIirValue = (enableTemperatureIir ? uint8_t(DspConfig::FifoSelIir_T) : 0) |
	                             (enablePressureIir ? uint8_t(DspConfig::FifoSelIir_P) : 0);

	const uint8_t fifoSelMask = uint8_t(FifoSel::FrameSel0) | uint8_t(FifoSel::FrameSel1) |
	                            uint8_t(FifoSel::DecSel0) | uint8_t(FifoSel::DecSel1) |
	                            uint8_t(FifoSel::DecSel2);
	const uint8_t fifoSelValue = (static_cast<uint8_t>(frameSelection) << 0) |
	                             (static_cast<uint8_t>(decimation) << 2);

	const uint8_t fifoModeValue = static_cast<uint8_t>(mode) << 5;

	bool fifoIirOk = false;
	bool fifoSelOk = false;
	bool fifoModeOk = false;
	if (standbyOk) {
		fifoIirOk = updateRegister(Register::DspConfig, fifoIirMask, fifoIirValue);
		if (fifoIirOk) {
			fifoSelOk = updateRegister(Register::FifoSel, fifoSelMask, fifoSelValue);
		}
		if (fifoSelOk) {
			fifoModeOk = updateRegister(Register::FifoConfig, uint8_t(FifoConfig::Mode), fifoModeValue);
		}
	}

	bool restoreOk = true;
	if (needsStandbyTransition) {
		restoreOk = writeRegister(Register::OdrConfig, *odrConfig);
	}

	modm::this_fiber::sleep_for(std::chrono::microseconds{2});
	return standbyOk && fifoIirOk && fifoSelOk && fifoModeOk && restoreOk;
}

template<Bmp581Transport Transport>
bool
Bmp581<Transport>::setFifoWatermark(uint8_t threshold)
{
	waitForCommandGap();

	const auto frameSelection = readFifoFrameSelection();
	if (!frameSelection) {
		return false;
	}

	const uint8_t maxThreshold = fifoMaxWatermark(*frameSelection);
	if ((maxThreshold == 0) || (threshold > maxThreshold)) {
		return false;
	}

	const uint8_t thresholdMask = uint8_t(FifoConfig::Threshold0) | uint8_t(FifoConfig::Threshold1) |
	                              uint8_t(FifoConfig::Threshold2) | uint8_t(FifoConfig::Threshold3) |
	                              uint8_t(FifoConfig::Threshold4);

	const bool ok = updateRegister(Register::FifoConfig, thresholdMask, threshold);
	modm::this_fiber::sleep_for(std::chrono::microseconds{2});
	return ok;
}

template<Bmp581Transport Transport>
std::optional<uint8_t>
Bmp581<Transport>::readFifoCount()
{
	const auto value = readRegister(Register::FifoCount);
	if (!value) {
		return std::nullopt;
	}
	return (*value & FifoCountMask);
}

template<Bmp581Transport Transport>
bool
Bmp581<Transport>::readFifoData(Data* frames, std::size_t frameCapacity, std::size_t& framesRead)
{
	framesRead = 0;
	if ((frames == nullptr) && (frameCapacity > 0)) {
		return false;
	}

	const auto frameSelection = readFifoFrameSelection();
	if (!frameSelection) {
		return false;
	}

	const uint8_t frameSize = fifoFrameSize(*frameSelection);
	const uint8_t maxFrameCount = fifoMaxFrameCount(*frameSelection);
	if ((frameSize == 0) || (maxFrameCount == 0)) {
		return false;
	}

	const auto fifoCount = readFifoCount();
	if (!fifoCount) {
		return false;
	}

	const std::size_t availableFrames = std::min<std::size_t>(*fifoCount, maxFrameCount);
	framesRead = std::min(availableFrames, frameCapacity);
	if (framesRead == 0) {
		return true;
	}

	const std::size_t readLength = framesRead * frameSize;
	std::array<uint8_t, FifoMaxReadBytes> rawBuffer{};
	if (!this->read(i(Register::FifoData), rawBuffer.data(), readLength)) {
		return false;
	}

	std::size_t index = 0;
	for (std::size_t ii = 0; ii < framesRead; ++ii)
	{
		frames[ii].rawTemp.fill(0);
		frames[ii].rawPress.fill(0);

		switch (*frameSelection)
		{
			case FifoFrameSelection::Temperature:
				frames[ii].rawTemp[0] = rawBuffer[index + 0];
				frames[ii].rawTemp[1] = rawBuffer[index + 1];
				frames[ii].rawTemp[2] = rawBuffer[index + 2];
				break;
			case FifoFrameSelection::Pressure:
				frames[ii].rawPress[0] = rawBuffer[index + 0];
				frames[ii].rawPress[1] = rawBuffer[index + 1];
				frames[ii].rawPress[2] = rawBuffer[index + 2];
				break;
			case FifoFrameSelection::PressureTemperature:
				frames[ii].rawTemp[0] = rawBuffer[index + 0];
				frames[ii].rawTemp[1] = rawBuffer[index + 1];
				frames[ii].rawTemp[2] = rawBuffer[index + 2];
				frames[ii].rawPress[0] = rawBuffer[index + 3];
				frames[ii].rawPress[1] = rawBuffer[index + 4];
				frames[ii].rawPress[2] = rawBuffer[index + 5];
				break;
			case FifoFrameSelection::Disabled:
				return false;
		}

		index += frameSize;
	}

	return true;
}

template<Bmp581Transport Transport>
bool
Bmp581<Transport>::readData(Data& data)
{
	// Read temperature (3 bytes) and pressure (3 bytes) in one transaction
	// Registers are contiguous: TEMP_XLSB(0x1D) to PRESS_MSB(0x22)
	uint8_t buffer[6];
	if (!this->read(i(Register::TempDataXlsb), buffer, 6)) {
		return false;
	}

	data.rawTemp[0] = buffer[0];
	data.rawTemp[1] = buffer[1];
	data.rawTemp[2] = buffer[2];
	data.rawPress[0] = buffer[3];
	data.rawPress[1] = buffer[4];
	data.rawPress[2] = buffer[5];

	return true;
}

template<Bmp581Transport Transport>
std::optional<float>
Bmp581<Transport>::readTemperature()
{
	Data data;
	if (!this->read(i(Register::TempDataXlsb), data.rawTemp.data(), 3)) {
		return std::nullopt;
	}
	return data.getTemperature();
}

template<Bmp581Transport Transport>
std::optional<float>
Bmp581<Transport>::readPressure()
{
	Data data;
	if (!this->read(i(Register::PressDataXlsb), data.rawPress.data(), 3)) {
		return std::nullopt;
	}
	return data.getPressure();
}

template<Bmp581Transport Transport>
bool
Bmp581<Transport>::isDataReady()
{
	const auto status = readIntStatus();
	return status && (*status & IntStatus::DataReady);
}

template<Bmp581Transport Transport>
std::optional<uint8_t>
Bmp581<Transport>::readRegister(Register reg)
{
	uint8_t value;
	if (!this->read(i(reg), &value, 1)) {
		return std::nullopt;
	}
	return value;
}

template<Bmp581Transport Transport>
std::optional<bmp581::FifoFrameSelection>
Bmp581<Transport>::readFifoFrameSelection()
{
	const auto fifoSel = readRegister(Register::FifoSel);
	if (!fifoSel) {
		return std::nullopt;
	}

	constexpr uint8_t frameSelectionMask = uint8_t(FifoSel::FrameSel0) | uint8_t(FifoSel::FrameSel1);
	return static_cast<FifoFrameSelection>(*fifoSel & frameSelectionMask);
}

template<Bmp581Transport Transport>
bool
Bmp581<Transport>::writeRegister(Register reg, uint8_t value)
{
	return this->write(i(reg), value);
}

template<Bmp581Transport Transport>
bool
Bmp581<Transport>::updateRegister(Register reg, uint8_t mask, uint8_t value)
{
	const auto current = readRegister(reg);
	if (!current) {
		return false;
	}
	const uint8_t newValue = (*current & ~mask) | (value & mask);
	return writeRegister(reg, newValue);
}

} // namespace modm
