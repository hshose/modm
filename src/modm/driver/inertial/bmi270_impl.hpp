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

#include <algorithm>
#ifndef MODM_BMI270_HPP
#error "Don't include this file directly, use 'bmi270.hpp' instead!"
#endif

namespace modm
{

// SPI transport -------------------------------------------------------------------------------

template<typename SpiMaster, typename Cs>
void
Bmi270SpiTransport<SpiMaster, Cs>::initialize()
{
	Cs::setOutput(modm::Gpio::High);
}

template<typename SpiMaster, typename Cs>
std::span<uint8_t>
Bmi270SpiTransport<SpiMaster, Cs>::readRegisters(Register startReg, uint8_t count)
{
	if (count > MaxRegisterSequence) {
		return {};
	}

	modm::this_fiber::poll([&]{ return this->acquireMaster(); });
	Cs::reset();

	txBuffer_[0] = static_cast<uint8_t>(startReg) | ReadFlag;
	txBuffer_[1] = 0;
	std::fill_n(txBuffer_.begin() + 2, count, 0);
	SpiMaster::transfer(txBuffer_.data(), rxBuffer_.data(), count + 2);

	if (this->releaseMaster()) {
		Cs::set();
	}

	return std::span{&rxBuffer_[2], count};
}

template<typename SpiMaster, typename Cs>
bool
Bmi270SpiTransport<SpiMaster, Cs>::writeRegister(Register reg, uint8_t data)
{
	const std::array<uint8_t, 1> value{data};
	return writeRegisters(reg, std::span{value});
}

template<typename SpiMaster, typename Cs>
bool
Bmi270SpiTransport<SpiMaster, Cs>::writeRegisters(Register startReg,
										 std::span<const uint8_t> data)
{
	if (data.size() > MaxRegisterSequence) {
		return false;
	}

	modm::this_fiber::poll([&]{ return this->acquireMaster(); });
	Cs::reset();

	txBuffer_[0] = static_cast<uint8_t>(startReg);
	std::copy(data.begin(), data.end(), txBuffer_.begin() + 1);
	SpiMaster::transfer(txBuffer_.data(), nullptr, data.size() + 1);

	if (this->releaseMaster()) {
		Cs::set();
	}

	return true;
}

// I2C transport -------------------------------------------------------------------------------

template<typename I2cMaster>
Bmi270I2cTransport<I2cMaster>::Bmi270I2cTransport(uint8_t address)
	: I2cDevice<I2cMaster>(address)
{
}

template<typename I2cMaster>
void
Bmi270I2cTransport<I2cMaster>::initialize()
{
}

template<typename I2cMaster>
std::span<uint8_t>
Bmi270I2cTransport<I2cMaster>::readRegisters(Register startReg, uint8_t count)
{
	if (count > MaxRegisterSequence) {
		return {};
	}

	uint8_t reg = static_cast<uint8_t>(startReg);
	if (I2cDevice<I2cMaster>::writeRead(&reg, 1, &buffer_[0], count)) {
		return std::span{&buffer_[0], count};
	}

	return {};
}

template<typename I2cMaster>
bool
Bmi270I2cTransport<I2cMaster>::writeRegister(Register reg, uint8_t data)
{
	const std::array<uint8_t, 1> value{data};
	return writeRegisters(reg, std::span{value});
}

template<typename I2cMaster>
bool
Bmi270I2cTransport<I2cMaster>::writeRegisters(Register startReg,
										 std::span<const uint8_t> data)
{
	if (data.size() > MaxRegisterSequence) {
		return false;
	}

	buffer_[0] = static_cast<uint8_t>(startReg);
	std::copy(data.begin(), data.end(), buffer_.begin() + 1);
	return I2cDevice<I2cMaster>::write(buffer_.data(), data.size() + 1);
}

// Driver --------------------------------------------------------------------------------------

template<Bmi270Transport Transport>
template<typename... Args>
Bmi270<Transport>::Bmi270(Args... transportArgs)
	: Transport{transportArgs...}
{
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::initialize(std::span<const uint8_t> configFile)
{
	Transport::initialize();

	if (!checkChipId() or !reset() or !setAdvancedPowerSave(false)) {
		return false;
	}

	bool configOk = false;
	if (configFile.empty()) {
		configOk = uploadConfig(bmi270_private::configuration, bmi270_private::ConfigurationSize);
	}
	else {
		configOk = uploadConfig(configFile);
	}

	if (!configOk) {
		return false;
	}

	bool ok = setAccRate(AccRate::Rate100Hz_Normal);
	ok &= setAccRange(AccRange::Range2g);
	ok &= setGyroRate(GyroRate::Rate100Hz_Normal);
	ok &= setGyroRange(GyroRange::Range2000dps);
	ok &= enableSensors();
	ok &= setAdvancedPowerSave(true);
	return ok;
}

template<Bmi270Transport Transport>
std::optional<bmi270::Data>
Bmi270<Transport>::readData()
{
	const auto data = this->readRegisters(Register::AccDataXLow, 15);
	if (data.empty()) {
		return {};
	}

	return Data{
		.acc = AccData{
			.raw = Vector3i(
				static_cast<int16_t>(uint16_t(data[0]) | (uint16_t(data[1]) << 8)),
				static_cast<int16_t>(uint16_t(data[2]) | (uint16_t(data[3]) << 8)),
				static_cast<int16_t>(uint16_t(data[4]) | (uint16_t(data[5]) << 8))),
			.range = accRange_},
		.gyro = GyroData{
			.raw = Vector3i(
				static_cast<int16_t>(uint16_t(data[6]) | (uint16_t(data[7]) << 8)),
				static_cast<int16_t>(uint16_t(data[8]) | (uint16_t(data[9]) << 8)),
				static_cast<int16_t>(uint16_t(data[10]) | (uint16_t(data[11]) << 8))),
			.range = gyroRange_},
		.sensorTime = (uint32_t(data[12]) | (uint32_t(data[13]) << 8) | (uint32_t(data[14]) << 16))};
}

template<Bmi270Transport Transport>
std::optional<bmi270::AccData>
Bmi270<Transport>::readAccData()
{
	const auto data = this->readRegisters(Register::AccDataXLow, 6);
	if (data.empty()) {
		return {};
	}

	return AccData{
		.raw = Vector3i(
			static_cast<int16_t>(uint16_t(data[0]) | (uint16_t(data[1]) << 8)),
			static_cast<int16_t>(uint16_t(data[2]) | (uint16_t(data[3]) << 8)),
			static_cast<int16_t>(uint16_t(data[4]) | (uint16_t(data[5]) << 8))),
		.range = accRange_};
}

template<Bmi270Transport Transport>
std::optional<bmi270::GyroData>
Bmi270<Transport>::readGyroData()
{
	const auto data = this->readRegisters(Register::GyroDataXLow, 6);
	if (data.empty()) {
		return {};
	}

	return GyroData{
		.raw = Vector3i(
			static_cast<int16_t>(uint16_t(data[0]) | (uint16_t(data[1]) << 8)),
			static_cast<int16_t>(uint16_t(data[2]) | (uint16_t(data[3]) << 8)),
			static_cast<int16_t>(uint16_t(data[4]) | (uint16_t(data[5]) << 8))),
		.range = gyroRange_};
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::readAccDataReady()
{
	const auto value = readRegister(Register::Status).value_or(0);
	return bool(Status_t{value} & Status::AccDataReady);
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::readGyroDataReady()
{
	const auto value = readRegister(Register::Status).value_or(0);
	return bool(Status_t{value} & Status::GyroDataReady);
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::readCommandReady()
{
	const auto value = readRegister(Register::Status).value_or(0);
	return bool(Status_t{value} & Status::CommandReady);
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::setAccRate(AccRate rate)
{
	timer_.wait();
	const bool ok = this->writeRegister(Register::AccConf, static_cast<uint8_t>(rate));
	timer_.restart(WriteTimeout);
	return ok;
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::setAccRange(AccRange range)
{
	timer_.wait();
	const bool ok = this->writeRegister(Register::AccRange, static_cast<uint8_t>(range));
	timer_.restart(WriteTimeout);
	if (ok) {
		accRange_ = range;
	}
	return ok;
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::setGyroRate(GyroRate rate)
{
	timer_.wait();
	const bool ok = this->writeRegister(Register::GyroConf, static_cast<uint8_t>(rate));
	timer_.restart(WriteTimeout);
	return ok;
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::setGyroRange(GyroRange range)
{
	timer_.wait();
	const bool ok = this->writeRegister(Register::GyroRange, static_cast<uint8_t>(range));
	timer_.restart(WriteTimeout);
	if (ok) {
		gyroRange_ = range;
	}
	return ok;
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::setPowerControl(PowerControl_t control)
{
	timer_.wait();
	const bool ok = this->writeRegister(Register::PowerCtrl, control.value);
	timer_.restart(WriteTimeout);
	return ok;
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::flushFifo()
{
	timer_.wait();
	const bool ok = this->writeRegister(Register::Command, FifoFlushCommand);
	timer_.restart(WriteTimeout);
	return ok;
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::checkChipId()
{
	const auto id = readRegister(Register::ChipId);
	return id.has_value() and id.value() == ChipId;
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::reset()
{
	timer_.wait();
	const bool ok = this->writeRegister(Register::Command, SoftResetCommand);
	timer_.restart(ResetTimeout);
	if (!ok) {
		return false;
	}

	timer_.wait();
	Transport::initialize();
	return true;
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::uploadConfig(std::span<const uint8_t> configFile)
{
	if (configFile.empty() or (configFile.size() & 1u) != 0) {
		return false;
	}

	timer_.wait();
	if (!this->writeRegister(Register::InitControl, InitControlLoadDisabled)) {
		return false;
	}
	timer_.restart(PowerModeTimeout);
	timer_.wait();

	constexpr std::size_t maxChunkSize = Transport::MaxRegisterSequence & ~std::size_t{1};
	if constexpr (maxChunkSize == 0) {
		return false;
	}

	std::size_t offset = 0;
	while (offset < configFile.size())
	{
		const uint16_t wordAddress = offset / 2;
		const std::array<uint8_t, 2> initAddress{
			uint8_t(wordAddress & 0x0F),
			uint8_t((wordAddress >> 4) & 0xFF),
		};

		if (!this->writeRegisters(Register::InitAddress0, std::span{initAddress})) {
			return false;
		}

		std::size_t chunkSize = std::min(configFile.size() - offset, maxChunkSize);
		chunkSize &= ~std::size_t{1};
		if (chunkSize == 0) {
			return false;
		}

		if (!this->writeRegisters(Register::InitData, configFile.subspan(offset, chunkSize))) {
			return false;
		}

		offset += chunkSize;
		timer_.restart(WriteTimeout);
		timer_.wait();
	}

	if (!this->writeRegister(Register::InitControl, InitControlLoadEnabled)) {
		return false;
	}
	timer_.restart(ConfigLoadTimeout);
	timer_.wait();

	const uint8_t internalStatus = readRegister(Register::InternalStatus).value_or(0);
	return (internalStatus & 0x07) == InternalStatusInitOk;
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::uploadConfig(modm::accessor::Flash<uint8_t> configFile, std::size_t configSize)
{
	if (!configFile.isValid() or configSize == 0 or (configSize & 1u) != 0) {
		return false;
	}

	timer_.wait();
	if (!this->writeRegister(Register::InitControl, InitControlLoadDisabled)) {
		return false;
	}
	timer_.restart(PowerModeTimeout);
	timer_.wait();

	constexpr std::size_t maxChunkSize = Transport::MaxRegisterSequence & ~std::size_t{1};
	if constexpr (maxChunkSize == 0) {
		return false;
	}

	std::array<uint8_t, maxChunkSize> chunkBuffer{};
	std::size_t offset = 0;
	while (offset < configSize)
	{
		const uint16_t wordAddress = offset / 2;
		const std::array<uint8_t, 2> initAddress{
			uint8_t(wordAddress & 0x0F),
			uint8_t((wordAddress >> 4) & 0xFF),
		};

		if (!this->writeRegisters(Register::InitAddress0, std::span{initAddress})) {
			return false;
		}

		std::size_t chunkSize = std::min(configSize - offset, maxChunkSize);
		chunkSize &= ~std::size_t{1};
		if (chunkSize == 0) {
			return false;
		}

		for (std::size_t index = 0; index < chunkSize; ++index) {
			chunkBuffer[index] = configFile[offset + index];
		}

		if (!this->writeRegisters(Register::InitData, std::span{chunkBuffer}.subspan(0, chunkSize))) {
			return false;
		}

		offset += chunkSize;
		timer_.restart(WriteTimeout);
		timer_.wait();
	}

	if (!this->writeRegister(Register::InitControl, InitControlLoadEnabled)) {
		return false;
	}
	timer_.restart(ConfigLoadTimeout);
	timer_.wait();

	const uint8_t internalStatus = readRegister(Register::InternalStatus).value_or(0);
	return (internalStatus & 0x07) == InternalStatusInitOk;
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::setAdvancedPowerSave(bool enable)
{
	uint8_t value = readRegister(Register::PowerConf).value_or(0);
	if (enable) {
		value |= uint8_t(PowerConfiguration::AdvancedPowerSave);
	}
	else {
		value &= ~uint8_t(PowerConfiguration::AdvancedPowerSave);
	}

	timer_.wait();
	const bool ok = this->writeRegister(Register::PowerConf, value);
	timer_.restart(PowerModeTimeout);
	return ok;
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::enableSensors()
{
	const auto sensors = (PowerControl::Accelerometer |
						  PowerControl::Gyroscope |
						  PowerControl::Temperature);
	return setPowerControl(sensors);
}

template<Bmi270Transport Transport>
std::optional<uint8_t>
Bmi270<Transport>::readRegister(Register reg)
{
	const auto data = this->readRegisters(reg, 1);
	if (data.empty()) {
		return {};
	}
	return data[0];
}

inline Vector3f
bmi270::AccData::getFloat() const
{
	float scale = 0.0f;
	switch (range)
	{
	case AccRange::Range2g: scale = 1000.f / 16384.f; break;
	case AccRange::Range4g: scale = 1000.f / 8192.f; break;
	case AccRange::Range8g: scale = 1000.f / 4096.f; break;
	case AccRange::Range16g: scale = 1000.f / 2048.f; break;
	}

	return Vector3f(raw[0] * scale, raw[1] * scale, raw[2] * scale);
}

inline Vector3f
bmi270::GyroData::getFloat() const
{
	float scale = 0.0f;
	switch (range)
	{
	case GyroRange::Range2000dps: scale = 1.f / 16.4f; break;
	case GyroRange::Range1000dps: scale = 1.f / 32.8f; break;
	case GyroRange::Range500dps: scale = 1.f / 65.6f; break;
	case GyroRange::Range250dps: scale = 1.f / 131.2f; break;
	case GyroRange::Range125dps: scale = 1.f / 262.4f; break;
	}

	return Vector3f(raw[0] * scale, raw[1] * scale, raw[2] * scale);
}

} // namespace modm
