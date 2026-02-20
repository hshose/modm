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

template<typename SpiMaster, typename Cs>
bool
Bmi270SpiTransport<SpiMaster, Cs>::readFifoData(std::span<uint8_t> data)
{
	if (data.empty()) {
		return true;
	}

	modm::this_fiber::poll([&]{ return this->acquireMaster(); });
	Cs::reset();

	const std::array<uint8_t, 2> header{
		uint8_t(static_cast<uint8_t>(Register::FifoData) | ReadFlag),
		0u};
	std::array<uint8_t, 2> discard{};
	SpiMaster::transfer(header.data(), discard.data(), header.size());

	std::size_t offset = 0;
	while (offset < data.size())
	{
		const std::size_t chunk = std::min<std::size_t>(data.size() - offset, MaxRegisterSequence);
		std::fill_n(txBuffer_.begin(), chunk, 0);
		SpiMaster::transfer(txBuffer_.data(), data.data() + offset, chunk);
		offset += chunk;
	}

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

template<typename I2cMaster>
bool
Bmi270I2cTransport<I2cMaster>::readFifoData(std::span<uint8_t> data)
{
	if (data.empty()) {
		return true;
	}

	uint8_t reg = static_cast<uint8_t>(Register::FifoData);
	return I2cDevice<I2cMaster>::writeRead(&reg, 1, data.data(), data.size());
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
std::optional<uint8_t>
Bmi270<Transport>::getChipId()
{
	return readRegister(Register::ChipId);
}

template<Bmi270Transport Transport>
std::optional<bmi270::ErrorInfo>
Bmi270<Transport>::getErrors()
{
	const auto value = readRegister(Register::Error);
	if (!value) {
		return {};
	}

	return ErrorInfo{
		.fatalError = bool(*value & 0x01),
		.internalError = uint8_t(*value & 0x1E),
		.fifoError = bool(*value & 0x40),
		.auxError = bool(*value & 0x80),
	};
}

template<Bmi270Transport Transport>
std::optional<bmi270::SensorStatus>
Bmi270<Transport>::getStatus()
{
	const auto value = readRegister(Register::Status);
	if (!value) {
		return {};
	}

	return SensorStatus{
		.accDataReady = bool(*value & 0x80),
		.gyroDataReady = bool(*value & 0x40),
		.auxDataReady = bool(*value & 0x20),
		.commandReady = bool(*value & 0x10),
		.auxBusy = bool(*value & 0x04),
	};
}

template<Bmi270Transport Transport>
std::optional<bmi270::InternalStatus>
Bmi270<Transport>::getInternalStatus()
{
	const auto value = readRegister(Register::InternalStatus);
	if (!value) {
		return {};
	}

	return InternalStatus{
		.message = static_cast<InternalMessage>(*value & 0x07),
		.axesRemapError = bool(*value & 0x20),
		.odr50HzError = bool(*value & 0x40),
	};
}

template<Bmi270Transport Transport>
std::optional<bmi270::Temperature>
Bmi270<Transport>::getTemperature()
{
	const auto data = this->readRegisters(Register::Temperature0, 2);
	if (data.empty()) {
		return {};
	}

	const uint16_t rawTemp = uint16_t(data[0]) | (uint16_t(data[1]) << 8);
	if (rawTemp == 0x8000u) {
		return Temperature{
			.valid = false,
			.celsius = 0.f,
		};
	}

	const auto signedRaw = static_cast<int16_t>(rawTemp);
	return Temperature{
		.valid = true,
		.celsius = static_cast<float>(signedRaw) * (1.f / 512.f) + 23.f,
	};
}

template<Bmi270Transport Transport>
std::optional<bmi270::InterruptStatus>
Bmi270<Transport>::getInterruptStatus()
{
	const auto data = this->readRegisters(Register::InterruptStatus0, 2);
	if (data.empty()) {
		return {};
	}

	const uint8_t value = data[1];
	return InterruptStatus{
		.fifoFull = bool(value & 0x01),
		.fifoWatermark = bool(value & 0x02),
		.error = bool(value & 0x04),
		.auxDataReady = bool(value & 0x20),
		.gyroDataReady = bool(value & 0x40),
		.accDataReady = bool(value & 0x80),
	};
}

template<Bmi270Transport Transport>
std::optional<bmi270::ErrorInterruptMask>
Bmi270<Transport>::getErrorInterruptMask()
{
	const auto value = readRegister(Register::ErrRegMask);
	if (!value) {
		return {};
	}

	return ErrorInterruptMask{
		.fatalError = bool(*value & 0x01),
		.internalError = bool(*value & 0x1E),
		.fifoError = bool(*value & 0x40),
		.auxError = bool(*value & 0x80),
	};
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::setErrorInterruptMask(ErrorInterruptMask mask)
{
	uint8_t value = 0;
	if (mask.fatalError) {
		value |= 0x01;
	}
	if (mask.internalError) {
		value |= 0x1E;
	}
	if (mask.fifoError) {
		value |= 0x40;
	}
	if (mask.auxError) {
		value |= 0x80;
	}

	const bool ok = this->writeRegister(Register::ErrRegMask, value);
	modm::this_fiber::sleep_for(WriteTimeout);
	return ok;
}

template<Bmi270Transport Transport>
std::optional<bmi270::InterruptIoControl>
Bmi270<Transport>::getInt1IoControl()
{
	const auto value = readRegister(Register::Int1IoCtrl);
	if (!value) {
		return {};
	}

	return InterruptIoControl{
		.level = (*value & 0x02) ? InterruptOutputLevel::ActiveHigh : InterruptOutputLevel::ActiveLow,
		.outputType = (*value & 0x04) ? InterruptOutputType::OpenDrain : InterruptOutputType::PushPull,
		.outputEnable = bool(*value & 0x08),
		.inputEnable = bool(*value & 0x10),
	};
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::setInt1IoControl(InterruptIoControl control)
{
	uint8_t value = 0;
	if (control.level == InterruptOutputLevel::ActiveHigh) {
		value |= 0x02;
	}
	if (control.outputType == InterruptOutputType::OpenDrain) {
		value |= 0x04;
	}
	if (control.outputEnable) {
		value |= 0x08;
	}
	if (control.inputEnable) {
		value |= 0x10;
	}

	const bool ok = this->writeRegister(Register::Int1IoCtrl, value);
	modm::this_fiber::sleep_for(WriteTimeout);
	return ok;
}

template<Bmi270Transport Transport>
std::optional<bmi270::InterruptIoControl>
Bmi270<Transport>::getInt2IoControl()
{
	const auto value = readRegister(Register::Int2IoCtrl);
	if (!value) {
		return {};
	}

	return InterruptIoControl{
		.level = (*value & 0x02) ? InterruptOutputLevel::ActiveHigh : InterruptOutputLevel::ActiveLow,
		.outputType = (*value & 0x04) ? InterruptOutputType::OpenDrain : InterruptOutputType::PushPull,
		.outputEnable = bool(*value & 0x08),
		.inputEnable = bool(*value & 0x10),
	};
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::setInt2IoControl(InterruptIoControl control)
{
	uint8_t value = 0;
	if (control.level == InterruptOutputLevel::ActiveHigh) {
		value |= 0x02;
	}
	if (control.outputType == InterruptOutputType::OpenDrain) {
		value |= 0x04;
	}
	if (control.outputEnable) {
		value |= 0x08;
	}
	if (control.inputEnable) {
		value |= 0x10;
	}

	const bool ok = this->writeRegister(Register::Int2IoCtrl, value);
	modm::this_fiber::sleep_for(WriteTimeout);
	return ok;
}

template<Bmi270Transport Transport>
std::optional<bmi270::InterruptLatch>
Bmi270<Transport>::getInterruptLatch()
{
	const auto value = readRegister(Register::IntLatch);
	if (!value) {
		return {};
	}

	return ((*value & 0x01) != 0) ? InterruptLatch::Permanent : InterruptLatch::None;
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::setInterruptLatch(InterruptLatch mode)
{
	const bool ok = this->writeRegister(Register::IntLatch, static_cast<uint8_t>(mode));
	modm::this_fiber::sleep_for(WriteTimeout);
	return ok;
}

template<Bmi270Transport Transport>
std::optional<bmi270::InterruptMapData>
Bmi270<Transport>::getInterruptMapData()
{
	const auto value = readRegister(Register::IntMapData);
	if (!value) {
		return {};
	}

	return InterruptMapData{
		.int1FifoFull = bool(*value & 0x01),
		.int1FifoWatermark = bool(*value & 0x02),
		.int1DataReady = bool(*value & 0x04),
		.int1Error = bool(*value & 0x08),
		.int2FifoFull = bool(*value & 0x10),
		.int2FifoWatermark = bool(*value & 0x20),
		.int2DataReady = bool(*value & 0x40),
		.int2Error = bool(*value & 0x80),
	};
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::setInterruptMapData(InterruptMapData map)
{
	uint8_t value = 0;
	if (map.int1FifoFull) {
		value |= 0x01;
	}
	if (map.int1FifoWatermark) {
		value |= 0x02;
	}
	if (map.int1DataReady) {
		value |= 0x04;
	}
	if (map.int1Error) {
		value |= 0x08;
	}
	if (map.int2FifoFull) {
		value |= 0x10;
	}
	if (map.int2FifoWatermark) {
		value |= 0x20;
	}
	if (map.int2DataReady) {
		value |= 0x40;
	}
	if (map.int2Error) {
		value |= 0x80;
	}

	const bool ok = this->writeRegister(Register::IntMapData, value);
	modm::this_fiber::sleep_for(WriteTimeout);
	return ok;
}

template<Bmi270Transport Transport>
std::optional<bmi270::InternalError>
Bmi270<Transport>::getInternalError()
{
	const auto value = readRegister(Register::InternalError);
	if (!value) {
		return {};
	}

	return InternalError{
		.longProcessingTime = bool(*value & 0x01),
		.fatalError = bool(*value & 0x04),
		.featureEngineDisabled = bool(*value & 0x10),
	};
}

template<Bmi270Transport Transport>
std::optional<bmi270::PullUpConfiguration>
Bmi270<Transport>::getPullUpConfiguration()
{
	const auto value = readRegister(Register::AuxIfTrim);
	if (!value) {
		return {};
	}

	return static_cast<PullUpConfiguration>(*value & 0x03);
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::setPullUpConfiguration(PullUpConfiguration configuration)
{
	const auto current = readRegister(Register::AuxIfTrim);
	if (!current) {
		return false;
	}

	const uint8_t value = (*current & ~0x03u) | (static_cast<uint8_t>(configuration) & 0x03u);
	const bool ok = this->writeRegister(Register::AuxIfTrim, value);
	modm::this_fiber::sleep_for(WriteTimeout);
	return ok;
}

template<Bmi270Transport Transport>
std::optional<bmi270::GyroCrtConfig>
Bmi270<Transport>::getGyroCrtConfig()
{
	const auto value = readRegister(Register::GyroCrtConf);
	if (!value) {
		return {};
	}

	return GyroCrtConfig{
		.running = bool(*value & 0x04),
		.readyForDownload = bool(*value & 0x08),
	};
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::setGyroCrtConfig(GyroCrtConfig configuration)
{
	const uint8_t value = configuration.running ? 0x04 : 0x00;
	const bool ok = this->writeRegister(Register::GyroCrtConf, value);
	modm::this_fiber::sleep_for(WriteTimeout);
	return ok;
}

template<Bmi270Transport Transport>
std::optional<bool>
Bmi270<Transport>::getNvmCrtEnabled()
{
	const auto value = readRegister(Register::NvmConf);
	if (!value) {
		return {};
	}

	return bool(*value & 0x02);
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::setNvmCrtEnabled(bool enable)
{
	const auto current = readRegister(Register::NvmConf);
	if (!current) {
		return false;
	}

	uint8_t value = *current;
	if (enable) {
		value |= 0x02;
	}
	else {
		value &= ~0x02u;
	}

	const bool ok = this->writeRegister(Register::NvmConf, value);
	modm::this_fiber::sleep_for(WriteTimeout);
	return ok;
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::triggerGyroCrt()
{
	return sendCommand(Command::TriggerGyro);
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::doCrt(std::span<const uint8_t> configFile)
{
	static constexpr uint8_t FeaturePageSize{16};
	static constexpr uint8_t FeaturePage0{0};
	static constexpr uint8_t FeaturePage1{1};
	static constexpr uint8_t MaxBurstLengthOffset{2};
	static constexpr uint8_t GyroSelfTestCrtOffset{3};
	static constexpr uint8_t GyroSelfTestCrtMask{0x01};
	static constexpr uint8_t AbortFeatureMask{0x02};
	static constexpr uint8_t GyroUserGainStatusOffset{8};
	static constexpr uint8_t GTriggerStatusMask{0x38};
	static constexpr uint8_t GTriggerStatusShift{3};
	static constexpr uint8_t GTriggerNoError{0};
	static constexpr uint8_t GTriggerPreconditionError{1};
	static constexpr uint8_t GTriggerDownloadError{2};
	static constexpr uint8_t GTriggerAbortError{3};
	static constexpr uint8_t CrtReadyRetryCount{100};
	static constexpr uint8_t CrtRunningRetryCount{200};
	static constexpr std::chrono::microseconds CrtReadyDelay{2000};
	static constexpr std::chrono::microseconds CrtRunningDelay{10000};
	static constexpr std::size_t CrtConfigStartIndex{0x1800};
	static constexpr std::size_t CrtConfigLength{2048};
	static constexpr uint8_t CrtMinBurstBytes{2};
	static constexpr uint16_t CrtMaxBurstWords{255};

	auto readFeaturePage = [&](uint8_t page, std::array<uint8_t, FeaturePageSize>& data) -> bool {
		if (!this->writeRegister(Register::FeatPage, page)) {
			return false;
		}
		modm::this_fiber::sleep_for(WriteTimeout);
		const auto pageData = this->readRegisters(Register::Features, FeaturePageSize);
		if (pageData.empty()) {
			return false;
		}
		std::copy_n(pageData.begin(), FeaturePageSize, data.begin());
		return true;
	};

	auto writeFeaturePage = [&](uint8_t page, const std::array<uint8_t, FeaturePageSize>& data) -> bool {
		if (!this->writeRegister(Register::FeatPage, page)) {
			return false;
		}
		modm::this_fiber::sleep_for(WriteTimeout);
		const bool ok = this->writeRegisters(Register::Features, std::span{data});
		modm::this_fiber::sleep_for(WriteTimeout);
		return ok;
	};

	auto getMaxBurstLength = [&]() -> std::optional<uint8_t> {
		std::array<uint8_t, FeaturePageSize> page{};
		if (!readFeaturePage(FeaturePage1, page)) {
			return {};
		}
		return page[MaxBurstLengthOffset];
	};

	auto setMaxBurstLength = [&](uint8_t burstWords) -> bool {
		std::array<uint8_t, FeaturePageSize> page{};
		if (!readFeaturePage(FeaturePage1, page)) {
			return false;
		}
		page[MaxBurstLengthOffset] = burstWords;
		return writeFeaturePage(FeaturePage1, page);
	};

	auto setSelfTestSelection = [&](bool selectCrt) -> bool {
		std::array<uint8_t, FeaturePageSize> page{};
		if (!readFeaturePage(FeaturePage1, page)) {
			return false;
		}
		if (selectCrt) {
			page[GyroSelfTestCrtOffset] |= GyroSelfTestCrtMask;
		}
		else {
			page[GyroSelfTestCrtOffset] &= ~GyroSelfTestCrtMask;
		}
		return writeFeaturePage(FeaturePage1, page);
	};

	auto setAbortFeature = [&](bool enable) -> bool {
		std::array<uint8_t, FeaturePageSize> page{};
		if (!readFeaturePage(FeaturePage1, page)) {
			return false;
		}
		if (enable) {
			page[GyroSelfTestCrtOffset] |= AbortFeatureMask;
		}
		else {
			page[GyroSelfTestCrtOffset] &= ~AbortFeatureMask;
		}
		return writeFeaturePage(FeaturePage1, page);
	};

	auto getGTriggerStatus = [&]() -> std::optional<uint8_t> {
		std::array<uint8_t, FeaturePageSize> page{};
		if (!readFeaturePage(FeaturePage0, page)) {
			return {};
		}
		return uint8_t((page[GyroUserGainStatusOffset] & GTriggerStatusMask) >> GTriggerStatusShift);
	};

	auto waitStRunningComplete = [&]() -> bool {
		for (uint8_t retry = 0; retry < CrtRunningRetryCount; ++retry)
		{
			const auto crtConf = getGyroCrtConfig();
			if (!crtConf) {
				return false;
			}
			if (!crtConf->running) {
				return true;
			}
			modm::this_fiber::sleep_for(CrtRunningDelay);
		}
		return false;
	};

	auto waitReadyForDownloadToggle = [&](bool previous) -> bool {
		for (uint8_t retry = 0; retry < CrtReadyRetryCount; ++retry)
		{
			const auto crtConf = getGyroCrtConfig();
			if (!crtConf) {
				return false;
			}
			if (crtConf->readyForDownload != previous) {
				return crtConf->running;
			}
			modm::this_fiber::sleep_for(CrtReadyDelay);
		}
		return false;
	};

	auto processCrtDownload = [&](bool lastChunk) -> bool {
		const auto crtConf = getGyroCrtConfig();
		if (!crtConf) {
			return false;
		}
		const bool readyForDownload = crtConf->readyForDownload;
		if (!sendCommand(Command::TriggerGyro)) {
			return false;
		}
		if (!lastChunk and !waitReadyForDownloadToggle(readyForDownload)) {
			return false;
		}
		return true;
	};

	auto writeInitBytes = [&](uint16_t index, std::span<const uint8_t> bytes) -> bool {
		const uint16_t wordAddress = index / 2;
		const std::array<uint8_t, 2> initAddress{
			uint8_t(wordAddress & 0x0F),
			uint8_t((wordAddress >> 4) & 0xFF),
		};

		if (!this->writeRegisters(Register::InitAddress0, std::span{initAddress})) {
			return false;
		}
		if (!this->writeRegisters(Register::InitData, bytes)) {
			return false;
		}
		modm::this_fiber::sleep_for(WriteTimeout);
		return true;
	};

	auto evaluateCrtResult = [&](uint8_t writeBurstWords) -> bool {
		const auto status = getGTriggerStatus();
		if (!status) {
			return false;
		}

		switch (*status)
		{
		case GTriggerNoError:
			return setMaxBurstLength(0);
		case GTriggerDownloadError:
		case GTriggerAbortError:
			(void)setMaxBurstLength(writeBurstWords);
			return false;
		case GTriggerPreconditionError:
		default:
			return false;
		}
	};

	bool ok = true;
	bool restoreOk = true;
	bool haveSavedPower = false;
	PowerControl_t savedPower{};

	const auto powerControl = getPowerControl();
	if (!powerControl) {
		return false;
	}
	savedPower = *powerControl;
	haveSavedPower = true;

	const auto powerConfiguration = getPowerConfiguration();
	if (!powerConfiguration) {
		return false;
	}
	const bool apsWasEnabled = bool(*powerConfiguration & PowerConfiguration::AdvancedPowerSave);

	if (apsWasEnabled) {
		ok &= setAdvancedPowerSave(false);
	}

	auto currentCrt = getGyroCrtConfig();
	if (!currentCrt or currentCrt->running) {
		ok = false;
	}

	uint8_t maxBurstLengthWords{0};
	if (ok) {
		const auto maxBurstLength = getMaxBurstLength();
		if (!maxBurstLength) {
			ok = false;
		}
		else {
			maxBurstLengthWords = *maxBurstLength;
		}
	}

	if (ok) {
		PowerControl_t preparePower{savedPower.value};
		preparePower.value &= ~uint8_t(PowerControl::Gyroscope);
		preparePower.value |= uint8_t(PowerControl::Accelerometer);
		ok &= setPowerControl(preparePower);
	}

	if (ok) {
		GyroCrtConfig crt{};
		crt.running = true;
		crt.readyForDownload = false;
		ok &= setGyroCrtConfig(crt);
	}

	if (ok) {
		modm::this_fiber::sleep_for(1ms);
		ok &= setAbortFeature(false);
	}

	if (ok) {
		ok &= setSelfTestSelection(true);
	}

	uint8_t writeBurstWords = 0;
	if (ok and (maxBurstLengthWords == 0)) {
		ok &= sendCommand(Command::TriggerGyro);
		ok &= waitStRunningComplete();
		if (ok) {
			ok &= evaluateCrtResult(0);
		}
	}
	else if (ok) {
		uint16_t writeBurstBytes = Transport::MaxRegisterSequence;
		writeBurstBytes &= ~uint16_t{1};
		if (writeBurstBytes < CrtMinBurstBytes) {
			writeBurstBytes = CrtMinBurstBytes;
		}
		const uint16_t maxBurstBytes = CrtMaxBurstWords * 2;
		if (writeBurstBytes > maxBurstBytes) {
			writeBurstBytes = maxBurstBytes;
		}
		writeBurstWords = uint8_t(writeBurstBytes / 2);
		ok &= setMaxBurstLength(writeBurstWords);

		bool readyForDownload = false;
		if (ok) {
			currentCrt = getGyroCrtConfig();
			if (!currentCrt) {
				ok = false;
			}
			else {
				readyForDownload = currentCrt->readyForDownload;
			}
		}
		if (ok) {
			ok &= sendCommand(Command::TriggerGyro);
		}
		if (ok) {
			ok &= waitReadyForDownloadToggle(readyForDownload);
		}

		if (ok) {
			if (!configFile.empty()) {
				if (configFile.size() < (CrtConfigStartIndex + CrtConfigLength)) {
					ok = false;
				}
				else {
					std::size_t index = CrtConfigStartIndex;
					const std::size_t end = CrtConfigStartIndex + CrtConfigLength;
					while (ok and (index < end))
					{
						std::size_t chunkSize = std::min<std::size_t>(writeBurstBytes, end - index);
						chunkSize &= ~std::size_t{1};
						if (chunkSize == 0) {
							ok = false;
							break;
						}

						const bool lastChunk = ((index + chunkSize) >= end);
						ok &= writeInitBytes(uint16_t(index), configFile.subspan(index, chunkSize));
						if (ok) {
							ok &= processCrtDownload(lastChunk);
						}
						index += chunkSize;
					}
				}
			}
			else {
				std::size_t index = CrtConfigStartIndex;
				const std::size_t end = CrtConfigStartIndex + CrtConfigLength;
				std::array<uint8_t, Transport::MaxRegisterSequence> chunkBuffer{};
				while (ok and (index < end))
				{
					std::size_t chunkSize = std::min<std::size_t>(writeBurstBytes, end - index);
					chunkSize &= ~std::size_t{1};
					if (chunkSize == 0) {
						ok = false;
						break;
					}

					const bool lastChunk = ((index + chunkSize) >= end);
					for (std::size_t ii = 0; ii < chunkSize; ++ii) {
						chunkBuffer[ii] = bmi270_private::configuration[index + ii];
					}

					ok &= writeInitBytes(uint16_t(index), std::span{chunkBuffer}.subspan(0, chunkSize));
					if (ok) {
						ok &= processCrtDownload(lastChunk);
					}
					index += chunkSize;
				}
			}
		}

		if (ok) {
			ok &= waitStRunningComplete();
		}
		if (ok) {
			ok &= evaluateCrtResult(writeBurstWords);
		}
	}

	if (haveSavedPower) {
		restoreOk &= setPowerControl(savedPower);
	}
	if (apsWasEnabled) {
		restoreOk &= setAdvancedPowerSave(true);
	}

	return ok and restoreOk;
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::performAccelFoc(AccelFocTarget target)
{
	static constexpr uint8_t FocSampleCount{128};
	static constexpr uint8_t DataReadyRetries{5};
	static constexpr std::chrono::milliseconds DataReadyDelay{20};
	static constexpr uint8_t FocAccConfValue{0xB7};
	static constexpr uint8_t NvAccOffsetMask{0x08};

	auto setAccelOffsetCompensation = [&](bool enable) -> bool {
		const auto nvConf = readRegister(Register::NvConf);
		if (!nvConf) {
			return false;
		}

		uint8_t value = *nvConf;
		if (enable) {
			value |= NvAccOffsetMask;
		}
		else {
			value &= ~NvAccOffsetMask;
		}

		const bool ok = this->writeRegister(Register::NvConf, value);
		modm::this_fiber::sleep_for(WriteTimeout);
		return ok;
	};

	const auto savedAcc = this->readRegisters(Register::AccConf, 2);
	const auto savedPowerControl = getPowerControl();
	const auto savedPowerConfiguration = getPowerConfiguration();
	if (savedAcc.empty() or !savedPowerControl or !savedPowerConfiguration) {
		return false;
	}

	const bool apsWasEnabled = bool(*savedPowerConfiguration & PowerConfiguration::AdvancedPowerSave);
	const uint8_t savedAccConf = savedAcc[0];
	const uint8_t savedAccRange = savedAcc[1] & 0x03;

	bool ok = true;
	bool restoreOk = true;

	ok &= setAccelOffsetCompensation(false);
	if (ok) {
		ok &= this->writeRegister(Register::AccConf, FocAccConfValue);
		modm::this_fiber::sleep_for(WriteTimeout);
	}

	if (ok) {
		PowerControl_t powerControl{savedPowerControl->value};
		powerControl.value |= uint8_t(PowerControl::Accelerometer);
		ok &= setPowerControl(powerControl);
	}

	if (ok and apsWasEnabled) {
		ok &= setAdvancedPowerSave(false);
	}

	Vector3i average{0, 0, 0};
	if (ok) {
		int32_t sumX = 0;
		int32_t sumY = 0;
		int32_t sumZ = 0;

		for (uint8_t sample = 0; sample < FocSampleCount; ++sample)
		{
			bool ready = false;
			for (uint8_t retry = 0; retry < DataReadyRetries; ++retry)
			{
				modm::this_fiber::sleep_for(DataReadyDelay);
				const auto status = readRegister(Register::Status);
				if (status and (*status & uint8_t(Status::AccDataReady))) {
					ready = true;
					break;
				}
			}

			if (!ready) {
				ok = false;
				break;
			}

			const auto data = readAccData();
			if (!data) {
				ok = false;
				break;
			}

			sumX += data->raw[0];
			sumY += data->raw[1];
			sumZ += data->raw[2];
		}

		if (ok) {
			average[0] = int(sumX / FocSampleCount);
			average[1] = int(sumY / FocSampleCount);
			average[2] = int(sumZ / FocSampleCount);
		}
	}

	if (ok) {
		uint8_t rangeG = 2;
		switch (savedAccRange)
		{
		case 0: rangeG = 2; break;
		case 1: rangeG = 4; break;
		case 2: rangeG = 8; break;
		case 3: rangeG = 16; break;
		default: rangeG = 8; break;
		}

		const int16_t lsbPerG = int16_t(32768 / rangeG);
		int16_t referenceX = 0;
		int16_t referenceY = 0;
		int16_t referenceZ = 0;
		const int16_t sign = target.negative ? -1 : 1;
		switch (target.axis)
		{
		case FocAxis::X: referenceX = sign; break;
		case FocAxis::Y: referenceY = sign; break;
		case FocAxis::Z: referenceZ = sign; break;
		}

		const int16_t deltaX = int16_t(average[0]) - int16_t(lsbPerG * referenceX);
		const int16_t deltaY = int16_t(average[1]) - int16_t(lsbPerG * referenceY);
		const int16_t deltaZ = int16_t(average[2]) - int16_t(lsbPerG * referenceZ);

		uint32_t temp = 65536u / (uint32_t(rangeG) * 256u);
		int8_t bitPos3p9mg = -1;
		while (temp != 1u) {
			++bitPos3p9mg;
			temp >>= 1u;
		}

		const int32_t scale = (1 << bitPos3p9mg);
		const int32_t roundOff = (1 << (bitPos3p9mg - 1));

		const int8_t offX = int8_t(-((deltaX + roundOff) / scale));
		const int8_t offY = int8_t(-((deltaY + roundOff) / scale));
		const int8_t offZ = int8_t(-((deltaZ + roundOff) / scale));

		ok &= setAccOffsets(AccOffsets{
			.x = uint8_t(offX),
			.y = uint8_t(offY),
			.z = uint8_t(offZ),
		});

		if (ok) {
			ok &= setAccelOffsetCompensation(true);
		}
	}

	restoreOk &= this->writeRegister(Register::AccConf, savedAccConf);
	modm::this_fiber::sleep_for(WriteTimeout);
	restoreOk &= this->writeRegister(Register::AccRange, savedAccRange);
	modm::this_fiber::sleep_for(WriteTimeout);
	restoreOk &= setPowerControl(*savedPowerControl);
	if (apsWasEnabled) {
		restoreOk &= setAdvancedPowerSave(true);
	}
	accRange_ = static_cast<AccRange>(savedAccRange);

	return ok and restoreOk;
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::performGyroFoc()
{
	static constexpr uint8_t FocSampleCount{128};
	static constexpr std::chrono::milliseconds DataReadyDelay{50};
	static constexpr uint8_t FocGyroConfValue{0xB6};
	static constexpr uint8_t FocGyroRangeValue{0x00};
	static constexpr uint8_t GyroOffsetEnableMask{0x40};

	auto setGyroOffsetCompensation = [&](bool enable) -> bool {
		const auto offset6 = readRegister(Register::Offset6);
		if (!offset6) {
			return false;
		}

		uint8_t value = *offset6;
		if (enable) {
			value |= GyroOffsetEnableMask;
		}
		else {
			value &= ~GyroOffsetEnableMask;
		}

		const bool ok = this->writeRegister(Register::Offset6, value);
		modm::this_fiber::sleep_for(WriteTimeout);
		return ok;
	};

	const auto savedGyro = this->readRegisters(Register::GyroConf, 2);
	const auto savedPowerControl = getPowerControl();
	const auto savedPowerConfiguration = getPowerConfiguration();
	const auto savedOffsets = getGyroOffsets();
	if (savedGyro.empty() or !savedPowerControl or !savedPowerConfiguration or !savedOffsets) {
		return false;
	}

	const bool apsWasEnabled = bool(*savedPowerConfiguration & PowerConfiguration::AdvancedPowerSave);
	const uint8_t savedGyroConf = savedGyro[0];
	const uint8_t savedGyroRange = savedGyro[1] & 0x07;

	bool ok = true;
	bool restoreOk = true;

	ok &= setGyroOffsetCompensation(false);
	if (ok) {
		const std::array<uint8_t, 2> focConfig{FocGyroConfValue, FocGyroRangeValue};
		ok &= this->writeRegisters(Register::GyroConf, std::span{focConfig});
		modm::this_fiber::sleep_for(WriteTimeout);
	}

	if (ok) {
		PowerControl_t powerControl{savedPowerControl->value};
		powerControl.value |= uint8_t(PowerControl::Gyroscope);
		ok &= setPowerControl(powerControl);
	}

	if (ok and apsWasEnabled) {
		ok &= setAdvancedPowerSave(false);
	}

	Vector3i average{0, 0, 0};
	if (ok) {
		int32_t sumX = 0;
		int32_t sumY = 0;
		int32_t sumZ = 0;

		for (uint8_t sample = 0; sample < FocSampleCount; ++sample)
		{
			modm::this_fiber::sleep_for(DataReadyDelay);

			const auto status = readRegister(Register::Status);
			if (!status or ((*status & uint8_t(Status::GyroDataReady)) == 0)) {
				ok = false;
				break;
			}

			const auto data = readGyroData();
			if (!data) {
				ok = false;
				break;
			}

			sumX += data->raw[0];
			sumY += data->raw[1];
			sumZ += data->raw[2];
		}

		if (ok) {
			average[0] = int(sumX / FocSampleCount);
			average[1] = int(sumY / FocSampleCount);
			average[2] = int(sumZ / FocSampleCount);
		}
	}

	if (ok) {
		const int16_t offX = int16_t(-std::clamp<int>(average[0], -512, 511));
		const int16_t offY = int16_t(-std::clamp<int>(average[1], -512, 511));
		const int16_t offZ = int16_t(-std::clamp<int>(average[2], -512, 511));

		GyroOffsets offsets = *savedOffsets;
		offsets.x = uint16_t(offX);
		offsets.y = uint16_t(offY);
		offsets.z = uint16_t(offZ);
		ok &= setGyroOffsets(offsets);
		if (ok) {
			ok &= setGyroOffsetCompensation(true);
		}
	}

	restoreOk &= this->writeRegister(Register::GyroConf, savedGyroConf);
	modm::this_fiber::sleep_for(WriteTimeout);
	restoreOk &= this->writeRegister(Register::GyroRange, savedGyroRange);
	modm::this_fiber::sleep_for(WriteTimeout);
	restoreOk &= setPowerControl(*savedPowerControl);
	if (apsWasEnabled) {
		restoreOk &= setAdvancedPowerSave(true);
	}
	gyroRange_ = static_cast<GyroRange>(savedGyroRange);

	return ok and restoreOk;
}

template<Bmi270Transport Transport>
std::optional<bmi270::InterfaceConfig>
Bmi270<Transport>::getInterfaceConfig()
{
	const auto value = readRegister(Register::IfConf);
	if (!value) {
		return {};
	}

	return InterfaceConfig{
		.primarySpiMode = (*value & 0x01) ? InterfaceSpiMode::Spi3Wire : InterfaceSpiMode::Spi4Wire,
		.oisSpiMode = (*value & 0x02) ? InterfaceSpiMode::Spi3Wire : InterfaceSpiMode::Spi4Wire,
		.oisEnabled = bool(*value & 0x10),
		.auxEnabled = bool(*value & 0x20),
	};
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::setInterfaceConfig(InterfaceConfig configuration)
{
	uint8_t value = 0;
	if (configuration.primarySpiMode == InterfaceSpiMode::Spi3Wire) {
		value |= 0x01;
	}
	if (configuration.oisSpiMode == InterfaceSpiMode::Spi3Wire) {
		value |= 0x02;
	}
	if (configuration.oisEnabled) {
		value |= 0x10;
	}
	if (configuration.auxEnabled) {
		value |= 0x20;
	}

	const bool ok = this->writeRegister(Register::IfConf, value);
	modm::this_fiber::sleep_for(WriteTimeout);
	return ok;
}

template<Bmi270Transport Transport>
std::optional<bmi270::DriveConfig>
Bmi270<Transport>::getDriveConfig()
{
	const auto value = readRegister(Register::Drv);
	if (!value) {
		return {};
	}

	return DriveConfig{
		.ioPadDrv1 = static_cast<DriveStrength>(*value & 0x07),
		.ioPadI2cBoost1 = bool(*value & 0x08),
		.ioPadDrv2 = static_cast<DriveStrength>((*value >> 4) & 0x07),
		.ioPadI2cBoost2 = bool(*value & 0x80),
	};
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::setDriveConfig(DriveConfig configuration)
{
	uint8_t value = static_cast<uint8_t>(configuration.ioPadDrv1) & 0x07;
	if (configuration.ioPadI2cBoost1) {
		value |= 0x08;
	}
	value |= (static_cast<uint8_t>(configuration.ioPadDrv2) & 0x07) << 4;
	if (configuration.ioPadI2cBoost2) {
		value |= 0x80;
	}

	const bool ok = this->writeRegister(Register::Drv, value);
	modm::this_fiber::sleep_for(WriteTimeout);
	return ok;
}

template<Bmi270Transport Transport>
std::optional<bmi270::AccOffsets>
Bmi270<Transport>::getAccOffsets()
{
	const auto data = this->readRegisters(Register::Offset0, 3);
	if (data.empty()) {
		return {};
	}

	return AccOffsets{
		.x = data[0],
		.y = data[1],
		.z = data[2],
	};
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::setAccOffsets(AccOffsets offsets)
{
	const std::array<uint8_t, 3> data{offsets.x, offsets.y, offsets.z};
	const bool ok = this->writeRegisters(Register::Offset0, std::span{data});
	modm::this_fiber::sleep_for(WriteTimeout);
	return ok;
}

template<Bmi270Transport Transport>
std::optional<bmi270::GyroOffsets>
Bmi270<Transport>::getGyroOffsets()
{
	const auto data = this->readRegisters(Register::Offset3, 4);
	if (data.empty()) {
		return {};
	}

	return GyroOffsets{
		.x = static_cast<uint16_t>(uint16_t(data[0]) | (uint16_t(data[3] & 0x03) << 8)),
		.y = static_cast<uint16_t>(uint16_t(data[1]) | (uint16_t(data[3] & 0x0C) << 6)),
		.z = static_cast<uint16_t>(uint16_t(data[2]) | (uint16_t(data[3] & 0x30) << 4)),
		.offsetEnabled = bool(data[3] & 0x40),
		.gainEnabled = bool(data[3] & 0x80),
	};
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::setGyroOffsets(GyroOffsets offsets)
{
	std::array<uint8_t, 4> data{};
	data[0] = static_cast<uint8_t>(offsets.x);
	data[1] = static_cast<uint8_t>(offsets.y);
	data[2] = static_cast<uint8_t>(offsets.z);
	data[3] |= static_cast<uint8_t>((offsets.x >> 8) & 0x03);
	data[3] |= static_cast<uint8_t>((offsets.y >> 6) & 0x0C);
	data[3] |= static_cast<uint8_t>((offsets.z >> 4) & 0x30);
	if (offsets.offsetEnabled) {
		data[3] |= 0x40;
	}
	if (offsets.gainEnabled) {
		data[3] |= 0x80;
	}

	const bool ok = this->writeRegisters(Register::Offset3, std::span{data});
	modm::this_fiber::sleep_for(WriteTimeout);
	return ok;
}

template<Bmi270Transport Transport>
std::optional<bmi270::GyroGainUpdate>
Bmi270<Transport>::getGyroGainUpdate()
{
	static constexpr uint8_t FeaturePage1{1};
	static constexpr uint8_t FeatureRegisterBase{0x30};
	static constexpr uint8_t GyroGainUpd1Address{0x36};
	static constexpr uint8_t GyroGainUpd2Address{0x38};
	static constexpr uint8_t GyroGainUpd3Address{0x3A};
	static constexpr uint8_t RatioOffsetX{GyroGainUpd1Address - FeatureRegisterBase};
	static constexpr uint8_t RatioOffsetY{GyroGainUpd2Address - FeatureRegisterBase};
	static constexpr uint8_t RatioOffsetZ{GyroGainUpd3Address - FeatureRegisterBase};
	static constexpr uint8_t EnableOffset{uint8_t(RatioOffsetZ + 1)};
	static constexpr uint16_t RatioMask{0x07FF};
	// Bit 11 of GYR_GAIN_UPD_3 (0x3A) lives in the MSB byte at offset +1, bit3.
	static constexpr uint8_t EnableMask{0x08};
	static_assert(RatioOffsetX == 6 and RatioOffsetY == 8 and RatioOffsetZ == 10 and EnableOffset == 11);

	std::array<uint8_t, 16> featurePage{};
	if (!readFeaturePage(FeaturePage1, featurePage)) {
		return {};
	}

	const uint16_t ratioX = uint16_t(featurePage[RatioOffsetX]) |
							(uint16_t(featurePage[RatioOffsetX + 1]) << 8);
	const uint16_t ratioY = uint16_t(featurePage[RatioOffsetY]) |
							(uint16_t(featurePage[RatioOffsetY + 1]) << 8);
	const uint16_t ratioZ = uint16_t(featurePage[RatioOffsetZ]) |
							(uint16_t(featurePage[RatioOffsetZ + 1]) << 8);

	return GyroGainUpdate{
		.ratioX = uint16_t(ratioX & RatioMask),
		.ratioY = uint16_t(ratioY & RatioMask),
		.ratioZ = uint16_t(ratioZ & RatioMask),
		.enable = bool(featurePage[EnableOffset] & EnableMask),
	};
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::setGyroGainUpdate(GyroGainUpdate update)
{
	static constexpr uint8_t FeaturePage1{1};
	static constexpr uint8_t FeatureRegisterBase{0x30};
	static constexpr uint8_t GyroGainUpd1Address{0x36};
	static constexpr uint8_t GyroGainUpd2Address{0x38};
	static constexpr uint8_t GyroGainUpd3Address{0x3A};
	static constexpr uint8_t RatioOffsetX{GyroGainUpd1Address - FeatureRegisterBase};
	static constexpr uint8_t RatioOffsetY{GyroGainUpd2Address - FeatureRegisterBase};
	static constexpr uint8_t RatioOffsetZ{GyroGainUpd3Address - FeatureRegisterBase};
	static constexpr uint8_t EnableOffset{uint8_t(RatioOffsetZ + 1)};
	static constexpr uint16_t RatioMask{0x07FF};
	// Bit 11 of GYR_GAIN_UPD_3 (0x3A) lives in the MSB byte at offset +1, bit3.
	static constexpr uint8_t EnableMask{0x08};
	static_assert(RatioOffsetX == 6 and RatioOffsetY == 8 and RatioOffsetZ == 10 and EnableOffset == 11);

	if ((update.ratioX > RatioMask) or (update.ratioY > RatioMask) or (update.ratioZ > RatioMask)) {
		return false;
	}

	std::array<uint8_t, 16> featurePage{};
	if (!readFeaturePage(FeaturePage1, featurePage)) {
		return false;
	}

	auto setRatioWord = [&](uint8_t offset, uint16_t ratio) {
		uint16_t word = uint16_t(featurePage[offset]) | (uint16_t(featurePage[offset + 1]) << 8);
		word = uint16_t((word & ~RatioMask) | (ratio & RatioMask));
		featurePage[offset] = uint8_t(word & 0xFF);
		featurePage[offset + 1] = uint8_t((word >> 8) & 0xFF);
	};

	setRatioWord(RatioOffsetX, update.ratioX);
	setRatioWord(RatioOffsetY, update.ratioY);
	setRatioWord(RatioOffsetZ, update.ratioZ);

	if (update.enable) {
		featurePage[EnableOffset] |= EnableMask;
	}
	else {
		featurePage[EnableOffset] &= ~EnableMask;
	}

	return writeFeaturePage(FeaturePage1, featurePage);
}

template<Bmi270Transport Transport>
std::optional<bmi270::GyroGainStatus>
Bmi270<Transport>::getGyroGainStatus()
{
	static constexpr uint8_t FeaturePage0{0};
	static constexpr uint8_t StatusOffset{8};
	static constexpr uint8_t SaturationXMask{0x01};
	static constexpr uint8_t SaturationYMask{0x02};
	static constexpr uint8_t SaturationZMask{0x04};
	static constexpr uint8_t TriggerStatusMask{0x38};
	static constexpr uint8_t TriggerStatusShift{3};

	std::array<uint8_t, 16> featurePage{};
	if (!readFeaturePage(FeaturePage0, featurePage)) {
		return {};
	}

	const uint8_t value = featurePage[StatusOffset];
	return GyroGainStatus{
		.saturationX = bool(value & SaturationXMask),
		.saturationY = bool(value & SaturationYMask),
		.saturationZ = bool(value & SaturationZMask),
		.triggerStatus = uint8_t((value & TriggerStatusMask) >> TriggerStatusShift),
	};
}

template<Bmi270Transport Transport>
std::optional<bmi270::GyroUserGain>
Bmi270<Transport>::getGyroUserGain()
{
	static constexpr uint8_t UserGainMask{0x7F};

	const auto data = this->readRegisters(Register::GyroUserGain0, 3);
	if (data.empty()) {
		return {};
	}

	return GyroUserGain{
		.x = int8_t(data[0] & UserGainMask),
		.y = int8_t(data[1] & UserGainMask),
		.z = int8_t(data[2] & UserGainMask),
	};
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::applyGyroGainUpdate(bool enableCompensation)
{
	static constexpr uint8_t GyroEnableMask{0x02};
	static constexpr uint8_t GyroGainEnableMask{0x80};
	static constexpr uint8_t MaxPollCount{100};
	static constexpr std::chrono::milliseconds PollDelay{10};

	bool ok = true;
	bool restoreOk = true;

	const auto savedPowerControl = getPowerControl();
	const auto savedPowerConfiguration = getPowerConfiguration();
	if (!savedPowerControl or !savedPowerConfiguration) {
		return false;
	}

	const bool apsWasEnabled = bool(*savedPowerConfiguration & PowerConfiguration::AdvancedPowerSave);
	if (apsWasEnabled) {
		ok &= setAdvancedPowerSave(false);
	}

	if (ok) {
		auto gainUpdate = getGyroGainUpdate();
		if (!gainUpdate) {
			ok = false;
		}
		else {
			gainUpdate->enable = true;
			ok &= setGyroGainUpdate(*gainUpdate);
		}
	}

	if (ok) {
		PowerControl_t powerControl{savedPowerControl->value};
		powerControl.value &= ~GyroEnableMask;
		ok &= setPowerControl(powerControl);
	}

	if (ok) {
		ok &= sendCommand(Command::ApplyUserGain);
	}

	if (ok) {
		bool completed = false;
		for (uint8_t count = 0; count < MaxPollCount; ++count)
		{
			modm::this_fiber::sleep_for(PollDelay);

			const auto gainUpdate = getGyroGainUpdate();
			if (!gainUpdate) {
				ok = false;
				break;
			}
			if (!gainUpdate->enable) {
				completed = true;
				break;
			}
		}

		if (!completed) {
			ok = false;
		}
	}

	if (ok and enableCompensation) {
		const auto offset6 = readRegister(Register::Offset6);
		if (!offset6) {
			ok = false;
		}
		else {
			ok &= this->writeRegister(Register::Offset6, uint8_t(*offset6 | GyroGainEnableMask));
			modm::this_fiber::sleep_for(WriteTimeout);
		}
	}

	restoreOk &= setPowerControl(*savedPowerControl);
	if (apsWasEnabled) {
		restoreOk &= setAdvancedPowerSave(true);
	}

	return ok and restoreOk;
}

template<Bmi270Transport Transport>
std::optional<bmi270::PowerConfiguration_t>
Bmi270<Transport>::getPowerConfiguration()
{
	const auto value = readRegister(Register::PowerConf);
	if (!value) {
		return {};
	}

	return PowerConfiguration_t{*value};
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::setPowerConfiguration(PowerConfiguration_t configuration)
{
	const bool ok = this->writeRegister(Register::PowerConf, configuration.value);
	modm::this_fiber::sleep_for(PowerModeTimeout);
	return ok;
}

template<Bmi270Transport Transport>
std::optional<bmi270::PowerControl_t>
Bmi270<Transport>::getPowerControl()
{
	const auto value = readRegister(Register::PowerCtrl);
	if (!value) {
		return {};
	}

	return PowerControl_t{*value};
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::sendCommand(Command command)
{
	const bool ok = this->writeRegister(Register::Command, static_cast<uint8_t>(command));
	modm::this_fiber::sleep_for(WriteTimeout);
	return ok;
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::setAccRate(AccRate rate)
{
	const bool ok = this->writeRegister(Register::AccConf, static_cast<uint8_t>(rate));
	modm::this_fiber::sleep_for(WriteTimeout);
	return ok;
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::setAccRange(AccRange range)
{
	const bool ok = this->writeRegister(Register::AccRange, static_cast<uint8_t>(range));
	modm::this_fiber::sleep_for(WriteTimeout);
	if (ok) {
		accRange_ = range;
	}
	return ok;
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::setGyroRate(GyroRate rate)
{
	const bool ok = this->writeRegister(Register::GyroConf, static_cast<uint8_t>(rate));
	modm::this_fiber::sleep_for(WriteTimeout);
	return ok;
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::setGyroRange(GyroRange range)
{
	const bool ok = this->writeRegister(Register::GyroRange, static_cast<uint8_t>(range));
	modm::this_fiber::sleep_for(WriteTimeout);
	if (ok) {
		gyroRange_ = range;
	}
	return ok;
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::setPowerControl(PowerControl_t control)
{
	const bool ok = this->writeRegister(Register::PowerCtrl, control.value);
	modm::this_fiber::sleep_for(WriteTimeout);
	return ok;
}

template<Bmi270Transport Transport>
std::optional<uint16_t>
Bmi270<Transport>::getFifoLength()
{
	static constexpr uint8_t FifoLengthMsbMask{0x3F};

	const auto data = this->readRegisters(Register::FifoLength0, 2);
	if (data.empty()) {
		return {};
	}

	return uint16_t(uint16_t(data[0]) | (uint16_t(data[1] & FifoLengthMsbMask) << 8));
}

template<Bmi270Transport Transport>
std::optional<bmi270::FifoDownsampling>
Bmi270<Transport>::getFifoDownsampling()
{
	static constexpr uint8_t GyroDownsamplingMask{0x07};
	static constexpr uint8_t GyroFilterMask{0x08};
	static constexpr uint8_t AccDownsamplingMask{0x70};
	static constexpr uint8_t AccFilterMask{0x80};

	const auto value = readRegister(Register::FifoDowns);
	if (!value) {
		return {};
	}

	return FifoDownsampling{
		.gyroDownsampling = uint8_t(*value & GyroDownsamplingMask),
		.gyroFilterData = (*value & GyroFilterMask) ? FifoFilterData::Filtered : FifoFilterData::Unfiltered,
		.accDownsampling = uint8_t((*value & AccDownsamplingMask) >> 4),
		.accFilterData = (*value & AccFilterMask) ? FifoFilterData::Filtered : FifoFilterData::Unfiltered,
	};
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::setFifoDownsampling(FifoDownsampling configuration)
{
	if ((configuration.gyroDownsampling > 0x07) or (configuration.accDownsampling > 0x07)) {
		return false;
	}

	uint8_t value = uint8_t(configuration.gyroDownsampling & 0x07);
	if (configuration.gyroFilterData == FifoFilterData::Filtered) {
		value |= 0x08;
	}
	value |= uint8_t((configuration.accDownsampling & 0x07) << 4);
	if (configuration.accFilterData == FifoFilterData::Filtered) {
		value |= 0x80;
	}

	const bool ok = this->writeRegister(Register::FifoDowns, value);
	modm::this_fiber::sleep_for(WriteTimeout);
	return ok;
}

template<Bmi270Transport Transport>
std::optional<uint16_t>
Bmi270<Transport>::getFifoWatermark()
{
	const auto data = this->readRegisters(Register::FifoWtm0, 2);
	if (data.empty()) {
		return {};
	}

	return uint16_t(uint16_t(data[0]) | (uint16_t(data[1]) << 8));
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::setFifoWatermark(uint16_t watermark)
{
	const std::array<uint8_t, 2> data{
		uint8_t(watermark & 0xFF),
		uint8_t((watermark >> 8) & 0xFF),
	};
	const bool ok = this->writeRegisters(Register::FifoWtm0, std::span{data});
	modm::this_fiber::sleep_for(WriteTimeout);
	return ok;
}

template<Bmi270Transport Transport>
std::optional<bmi270::FifoConfiguration>
Bmi270<Transport>::getFifoConfiguration()
{
	static constexpr uint8_t StopOnFullMask{0x01};
	static constexpr uint8_t TimeEnableMask{0x02};
	static constexpr uint8_t TagInt1Mask{0x03};
	static constexpr uint8_t TagInt2Mask{0x0C};
	static constexpr uint8_t HeaderEnableMask{0x10};
	static constexpr uint8_t AuxEnableMask{0x20};
	static constexpr uint8_t AccEnableMask{0x40};
	static constexpr uint8_t GyroEnableMask{0x80};

	const auto data = this->readRegisters(Register::FifoConfig0, 2);
	if (data.empty()) {
		return {};
	}

	const uint8_t config0 = data[0];
	const uint8_t config1 = data[1];
	return FifoConfiguration{
		.stopOnFull = bool(config0 & StopOnFullMask),
		.timeEnable = bool(config0 & TimeEnableMask),
		.tagInt1 = static_cast<FifoTagInterrupt>(config1 & TagInt1Mask),
		.tagInt2 = static_cast<FifoTagInterrupt>((config1 & TagInt2Mask) >> 2),
		.headerEnable = bool(config1 & HeaderEnableMask),
		.auxEnable = bool(config1 & AuxEnableMask),
		.accEnable = bool(config1 & AccEnableMask),
		.gyroEnable = bool(config1 & GyroEnableMask),
	};
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::setFifoConfiguration(FifoConfiguration configuration)
{
	uint8_t config0 = 0;
	if (configuration.stopOnFull) {
		config0 |= 0x01;
	}
	if (configuration.timeEnable) {
		config0 |= 0x02;
	}

	uint8_t config1 = uint8_t(uint8_t(configuration.tagInt1) & 0x03);
	config1 |= uint8_t((uint8_t(configuration.tagInt2) & 0x03) << 2);
	if (configuration.headerEnable) {
		config1 |= 0x10;
	}
	if (configuration.auxEnable) {
		config1 |= 0x20;
	}
	if (configuration.accEnable) {
		config1 |= 0x40;
	}
	if (configuration.gyroEnable) {
		config1 |= 0x80;
	}

	const std::array<uint8_t, 2> data{config0, config1};
	const bool ok = this->writeRegisters(Register::FifoConfig0, std::span{data});
	modm::this_fiber::sleep_for(WriteTimeout);
	return ok;
}

template<Bmi270Transport Transport>
std::optional<uint16_t>
Bmi270<Transport>::readFifo(std::span<uint8_t> buffer)
{
	const auto length = getFifoLength();
	if (!length) {
		return {};
	}

	if (*length > buffer.size()) {
		return {};
	}

	if (!this->readFifoData(buffer.subspan(0, *length))) {
		return {};
	}

	return length;
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::flushFifo()
{
	const bool ok = this->writeRegister(Register::Command, FifoFlushCommand);
	modm::this_fiber::sleep_for(WriteTimeout);
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
	const bool ok = this->writeRegister(Register::Command, SoftResetCommand);
	modm::this_fiber::sleep_for(ResetTimeout);
	if (!ok) {
		return false;
	}

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

	if (!this->writeRegister(Register::InitControl, InitControlLoadDisabled)) {
		return false;
	}
	modm::this_fiber::sleep_for(PowerModeTimeout);

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
		modm::this_fiber::sleep_for(WriteTimeout);
	}

	if (!this->writeRegister(Register::InitControl, InitControlLoadEnabled)) {
		return false;
	}
	modm::this_fiber::sleep_for(ConfigLoadTimeout);

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

	if (!this->writeRegister(Register::InitControl, InitControlLoadDisabled)) {
		return false;
	}
	modm::this_fiber::sleep_for(PowerModeTimeout);

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
		modm::this_fiber::sleep_for(WriteTimeout);
	}

	if (!this->writeRegister(Register::InitControl, InitControlLoadEnabled)) {
		return false;
	}
	modm::this_fiber::sleep_for(ConfigLoadTimeout);

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

	const bool ok = this->writeRegister(Register::PowerConf, value);
	modm::this_fiber::sleep_for(PowerModeTimeout);
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
bool
Bmi270<Transport>::readFeaturePage(uint8_t page, std::array<uint8_t, 16>& data)
{
	if (!this->writeRegister(Register::FeatPage, page)) {
		return false;
	}
	modm::this_fiber::sleep_for(WriteTimeout);

	const auto pageData = this->readRegisters(Register::Features, data.size());
	if (pageData.empty()) {
		return false;
	}

	std::copy_n(pageData.begin(), data.size(), data.begin());
	return true;
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::writeFeaturePage(uint8_t page, const std::array<uint8_t, 16>& data)
{
	if (!this->writeRegister(Register::FeatPage, page)) {
		return false;
	}
	modm::this_fiber::sleep_for(WriteTimeout);

	const bool ok = this->writeRegisters(Register::Features, std::span{data});
	modm::this_fiber::sleep_for(WriteTimeout);
	return ok;
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
