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
	static constexpr uint8_t RunningMask{0x04};

	const auto current = readRegister(Register::GyroCrtConf);
	if (!current) {
		return false;
	}

	uint8_t value = *current;
	if (configuration.running) {
		value |= RunningMask;
	}
	else {
		value &= ~RunningMask;
	}

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
	return doCrtAndReadGainUpdate(configFile).has_value();
}

template<Bmi270Transport Transport>
std::optional<bmi270::GyroGainUpdate>
Bmi270<Transport>::doCrtAndReadGainUpdate(std::span<const uint8_t> configFile)
{
	static constexpr uint8_t FeaturePage0{0};
	static constexpr uint8_t FeaturePage1{1};
	static constexpr uint8_t FeaturePageSize{16};
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
	static constexpr uint16_t CrtMinBurstBytes{2};
	static constexpr uint16_t CrtMaxBurstBytes{uint16_t(255u * 2u)};
	static constexpr uint8_t GyroGainEnableMask{0x80};

	auto getMaxBurstLengthWords = [&]() -> std::optional<uint8_t> {
		std::array<uint8_t, FeaturePageSize> page{};
		if (!readFeaturePage(FeaturePage1, page)) {
			return {};
		}
		return page[MaxBurstLengthOffset];
	};

	auto setMaxBurstLengthBytes = [&](uint16_t writeLengthBytes) -> bool {
		std::array<uint8_t, FeaturePageSize> page{};
		if (!readFeaturePage(FeaturePage1, page)) {
			return false;
		}

		uint16_t burstWords = writeLengthBytes / 2u;
		if (burstWords > 255u) {
			burstWords = 255u;
		}
		page[MaxBurstLengthOffset] = uint8_t(burstWords);
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

	auto getCrtRunning = [&]() -> std::optional<bool> {
		const auto value = readRegister(Register::GyroCrtConf);
		if (!value) {
			return {};
		}
		return bool(*value & 0x04);
	};

	auto getReadyForDownload = [&]() -> std::optional<bool> {
		const auto value = readRegister(Register::GyroCrtConf);
		if (!value) {
			return {};
		}
		return bool(*value & 0x08);
	};

	auto setCrtRunning = [&](bool enable) -> bool {
		GyroCrtConfig config{};
		config.running = enable;
		config.readyForDownload = false;
		return setGyroCrtConfig(config);
	};

	auto waitStRunningComplete = [&]() -> bool {
		for (uint8_t retry = 0; retry < CrtRunningRetryCount; ++retry)
		{
			const auto running = getCrtRunning();
			if (!running) {
				return false;
			}
			if (!*running) {
				return true;
			}
			modm::this_fiber::sleep_for(CrtRunningDelay);
		}
		return false;
	};

	auto waitReadyForDownloadToggle = [&](bool previous) -> bool {
		bool toggled = false;
		for (uint8_t retry = 0; retry < CrtReadyRetryCount; ++retry)
		{
			const auto ready = getReadyForDownload();
			if (!ready) {
				return false;
			}
			if (*ready != previous) {
				toggled = true;
				break;
			}
			modm::this_fiber::sleep_for(CrtReadyDelay);
		}

		if (!toggled) {
			return false;
		}

		const auto running = getCrtRunning();
		return running.has_value() and *running;
	};

	auto processCrtDownload = [&](bool lastChunk) -> bool {
		const auto readyForDownload = getReadyForDownload();
		if (!readyForDownload) {
			return false;
		}

		if (!sendCommand(Command::TriggerGyro)) {
			return false;
		}

		if (!lastChunk and !waitReadyForDownloadToggle(*readyForDownload)) {
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

	auto writeCrtConfigFile = [&](uint16_t writeLengthBytes) -> bool {
		if (writeLengthBytes < CrtMinBurstBytes) {
			writeLengthBytes = CrtMinBurstBytes;
		}
		writeLengthBytes = std::min<uint16_t>(writeLengthBytes, CrtMaxBurstBytes);
		writeLengthBytes &= ~uint16_t{1};
		if (writeLengthBytes == 0) {
			return false;
		}

		const std::size_t configEnd = CrtConfigStartIndex + CrtConfigLength;
		if (!configFile.empty() and configFile.size() < configEnd) {
			return false;
		}

		auto fillChunk = [&](std::size_t index, std::span<uint8_t> chunk) {
			if (!configFile.empty()) {
				std::copy_n(configFile.begin() + index, chunk.size(), chunk.begin());
			}
			else {
				for (std::size_t ii = 0; ii < chunk.size(); ++ii) {
					chunk[ii] = bmi270_private::configuration[index + ii];
				}
			}
		};

		const uint16_t remainder = uint16_t(CrtConfigLength % writeLengthBytes);
		const std::size_t balanceEnd = configEnd - remainder;
		std::array<uint8_t, Transport::MaxRegisterSequence> chunkBuffer{};

		if (remainder == 0)
		{
			for (std::size_t index = CrtConfigStartIndex; index < configEnd; index += writeLengthBytes)
			{
				const bool lastChunk = (index >= (configEnd - writeLengthBytes));
				auto chunk = std::span{chunkBuffer}.subspan(0, writeLengthBytes);
				fillChunk(index, chunk);
				if (!writeInitBytes(uint16_t(index), chunk) or !processCrtDownload(lastChunk)) {
					return false;
				}
			}
			return true;
		}

		for (std::size_t index = CrtConfigStartIndex; index < balanceEnd; index += writeLengthBytes)
		{
			auto chunk = std::span{chunkBuffer}.subspan(0, writeLengthBytes);
			fillChunk(index, chunk);
			if (!writeInitBytes(uint16_t(index), chunk) or !processCrtDownload(false)) {
				return false;
			}
		}

		if (!setMaxBurstLengthBytes(CrtMinBurstBytes)) {
			return false;
		}

		for (std::size_t index = balanceEnd; index < configEnd; index += CrtMinBurstBytes)
		{
			const bool lastChunk = (index >= (configEnd - CrtMinBurstBytes));
			auto chunk = std::span{chunkBuffer}.subspan(0, CrtMinBurstBytes);
			fillChunk(index, chunk);
			if (!writeInitBytes(uint16_t(index), chunk) or !processCrtDownload(lastChunk)) {
				return false;
			}
		}

		return true;
	};

	auto evaluateCrtResult = [&](uint16_t configuredBurstBytes) -> bool {
		const auto status = getGTriggerStatus();
		if (!status) {
			return false;
		}

		switch (*status)
		{
		case GTriggerNoError:
			return setMaxBurstLengthBytes(0);

		case GTriggerDownloadError:
		case GTriggerAbortError:
			(void)setMaxBurstLengthBytes(configuredBurstBytes);
			return false;

		case GTriggerPreconditionError:
		default:
			return false;
		}
	};

	auto disableFifoSensors = [&]() -> bool {
		FifoConfiguration config{
			.stopOnFull = false,
			.timeEnable = false,
			.tagInt1 = FifoTagInterrupt::Edge,
			.tagInt2 = FifoTagInterrupt::Edge,
			.headerEnable = false,
			.auxEnable = false,
			.accEnable = false,
			.gyroEnable = false,
		};
		return setFifoConfiguration(config);
	};

	auto crtPrepareSetup = [&](PowerControl_t savedPower) -> bool {
		PowerControl_t prepPower{savedPower.value};
		prepPower.value &= ~uint8_t(PowerControl::Gyroscope);
		if (!setPowerControl(prepPower)) {
			return false;
		}

		if (!disableFifoSensors()) {
			return false;
		}

		prepPower.value |= uint8_t(PowerControl::Accelerometer);
		if (!setPowerControl(prepPower)) {
			return false;
		}

		modm::this_fiber::sleep_for(1ms);
		return setAbortFeature(false);
	};

	auto getGyroGainEnable = [&]() -> std::optional<bool> {
		const auto offset6 = readRegister(Register::Offset6);
		if (!offset6) {
			return {};
		}
		return bool(*offset6 & GyroGainEnableMask);
	};

	auto setGyroGainEnable = [&](bool enable) -> bool {
		const auto offset6 = readRegister(Register::Offset6);
		if (!offset6) {
			return false;
		}

		uint8_t value = *offset6;
		if (enable) {
			value |= GyroGainEnableMask;
		}
		else {
			value &= ~GyroGainEnableMask;
		}

		const bool ok = this->writeRegister(Register::Offset6, value);
		modm::this_fiber::sleep_for(WriteTimeout);
		return ok;
	};

	const auto savedPowerControl = getPowerControl();
	const auto savedPowerConfiguration = getPowerConfiguration();
	const auto savedGyroGainEnable = getGyroGainEnable();
	if (!savedPowerControl or !savedPowerConfiguration or !savedGyroGainEnable) {
		return {};
	}

	const bool apsWasEnabled = bool(*savedPowerConfiguration & PowerConfiguration::AdvancedPowerSave);
	bool ok = true;
	bool restoreOk = true;

	if (apsWasEnabled) {
		ok &= setAdvancedPowerSave(false);
	}
	if (ok) {
		ok &= setGyroGainEnable(true);
	}

	if (ok) {
		const auto running = getCrtRunning();
		ok &= running.has_value() and !*running;
	}

	auto maxBurstLengthWords = uint8_t{0};
	if (ok) {
		const auto maxBurst = getMaxBurstLengthWords();
		if (!maxBurst) {
			ok = false;
		}
		else {
			maxBurstLengthWords = *maxBurst;
		}
	}

	if (ok) {
		ok &= setCrtRunning(true);
	}
	if (ok) {
		ok &= crtPrepareSetup(*savedPowerControl);
	}
	if (ok) {
		ok &= setSelfTestSelection(true);
	}

	uint16_t configuredBurstBytes = 0;
	if (ok and (maxBurstLengthWords == 0))
	{
		ok &= sendCommand(Command::TriggerGyro);
		ok &= waitStRunningComplete();
		if (ok) {
			ok &= evaluateCrtResult(0);
		}
	}
	else if (ok)
	{
		configuredBurstBytes = Transport::MaxRegisterSequence;
		if (configuredBurstBytes < CrtMinBurstBytes) {
			configuredBurstBytes = CrtMinBurstBytes;
		}
		configuredBurstBytes = std::min<uint16_t>(configuredBurstBytes, CrtMaxBurstBytes);
		if (configuredBurstBytes & 1u) {
			--configuredBurstBytes;
		}
		if (configuredBurstBytes == 0) {
			ok = false;
		}

		if (ok) {
			ok &= setMaxBurstLengthBytes(configuredBurstBytes);
		}

		bool downloadReady = false;
		if (ok) {
			const auto ready = getReadyForDownload();
			if (!ready) {
				ok = false;
			}
			else {
				downloadReady = *ready;
			}
		}
		if (ok) {
			ok &= sendCommand(Command::TriggerGyro);
		}
		if (ok) {
			ok &= waitReadyForDownloadToggle(downloadReady);
		}
		if (ok) {
			ok &= writeCrtConfigFile(configuredBurstBytes);
		}
		if (ok) {
			ok &= waitStRunningComplete();
		}
		if (ok) {
			ok &= evaluateCrtResult(configuredBurstBytes);
		}
	}

	std::optional<GyroGainUpdate> gainUpdate;
	if (ok) {
		gainUpdate = getGyroGainUpdate();
		ok &= gainUpdate.has_value();
	}

	restoreOk &= setPowerControl(*savedPowerControl);
	restoreOk &= setGyroGainEnable(*savedGyroGainEnable);
	if (apsWasEnabled) {
		restoreOk &= setAdvancedPowerSave(true);
	}

	if (!(ok and restoreOk)) {
		return {};
	}

	return gainUpdate;
}

template<Bmi270Transport Transport>
bool
Bmi270<Transport>::performAccelFoc(AccelFocTarget target,
								   uint16_t sampleCount,
								   std::chrono::microseconds sampleDelay,
								   AccRate rate)
{
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
	if ((sampleCount == 0) or savedAcc.empty() or !savedPowerControl or !savedPowerConfiguration) {
		return false;
	}

	const bool apsWasEnabled = bool(*savedPowerConfiguration & PowerConfiguration::AdvancedPowerSave);
	const uint8_t savedAccConf = savedAcc[0];
	const uint8_t savedAccRange = savedAcc[1] & 0x03;

	bool ok = true;
	bool restoreOk = true;

	ok &= setAccelOffsetCompensation(false);
	if (ok) {
		ok &= this->writeRegister(Register::AccConf, static_cast<uint8_t>(rate));
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

	modm::this_fiber::sleep_for(std::chrono::seconds(1));
	
	Vector3i average{0, 0, 0};
	if (ok) {
		int64_t sumX = 0;
		int64_t sumY = 0;
		int64_t sumZ = 0;

		for (uint16_t sample = 0; sample < sampleCount; ++sample)
		{
			modm::this_fiber::sleep_for(sampleDelay);

			const auto status = readRegister(Register::Status);
			if (!status or ((*status & uint8_t(Status::AccDataReady)) == 0)) {
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
			average[0] = int(sumX / sampleCount);
			average[1] = int(sumY / sampleCount);
			average[2] = int(sumZ / sampleCount);
		}
	}

	const auto toOffset = [](int avg, bool plusOneG, bool minusOneG) -> uint8_t {
		int target = 0;
		if (plusOneG) {
			target = 16384;
		}
		else if (minusOneG) {
			target = -16384;
		}

		int delta = target - avg;
		delta = std::clamp(delta / 64, -128, 127);
		return static_cast<uint8_t>(int8_t(delta));
	};

	if (ok) {
		AccOffsets offsets{};
		offsets.x = toOffset(average[0], target.axis == FocAxis::X and !target.negative, target.axis == FocAxis::X and target.negative);
		offsets.y = toOffset(average[1], target.axis == FocAxis::Y and !target.negative, target.axis == FocAxis::Y and target.negative);
		offsets.z = toOffset(average[2], target.axis == FocAxis::Z and !target.negative, target.axis == FocAxis::Z and target.negative);

		ok &= setAccOffsets(offsets);
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
Bmi270<Transport>::performGyroFoc(uint16_t sampleCount,
								  std::chrono::microseconds sampleDelay,
								  GyroRate rate)
{
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
	if ((sampleCount == 0) or savedGyro.empty() or !savedPowerControl or !savedPowerConfiguration or !savedOffsets) {
		return false;
	}

	const bool apsWasEnabled = bool(*savedPowerConfiguration & PowerConfiguration::AdvancedPowerSave);
	const uint8_t savedGyroConf = savedGyro[0];
	const uint8_t savedGyroRange = savedGyro[1] & 0x07;

	bool ok = true;
	bool restoreOk = true;

	ok &= setGyroOffsetCompensation(false);
	if (ok) {
		const std::array<uint8_t, 2> focConfig{static_cast<uint8_t>(rate), FocGyroRangeValue};
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

	modm::this_fiber::sleep_for(std::chrono::seconds(1));

	Vector3i average{0, 0, 0};
	if (ok) {
		int64_t sumX = 0;
		int64_t sumY = 0;
		int64_t sumZ = 0;

		for (uint16_t sample = 0; sample < sampleCount; ++sample)
		{
			modm::this_fiber::sleep_for(sampleDelay);

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
			average[0] = int(sumX / sampleCount);
			average[1] = int(sumY / sampleCount);
			average[2] = int(sumZ / sampleCount);
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
Bmi270<Transport>::setGyroUserGain(GyroUserGain gain)
{
	static constexpr uint8_t UserGainMask{0x7F};

	const std::array<uint8_t, 3> data{
		uint8_t(gain.x) & UserGainMask,
		uint8_t(gain.y) & UserGainMask,
		uint8_t(gain.z) & UserGainMask,
	};

	const bool ok = this->writeRegisters(Register::GyroUserGain0, std::span{data});
	modm::this_fiber::sleep_for(WriteTimeout);
	return ok;
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

} // namespace modm
