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

#ifndef MODM_BMI270_HPP
#define MODM_BMI270_HPP

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <modm/architecture/interface/accessor.hpp>
#include <modm/architecture/interface/gpio.hpp>
#include <modm/architecture/interface/i2c_device.hpp>
#include <modm/architecture/interface/register.hpp>
#include <modm/architecture/interface/spi_device.hpp>
#include <modm/math/geometry/vector3.hpp>
#include <modm/processing/fiber.hpp>
#include <modm/processing/timer/timeout.hpp>
#include <optional>
#include <span>

namespace modm
{

/// @ingroup modm_driver_bmi270
struct bmi270
{
	enum class AccRange : uint8_t
	{
		Range2g = 0x00,  //< +-2g
		Range4g = 0x01,  //< +-4g
		Range8g = 0x02,  //< +-8g
		Range16g = 0x03  //< +-16g
	};

	enum class GyroRange : uint8_t
	{
		Range2000dps = 0x00,  //< +-2000 deg/s
		Range1000dps = 0x01,  //< +-1000 deg/s
		Range500dps = 0x02,   //< +-500 deg/s
		Range250dps = 0x03,   //< +-250 deg/s
		Range125dps = 0x04    //< +-125 deg/s
	};

	enum class AccRate : uint8_t
	{
		Rate25Hz_Osr4 = 0x06 | 0x80,
		Rate25Hz_Normal = 0x06 | 0x20 | 0x80,
		Rate50Hz_Normal = 0x07 | 0x20 | 0x80,
		Rate100Hz_Normal = 0x08 | 0x20 | 0x80,
		Rate200Hz_Normal = 0x09 | 0x20 | 0x80,
		Rate400Hz_Normal = 0x0A | 0x20 | 0x80,
		Rate800Hz_Normal = 0x0B | 0x20 | 0x80,
		Rate1600Hz_Normal = 0x0C | 0x20 | 0x80
	};

	enum class GyroRate : uint8_t
	{
		Rate25Hz_Normal = 0x06 | 0x20 | 0x40 | 0x80,
		Rate50Hz_Normal = 0x07 | 0x20 | 0x40 | 0x80,
		Rate100Hz_Normal = 0x08 | 0x20 | 0x40 | 0x80,
		Rate200Hz_Normal = 0x09 | 0x20 | 0x40 | 0x80,
		Rate400Hz_Normal = 0x0A | 0x20 | 0x40 | 0x80,
		Rate800Hz_Normal = 0x0B | 0x20 | 0x40 | 0x80,
		Rate1600Hz_Normal = 0x0C | 0x20 | 0x40 | 0x80
	};

	enum class Status : uint8_t
	{
		AccDataReady = Bit7,
		GyroDataReady = Bit6,
		CommandReady = Bit4
	};
	MODM_FLAGS8(Status);

	enum class PowerControl : uint8_t
	{
		Auxiliary = Bit0,
		Gyroscope = Bit1,
		Accelerometer = Bit2,
		Temperature = Bit3
	};
	MODM_FLAGS8(PowerControl);

	enum class PowerConfiguration : uint8_t
	{
		AdvancedPowerSave = Bit0,
		FifoSelfWakeup = Bit1,
		FastPowerUp = Bit2
	};
	MODM_FLAGS8(PowerConfiguration);

	struct AccData
	{
		/// acceleration in milli-g
		Vector3f
		getFloat() const;

		Vector3i raw;
		AccRange range;
	};

	struct GyroData
	{
		/// angular rate in deg/s
		Vector3f
		getFloat() const;

		Vector3i raw;
		GyroRange range;
	};

	struct Data
	{
		AccData acc;
		GyroData gyro;
		uint32_t sensorTime;
	};
};

/// @cond
namespace bmi270_private
{
static constexpr std::size_t ConfigurationSize{8192};
extern modm::accessor::Flash<uint8_t> configuration;
}  // namespace bmi270_private
/// @endcond

/// @ingroup modm_driver_bmi270
struct Bmi270TransportBase
{
	enum class Register : uint8_t
	{
		ChipId = 0x00,
		Error = 0x02,
		Status = 0x03,
		AccDataXLow = 0x0C,
		GyroDataXLow = 0x12,
		SensorTime0 = 0x18,
		InterruptStatus0 = 0x1C,
		InterruptStatus1 = 0x1D,
		InternalStatus = 0x21,
		AccConf = 0x40,
		AccRange = 0x41,
		GyroConf = 0x42,
		GyroRange = 0x43,
		InitControl = 0x59,
		InitAddress0 = 0x5B,
		InitAddress1 = 0x5C,
		InitData = 0x5E,
		PowerConf = 0x7C,
		PowerCtrl = 0x7D,
		Command = 0x7E
	};

	static constexpr uint8_t MaxRegisterSequence{32};
};

/// @ingroup modm_driver_bmi270
template<typename T>
concept Bmi270Transport = requires(T& transport, Bmi270TransportBase::Register reg, uint8_t count,
								   uint8_t data, const std::array<uint8_t, 2>& values) {
	{ transport.initialize() };
	{ transport.readRegisters(reg, count) } -> std::same_as<std::span<uint8_t>>;
	{ transport.writeRegister(reg, data) } -> std::same_as<bool>;
	{ transport.writeRegisters(reg, std::span{values}) } -> std::same_as<bool>;
};

/**
 * BMI270 SPI transport. Pass as template parameter to Bmi270 driver class.
 *
 * @tparam SpiMaster SPI master the device is connected to
 * @tparam Cs chip-select GPIO
 * @ingroup modm_driver_bmi270
 */
template<typename SpiMaster, typename Cs>
class Bmi270SpiTransport : public Bmi270TransportBase, public SpiDevice<SpiMaster>
{
public:
	Bmi270SpiTransport() = default;

	Bmi270SpiTransport(const Bmi270SpiTransport&) = delete;

	Bmi270SpiTransport&
	operator=(const Bmi270SpiTransport&) = delete;

	void
	initialize();

	std::span<uint8_t>
	readRegisters(Register startReg, uint8_t count);

	bool
	writeRegister(Register reg, uint8_t data);

	bool
	writeRegisters(Register startReg, std::span<const uint8_t> data);

private:
	static constexpr uint8_t ReadFlag{0x80};
	std::array<uint8_t, MaxRegisterSequence + 2> rxBuffer_{};
	std::array<uint8_t, MaxRegisterSequence + 2> txBuffer_{};
};

/**
 * BMI270 I2C transport. Pass as template parameter to Bmi270 driver class.
 *
 * @tparam I2cMaster I2C master the device is connected to
 * @ingroup modm_driver_bmi270
 */
template<typename I2cMaster>
class Bmi270I2cTransport : public Bmi270TransportBase, public I2cDevice<I2cMaster>
{
public:
	explicit Bmi270I2cTransport(uint8_t address = 0x68);

	Bmi270I2cTransport(const Bmi270I2cTransport&) = delete;

	Bmi270I2cTransport&
	operator=(const Bmi270I2cTransport&) = delete;

	void
	initialize();

	std::span<uint8_t>
	readRegisters(Register startReg, uint8_t count);

	bool
	writeRegister(Register reg, uint8_t data);

	bool
	writeRegisters(Register startReg, std::span<const uint8_t> data);

private:
	std::array<uint8_t, MaxRegisterSequence + 1> buffer_{};
};

/**
 * Bosch BMI270 IMU
 *
 * The device contains an accelerometer and a gyroscope.
 *
 * @tparam Transport Transport layer (use @ref Bmi270SpiTransport or @ref Bmi270I2cTransport)
 * @ingroup modm_driver_bmi270
 */
template<Bmi270Transport Transport>
class Bmi270 : public bmi270, public Transport
{
public:
	/// @arg transportArgs Arguments to transport layer.
	template<typename... Args>
	Bmi270(Args... transportArgs);

	/// Initialize device. Call before any other member function.
	/// @return true on success, false on error
	bool
	initialize(std::span<const uint8_t> configFile = {});

	/// Read acceleration and gyroscope data and sensor time from one burst read.
	std::optional<Data>
	readData();

	std::optional<AccData>
	readAccData();

	std::optional<GyroData>
	readGyroData();

	bool
	readAccDataReady();

	bool
	readGyroDataReady();

	bool
	readCommandReady();

	bool
	setAccRate(AccRate rate);

	bool
	setAccRange(AccRange range);

	bool
	setGyroRate(GyroRate rate);

	bool
	setGyroRange(GyroRange range);

	/// Enable/disable sensor power domains.
	bool
	setPowerControl(PowerControl_t control);

	bool
	flushFifo();

private:
	using Register = Transport::Register;

	static constexpr std::chrono::microseconds WriteTimeout{2};
	static constexpr std::chrono::microseconds PowerModeTimeout{450};
	static constexpr std::chrono::milliseconds ResetTimeout{2};
	static constexpr std::chrono::milliseconds ConfigLoadTimeout{20};

	static constexpr uint8_t ChipId{0x24};
	static constexpr uint8_t SoftResetCommand{0xB6};
	static constexpr uint8_t FifoFlushCommand{0xB0};
	static constexpr uint8_t InitControlLoadDisabled{0x00};
	static constexpr uint8_t InitControlLoadEnabled{0x01};
	static constexpr uint8_t InternalStatusInitOk{0x01};

	bool
	checkChipId();

	bool
	reset();

	bool
	uploadConfig(std::span<const uint8_t> configFile);

	bool
	uploadConfig(modm::accessor::Flash<uint8_t> configFile, std::size_t configSize);

	bool
	setAdvancedPowerSave(bool enable);

	bool
	enableSensors();

	std::optional<uint8_t>
	readRegister(Register reg);

	modm::PreciseTimeout timer_;
	AccRange accRange_{AccRange::Range2g};
	GyroRange gyroRange_{GyroRange::Range2000dps};
};

}  // namespace modm

#include "bmi270_impl.hpp"

#endif  // MODM_BMI270_HPP
