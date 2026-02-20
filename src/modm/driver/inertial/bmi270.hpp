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

	enum class InterruptOutputLevel : uint8_t
	{
		ActiveLow = 0x00,
		ActiveHigh = 0x01
	};

	enum class InterruptOutputType : uint8_t
	{
		PushPull = 0x00,
		OpenDrain = 0x01
	};

	enum class InterruptLatch : uint8_t
	{
		None = 0x00,
		Permanent = 0x01
	};

	enum class InternalMessage : uint8_t
	{
		NotInitialized = 0x00,
		Initialized = 0x01,
		InitError = 0x02,
		DriverError = 0x03,
		SensorError = 0x04,
		NvmError = 0x05,
		StartupError = 0x06,
		CompatibilityError = 0x07
	};

	enum class PullUpConfiguration : uint8_t
	{
		Off = 0x00,
		PullUp40k = 0x01,
		PullUp10k = 0x02,
		PullUp2k = 0x03
	};

	enum class InterfaceSpiMode : uint8_t
	{
		Spi4Wire = 0x00,
		Spi3Wire = 0x01
	};

	enum class DriveStrength : uint8_t
	{
		Level0 = 0b000,
		Level1 = 0b001,
		Level2 = 0b010,
		Level3 = 0b011,
		Level4 = 0b100,
		Level5 = 0b101,
		Level6 = 0b110,
		Level7 = 0b111
	};

	enum class Command : uint8_t
	{
		TriggerGyro = 0x02,
		ApplyUserGain = 0x03,
		ProgramNvm = 0xA0,
		FlushFifo = 0xB0,
		SoftReset = 0xB6
	};

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

	struct ErrorInfo
	{
		bool fatalError;
		uint8_t internalError;
		bool fifoError;
		bool auxError;
	};

	struct SensorStatus
	{
		bool accDataReady;
		bool gyroDataReady;
		bool auxDataReady;
		bool commandReady;
		bool auxBusy;
	};

	struct InternalStatus
	{
		InternalMessage message;
		bool axesRemapError;
		bool odr50HzError;
	};

	struct InterruptStatus
	{
		bool fifoFull;
		bool fifoWatermark;
		bool error;
		bool auxDataReady;
		bool gyroDataReady;
		bool accDataReady;
	};

	struct InterruptIoControl
	{
		InterruptOutputLevel level;
		InterruptOutputType outputType;
		bool outputEnable;
		bool inputEnable;
	};

	struct InterruptMapData
	{
		bool int1FifoFull;
		bool int1FifoWatermark;
		bool int1DataReady;
		bool int1Error;
		bool int2FifoFull;
		bool int2FifoWatermark;
		bool int2DataReady;
		bool int2Error;
	};

	struct ErrorInterruptMask
	{
		bool fatalError;
		bool internalError;
		bool fifoError;
		bool auxError;
	};

	struct InternalError
	{
		bool longProcessingTime;
		bool fatalError;
		bool featureEngineDisabled;
	};

	struct GyroCrtConfig
	{
		bool running;
		bool readyForDownload;
	};

	struct InterfaceConfig
	{
		InterfaceSpiMode primarySpiMode;
		InterfaceSpiMode oisSpiMode;
		bool oisEnabled;
		bool auxEnabled;
	};

	struct DriveConfig
	{
		DriveStrength ioPadDrv1;
		bool ioPadI2cBoost1;
		DriveStrength ioPadDrv2;
		bool ioPadI2cBoost2;
	};

	struct AccOffsets
	{
		uint8_t x;
		uint8_t y;
		uint8_t z;
	};

	struct GyroOffsets
	{
		uint16_t x;
		uint16_t y;
		uint16_t z;
		bool offsetEnabled;
		bool gainEnabled;
	};

	struct GyroGainUpdate
	{
		/// GYR_GAIN_UPD_1 @ feature register 0x36, bits [10:0] (1.10 fixed-point).
		uint16_t ratioX;
		/// GYR_GAIN_UPD_2 @ feature register 0x38, bits [10:0] (1.10 fixed-point).
		uint16_t ratioY;
		/// GYR_GAIN_UPD_3 @ feature register 0x3A, bits [10:0] (1.10 fixed-point).
		uint16_t ratioZ;
		/// GYR_GAIN_UPD_3 bit 11 enable bit (applies all 3 gain ratios).
		/// This bit is auto-cleared by the IMU once the update command completes.
		bool enable;
	};

	struct GyroGainStatus
	{
		bool saturationX;
		bool saturationY;
		bool saturationZ;
		uint8_t triggerStatus;
	};

	struct GyroUserGain
	{
		int8_t x;
		int8_t y;
		int8_t z;
	};

	struct Temperature
	{
		bool valid;
		float celsius;
	};

	enum class FocAxis : uint8_t
	{
		X,
		Y,
		Z
	};

	struct AccelFocTarget
	{
		FocAxis axis;
		bool negative;
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
		Temperature0 = 0x22,
		FeatPage = 0x2F,
		Features = 0x30,
		AccConf = 0x40,
		AccRange = 0x41,
		GyroConf = 0x42,
		GyroRange = 0x43,
		ErrRegMask = 0x52,
		Int1IoCtrl = 0x53,
		Int2IoCtrl = 0x54,
		IntLatch = 0x55,
		IntMapData = 0x58,
		InitControl = 0x59,
		InitAddress0 = 0x5B,
		InitAddress1 = 0x5C,
		InitData = 0x5E,
		InternalError = 0x5F,
		AuxIfTrim = 0x68,
		GyroCrtConf = 0x69,
		NvmConf = 0x6A,
		IfConf = 0x6B,
		Drv = 0x6C,
		NvConf = 0x70,
		Offset0 = 0x71,
		Offset3 = 0x74,
		Offset6 = 0x77,
		GyroUserGain0 = 0x78,
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

	std::optional<uint8_t>
	getChipId();

	std::optional<ErrorInfo>
	getErrors();

	std::optional<SensorStatus>
	getStatus();

	std::optional<InternalStatus>
	getInternalStatus();

	std::optional<Temperature>
	getTemperature();

	std::optional<InterruptStatus>
	getInterruptStatus();

	std::optional<ErrorInterruptMask>
	getErrorInterruptMask();

	bool
	setErrorInterruptMask(ErrorInterruptMask mask);

	std::optional<InterruptIoControl>
	getInt1IoControl();

	bool
	setInt1IoControl(InterruptIoControl control);

	std::optional<InterruptIoControl>
	getInt2IoControl();

	bool
	setInt2IoControl(InterruptIoControl control);

	std::optional<InterruptLatch>
	getInterruptLatch();

	bool
	setInterruptLatch(InterruptLatch mode);

	std::optional<InterruptMapData>
	getInterruptMapData();

	bool
	setInterruptMapData(InterruptMapData map);

	std::optional<InternalError>
	getInternalError();

	std::optional<PullUpConfiguration>
	getPullUpConfiguration();

	bool
	setPullUpConfiguration(PullUpConfiguration configuration);

	std::optional<GyroCrtConfig>
	getGyroCrtConfig();

	bool
	setGyroCrtConfig(GyroCrtConfig configuration);

	std::optional<bool>
	getNvmCrtEnabled();

	bool
	setNvmCrtEnabled(bool enable);

	bool
	triggerGyroCrt();

	/// Perform component retrim (CRT) for the gyroscope.
	///
	/// If no configuration blob is passed, the built-in BMI270 configuration is used.
	bool
	doCrt(std::span<const uint8_t> configFile = {});

	std::optional<InterfaceConfig>
	getInterfaceConfig();

	bool
	setInterfaceConfig(InterfaceConfig configuration);

	std::optional<DriveConfig>
	getDriveConfig();

	bool
	setDriveConfig(DriveConfig configuration);

	std::optional<AccOffsets>
	getAccOffsets();

	bool
	setAccOffsets(AccOffsets offsets);

	std::optional<GyroOffsets>
	getGyroOffsets();

	bool
	setGyroOffsets(GyroOffsets offsets);

	/// Read gyroscope user-gain update ratios from volatile feature memory:
	/// GYR_GAIN_UPD_1/2/3 at 0x36/0x38/0x3A (feature page 1).
	std::optional<GyroGainUpdate>
	getGyroGainUpdate();

	/// Write gyroscope user-gain update ratios to volatile feature memory:
	/// GYR_GAIN_UPD_1/2/3 at 0x36/0x38/0x3A (feature page 1).
	///
	/// Ratio fields use bits [10:0] (1.10 fixed-point), valid range: 0x000..0x7FF.
	/// Enable uses bit 11 of GYR_GAIN_UPD_3.
	bool
	setGyroGainUpdate(GyroGainUpdate update);

	/// Read user-gain update status from feature output memory.
	std::optional<GyroGainStatus>
	getGyroGainStatus();

	/// Read compensated gyroscope user-gain values.
	std::optional<GyroUserGain>
	getGyroUserGain();

	/// Trigger manual user-gain update (`CMD=usr_gain`) using the values in
	/// @ref GyroGainUpdate.
	///
	/// The function disables gyroscope power, waits for completion
	/// (`GyroGainUpdate.enable -> 0`), optionally enables gain compensation and
	/// restores previous gyro power state.
	bool
	applyGyroGainUpdate(bool enableCompensation = true);

	/// Perform accelerometer fast offset compensation.
	///
	/// Keep the board stationary in the orientation matching \p target.
	bool
	performAccelFoc(AccelFocTarget target);

	/// Perform gyroscope fast offset compensation.
	///
	/// Keep the board stationary while this function runs.
	bool
	performGyroFoc();

	std::optional<PowerConfiguration_t>
	getPowerConfiguration();

	bool
	setPowerConfiguration(PowerConfiguration_t configuration);

	std::optional<PowerControl_t>
	getPowerControl();

	bool
	sendCommand(Command command);

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
	static constexpr std::chrono::milliseconds ResetTimeout{45};
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

	bool
	readFeaturePage(uint8_t page, std::array<uint8_t, 16>& data);

	bool
	writeFeaturePage(uint8_t page, const std::array<uint8_t, 16>& data);

	std::optional<uint8_t>
	readRegister(Register reg);

	AccRange accRange_{AccRange::Range2g};
	GyroRange gyroRange_{GyroRange::Range2000dps};
};

}  // namespace modm

#include "bmi270_impl.hpp"

#endif  // MODM_BMI270_HPP
