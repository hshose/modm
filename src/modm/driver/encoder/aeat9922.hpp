/*
 * Copyright (c) 2026, Lukas Wildberger
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#pragma once
#define MODM_AEAT9922_HPP

#include <cstdint>

#include <modm/architecture/interface/register.hpp>
#include <modm/architecture/interface/spi_device.hpp>
#include <modm/math/geometry/angle_int.hpp>
#include <modm/processing/fiber.hpp>

namespace modm
{

/// @ingroup modm_driver_aeat9922
struct aeat9922
{
	static constexpr uint8_t UnlockKey = 0xab;

	enum class Error : uint8_t
	{
		Ready = Bit7,
		MagnetTooStrong = Bit6,
		MagnetTooWeak = Bit5,
		MemoryError = Bit4,
	};
	MODM_FLAGS8(Error)

	enum class Register : uint8_t
	{
		CustomerConfig00 = 0x07,
		CustomerConfig01 = 0x08,
		Unlock = 0x10,
		Error = 0x21,
		Position = 0x3f,
	};

	enum class AxisMode : uint8_t
	{
		OnAxis = 0,
		OffAxis = Bit5,
	};

protected:
	static constexpr uint16_t
	frame(bool read, uint8_t payload)
	{
		uint16_t value = (uint16_t(read) << 14) | payload;
		uint16_t parity = value & 0x7fff;
		parity ^= parity >> 8;
		parity ^= parity >> 4;
		parity ^= parity >> 2;
		parity ^= parity >> 1;
		return value | ((parity & 1u) << 15);
	}
};

/**
 * Broadcom AEAT-9922 absolute magnetic encoder SPI driver.
 *
 * Uses the SPI4(A) interface. Resolution is configurable from 10 to 18 bit.
 *
 * @tparam SpiMaster  SPI master connected to the encoder
 * @tparam Cs         Active-low chip-select GPIO
 * @tparam Resolution Encoder resolution in bits (10–18), default 14
 * @tparam Axis       Magnet axis mode, default on-axis
 * @ingroup modm_driver_aeat9922
 */
template<typename SpiMaster, typename Cs, uint8_t Resolution = 14,
         aeat9922::AxisMode Axis = aeat9922::AxisMode::OnAxis>
class Aeat9922 : public aeat9922, public modm::SpiDevice<SpiMaster>
{
	static_assert(Resolution >= 10 && Resolution <= 18,
	              "Aeat9922: Resolution must be between 10 and 18 bits");

public:
	static constexpr uint8_t resolution = Resolution;
	static constexpr AxisMode axisMode = Axis;

	using Data = modm::IntegerAngle<Resolution>;

	Aeat9922(Data &data);

	/// Configure the volatile absolute resolution and magnet axis mode fields.
	bool
	initialize();

	/// Read the absolute angle. Returns false on parity error or when the encoder sets EF.
	bool
	read();

	/**
	 * Read the absolute angle without SPI bus arbitration.
	 *
	 * This is intended for deterministic interrupt handlers. The caller must ensure
	 * exclusive access to the SPI master and must call initialize() before using it.
	 * Do not call any other device on the same SPI master while interrupt-driven
	 * reads are enabled.
	 */
	bool
	readFromInterrupt();

	/// Read the ERROR register.
	Error_t
	readStatus();

	inline Data &
	getData()
	{
		return data;
	}

private:
	uint16_t
	transferFrame(uint16_t value);

	uint32_t
	readPositionFrame();

	bool
	readPosition();

	uint8_t
	readRegister(Register reg);

	void
	writeRegister(Register reg, uint8_t value);

	static constexpr uint8_t WireDataBits = (Resolution <= 14) ? 14 : Resolution;
	static constexpr uint8_t PositionFrameBits = WireDataBits + 2;
	static constexpr uint8_t PositionFrameBytes = (PositionFrameBits + 7) / 8;

	Data &data;
	uint8_t inBuffer[3];
	uint8_t outBuffer[3];
};

}  // namespace modm

#include "aeat9922_impl.hpp"
