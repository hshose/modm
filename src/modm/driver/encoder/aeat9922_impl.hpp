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

#ifndef MODM_AEAT9922_HPP
#	error "Don't include this file directly, use 'aeat9922.hpp' instead!"
#endif

#include <chrono>

#include <modm/architecture/interface/delay.hpp>
#include <modm/architecture/interface/gpio.hpp>

namespace modm
{

using namespace std::chrono_literals;

template<typename SpiMaster, typename Cs, uint8_t Resolution, aeat9922::AxisMode Axis>
Aeat9922<SpiMaster, Cs, Resolution, Axis>::Aeat9922(Data &data) : data(data)
{
	this->attachConfigurationHandler([] {
		SpiMaster::setDataMode(SpiMaster::DataMode::Mode1);
		SpiMaster::setDataOrder(SpiMaster::DataOrder::MsbFirst);
	});
	Cs::setOutput(modm::Gpio::High);
}

template<typename SpiMaster, typename Cs, uint8_t Resolution, aeat9922::AxisMode Axis>
uint16_t
Aeat9922<SpiMaster, Cs, Resolution, Axis>::transferFrame(uint16_t value)
{
	outBuffer[0] = uint8_t(value >> 8);
	outBuffer[1] = uint8_t(value);

	Cs::reset();
	modm::delay(350ns);
	SpiMaster::transfer(outBuffer, inBuffer, 2);
	modm::delay(50ns);
	Cs::set();
	modm::delay(350ns);

	return (uint16_t(inBuffer[0]) << 8) | inBuffer[1];
}

template<typename SpiMaster, typename Cs, uint8_t Resolution, aeat9922::AxisMode Axis>
uint32_t
Aeat9922<SpiMaster, Cs, Resolution, Axis>::readPositionFrame()
{
	outBuffer[0] = 0;
	outBuffer[1] = 0;
	outBuffer[2] = 0;

	Cs::reset();
	modm::delay(350ns);
	SpiMaster::transfer(outBuffer, inBuffer, PositionFrameBytes);
	modm::delay(50ns);
	Cs::set();
	modm::delay(350ns);

	uint32_t value = 0;
	for (uint8_t ii = 0; ii < PositionFrameBytes; ++ii)
	{
		value = (value << 8) | inBuffer[ii];
	}
	return value >> ((PositionFrameBytes * 8) - PositionFrameBits);
}

template<typename SpiMaster, typename Cs, uint8_t Resolution, aeat9922::AxisMode Axis>
uint8_t
Aeat9922<SpiMaster, Cs, Resolution, Axis>::readRegister(Register reg)
{
	const uint16_t command = frame(true, static_cast<uint8_t>(reg));
	transferFrame(command);
	return uint8_t(transferFrame(command));
}

template<typename SpiMaster, typename Cs, uint8_t Resolution, aeat9922::AxisMode Axis>
void
Aeat9922<SpiMaster, Cs, Resolution, Axis>::writeRegister(Register reg, uint8_t value)
{
	transferFrame(frame(false, static_cast<uint8_t>(reg)));
	transferFrame(frame(false, value));
}

template<typename SpiMaster, typename Cs, uint8_t Resolution, aeat9922::AxisMode Axis>
bool
Aeat9922<SpiMaster, Cs, Resolution, Axis>::initialize()
{
	modm::this_fiber::poll([&] { return this->acquireMaster(); });

	if (!(Error_t{readRegister(Register::Error)} & Error::Ready))
	{
		this->releaseMaster();
		return false;
	}

	const uint8_t config0 = readRegister(Register::CustomerConfig00);
	const uint8_t config1 = readRegister(Register::CustomerConfig01);
	writeRegister(Register::Unlock, UnlockKey);
	modm::this_fiber::sleep_for(1ms);
	writeRegister(Register::CustomerConfig00, (config0 & ~Bit5) | static_cast<uint8_t>(Axis));
	writeRegister(Register::CustomerConfig01, (config1 & 0xf0u) | uint8_t(18u - Resolution));
	modm::this_fiber::sleep_for(1ms);
	writeRegister(Register::Unlock, 0);

	this->releaseMaster();
	return true;
}

template<typename SpiMaster, typename Cs, uint8_t Resolution, aeat9922::AxisMode Axis>
bool
Aeat9922<SpiMaster, Cs, Resolution, Axis>::read()
{
	modm::this_fiber::poll([&] { return this->acquireMaster(); });
	const bool success = readPosition();
	this->releaseMaster();
	return success;
}

template<typename SpiMaster, typename Cs, uint8_t Resolution, aeat9922::AxisMode Axis>
bool
Aeat9922<SpiMaster, Cs, Resolution, Axis>::readFromInterrupt()
{
	return readPosition();
}

template<typename SpiMaster, typename Cs, uint8_t Resolution, aeat9922::AxisMode Axis>
bool
Aeat9922<SpiMaster, Cs, Resolution, Axis>::readPosition()
{

	transferFrame(frame(true, static_cast<uint8_t>(Register::Position)));
	const uint32_t value = readPositionFrame();

	if (__builtin_parity(value)) { return false; }
	if (value & (uint32_t(1) << WireDataBits)) { return false; }

	data.data = (value >> (WireDataBits - Resolution)) & ((uint32_t(1) << Resolution) - 1u);
	return true;
}

template<typename SpiMaster, typename Cs, uint8_t Resolution, aeat9922::AxisMode Axis>
typename Aeat9922<SpiMaster, Cs, Resolution, Axis>::Error_t
Aeat9922<SpiMaster, Cs, Resolution, Axis>::readStatus()
{
	modm::this_fiber::poll([&] { return this->acquireMaster(); });
	const auto status = Error_t{readRegister(Register::Error)};
	this->releaseMaster();
	return status;
}

}  // namespace modm
