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

#include <chrono>
#include <cstdint>

#include <modm/board.hpp>
#include <modm/driver/encoder/aeat9922.hpp>
#include <modm/platform.hpp>

namespace Board
{

namespace Encoder
{

struct Aeat9922
{
	using Cs = GpioA4;
	using Sck = GpioB3;
	using Miso = GpioB4;
	using Mosi = GpioB5;

	using Spi = SpiMaster3;
	using SpiHal = SpiHal3;
	using Timer = Timer2;

	static constexpr uint8_t Resolution = 18;
	static constexpr modm::aeat9922::AxisMode AxisMode =
		modm::aeat9922::AxisMode::OnAxis;
	static constexpr auto SamplePeriod = std::chrono::microseconds{100};
	static constexpr uint32_t SampleFrequency = 10'000;

	/**
	 * Direct access for an interrupt-driven transfer sequence.
	 *
	 * SPI3 remains configured for 8-bit frames. A half-word access therefore
	 * pushes/pops two consecutive bytes through the STM32H7 SPI FIFO.
	 */
	struct SpiTransfer
	{
		static constexpr uint16_t PositionReadCommand = 0xc03f;

		static inline uint16_t
		byteSwap16(uint16_t value)
		{
			return uint16_t((value << 8) | (value >> 8));
		}

		static inline void
		write16(uint16_t value)
		{
			// With 8-bit frames, half-word FIFO accesses use little-endian byte
			// packing. Swap to keep multi-byte values MSB-first on the wire.
			SpiHal::write16(byteSwap16(value));
		}

		static inline void
		read16(uint16_t& value)
		{
			value = byteSwap16(SpiHal::read16());
		}

		static inline void
		write24(uint32_t value)
		{
			write16(uint16_t(value >> 8));
			SpiHal::write(uint8_t(value));
		}

		static inline void
		read24(uint32_t& value)
		{
			uint16_t firstTwoBytes;
			read16(firstTwoBytes);
			value = (uint32_t{firstTwoBytes} << 8) | SpiHal::read();
		}

		static inline void
		beginFrame()
		{
			Cs::reset();
			modm::delay(std::chrono::nanoseconds{350});
		}

		static inline void
		endFrame()
		{
			modm::delay(std::chrono::nanoseconds{50});
			Cs::set();
		}

		static inline void
		waitBetweenFrames()
		{
			modm::delay(std::chrono::nanoseconds{350});
		}

		static inline void
		waitUntilTransmitted()
		{
			while (!SpiHal::isTxCompleted()) {}
		}

		/** Perform the bare command and 24-bit position-response transfers. */
		static inline void
		readPosition(uint32_t& response)
		{
			// Send the 16-bit POSITION_ABS read command and discard its response.
			beginFrame();
			while (SpiHal::isTxFifoFull()) {}
			write16(PositionReadCommand);
			waitUntilTransmitted();
			uint16_t commandResponse;
			read16(commandResponse);
			endFrame();
			waitBetweenFrames();

			// Clock out the 24-bit response, then consume it as 16 + 8 bits.
			beginFrame();
			while (SpiHal::isTxFifoFull()) {}
			write24(0);
			waitUntilTransmitted();
			read24(response);
			endFrame();
			// No explicit t_CSn wait is needed here: the 100 us sampling
			// interval is much longer than the required 350 ns CS-high time.
		}
	};

	using Device = modm::Aeat9922<Spi, Cs, Resolution, AxisMode>;

	inline static Device::Data data{};
	inline static Device device{data};

	/** Initialize and configure the complete encoder sampling hardware. */
	static inline bool
	initialize()
	{
		Cs::setOutput(modm::Gpio::High);
		Spi::connect<Sck::Sck, Miso::Miso, Mosi::Mosi>();
		Spi::initialize<SystemClock, 12.5_MHz>();
		Spi::setDataMode(Spi::DataMode::Mode1);
		Spi::setDataOrder(Spi::DataOrder::MsbFirst);
		Spi::setDataSize(Spi::DataSize::Bit8);

		// Programs the volatile absolute-resolution and axis-mode fields.
		if (!device.initialize()) { return false; }

		Timer::enable();
		Timer::setMode(Timer::Mode::UpCounter);
		Timer::setPeriod<SystemClock>(SamplePeriod);
		Timer::enableInterrupt(Timer::Interrupt::Update);
		Timer::enableInterruptVector(true, 4);
		Timer::start();
		return true;
	}
};

} // namespace Encoder

} // namespace Board
