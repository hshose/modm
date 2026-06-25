#pragma once

#include <chrono>
#include <cstdint>

#include <modm/board.hpp>
#include <modm/driver/encoder/aeat9922.hpp>
#include <modm/platform.hpp>

namespace Board
{

namespace EncoderHardware
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

	struct SpiTransfer
	{
		static constexpr uint16_t PositionReadCommand = 0xc03f;

		static inline uint16_t byteSwap16(uint16_t value)
		{
			return uint16_t((value << 8) | (value >> 8));
		}

		static inline void write16(uint16_t value)
		{
			SpiHal::write16(byteSwap16(value));
		}

		static inline void read16(uint16_t& value)
		{
			value = byteSwap16(SpiHal::read16());
		}

		static inline void write24(uint32_t value)
		{
			write16(uint16_t(value >> 8));
			SpiHal::write(uint8_t(value));
		}

		static inline void read24(uint32_t& value)
		{
			uint16_t firstTwoBytes;
			read16(firstTwoBytes);
			value = (uint32_t{firstTwoBytes} << 8) | SpiHal::read();
		}

		static inline void beginFrame()
		{
			Cs::reset();
			modm::delay(std::chrono::nanoseconds{350});
		}

		static inline void endFrame()
		{
			modm::delay(std::chrono::nanoseconds{50});
			Cs::set();
		}

		static inline void waitBetweenFrames()
		{
			modm::delay(std::chrono::nanoseconds{350});
		}

		static inline void waitUntilTransmitted()
		{
			while (!SpiHal::isTxCompleted()) {
			}
		}

		static inline void readPosition(uint32_t& response)
		{
			beginFrame();
			while (SpiHal::isTxFifoFull()) {
			}
			write16(PositionReadCommand);
			waitUntilTransmitted();
			uint16_t commandResponse;
			read16(commandResponse);
			endFrame();
			waitBetweenFrames();

			beginFrame();
			while (SpiHal::isTxFifoFull()) {
			}
			write24(0);
			waitUntilTransmitted();
			read24(response);
			endFrame();
		}
	};

	using Device = modm::Aeat9922<Spi, Cs, Resolution, AxisMode>;

	inline static Device::Data data {};
	inline static Device device {data};

	static inline bool initialize()
	{
		Cs::setOutput(modm::Gpio::High);
		Spi::connect<Sck::Sck, Miso::Miso, Mosi::Mosi>();
		Spi::initialize<SystemClock, 12.5_MHz>();
		Spi::setDataMode(Spi::DataMode::Mode1);
		Spi::setDataOrder(Spi::DataOrder::MsbFirst);
		Spi::setDataSize(Spi::DataSize::Bit8);

		if (!device.initialize()) {
			return false;
		}

		Timer::enable();
		Timer::setMode(Timer::Mode::UpCounter);
		Timer::setPeriod<SystemClock>(SamplePeriod);
		Timer::enableInterrupt(Timer::Interrupt::Update);
		Timer::enableInterruptVector(true, 4);
		Timer::start();
		return true;
	}
};

} // namespace EncoderHardware

} // namespace Board
