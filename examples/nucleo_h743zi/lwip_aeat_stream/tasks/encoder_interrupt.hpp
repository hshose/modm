#pragma once

#include <atomic>
#include <cstdint>

#include "../encoder.hpp"
#include "../encoder_sample_buffer.hpp"
#include "../hardware.hpp"

namespace EncoderInterrupt
{

using Hardware = Board::EncoderHardware::Aeat9922;
using Timer = Hardware::Timer;
static_assert(Encoder::Resolution == Hardware::Resolution);

inline uint32_t positionBuffer {0};

inline uint32_t
timestampUsFromCycles()
{
	return uint32_t(uint64_t(DWT->CYCCNT) * 1'000'000ULL / Board::SystemClock::Frequency);
}

} // namespace EncoderInterrupt

MODM_ISR(TIM2)
{
	using namespace EncoderInterrupt;

	Timer::acknowledgeInterruptFlags(Timer::InterruptFlag::Update);

	Hardware::SpiTransfer::readPosition(positionBuffer);

	uint32_t position;
	uint32_t status;
	Encoder::decodeRawFrame(positionBuffer, position, status);

	EncoderSample sample {};
	sample.sequence = Encoder::sampleSequence.fetch_add(1, std::memory_order_relaxed);
	sample.timestamp_us = timestampUsFromCycles();
	sample.position_raw = position;
	sample.status = status;
	encoderSampleBuffer.pushFromInterrupt(sample);
}
