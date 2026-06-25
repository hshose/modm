#pragma once

#include <atomic>
#include <cstdint>

#include "encoder_stream_protocol.h"

namespace Encoder
{

static constexpr uint8_t Resolution = 18;
static constexpr uint32_t CountsPerRevolution = uint32_t{1} << Resolution;
static constexpr uint32_t PositionMask = CountsPerRevolution - 1u;
static constexpr uint32_t StatusParityError = 1u << 0;
static constexpr uint32_t StatusEncoderError = 1u << 1;

inline std::atomic<uint32_t> lastRawFrame {0};
inline std::atomic<uint32_t> lastPosition {0};
inline std::atomic<uint32_t> validSampleCount {0};
inline std::atomic<uint32_t> parityErrorCount {0};
inline std::atomic<uint32_t> encoderErrorCount {0};
inline std::atomic<uint32_t> sampleSequence {0};

inline bool
decodeRawFrame(uint32_t rawFrame, uint32_t& position, uint32_t& status)
{
	lastRawFrame.store(rawFrame, std::memory_order_relaxed);
	status = 0;

	// P | EF | position[17:0] | four zero padding bits
	const uint32_t frame = rawFrame >> 4;
	if (__builtin_parity(frame)) {
		status |= StatusParityError;
		parityErrorCount.fetch_add(1, std::memory_order_relaxed);
	}

	if (frame & (uint32_t{1} << Resolution)) {
		status |= StatusEncoderError;
		encoderErrorCount.fetch_add(1, std::memory_order_relaxed);
	}

	position = frame & PositionMask;
	lastPosition.store(position, std::memory_order_relaxed);
	if (status == 0) {
		validSampleCount.fetch_add(1, std::memory_order_relaxed);
	}
	return status == 0;
}

} // namespace Encoder
