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

#include <cstdint>

#include "data.hpp"

/** Validate and publish one raw AEAT-9922 24-bit position frame. */
inline bool
updateEncoder(uint32_t rawFrame)
{
	auto& encoder = Data::encoder;
	encoder.rawFrame.store(rawFrame, std::memory_order_relaxed);

	// P | EF | position[17:0] | four zero padding bits
	const uint32_t frame = rawFrame >> 4;
	if (__builtin_parity(frame))
	{
		encoder.parityErrorCount.fetch_add(1, std::memory_order_relaxed);
		return false;
	}

	if (frame & (uint32_t{1} << Data::Encoder::Resolution))
	{
		encoder.encoderErrorCount.fetch_add(1, std::memory_order_relaxed);
		return false;
	}

	constexpr uint32_t PositionMask = Data::Encoder::CountsPerRevolution - 1u;
	const uint32_t rawValue = frame & PositionMask;
	constexpr float DegreesPerTick = 360.f / Data::Encoder::CountsPerRevolution;
	encoder.rawValue.store(rawValue, std::memory_order_relaxed);
	encoder.angleDegrees.store(float(rawValue) * DegreesPerTick, std::memory_order_relaxed);
	encoder.validSampleCount.fetch_add(1, std::memory_order_relaxed);
	return true;
}
