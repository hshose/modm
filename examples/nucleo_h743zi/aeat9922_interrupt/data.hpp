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

#include <atomic>
#include <cstdint>

namespace Data
{

struct Encoder
{
	static constexpr uint8_t Resolution = 18;
	static constexpr uint32_t CountsPerRevolution = uint32_t{1} << Resolution;

	std::atomic<uint32_t> rawFrame{0};
	std::atomic<uint32_t> rawValue{0};
	std::atomic<float> angleDegrees{0.f};

	std::atomic<uint32_t> validSampleCount{0};
	std::atomic<uint32_t> parityErrorCount{0};
	std::atomic<uint32_t> encoderErrorCount{0};
};

inline Encoder encoder{};

} // namespace Data
