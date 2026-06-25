/*
 * Copyright (c) 2026, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#pragma once

#include <modm_lwip_ethernet.hpp>

#include <cstddef>
#include <cstdint>

struct pbuf;

namespace modm::lwip::ethernet
{

inline constexpr uint8_t DefaultMac[] { 0x02, 0x00, 0x00, 0x00, 0x00, 0x01 };

}
