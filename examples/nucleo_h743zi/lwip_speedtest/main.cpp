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

#include <modm/board.hpp>
#include <modm/processing.hpp>

#include <modm_lwip.hpp>

#include "udp_speedtest.h"

using namespace Board;

int
main()
{
	SCB_DisableICache();
	SCB_DisableDCache();
	Board::initialize();

	Leds::setOutput();
	MODM_LOG_INFO << "\n\nReboot: lwIP UDP speedtest example" << modm::endl;

	static constexpr modm::lwip::EthernetConfig network {
		{ 192, 168, 1, 50 },
		{ 255, 255, 255, 0 },
		{ 192, 168, 1, 1 },
	};

	if (!modm::lwip::Ethernet::initialize(network)) {
		MODM_LOG_ERROR << "Ethernet/lwIP initialization failed" << modm::endl;
	}

	udp_speedtest_init();

	modm::ShortPeriodicTimer statusTimer { 1s };
	while (true) {
		if (statusTimer.execute()) {
			Leds::toggle();
			modm::lwip::Ethernet::pollStatus();
		}

		modm::lwip::Ethernet::poll();
		udp_speedtest_poll();
	}
}
