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

#include "ethernet_dma.hpp"
#include "network.hpp"

using namespace Board;

int
main()
{
	ethernet_dma::configureMpuRegion();

	SCB_DisableICache();
    SCB_DisableDCache();
	Board::initialize();
	ethernet_dma::configureMpuRegion();

	Leds::setOutput();
	MODM_LOG_INFO << "\n\nReboot: Ethernet DMA + lwIP ping example" << modm::endl;
	ethernet_dma::printMemoryLayout();
	ethernet_dma::initialize();
	network::initialize();

	modm::ShortPeriodicTimer txTimer { 1s };

	while (true) {
		if (txTimer.execute()) {
			Leds::toggle();
			ethernet_dma::pollDmaStatus();
		}

		network::poll();
	}
}
