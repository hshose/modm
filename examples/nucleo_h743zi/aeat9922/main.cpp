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

#include <modm/board.hpp>
#include <modm/driver/encoder/aeat9922.hpp>

using namespace Board;
using namespace std::chrono_literals;

using SpiMaster = SpiMaster3;
using Cs = GpioOutputA4;
using Sck = GpioB3;
using Miso = GpioB4;
using Mosi = GpioB5;

using Encoder = modm::Aeat9922<SpiMaster, Cs, 18, modm::aeat9922::AxisMode::OffAxis>;

Encoder::Data data{0};
Encoder encoder{data};

bool
writeByte(uint8_t data)
{
	const auto start = modm::PreciseClock::now();
	while ((modm::PreciseClock::now() - start) < 2ms)
	{
		if (stlink::Uart::write(data)) { return true; }
	}
	return false;
}

void
writeText(const char* text)
{
	while (*text) { writeByte(uint8_t(*text++)); }
}

void
writeUnsigned(uint32_t value, uint8_t width = 0, char pad = ' ')
{
	char buffer[10];
	uint8_t length = 0;
	do
	{
		buffer[length++] = char('0' + (value % 10u));
		value /= 10u;
	} while (value != 0);

	while (length < width)
	{
		writeByte(uint8_t(pad));
		--width;
	}
	while (length != 0) { writeByte(buffer[--length]); }
}

void
writeHex8(uint8_t value)
{
	const char hex[] = "0123456789abcdef";
	writeText("0x");
	writeByte(hex[value >> 4]);
	writeByte(hex[value & 0x0f]);
}

int
main()
{
	Board::initialize();
	Board::Leds::setOutput();

	Cs::setOutput(modm::Gpio::High);

	SpiMaster::connect<Sck::Sck, Miso::Miso, Mosi::Mosi>();
	SpiMaster::initialize<Board::SystemClock, 12'500'000>();

	writeText("==========AEAT-9922 Test==========\n");
	writeText("SPI3: SCK=PB3, MISO=PB4, MOSI=PB5, CS=PA4\n");

	while (!encoder.initialize())
	{
		Board::LedRed::toggle();
		writeText("Waiting for encoder ready\n");
		modm::delay(250ms);
	}

	Board::LedRed::reset();
	Board::LedGreen::set();
	writeText("Encoder initialized\n");
	writeText("Axis mode: ");
	writeText(Encoder::axisMode == modm::aeat9922::AxisMode::OffAxis ? "off-axis\n" : "on-axis\n");

	while (true)
	{
		const bool readOk = encoder.read();
		const auto status = encoder.readStatus();
		const uint32_t milliDegrees = uint32_t((uint64_t(data.data) * 360'000u) / (1u << Encoder::resolution));

		writeText("\nNew readout:\n");
		writeText("  angle degree: ");
		writeUnsigned(milliDegrees / 1000, 3);
		writeByte('.');
		writeUnsigned(milliDegrees % 1000, 3, '0');
		writeText(" degrees\n");
		writeText("     angle raw: ");
		writeUnsigned(data.data);
		writeText("\n        status: ");
		writeHex8(status.value);
		writeByte('\n');

		if (!readOk)
		{
			Board::LedRed::set();
			writeText("  position read error flag set\n");
		}
		else
		{
			Board::LedRed::reset();
			Board::LedBlue::toggle();
		}

		modm::delay(500ms);
	}

	return 0;
}
