/*
 * Copyright (c) 2026, Henrik Hose
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include <modm/board.hpp>
#include <modm/driver/radar/iwr6843aop.hpp>

using namespace Board;
using namespace modm::literals;

using ControlUart = BufferedUart<UsartHal1, UartTxBuffer<512>, UartRxBuffer<512>>;
using ControlTx = GpioA9;
using ControlRx = GpioA10;

using DataUart = BufferedUart<UartHal4, UartTxBuffer<128>, UartRxBuffer<4096>>;
using DataTx = GpioA0;
using DataRx = GpioA1;

using RadarNrst = GpioC9;

struct DummySyncPin
{
	static void set() {}
	static void reset() {}
};

using Radar = modm::Iwr6843aop<ControlUart, DataUart, DummySyncPin>;

static Radar radar;

static constexpr char RadarConfiguration[] = R"cfg(
sensorStop
flushCfg
dfeDataOutputMode 1
channelCfg 15 7 0
adcCfg 2 1
adcbufCfg -1 0 1 1 1
profileCfg 0 60.00 38.44 7.00 63.75 0 0 35.04 1.00 255 4675.80 0 0 30
chirpCfg 0 0 0 0 0 0 0 1
chirpCfg 1 1 0 0 0 0 0 2
chirpCfg 2 2 0 0 0 0 0 4
frameCfg 0 2 60 0 95.00 1 0.00
lowPower 0 0
guiMonitor -1 1 0 0 0 0 1
cfarCfg -1 0 2 8 4 3 0 8 0
cfarCfg -1 1 0 4 2 3 1 20 1
multiObjBeamForming -1 1 0.5
clutterRemoval -1 0
aoaFovCfg -1 -90 90 -90 90
cfarFovCfg -1 0 0.1 19.0
cfarFovCfg -1 1 -8.00 8.00
calibDcRangeSig -1 0 -5 8 256
extendedMaxVelocity -1 0
lvdsStreamCfg -1 0 0 0
compRangeBiasAndRxChanPhase 0.0 -1 0 1 0 -1 0 1 0 -1 0 1 0 -1 0 1 0 -1 0 1 0 -1 0 1 0
measureRangeBiasAndRxChanPhase 0 1.0 0.2
CQRxSatMonitor 0 3 5 103 0
CQSigImgMonitor 0 95 6
analogMonitor 0 0
calibData 0 0 0
sensorStart
)cfg";

int
main()
{
	Board::initialize();
	Leds::setOutput();

	RadarNrst::setOutput(modm::Gpio::Low);
	modm::delay(10ms);
	RadarNrst::set();
	modm::delay(10ms);

	ControlUart::connect<ControlTx::Tx, ControlRx::Rx>();
	DataUart::connect<DataTx::Tx, DataRx::Rx>();

	ControlUart::initialize<Board::SystemClock, 115200_Bd>();
	DataUart::initialize<Board::SystemClock, 921600_Bd>();

	MODM_LOG_INFO << "uart init\n";

	MODM_LOG_INFO << "IWR6843AOP example\n";
	MODM_LOG_INFO << "Uploading configuration...\n";

	if (not radar.configure(std::span<const char>{RadarConfiguration, sizeof(RadarConfiguration) - 1}))
	{
		MODM_LOG_ERROR << "Configuration failed, error=" << static_cast<int>(radar.getLastError()) << modm::endl;
		while (true)
		{
			Leds::toggle();
			modm::delay(250ms);
		}
	}

	MODM_LOG_INFO << "Configuration successful\n";

	Radar::FrameType frame{};
	modm::Timestamp lastFrameTimestamp{};
	bool hasLastFrameTimestamp{false};
	uint32_t processedFrames{0};

	while (true)
	{
		if (not radar.processData())
		{
			MODM_LOG_ERROR << "Processing error=" << static_cast<int>(radar.getLastError()) << modm::endl;
			radar.clearError();
			modm::delay(5ms);
		}

		while (radar.getFrame(frame))
		{
			processedFrames++;
			LedGreen::toggle();

			const auto now = modm::Clock::now();
			const auto age = now - frame.timestamp;

			uint32_t intervalMs{0};
			if (hasLastFrameTimestamp) {
				intervalMs = (frame.timestamp - lastFrameTimestamp).count();
			}
			lastFrameTimestamp = frame.timestamp;
			hasLastFrameTimestamp = true;

			float maxVelocity{0.f};
			for (std::size_t ii = 0; ii < frame.pointCount; ++ii)
			{
				float velocity = frame.points[ii].point.velocity;
				if (velocity < 0.f) {
					velocity = -velocity;
				}
				if (velocity > maxVelocity) {
					maxVelocity = velocity;
				}
			}

			MODM_LOG_INFO.printf(
				"Frame #%lu: %lu points detected, age: %lu ms, interval: %lu ms, max velocity: %.2f m/s\n",
				static_cast<unsigned long>(frame.frameHeader.frameNumber),
				static_cast<unsigned long>(frame.pointCount),
				static_cast<unsigned long>(age.count()),
				static_cast<unsigned long>(intervalMs),
				static_cast<double>(maxVelocity));

			if ((processedFrames % 10u) == 0u) {
				MODM_LOG_INFO.printf("Processed %lu frames\n", static_cast<unsigned long>(processedFrames));
			}
		}

		modm::delay(1ms);
	}

	return 0;
}
