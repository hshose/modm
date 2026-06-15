/*
 * Copyright (c) 2026, modm project
 * SPDX-License-Identifier: MPL-2.0
 */

#include <modm/board.hpp>

using namespace Board;
using namespace modm::literals;

using PhaseAHigh = GpioA8;
using PhaseALow = GpioA9;
using PhaseBHigh = GpioA10;
using PhaseBLow = GpioA11;
using PhaseCHigh = GpioB14;
using PhaseCLow = GpioB15;

namespace
{
constexpr auto timers = Hrtim1::TimerCounter::A | Hrtim1::TimerCounter::B | Hrtim1::TimerCounter::D;

constexpr auto prescaler = Hrtim1::Prescaler::Mul32;

// Nucleo-G474RE runs APB2 at 170 MHz. Mul32 drives the HRTIM waveform timers
// from a 5.44 GHz high-resolution time base.
constexpr auto timerClock = 5'440'000'000ull;

// The three waveform timers run independently. Outputs 1 and 2 of each timer
// share that timer's period, but Timer A/B/D can each use a different PERxR.
constexpr Hrtim1::Value periodA = timerClock / 2'000'000;  // 2 MHz, 500 ns
constexpr Hrtim1::Value periodB = timerClock / 1'000'000;  // 1 MHz, 1 us
constexpr Hrtim1::Value periodD = timerClock / 500'000;    // 500 kHz, 2 us

constexpr Hrtim1::Value dutyA = periodA / 2;        // 50%, high for 250 ns
constexpr Hrtim1::Value dutyB = periodB / 4;        // 25%, high for 250 ns
constexpr Hrtim1::Value dutyD = (periodD * 3) / 4;  // 75%, high for 1.5 us

// 54 ticks at 5.44 GHz are 9.9 ns. This keeps complementary outputs separated,
// although a 48 MHz logic analyzer will usually not resolve this dead time.
constexpr Hrtim1::Value deadTime = 54;

void
stopOnCalibrationError()
{
	MODM_LOG_ERROR << "HRTIM DLL calibration failed" << modm::endl;
	while (true) {}
}

void
configureTimer(Hrtim1::Timer timer, Hrtim1::Value period, Hrtim1::Value duty)
{
	Hrtim1::setMode(timer, prescaler);
	Hrtim1::setPeriod(timer, period);
	Hrtim1::setCompareValue(timer, Hrtim1::CompareUnit::Compare1, duty);
	Hrtim1::setDeadTime(timer, deadTime, deadTime);
}
}  // namespace

int
main()
{
	Board::initialize();
	LedD13::setOutput();

	Hrtim1::connect<PhaseAHigh::Cha1, PhaseALow::Cha2, PhaseBHigh::Chb1, PhaseBLow::Chb2,
					PhaseCHigh::Chd1, PhaseCLow::Chd2>();
	Hrtim1::enable();
	if (!Hrtim1::calibrate()) { stopOnCalibrationError(); }

	configureTimer(Hrtim1::Timer::A, periodA, dutyA);
	configureTimer(Hrtim1::Timer::B, periodB, dutyB);
	configureTimer(Hrtim1::Timer::D, periodD, dutyD);

	// Timer A: CHA1 is 2 MHz at 50% duty. CHA2 is its complementary output.
	Hrtim1::configurePwm<PhaseAHigh::Cha1>(Hrtim1::CompareUnit::Compare1, dutyA,
										   Hrtim1::OutputPolarity::ActiveHigh, false, false);
	Hrtim1::configurePwm<PhaseALow::Cha2>(Hrtim1::CompareUnit::Compare1, dutyA,
										  Hrtim1::OutputPolarity::ActiveHigh, true, false);

	// Timer B: CHB1 is 1 MHz at 25% duty. CHB2 is its complementary output.
	Hrtim1::configurePwm<PhaseBHigh::Chb1>(Hrtim1::CompareUnit::Compare1, dutyB,
										   Hrtim1::OutputPolarity::ActiveHigh, false, false);
	Hrtim1::configurePwm<PhaseBLow::Chb2>(Hrtim1::CompareUnit::Compare1, dutyB,
										  Hrtim1::OutputPolarity::ActiveHigh, true, false);

	// Timer D: CHD1 is 500 kHz at 75% duty. CHD2 is its complementary output.
	Hrtim1::configurePwm<PhaseCHigh::Chd1>(Hrtim1::CompareUnit::Compare1, dutyD,
										   Hrtim1::OutputPolarity::ActiveHigh, false, false);
	Hrtim1::configurePwm<PhaseCLow::Chd2>(Hrtim1::CompareUnit::Compare1, dutyD,
										  Hrtim1::OutputPolarity::ActiveHigh, true, false);

	Hrtim1::applyAndReset(timers);

	Hrtim1::enableOutput<PhaseAHigh::Cha1>();
	Hrtim1::enableOutput<PhaseALow::Cha2>();
	Hrtim1::enableOutput<PhaseBHigh::Chb1>();
	Hrtim1::enableOutput<PhaseBLow::Chb2>();
	Hrtim1::enableOutput<PhaseCHigh::Chd1>();
	Hrtim1::enableOutput<PhaseCLow::Chd2>();

	Hrtim1::start(timers);

	while (true)
	{
		LedD13::toggle();
		modm::delay(250ms);
	}
}
