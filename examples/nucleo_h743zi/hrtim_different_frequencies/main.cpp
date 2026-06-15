/*
 * Copyright (c) 2026, modm project
 * SPDX-License-Identifier: MPL-2.0
 */

#include <modm/board.hpp>

using namespace Board;
using namespace modm::literals;

using PhaseAHigh = GpioC6;
using PhaseALow  = GpioC7;
using PhaseBHigh = GpioC8;
using PhaseBLow  = GpioA8;
using PhaseCHigh = GpioA9;
using PhaseCLow  = GpioA10;

namespace
{
constexpr auto timers = Hrtim1::TimerCounter::A |
						Hrtim1::TimerCounter::B |
						Hrtim1::TimerCounter::C;

constexpr auto prescaler = Hrtim1::Prescaler::Mul32;

// Nucleo-H743ZI runs APB2 at 100 MHz by default. Since APB2 is prescaled, the
// HRTIM kernel clock is 2 * PCLK2 = 200 MHz. Mul32 therefore drives the HRTIM
// timer counters from a 6.4 GHz clock.
constexpr auto timerClock = 6'400'000'000ull;

// The three waveform timers run independently. Outputs 1 and 2 of each timer
// share that timer's period, but Timer A/B/C can each use a different PERxR.
constexpr Hrtim1::Value periodA = timerClock / 2'000'000; // 2 MHz, 500 ns
constexpr Hrtim1::Value periodB = timerClock / 1'000'000; // 1 MHz, 1 us
constexpr Hrtim1::Value periodC = timerClock /   500'000; // 500 kHz, 2 us

constexpr Hrtim1::Value dutyA = periodA / 2;       // 50%, high for 250 ns
constexpr Hrtim1::Value dutyB = periodB / 4;       // 25%, high for 250 ns
constexpr Hrtim1::Value dutyC = (periodC * 3) / 4; // 75%, high for 1.5 us

// 64 ticks at 6.4 GHz are 10 ns. This keeps complementary outputs separated,
// although a 48 MHz logic analyzer will usually not resolve this dead time.
constexpr Hrtim1::Value deadTime = 64;

void
configureTimer(Hrtim1::Timer timer, Hrtim1::Value period, Hrtim1::Value duty)
{
	Hrtim1::setMode(timer, prescaler);
	Hrtim1::setPeriod(timer, period);
	Hrtim1::setCompareValue(timer, Hrtim1::CompareUnit::Compare1, duty);
	Hrtim1::setDeadTime(timer, deadTime, deadTime);
}
}

int
main()
{
	Board::initialize();

	LedGreen::setOutput();

	Hrtim1::connect<
		PhaseAHigh::Cha1, PhaseALow::Cha2,
		PhaseBHigh::Chb1, PhaseBLow::Chb2,
		PhaseCHigh::Chc1, PhaseCLow::Chc2>();
	Hrtim1::enable();

	configureTimer(Hrtim1::Timer::A, periodA, dutyA);
	configureTimer(Hrtim1::Timer::B, periodB, dutyB);
	configureTimer(Hrtim1::Timer::C, periodC, dutyC);

	// Timer A: CHA1 is 2 MHz at 50% duty. CHA2 is its complementary output.
	Hrtim1::configurePwm<PhaseAHigh::Cha1>(
		Hrtim1::CompareUnit::Compare1, dutyA, Hrtim1::OutputPolarity::ActiveHigh,
		false, false);
	Hrtim1::configurePwm<PhaseALow::Cha2>(
		Hrtim1::CompareUnit::Compare1, dutyA, Hrtim1::OutputPolarity::ActiveHigh,
		true, false);

	// Timer B: CHB1 is 1 MHz at 25% duty. CHB2 is its complementary output.
	Hrtim1::configurePwm<PhaseBHigh::Chb1>(
		Hrtim1::CompareUnit::Compare1, dutyB, Hrtim1::OutputPolarity::ActiveHigh,
		false, false);
	Hrtim1::configurePwm<PhaseBLow::Chb2>(
		Hrtim1::CompareUnit::Compare1, dutyB, Hrtim1::OutputPolarity::ActiveHigh,
		true, false);

	// Timer C: CHC1 is 500 kHz at 75% duty. CHC2 is its complementary output.
	Hrtim1::configurePwm<PhaseCHigh::Chc1>(
		Hrtim1::CompareUnit::Compare1, dutyC, Hrtim1::OutputPolarity::ActiveHigh,
		false, false);
	Hrtim1::configurePwm<PhaseCLow::Chc2>(
		Hrtim1::CompareUnit::Compare1, dutyC, Hrtim1::OutputPolarity::ActiveHigh,
		true, false);

	Hrtim1::applyAndReset(timers);

	Hrtim1::enableOutput<PhaseAHigh::Cha1>();
	Hrtim1::enableOutput<PhaseALow::Cha2>();
	Hrtim1::enableOutput<PhaseBHigh::Chb1>();
	Hrtim1::enableOutput<PhaseBLow::Chb2>();
	Hrtim1::enableOutput<PhaseCHigh::Chc1>();
	Hrtim1::enableOutput<PhaseCLow::Chc2>();

	Hrtim1::start(timers);

	while (true)
	{
		LedGreen::toggle();
		modm::delay(250ms);
	}
}
