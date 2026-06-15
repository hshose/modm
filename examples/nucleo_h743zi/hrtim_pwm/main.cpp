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

using namespace Board;
using namespace modm::literals;

using PhaseAHigh = GpioC6;
using PhaseALow = GpioC7;
using PhaseBHigh = GpioC8;
using PhaseBLow = GpioA8;
using PhaseCHigh = GpioA9;
using PhaseCLow = GpioA10;

namespace
{

constexpr auto timers = Hrtim1::TimerCounter::A | Hrtim1::TimerCounter::B | Hrtim1::TimerCounter::C;

// Board::SystemClock configures APB2 to 100 MHz. With the default HRTIM clock
// selection and APB2 prescaler != 1, HRTIM1 runs at 2 * PCLK2 = 200 MHz.
// Prescaler::Mul32 selects the high-resolution 32x time base:
//
//     200 MHz * 32 = 6.4 GHz
//
// A period of 3'200 therefore generates a 2 MHz PWM carrier.
constexpr auto prescaler = Hrtim1::Prescaler::Mul32;
constexpr Hrtim1::Value period = 3'200;

// Dead time is inserted between each timer's output 1 and output 2.
// With the Mul32 time base this fixed value is 64 / 6.4 GHz = 10 ns.
constexpr Hrtim1::Value deadTime = 64;

void
configurePhase(Hrtim1::Timer timer, Hrtim1::CompareUnit compare, Hrtim1::Value duty)
{
	Hrtim1::setMode(timer, prescaler);
	Hrtim1::setPeriod(timer, period);
	Hrtim1::setCompareValue(timer, compare, duty);
	Hrtim1::setDeadTime(timer, deadTime, deadTime);
}

}  // namespace

int
main()
{
	Board::initialize();

	MODM_LOG_INFO << "modm example: HRTIM complementary PWM" << modm::endl;

	Hrtim1::connect<PhaseAHigh::Cha1, PhaseALow::Cha2, PhaseBHigh::Chb1, PhaseBLow::Chb2,
					PhaseCHigh::Chc1, PhaseCLow::Chc2>();

	Hrtim1::enable();

	configurePhase(Hrtim1::Timer::A, Hrtim1::CompareUnit::Compare1, period / 2);
	configurePhase(Hrtim1::Timer::B, Hrtim1::CompareUnit::Compare1, period / 2);
	configurePhase(Hrtim1::Timer::C, Hrtim1::CompareUnit::Compare1, period / 2);

	Hrtim1::configurePwm<PhaseAHigh::Cha1>(Hrtim1::CompareUnit::Compare1, period / 2,
										   Hrtim1::OutputPolarity::ActiveHigh, false, false);
	Hrtim1::configurePwm<PhaseALow::Cha2>(Hrtim1::CompareUnit::Compare1, period / 2,
										  Hrtim1::OutputPolarity::ActiveHigh, true, false);

	Hrtim1::configurePwm<PhaseBHigh::Chb1>(Hrtim1::CompareUnit::Compare1, period / 2,
										   Hrtim1::OutputPolarity::ActiveHigh, false, false);
	Hrtim1::configurePwm<PhaseBLow::Chb2>(Hrtim1::CompareUnit::Compare1, period / 2,
										  Hrtim1::OutputPolarity::ActiveHigh, true, false);

	Hrtim1::configurePwm<PhaseCHigh::Chc1>(Hrtim1::CompareUnit::Compare1, period / 2,
										   Hrtim1::OutputPolarity::ActiveHigh, false, false);
	Hrtim1::configurePwm<PhaseCLow::Chc2>(Hrtim1::CompareUnit::Compare1, period / 2,
										  Hrtim1::OutputPolarity::ActiveHigh, true, false);

	Hrtim1::applyAndReset(timers);

	Hrtim1::enableOutput<PhaseAHigh::Cha1>();
	Hrtim1::enableOutput<PhaseALow::Cha2>();
	Hrtim1::enableOutput<PhaseBHigh::Chb1>();
	Hrtim1::enableOutput<PhaseBLow::Chb2>();
	Hrtim1::enableOutput<PhaseCHigh::Chc1>();
	Hrtim1::enableOutput<PhaseCLow::Chc2>();

	Hrtim1::start(timers);

	// Sweep the high-side duty cycle from 10% to 90% in 0.5% steps.
	// At 2 MHz this moves the high-side pulse width from 50 ns to 450 ns.
	// The low-side outputs are inverted complements of the high-side outputs,
	// with the dead time above inserted by HRTIM.
	Hrtim1::Value duty = period / 2;
	int16_t step = period / 200;

	while (true)
	{
		duty += step;
		if (duty >= (period * 9) / 10 || duty <= period / 10) { step = -step; }

		Hrtim1::setCompareValue(Hrtim1::Timer::A, Hrtim1::CompareUnit::Compare1, duty);
		Hrtim1::setCompareValue(Hrtim1::Timer::B, Hrtim1::CompareUnit::Compare1, duty);
		Hrtim1::setCompareValue(Hrtim1::Timer::C, Hrtim1::CompareUnit::Compare1, duty);
		Hrtim1::applyUpdate(timers);

		LedGreen::toggle();
		modm::delay(20ms);
	}

	return 0;
}
