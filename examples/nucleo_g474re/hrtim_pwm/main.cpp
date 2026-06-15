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

using PhaseAHigh = GpioA8;
using PhaseALow = GpioA9;
using PhaseBHigh = GpioA10;
using PhaseBLow = GpioA11;
using PhaseCHigh = GpioB14;
using PhaseCLow = GpioB15;

namespace
{

constexpr auto timers = Hrtim1::TimerCounter::A | Hrtim1::TimerCounter::B | Hrtim1::TimerCounter::D;

// Board::SystemClock configures APB2 to 170 MHz. The G4 HRTIM kernel clock is
// PCLK2, so Prescaler::Mul32 selects a 5.44 GHz high-resolution time base:
//
//     170 MHz * 32 = 5.44 GHz
//
// A period of 2'720 therefore generates a 2 MHz PWM carrier.
constexpr auto prescaler = Hrtim1::Prescaler::Mul32;
constexpr auto timerClock = 5'440'000'000ull;
constexpr Hrtim1::Value period = timerClock / 2'000'000;
static_assert(period == 2'720);

// Dead time is inserted between each timer's output 1 and output 2.
// With the Mul32 time base this fixed value is 54 / 5.44 GHz = 9.9 ns.
constexpr Hrtim1::Value deadTime = 54;

void
stopOnCalibrationError()
{
	MODM_LOG_ERROR << "HRTIM DLL calibration failed" << modm::endl;
	while (true) {}
}

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
	LedD13::setOutput();

	MODM_LOG_INFO << "modm example: STM32G4 HRTIM complementary PWM" << modm::endl;

	Hrtim1::connect<PhaseAHigh::Cha1, PhaseALow::Cha2, PhaseBHigh::Chb1, PhaseBLow::Chb2,
					PhaseCHigh::Chd1, PhaseCLow::Chd2>();

	Hrtim1::enable();
	if (!Hrtim1::calibrate()) { stopOnCalibrationError(); }

	configurePhase(Hrtim1::Timer::A, Hrtim1::CompareUnit::Compare1, period / 2);
	configurePhase(Hrtim1::Timer::B, Hrtim1::CompareUnit::Compare1, period / 2);
	configurePhase(Hrtim1::Timer::D, Hrtim1::CompareUnit::Compare1, period / 2);

	Hrtim1::configurePwm<PhaseAHigh::Cha1>(Hrtim1::CompareUnit::Compare1, period / 2,
										   Hrtim1::OutputPolarity::ActiveHigh, false, false);
	Hrtim1::configurePwm<PhaseALow::Cha2>(Hrtim1::CompareUnit::Compare1, period / 2,
										  Hrtim1::OutputPolarity::ActiveHigh, true, false);

	Hrtim1::configurePwm<PhaseBHigh::Chb1>(Hrtim1::CompareUnit::Compare1, period / 2,
										   Hrtim1::OutputPolarity::ActiveHigh, false, false);
	Hrtim1::configurePwm<PhaseBLow::Chb2>(Hrtim1::CompareUnit::Compare1, period / 2,
										  Hrtim1::OutputPolarity::ActiveHigh, true, false);

	Hrtim1::configurePwm<PhaseCHigh::Chd1>(Hrtim1::CompareUnit::Compare1, period / 2,
										   Hrtim1::OutputPolarity::ActiveHigh, false, false);
	Hrtim1::configurePwm<PhaseCLow::Chd2>(Hrtim1::CompareUnit::Compare1, period / 2,
										  Hrtim1::OutputPolarity::ActiveHigh, true, false);

	Hrtim1::applyAndReset(timers);

	Hrtim1::enableOutput<PhaseAHigh::Cha1>();
	Hrtim1::enableOutput<PhaseALow::Cha2>();
	Hrtim1::enableOutput<PhaseBHigh::Chb1>();
	Hrtim1::enableOutput<PhaseBLow::Chb2>();
	Hrtim1::enableOutput<PhaseCHigh::Chd1>();
	Hrtim1::enableOutput<PhaseCLow::Chd2>();

	Hrtim1::start(timers);

	// Sweep the high-side duty cycle from 10% to 90% in about 0.5% steps.
	// At 2 MHz this moves the high-side pulse width from 50 ns to 450 ns.
	// The low-side outputs are inverted complements of the high-side outputs,
	// with the dead time above inserted by HRTIM.
	int32_t duty = period / 2;
	int32_t step = period / 200;

	while (true)
	{
		duty += step;
		if (duty >= (period * 9) / 10 || duty <= period / 10) { step = -step; }

		const auto compare = static_cast<Hrtim1::Value>(duty);
		Hrtim1::setCompareValue(Hrtim1::Timer::A, Hrtim1::CompareUnit::Compare1, compare);
		Hrtim1::setCompareValue(Hrtim1::Timer::B, Hrtim1::CompareUnit::Compare1, compare);
		Hrtim1::setCompareValue(Hrtim1::Timer::D, Hrtim1::CompareUnit::Compare1, compare);
		Hrtim1::applyUpdate(timers);

		LedD13::toggle();
		modm::delay(20ms);
	}

	return 0;
}
