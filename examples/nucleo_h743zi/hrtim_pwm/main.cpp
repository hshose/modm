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

constexpr Hrtim1::Value period = 20'000;
constexpr Hrtim1::Value deadTime = 64;

void
configurePhase(Hrtim1::Timer timer,
		Hrtim1::CompareUnit compare, Hrtim1::Value duty)
{
	Hrtim1::setMode(timer);
	Hrtim1::setPeriod(timer, period);
	Hrtim1::setCompareValue(timer, compare, duty);
	Hrtim1::setDeadTime(timer, deadTime, deadTime);
}

} // namespace

int
main()
{
	Board::initialize();

	MODM_LOG_INFO << "modm example: HRTIM complementary PWM" << modm::endl;

	Hrtim1::connect<
		PhaseAHigh::Cha1, PhaseALow::Cha2,
		PhaseBHigh::Chb1, PhaseBLow::Chb2,
		PhaseCHigh::Chc1, PhaseCLow::Chc2>();

	Hrtim1::enable();

	configurePhase(Hrtim1::Timer::A, Hrtim1::CompareUnit::Compare1, period / 2);
	configurePhase(Hrtim1::Timer::B, Hrtim1::CompareUnit::Compare1, period / 2);
	configurePhase(Hrtim1::Timer::C, Hrtim1::CompareUnit::Compare1, period / 2);

	Hrtim1::configurePwm<PhaseAHigh::Cha1>(
			Hrtim1::CompareUnit::Compare1, period / 2,
			Hrtim1::OutputPolarity::ActiveHigh, false, false);
	Hrtim1::configurePwm<PhaseALow::Cha2>(
			Hrtim1::CompareUnit::Compare1, period / 2,
			Hrtim1::OutputPolarity::ActiveHigh, true, false);

	Hrtim1::configurePwm<PhaseBHigh::Chb1>(
			Hrtim1::CompareUnit::Compare1, period / 2,
			Hrtim1::OutputPolarity::ActiveHigh, false, false);
	Hrtim1::configurePwm<PhaseBLow::Chb2>(
			Hrtim1::CompareUnit::Compare1, period / 2,
			Hrtim1::OutputPolarity::ActiveHigh, true, false);

	Hrtim1::configurePwm<PhaseCHigh::Chc1>(
			Hrtim1::CompareUnit::Compare1, period / 2,
			Hrtim1::OutputPolarity::ActiveHigh, false, false);
	Hrtim1::configurePwm<PhaseCLow::Chc2>(
			Hrtim1::CompareUnit::Compare1, period / 2,
			Hrtim1::OutputPolarity::ActiveHigh, true, false);

	Hrtim1::applyAndReset(timers);

	Hrtim1::enableOutput<PhaseAHigh::Cha1>();
	Hrtim1::enableOutput<PhaseALow::Cha2>();
	Hrtim1::enableOutput<PhaseBHigh::Chb1>();
	Hrtim1::enableOutput<PhaseBLow::Chb2>();
	Hrtim1::enableOutput<PhaseCHigh::Chc1>();
	Hrtim1::enableOutput<PhaseCLow::Chc2>();

	Hrtim1::start(timers);

	Hrtim1::Value duty = period / 2;
	int16_t step = period / 200;

	while (true)
	{
		duty += step;
		if (duty >= (period * 9) / 10 || duty <= period / 10) {
			step = -step;
		}

		Hrtim1::setCompareValue(Hrtim1::Timer::A, Hrtim1::CompareUnit::Compare1, duty);
		Hrtim1::setCompareValue(Hrtim1::Timer::B, Hrtim1::CompareUnit::Compare1, duty);
		Hrtim1::setCompareValue(Hrtim1::Timer::C, Hrtim1::CompareUnit::Compare1, duty);
		Hrtim1::applyUpdate(timers);

		LedGreen::toggle();
		modm::delay(20ms);
	}

	return 0;
}
