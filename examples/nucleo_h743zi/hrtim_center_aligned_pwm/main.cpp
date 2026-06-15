/*
 * Copyright (c) 2026, modm project
 * SPDX-License-Identifier: MPL-2.0
 */

#include <atomic>
#include <cstdint>
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

// Board::SystemClock configures APB2 to 100 MHz. Since APB2 is prescaled, the
// HRTIM kernel clock is 2 * PCLK2 = 200 MHz. Mul32 therefore drives the HRTIM
// waveform timers from a 6.4 GHz high-resolution time base.
constexpr auto prescaler = Hrtim1::Prescaler::Mul32;
constexpr auto timerClock = 6'400'000'000ull;

// 100 kHz PWM: 10 us carrier period with 156.25 ps HRTIM ticks.
// This is intentionally slower than the 2 MHz PWM example, because this example
// also enables two Timer A interrupt sources per PWM frame.
constexpr auto pwmFrequency = 100'000ull;
constexpr Hrtim1::Value period = timerClock / pwmFrequency;
static_assert(period == 64'000);

// The HRTIM counter is still edge-counting. The fake center-aligned waveform is
// built by setting the high-side output at center - duty / 2 and resetting it at
// center + duty / 2. The low-side output uses the opposite events.
constexpr Hrtim1::Value center = period / 2;
constexpr Hrtim1::Value phaseADuty = period / 2;         // 50%, high for 5 us
constexpr Hrtim1::Value phaseBDuty = (period * 3) / 10;  // 30%, high for 3 us
constexpr Hrtim1::Value phaseCDuty = (period * 7) / 10;  // 70%, high for 7 us

// Compare1 is the high-side midpoint interrupt at 5 us into the frame.
// The low-side interval wraps around the period boundary, therefore the Timer A
// repetition event is the exact center of the commanded low-side interval.
constexpr auto highCenterInterrupt = Hrtim1::CompareUnit::Compare1;

// Deadtime is inserted between each timer's output 1 and output 2.
// 64 ticks at 6.4 GHz are 10 ns.
constexpr Hrtim1::Value deadTime = 64;

std::atomic<uint32_t> highCenterInterrupts{0};
std::atomic<uint32_t> lowCenterInterrupts{0};

void
incrementFromInterrupt(std::atomic<uint32_t>& counter)
{ counter.store(counter.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed); }

void
setCenteredDuty(Hrtim1::Timer timer, Hrtim1::Value duty)
{
	const auto halfDuty = duty / 2;
	Hrtim1::setCompareValue(timer, highCenterInterrupt, center);
	Hrtim1::setCompareValue(timer, Hrtim1::CompareUnit::Compare2, center - halfDuty);
	Hrtim1::setCompareValue(timer, Hrtim1::CompareUnit::Compare3, center + halfDuty);
}

void
configurePhase(Hrtim1::Timer timer, Hrtim1::Value duty)
{
	Hrtim1::setMode(timer, prescaler);
	Hrtim1::setPeriod(timer, period);
	Hrtim1::setRepetitionCount(timer, 0);
	Hrtim1::setDeadTime(timer, deadTime, deadTime);
	setCenteredDuty(timer, duty);
}

template<typename HighSide, typename LowSide>
void
configureCenteredOutputs()
{
	// High side: active from Compare2 to Compare3.
	Hrtim1::configureOutput<HighSide>(Hrtim1::OutputEvent::Compare2, Hrtim1::OutputEvent::Compare3,
									  Hrtim1::OutputPolarity::ActiveHigh, false);

	// Low side: active from Compare3 until Compare2 in the next frame.
	Hrtim1::configureOutput<LowSide>(Hrtim1::OutputEvent::Compare3, Hrtim1::OutputEvent::Compare2,
									 Hrtim1::OutputPolarity::ActiveHigh, false);
}

void
enableTimerAInterrupts()
{
	auto& timerA = HRTIM1->sTimerxRegs[0];
	constexpr uint32_t interruptFlags = HRTIM_TIMICR_CMP1C | HRTIM_TIMICR_REPC;

	timerA.TIMxICR = interruptFlags;
	timerA.TIMxDIER |= HRTIM_TIMDIER_CMP1IE | HRTIM_TIMDIER_REPIE;

	NVIC_SetPriority(HRTIM1_TIMA_IRQn, 5);
	NVIC_ClearPendingIRQ(HRTIM1_TIMA_IRQn);
	NVIC_EnableIRQ(HRTIM1_TIMA_IRQn);
}
}  // namespace

MODM_ISR(HRTIM1_TIMA)
{
	auto& timerA = HRTIM1->sTimerxRegs[0];
	const auto flags = timerA.TIMxISR;
	uint32_t clearFlags = 0;

	if (flags & HRTIM_TIMISR_CMP1)
	{
		incrementFromInterrupt(highCenterInterrupts);
		clearFlags |= HRTIM_TIMICR_CMP1C;
	}

	if (flags & HRTIM_TIMISR_REP)
	{
		incrementFromInterrupt(lowCenterInterrupts);
		clearFlags |= HRTIM_TIMICR_REPC;
	}

	timerA.TIMxICR = clearFlags;
}

int
main()
{
	Board::initialize();

	MODM_LOG_INFO << "modm example: HRTIM fake center-aligned PWM" << modm::endl;

	Hrtim1::connect<PhaseAHigh::Cha1, PhaseALow::Cha2, PhaseBHigh::Chb1, PhaseBLow::Chb2,
					PhaseCHigh::Chc1, PhaseCLow::Chc2>();
	Hrtim1::enable();

	configurePhase(Hrtim1::Timer::A, phaseADuty);
	configurePhase(Hrtim1::Timer::B, phaseBDuty);
	configurePhase(Hrtim1::Timer::C, phaseCDuty);

	configureCenteredOutputs<PhaseAHigh::Cha1, PhaseALow::Cha2>();
	configureCenteredOutputs<PhaseBHigh::Chb1, PhaseBLow::Chb2>();
	configureCenteredOutputs<PhaseCHigh::Chc1, PhaseCLow::Chc2>();

	enableTimerAInterrupts();
	Hrtim1::applyAndReset(timers);

	Hrtim1::enableOutput<PhaseAHigh::Cha1>();
	Hrtim1::enableOutput<PhaseALow::Cha2>();
	Hrtim1::enableOutput<PhaseBHigh::Chb1>();
	Hrtim1::enableOutput<PhaseBLow::Chb2>();
	Hrtim1::enableOutput<PhaseCHigh::Chc1>();
	Hrtim1::enableOutput<PhaseCLow::Chc2>();

	Hrtim1::start(timers);

	uint32_t nextReport = 100'000;
	while (true)
	{
		const uint32_t highCenter = highCenterInterrupts.load(std::memory_order_relaxed);
		if (highCenter >= nextReport)
		{
			nextReport += 100'000;
			LedGreen::toggle();

			const uint32_t lowCenter = lowCenterInterrupts.load(std::memory_order_relaxed);
			MODM_LOG_INFO << "high-center=" << highCenter << " low-center=" << lowCenter
						  << modm::endl;
		}

		modm::delay(1ms);
	}
}
