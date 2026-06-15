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

#include <atomic>
#include <cstdint>
#include <modm/board.hpp>

using namespace Board;

using PwmOut = GpioC6;
using RegularInput = A0;
using InjectedInput = A1;

namespace
{

constexpr auto timer = Hrtim1::Timer::A;
constexpr auto timers = Hrtim1::TimerCounter::A;

// Board::SystemClock configures APB2 to 100 MHz. With the default HRTIM clock
// selection and APB2 prescaler != 1, HRTIM1 runs at 2 * PCLK2 = 200 MHz.
// Prescaler::Div4 selects a 50 MHz Timer A time base.
//
//     50 MHz / 50'000 = 1 kHz
//
// CHA1 on PC6 is configured as a 50% duty PWM reference. ADC trigger 1 fires
// at the Timer A period event once per PWM frame. ADC trigger 2 fires at
// Timer A compare 2, one quarter into the frame. Both triggers therefore run
// at 1 kHz, with the injected sequence delayed by 250 us from the regular
// conversion trigger.
constexpr auto prescaler = Hrtim1::Prescaler::Div4;
constexpr Hrtim1::Value period = 50'000;
constexpr Hrtim1::Value pwmCompare = period / 2;
constexpr Hrtim1::Value injectedTriggerCompare = period / 4;

std::atomic<uint32_t> regularCount{0};
std::atomic<uint32_t> injectedCount{0};
std::atomic<uint32_t> adcErrorCount{0};
std::atomic<uint32_t> regularValue{0};
std::atomic<uint32_t> injectedA1Value{0};
std::atomic<uint32_t> injectedA0Value{0};

void
configureHrtim()
{
	Hrtim1::connect<PwmOut::Cha1>();
	Hrtim1::enable();

	Hrtim1::setMode(timer, prescaler);
	Hrtim1::setPeriod(timer, period);
	Hrtim1::setCompareValue(timer, Hrtim1::CompareUnit::Compare2,
			injectedTriggerCompare);
	Hrtim1::configurePwm<PwmOut::Cha1>(
			Hrtim1::CompareUnit::Compare1, pwmCompare,
			Hrtim1::OutputPolarity::ActiveHigh, false, false);

	const bool triggersConfigured =
			Hrtim1::configureAdcTrigger(Hrtim1::AdcTrigger::Trigger1,
					Hrtim1::AdcTriggerSource::TimerAPeriod,
					Hrtim1::AdcTriggerUpdateSource::TimerA) &&
			Hrtim1::configureAdcTrigger(Hrtim1::AdcTrigger::Trigger2,
					Hrtim1::AdcTriggerSource::TimerACompare2,
					Hrtim1::AdcTriggerUpdateSource::TimerA);

	if (!triggersConfigured)
	{
		MODM_LOG_ERROR << "HRTIM ADC trigger configuration failed" << modm::endl;
		while (true) {
		}
	}
}

void
configureAdc()
{
	Adc1::connect<RegularInput::Inp15, InjectedInput::Inp10>();
	Adc1::initialize(Adc1::ClockMode::SynchronousPrescaler4,
			Adc1::ClockSource::NoClock,
			Adc1::Prescaler::Disabled,
			Adc1::CalibrationMode::SingleEndedInputsMode);

	const bool channelsConfigured =
			Adc1::setPinChannel<RegularInput>(Adc1::SampleTime::Cycles17) &&
			Adc1::setInjectedConversionSequenceLength(2) &&
			Adc1::setInjectedConversionChannel<InjectedInput>(0,
					Adc1::SampleTime::Cycles17) &&
			Adc1::setInjectedConversionChannel<RegularInput>(1,
					Adc1::SampleTime::Cycles17);

	if (!channelsConfigured)
	{
		MODM_LOG_ERROR << "ADC channel configuration failed" << modm::endl;
		while (true) {
		}
	}

	Adc1::enableRegularConversionExternalTrigger(
			Adc1::ExternalTriggerPolarity::RisingEdge,
			Adc1::ExternalTriggerEvent::HrtimRegularTrigger1);
	Adc1::enableInjectedConversionExternalTrigger(
			Adc1::ExternalTriggerPolarity::RisingEdge,
			Adc1::ExternalTriggerEvent::HrtimInjectedTrigger2);
}

void
handleAdcInterrupt()
{
	const auto flags = Adc1::getInterruptFlags();

	if (flags & Adc1::InterruptFlag::EndOfRegularConversion)
	{
		regularValue.store(Adc1::getValue(), std::memory_order_relaxed);
		regularCount.fetch_add(1, std::memory_order_relaxed);
		Adc1::acknowledgeInterruptFlags(Adc1::InterruptFlag::EndOfRegularConversion |
				Adc1::InterruptFlag::EndOfRegularSequenceOfConversions);
	}

	if (flags & Adc1::InterruptFlag::EndOfInjectedSequenceOfConversions)
	{
		injectedA1Value.store(Adc1::getInjectedConversionValue(0),
				std::memory_order_relaxed);
		injectedA0Value.store(Adc1::getInjectedConversionValue(1),
				std::memory_order_relaxed);
		injectedCount.fetch_add(1, std::memory_order_relaxed);
		Adc1::acknowledgeInterruptFlags(Adc1::InterruptFlag::EndOfInjectedConversion |
				Adc1::InterruptFlag::EndOfInjectedSequenceOfConversions);
	}

	const auto errorFlags = flags & (Adc1::InterruptFlag::Overrun |
			Adc1::InterruptFlag::InjectedContextQueueOverflow);
	if (errorFlags)
	{
		adcErrorCount.fetch_add(1, std::memory_order_relaxed);
		Adc1::acknowledgeInterruptFlags(errorFlags);
	}
}

} // namespace

int
main()
{
	Board::initialize();

	MODM_LOG_INFO << "modm example: HRTIM-triggered ADC" << modm::endl;

	configureHrtim();
	configureAdc();

	AdcInterrupt1::attachInterruptHandler(handleAdcInterrupt);
	Adc1::enableInterruptVector(5);
	Adc1::enableInterrupt(Adc1::Interrupt::EndOfRegularConversion |
			Adc1::Interrupt::EndOfInjectedSequenceOfConversions |
			Adc1::Interrupt::Overrun |
			Adc1::Interrupt::InjectedContextQueueOverflow);

	Hrtim1::applyAndReset(timers);

	// With the external trigger source selected, these calls arm the regular
	// and injected groups once. The conversions then start on their periodic
	// HRTIM trigger edges, not immediately here.
	Adc1::startConversion();
	Adc1::startInjectedConversionSequence();

	Hrtim1::enableOutput<PwmOut::Cha1>();
	Hrtim1::start(timers);

	uint32_t nextReport = 500;

	while (true)
	{
		const uint32_t count = regularCount.load(std::memory_order_relaxed);
		if (count >= nextReport)
		{
			nextReport += 500;
			LedGreen::toggle();

			const uint32_t injected = injectedCount.load(std::memory_order_relaxed);
			const uint32_t errors = adcErrorCount.load(std::memory_order_relaxed);
			const uint32_t regular = regularValue.load(std::memory_order_relaxed);
			const uint32_t injectedA1 = injectedA1Value.load(std::memory_order_relaxed);
			const uint32_t injectedA0 = injectedA0Value.load(std::memory_order_relaxed);

			MODM_LOG_INFO << "regular=" << count
					<< " adc1_in15=" << regular
					<< " injected=" << injected
					<< " adc1_in10=" << injectedA1
					<< " adc1_in15=" << injectedA0
					<< " errors=" << errors
					<< modm::endl;
		}
	}

	return 0;
}
