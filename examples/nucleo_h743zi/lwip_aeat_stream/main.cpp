#include <modm/board.hpp>
#include <modm/processing.hpp>

#include <modm_lwip.hpp>

#include "encoder_sample_buffer.hpp"
#include "encoder_udp_streamer.hpp"
#include "hardware.hpp"
#include "tasks/encoder_interrupt.hpp"

using namespace Board;
using namespace std::chrono_literals;

int
main()
{
	SCB_DisableICache();
	SCB_DisableDCache();
	Board::initialize();

	Leds::setOutput();
	MODM_LOG_INFO << "\n\nReboot: AEAT UDP encoder stream example" << modm::endl;
	MODM_LOG_INFO << "SPI3: SCK=PB3, MISO=PB4, MOSI=PB5, CS=PA4" << modm::endl;
	MODM_LOG_INFO << "Encoder timer sample frequency: "
	              << Board::EncoderHardware::Aeat9922::SampleFrequency << " Hz" << modm::endl;

	static constexpr modm::lwip::EthernetConfig network {
		{ 192, 168, 1, 50 },
		{ 255, 255, 255, 0 },
		{ 192, 168, 1, 1 },
	};

	if (!modm::lwip::Ethernet::initialize(network)) {
		MODM_LOG_ERROR << "Ethernet/lwIP initialization failed" << modm::endl;
	}

	while (!Board::EncoderHardware::Aeat9922::initialize()) {
		Board::LedRed::toggle();
		modm::delay(250ms);
	}
	Board::LedRed::reset();
	Board::LedGreen::set();

	encoder_udp_streamer_initialize();

	modm::ShortPeriodicTimer statusTimer { 1s };
	while (true) {
		modm::lwip::Ethernet::poll();
		encoder_udp_streamer_poll();

		if (statusTimer.execute()) {
			Leds::toggle();
			modm::lwip::Ethernet::pollStatus();
			MODM_LOG_INFO << "stream="
			              << static_cast<unsigned int>(encoder_udp_streamer_is_enabled())
			              << ", fill=" << encoderSampleBuffer.fillLevel()
			              << ", max_fill=" << encoderSampleBuffer.maxFillLevel()
			              << ", produced=" << encoderSampleBuffer.producedCount()
			              << ", sent=" << encoder_udp_streamer_samples_sent()
			              << ", overruns=" << encoderSampleBuffer.overrunCount()
			              << ", udp_packets=" << encoder_udp_streamer_packets_sent()
			              << ", udp_errors=" << encoder_udp_streamer_send_errors()
			              << modm::endl;
		}
	}
}
