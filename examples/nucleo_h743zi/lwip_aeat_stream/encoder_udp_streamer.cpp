#include "encoder_udp_streamer.hpp"

#include "encoder.hpp"
#include "encoder_sample_buffer.hpp"
#include "encoder_stream_protocol.h"

#include <modm_lwip/udp_socket.hpp>

#include <algorithm>
#include <cstddef>
#include <cstring>

EncoderSampleBuffer encoderSampleBuffer;

namespace
{

modm::lwip::UdpSocket streamSocket;
modm::lwip::UdpEndpoint streamRemote;

bool streamingEnabled;
uint32_t commandSequence;
uint32_t dataPacketSequence;
uint32_t samplesSent;
uint32_t streamDropCount;
uint32_t udpPacketsSent;
uint32_t udpSendErrors;
uint32_t decimation = 1;
uint32_t decimationPhase;
uint32_t maxSamplesPerPacket = ENCODER_STREAM_DEFAULT_SAMPLES_PER_PACKET;

constexpr uint32_t
maxSamplesByMtu()
{
	return (ENCODER_STREAM_MAX_UDP_PAYLOAD - sizeof(EncoderStreamDataPacket)) /
			sizeof(EncoderSample);
}

bool
validHeader(modm::lwip::PacketView packet, EncoderStreamHeader& header)
{
	return packet.copyAs(header) &&
			packet.size() >= sizeof(EncoderStreamHeader) &&
			header.magic == ENCODER_STREAM_MAGIC &&
			header.version == ENCODER_STREAM_VERSION &&
			header.payload_len == packet.size();
}

void
fillHeader(EncoderStreamHeader& header, uint16_t type, uint32_t sequence, uint32_t payloadLength)
{
	header.magic = ENCODER_STREAM_MAGIC;
	header.version = ENCODER_STREAM_VERSION;
	header.type = type;
	header.sequence = sequence;
	header.payload_len = payloadLength;
}

void
sendStatus(const modm::lwip::UdpEndpoint& remote)
{
	EncoderStreamStatusPacket status {};
	fillHeader(status.header, ENCODER_STREAM_STATUS, commandSequence++, sizeof(status));
	status.streaming_enabled = streamingEnabled ? 1u : 0u;
	status.samples_produced = encoderSampleBuffer.producedCount();
	status.samples_sent = samplesSent;
	status.ring_overruns = encoderSampleBuffer.overrunCount();
	status.ring_fill_level = encoderSampleBuffer.fillLevel();
	status.ring_max_fill_level = encoderSampleBuffer.maxFillLevel();
	status.udp_packets_sent = udpPacketsSent;
	status.udp_send_errors = udpSendErrors;
	status.last_encoder_position = Encoder::lastPosition.load(std::memory_order_relaxed);

	if (!streamSocket.sendTo(remote, &status, sizeof(status))) {
		udpSendErrors++;
	}
}

void
handleStart(modm::lwip::PacketView packet, const modm::lwip::UdpEndpoint& remote)
{
	EncoderStreamStartCommand command {};
	if (packet.size() < sizeof(EncoderStreamStartCommand) ||
			!packet.copyTo(&command, sizeof(command))) {
		sendStatus(remote);
		return;
	}

	streamRemote = remote;
	decimation = command.requested_sample_decimation;
	if (decimation == 0) {
		decimation = 1;
	}
	decimationPhase = 0;

	const uint32_t maxByMtu = std::min<uint32_t>(
			ENCODER_STREAM_MAX_SAMPLES_PER_PACKET, maxSamplesByMtu());
	maxSamplesPerPacket = command.max_samples_per_packet;
	if (maxSamplesPerPacket == 0) {
		maxSamplesPerPacket = ENCODER_STREAM_DEFAULT_SAMPLES_PER_PACKET;
	}
	if (maxSamplesPerPacket > maxByMtu) {
		maxSamplesPerPacket = maxByMtu;
	}

	dataPacketSequence = 0;
	streamingEnabled = true;
	sendStatus(remote);
}

void
handleStop(const modm::lwip::UdpEndpoint& remote)
{
	streamingEnabled = false;
	sendStatus(remote);
}

void
handleCommand(modm::lwip::UdpSocket&,
		const modm::lwip::UdpEndpoint& remote,
		modm::lwip::PacketView packet,
		void*)
{
	EncoderStreamHeader header {};
	if (!validHeader(packet, header)) {
		return;
	}

	switch (header.type) {
	case ENCODER_STREAM_CMD_START:
		handleStart(packet, remote);
		break;
	case ENCODER_STREAM_CMD_STOP:
		handleStop(remote);
		break;
	case ENCODER_STREAM_CMD_STATUS:
		sendStatus(remote);
		break;
	default:
		break;
	}
}

bool
samplePassesDecimation()
{
	const bool send = decimationPhase == 0;
	decimationPhase++;
	if (decimationPhase >= decimation) {
		decimationPhase = 0;
	}
	return send;
}

void
discardWhenStopped()
{
	EncoderSample sample {};
	uint32_t budget = 256;
	while (budget-- > 0 && encoderSampleBuffer.pop(sample)) {
		streamDropCount++;
	}
}

bool
sendDataPacket()
{
	EncoderSample packetSamples[ENCODER_STREAM_MAX_SAMPLES_PER_PACKET] {};
	uint32_t sampleCount = 0;
	uint32_t firstSequence = 0;
	EncoderSample sample {};

	while (sampleCount < maxSamplesPerPacket && encoderSampleBuffer.pop(sample)) {
		if (!samplePassesDecimation()) {
			streamDropCount++;
			continue;
		}
		if (sampleCount == 0) {
			firstSequence = sample.sequence;
		}
		packetSamples[sampleCount++] = sample;
	}

	if (sampleCount == 0) {
		return false;
	}

	const std::size_t payloadSize =
			sizeof(EncoderStreamDataPacket) + sampleCount * sizeof(EncoderSample);
	modm::lwip::PacketBuffer packet = modm::lwip::PacketBuffer::allocateTransport(payloadSize);
	if (!packet.isValid()) {
		udpSendErrors++;
		return false;
	}

	EncoderStreamDataPacket header {};
	fillHeader(header.header, ENCODER_STREAM_DATA, dataPacketSequence, payloadSize);
	header.first_sample_sequence = firstSequence;
	header.sample_count = sampleCount;
	header.sample_drop_count = streamDropCount;
	header.ring_overrun_count = encoderSampleBuffer.overrunCount();

	bool writeOk = packet.write(&header, sizeof(header)) &&
			packet.write(packetSamples, sampleCount * sizeof(EncoderSample), sizeof(header));
	if (!writeOk) {
		udpSendErrors++;
		return false;
	}

	if (streamSocket.sendTo(streamRemote, static_cast<modm::lwip::PacketBuffer&&>(packet))) {
		udpPacketsSent++;
		samplesSent += sampleCount;
		dataPacketSequence++;
		return true;
	}

	udpSendErrors++;
	return false;
}

} // namespace

extern "C" void
encoder_udp_streamer_initialize(void)
{
	if (streamSocket.isBound()) {
		return;
	}

	streamSocket.onReceive(handleCommand);
	if (!streamSocket.bind(ENCODER_STREAM_CONTROL_PORT)) {
		udpSendErrors++;
	}
}

extern "C" void
encoder_udp_streamer_poll(void)
{
	if (!streamSocket.isBound()) {
		return;
	}

	if (!streamingEnabled) {
		discardWhenStopped();
		return;
	}

	uint32_t budget = ENCODER_STREAM_MAX_PACKETS_PER_POLL;
	while (budget-- > 0) {
		if (!sendDataPacket()) {
			break;
		}
	}
}

extern "C" uint32_t encoder_udp_streamer_samples_sent(void) { return samplesSent; }
extern "C" uint32_t encoder_udp_streamer_packets_sent(void) { return udpPacketsSent; }
extern "C" uint32_t encoder_udp_streamer_send_errors(void) { return udpSendErrors; }
extern "C" bool encoder_udp_streamer_is_enabled(void) { return streamingEnabled; }
