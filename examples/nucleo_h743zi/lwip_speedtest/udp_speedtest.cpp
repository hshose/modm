#include "udp_speedtest.h"

#include <modm_lwip/udp_socket.hpp>

#include <cstddef>
#include <cstring>

#define UDP_SPEEDTEST_HEADER_SIZE 20U

extern "C" uint32_t sys_now(void);

namespace
{

struct __attribute__((packed)) UdpSpeedtestHeader
{
	uint32_t magic;
	uint16_t version;
	uint16_t type;
	uint32_t seq;
	uint32_t payload_len;
	uint32_t timestamp_us;
};

struct __attribute__((packed)) UdpSpeedtestRxStats
{
	UdpSpeedtestHeader header;
	uint32_t rx_packets;
	uint32_t rx_bytes;
	uint32_t missing_packets;
	uint32_t malformed_packets;
	uint32_t out_of_order_packets;
	uint32_t elapsed_us;
	uint32_t bytes_per_second;
};

struct __attribute__((packed)) UdpSpeedtestTxStart
{
	UdpSpeedtestHeader header;
	uint32_t packet_count;
	uint32_t payload_size;
	uint32_t inter_packet_delay_us;
	uint32_t tx_budget_per_poll;
	uint32_t fill_mode;
};

struct __attribute__((packed)) UdpSpeedtestTxStartVeryLegacy
{
	UdpSpeedtestHeader header;
	uint32_t packet_count;
	uint32_t payload_size;
	uint32_t inter_packet_delay_us;
};

struct __attribute__((packed)) UdpSpeedtestTxDone
{
	UdpSpeedtestHeader header;
	uint32_t attempted_packets;
	uint32_t sent_packets;
	uint32_t send_errors;
	uint32_t sent_bytes;
	uint32_t elapsed_us;
	uint32_t bytes_per_second;
	uint32_t active_tx_budget_per_poll;
	uint32_t udp_sendto_calls;
	uint32_t udp_sendto_errors;
	uint32_t tx_blocked_events;
	uint32_t tx_poll_calls;
	uint32_t tx_max_packets_per_poll;
	uint32_t active_fill_mode;
};

modm::lwip::UdpSocket speedtestSocket;

uint32_t rx_packets;
uint32_t rx_bytes;
uint32_t rx_missing_packets;
uint32_t rx_malformed_packets;
uint32_t rx_out_of_order_packets;
uint32_t rx_expected_seq;
uint32_t rx_first_time_us;
uint32_t rx_last_time_us;
bool rx_started;

bool tx_active;
modm::lwip::UdpEndpoint tx_endpoint;
uint32_t tx_requested_packets;
uint32_t tx_payload_size;
uint32_t tx_inter_packet_delay_us;
uint32_t tx_next_seq;
uint32_t tx_attempted_packets;
uint32_t tx_sent_packets;
uint32_t tx_send_errors;
uint32_t tx_sent_bytes;
uint32_t tx_start_time_us;
uint32_t tx_last_send_time_us;
uint32_t tx_budget_per_poll = UDP_SPEEDTEST_TX_BUDGET_DEFAULT;
uint32_t tx_sendto_calls;
uint32_t tx_sendto_errors;
uint32_t tx_blocked_events;
uint32_t tx_poll_calls;
uint32_t tx_current_poll_packets;
uint32_t tx_max_packets_per_poll;
UdpSpeedtestFillMode tx_fill_mode = UDP_SPEEDTEST_FILL_FULL_PATTERN;

uint8_t tx_payload[UDP_SPEEDTEST_MAX_PAYLOAD_SIZE];

uint32_t
udp_speedtest_time_us()
{
	return sys_now() * 1000UL;
}

uint32_t
elapsed_us(uint32_t start, uint32_t end)
{
	return end - start;
}

uint32_t
bytes_per_second(uint32_t bytes, uint32_t elapsed)
{
	if (elapsed == 0) {
		return 0;
	}
	return static_cast<uint32_t>((static_cast<uint64_t>(bytes) * 1000000ULL) / elapsed);
}

bool
copy_header(modm::lwip::PacketView packet, UdpSpeedtestHeader &header)
{
	if (packet.size() < sizeof(header)) {
		return false;
	}

	return packet.copyAs(header) &&
			header.magic == UDP_SPEEDTEST_MAGIC &&
			header.version == UDP_SPEEDTEST_VERSION &&
			header.payload_len == packet.size();
}

void
fill_header(UdpSpeedtestHeader &header, uint16_t type, uint32_t seq, uint32_t payload_len)
{
	header.magic = UDP_SPEEDTEST_MAGIC;
	header.version = UDP_SPEEDTEST_VERSION;
	header.type = type;
	header.seq = seq;
	header.payload_len = payload_len;
	header.timestamp_us = udp_speedtest_time_us();
}

void
reset_rx_stats()
{
	rx_packets = 0;
	rx_bytes = 0;
	rx_missing_packets = 0;
	rx_malformed_packets = 0;
	rx_out_of_order_packets = 0;
	rx_expected_seq = 0;
	rx_first_time_us = 0;
	rx_last_time_us = 0;
	rx_started = false;
}

void
stop_tx_test()
{
	tx_active = false;
	tx_requested_packets = 0;
	tx_payload_size = 0;
	tx_inter_packet_delay_us = 0;
	tx_next_seq = 0;
}

void
send_rx_stats(modm::lwip::UdpSocket &socket, const modm::lwip::UdpEndpoint &remote)
{
	UdpSpeedtestRxStats stats {};
	const uint32_t duration = rx_started ? elapsed_us(rx_first_time_us, rx_last_time_us) : 0;

	fill_header(stats.header, UDP_SPEEDTEST_RX_STATS, 0, sizeof(stats));
	stats.rx_packets = rx_packets;
	stats.rx_bytes = rx_bytes;
	stats.missing_packets = rx_missing_packets;
	stats.malformed_packets = rx_malformed_packets;
	stats.out_of_order_packets = rx_out_of_order_packets;
	stats.elapsed_us = duration;
	stats.bytes_per_second = bytes_per_second(rx_bytes, duration);

	(void) socket.sendTo(remote, &stats, sizeof(stats));
}

void
send_tx_done()
{
	UdpSpeedtestTxDone done {};
	const uint32_t now = udp_speedtest_time_us();
	const uint32_t duration = elapsed_us(tx_start_time_us, now);

	fill_header(done.header, UDP_SPEEDTEST_TX_DONE, tx_next_seq, sizeof(done));
	done.attempted_packets = tx_attempted_packets;
	done.sent_packets = tx_sent_packets;
	done.send_errors = tx_send_errors;
	done.sent_bytes = tx_sent_bytes;
	done.elapsed_us = duration;
	done.bytes_per_second = bytes_per_second(tx_sent_bytes, duration);
	done.active_tx_budget_per_poll = tx_budget_per_poll;
	done.udp_sendto_calls = tx_sendto_calls;
	done.udp_sendto_errors = tx_sendto_errors;
	done.tx_blocked_events = tx_blocked_events;
	done.tx_poll_calls = tx_poll_calls;
	done.tx_max_packets_per_poll = tx_max_packets_per_poll;
	done.active_fill_mode = static_cast<uint32_t>(tx_fill_mode);

	if (!speedtestSocket.sendTo(tx_endpoint, &done, sizeof(done))) {
		tx_send_errors++;
	}
	stop_tx_test();
}

void
handle_rx_data(const UdpSpeedtestHeader &header, uint16_t total_len)
{
	if (header.seq == 0) {
		reset_rx_stats();
	}

	const uint32_t now = udp_speedtest_time_us();

	if (!rx_started) {
		rx_started = true;
		rx_first_time_us = now;
		rx_expected_seq = header.seq + 1;
	}
	else if (header.seq == rx_expected_seq) {
		rx_expected_seq++;
	}
	else if (static_cast<int32_t>(header.seq - rx_expected_seq) > 0) {
		rx_missing_packets += header.seq - rx_expected_seq;
		rx_expected_seq = header.seq + 1;
	}
	else {
		rx_out_of_order_packets++;
	}

	rx_last_time_us = now;
	rx_packets++;
	rx_bytes += total_len;
}

void
handle_tx_start(modm::lwip::PacketView packet, const modm::lwip::UdpEndpoint &remote)
{
	UdpSpeedtestTxStart command {};
	if (packet.size() < sizeof(UdpSpeedtestTxStartVeryLegacy)) {
		rx_malformed_packets++;
		return;
	}

	const std::size_t command_size =
			(packet.size() < sizeof(command)) ? packet.size() : sizeof(command);
	packet.copyTo(&command, command_size);
	if (command.packet_count == 0 ||
			command.payload_size < sizeof(UdpSpeedtestHeader) ||
			command.payload_size > UDP_SPEEDTEST_MAX_PAYLOAD_SIZE) {
		rx_malformed_packets++;
		return;
	}

	tx_endpoint = remote;
	tx_requested_packets = command.packet_count;
	tx_payload_size = command.payload_size;
	tx_inter_packet_delay_us = command.inter_packet_delay_us;
	tx_budget_per_poll = command.tx_budget_per_poll;
	if (tx_budget_per_poll == 0) {
		tx_budget_per_poll = UDP_SPEEDTEST_TX_BUDGET_DEFAULT;
	}
	if (tx_budget_per_poll > UDP_SPEEDTEST_TX_BUDGET_MAX) {
		tx_budget_per_poll = UDP_SPEEDTEST_TX_BUDGET_MAX;
	}
	tx_fill_mode = static_cast<UdpSpeedtestFillMode>(command.fill_mode);
	if (tx_fill_mode != UDP_SPEEDTEST_FILL_FULL_PATTERN &&
			tx_fill_mode != UDP_SPEEDTEST_FILL_HEADER_ONLY) {
		tx_fill_mode = UDP_SPEEDTEST_FILL_FULL_PATTERN;
	}
	tx_next_seq = 0;
	tx_attempted_packets = 0;
	tx_sent_packets = 0;
	tx_send_errors = 0;
	tx_sent_bytes = 0;
	tx_sendto_calls = 0;
	tx_sendto_errors = 0;
	tx_blocked_events = 0;
	tx_poll_calls = 0;
	tx_current_poll_packets = 0;
	tx_max_packets_per_poll = 0;
	tx_start_time_us = udp_speedtest_time_us();
	tx_last_send_time_us = 0;
	tx_active = true;
}

bool
send_tx_packet()
{
	modm::lwip::PacketBuffer packet =
			modm::lwip::PacketBuffer::allocateTransport(tx_payload_size);
	if (!packet.isValid()) {
		tx_blocked_events++;
		return false;
	}

	tx_attempted_packets++;

	UdpSpeedtestHeader header {};
	fill_header(header, UDP_SPEEDTEST_TX_DATA, tx_next_seq, tx_payload_size);

	uint8_t *data = packet.data();
	bool write_ok = false;
	if (data != nullptr) {
		if (tx_fill_mode == UDP_SPEEDTEST_FILL_HEADER_ONLY) {
			std::memcpy(data, &header, sizeof(header));
		}
		else {
			std::memcpy(tx_payload, &header, sizeof(header));
			for (uint32_t index = sizeof(UdpSpeedtestHeader); index < tx_payload_size; ++index) {
				tx_payload[index] = static_cast<uint8_t>(tx_next_seq + index);
			}
			std::memcpy(data, tx_payload, tx_payload_size);
		}
		write_ok = true;
	}
	else if (tx_fill_mode == UDP_SPEEDTEST_FILL_HEADER_ONLY) {
		write_ok = packet.write(&header, sizeof(header));
	}
	else {
		std::memcpy(tx_payload, &header, sizeof(header));
		for (uint32_t index = sizeof(UdpSpeedtestHeader); index < tx_payload_size; ++index) {
			tx_payload[index] = static_cast<uint8_t>(tx_next_seq + index);
		}
		write_ok = packet.write(tx_payload, tx_payload_size);
	}

	if (!write_ok) {
		tx_send_errors++;
		return false;
	}

	tx_sendto_calls++;
	const bool sent = speedtestSocket.sendTo(tx_endpoint, static_cast<modm::lwip::PacketBuffer &&>(packet));
	if (sent) {
		tx_sent_packets++;
		tx_sent_bytes += tx_payload_size;
		tx_last_send_time_us = udp_speedtest_time_us();
		tx_next_seq++;
		tx_current_poll_packets++;
		if (tx_current_poll_packets > tx_max_packets_per_poll) {
			tx_max_packets_per_poll = tx_current_poll_packets;
		}
	}
	else {
		tx_send_errors++;
		tx_sendto_errors++;
	}

	return sent;
}

void
udp_speedtest_recv(modm::lwip::UdpSocket &socket,
		const modm::lwip::UdpEndpoint &remote,
		modm::lwip::PacketView packet,
		void *)
{
	UdpSpeedtestHeader header {};
	if (!copy_header(packet, header)) {
		rx_malformed_packets++;
		return;
	}

	switch (header.type) {
	case UDP_SPEEDTEST_RX_DATA:
		handle_rx_data(header, static_cast<uint16_t>(packet.size()));
		break;
	case UDP_SPEEDTEST_RX_DONE:
		send_rx_stats(socket, remote);
		break;
	case UDP_SPEEDTEST_TX_START:
		handle_tx_start(packet, remote);
		break;
	case UDP_SPEEDTEST_RESET:
		reset_rx_stats();
		stop_tx_test();
		break;
	default:
		rx_malformed_packets++;
		break;
	}
}

}

extern "C" void
udp_speedtest_init(void)
{
	if (speedtestSocket.isBound()) {
		return;
	}

	speedtestSocket.onReceive(udp_speedtest_recv);
	if (!speedtestSocket.bind(UDP_SPEEDTEST_PORT)) {
		rx_malformed_packets++;
	}
}

extern "C" void
udp_speedtest_poll(void)
{
	if (!tx_active || !speedtestSocket.isBound()) {
		return;
	}

	tx_poll_calls++;
	tx_current_poll_packets = 0;
	uint32_t budget = tx_budget_per_poll;
	while (tx_active && budget-- > 0) {
		const uint32_t now = udp_speedtest_time_us();
		if (tx_inter_packet_delay_us != 0 &&
				tx_last_send_time_us != 0 &&
				elapsed_us(tx_last_send_time_us, now) < tx_inter_packet_delay_us) {
			return;
		}

		if (tx_next_seq >= tx_requested_packets) {
			send_tx_done();
			return;
		}

		if (!send_tx_packet()) {
			break;
		}
	}
}

extern "C" uint32_t
udp_speedtest_get_rx_packet_count(void)
{
	return rx_packets;
}

extern "C" uint32_t
udp_speedtest_get_rx_byte_count(void)
{
	return rx_bytes;
}

extern "C" uint32_t
udp_speedtest_get_rx_drop_count(void)
{
	return rx_missing_packets;
}

extern "C" uint32_t
udp_speedtest_get_rx_malformed_count(void)
{
	return rx_malformed_packets;
}

extern "C" uint32_t
udp_speedtest_get_tx_packet_count(void)
{
	return tx_sent_packets;
}

extern "C" uint32_t
udp_speedtest_get_tx_byte_count(void)
{
	return tx_sent_bytes;
}

extern "C" uint32_t
udp_speedtest_get_tx_error_count(void)
{
	return tx_send_errors;
}

extern "C" bool
udp_speedtest_tx_active(void)
{
	return tx_active;
}
