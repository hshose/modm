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
#include <modm/driver/ethernet/lan8742a.hpp>
#include <modm/processing/rtos.hpp>

#include <FreeRTOS_IP.h>
#include <FreeRTOS_Sockets.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>

using namespace Board;

namespace Ethernet
{
	using RMII_Ref_Clk = GpioInputA1;
	using RMII_Mdio = GpioA2;
	using RMII_Crs_Dv = GpioInputA7;
	using RMII_Tx_En = GpioOutputG11;
	using RMII_Tx_D0 = GpioOutputG13;
	using RMII_Tx_D1 = GpioOutputB13;
	using RMII_Mdc = GpioOutputC1;
	using RMII_Rx_D0 = GpioInputC4;
	using RMII_Rx_D1 = GpioInputC5;
	using Port = Eth<modm::Lan8742a>;
}

namespace Benchmark
{
constexpr uint32_t Magic { 0x4d42544d };
constexpr uint16_t Version { 1 };
constexpr uint16_t ControlPort { 5000 };
constexpr uint16_t DataPort { 5001 };
constexpr std::size_t MaxPayloadBytes { 1456 };
constexpr TickType_t ReceiveTimeout { pdMS_TO_TICKS(50) };
constexpr TickType_t ControlTimeout { pdMS_TO_TICKS(250) };
constexpr TickType_t ReceiveGraceTime { pdMS_TO_TICKS(250) };

enum class Command : uint16_t
{
	StartRx = 1,
	StartTx = 2,
	Ack = 0x8000,
	Result = 0x8001,
	Error = 0xffff,
};

struct Message
{
	uint32_t magic;
	uint16_t version;
	uint16_t command;
	uint32_t runId;
	uint16_t payloadBytes;
	uint16_t dataPort;
	uint32_t durationMs;
	uint32_t packetCount;
	uint64_t payloadTotalBytes;
	uint32_t sequenceErrors;
	uint32_t reserved;
} modm_packed;

struct DataHeader
{
	uint32_t magic;
	uint32_t runId;
	uint32_t sequence;
	uint16_t payloadBytes;
	uint16_t reserved;
} modm_packed;

static_assert(sizeof(Message) == 40);
static_assert(sizeof(DataHeader) == 16);

using DatagramBuffer = std::array<uint8_t, sizeof(DataHeader) + MaxPayloadBytes>;

constexpr uint32_t
ticksToMilliseconds(TickType_t ticks)
{
	return uint32_t((uint64_t(ticks) * 1000ULL) / configTICK_RATE_HZ);
}

void
fillPayload(uint8_t *payload, std::size_t size)
{
	for (std::size_t index = 0; index < size; ++index)
		payload[index] = uint8_t(index);
}

bool
isValidRequest(const Message &message)
{
	if (message.magic != Magic or message.version != Version)
		return false;
	if (message.payloadBytes > MaxPayloadBytes)
		return false;

	const auto command = Command(message.command);
	return command == Command::StartRx or command == Command::StartTx;
}

void
sendReply(Socket_t socket,
		const freertos_sockaddr &address,
		Command command,
		uint32_t runId,
		uint16_t payloadBytes,
		uint16_t dataPort,
		uint32_t durationMs,
		uint32_t packetCount,
		uint64_t payloadTotalBytes,
		uint32_t sequenceErrors,
		uint32_t reserved = 0)
{
	const Message reply {
		.magic = Magic,
		.version = Version,
		.command = uint16_t(command),
		.runId = runId,
		.payloadBytes = payloadBytes,
		.dataPort = dataPort,
		.durationMs = durationMs,
		.packetCount = packetCount,
		.payloadTotalBytes = payloadTotalBytes,
		.sequenceErrors = sequenceErrors,
		.reserved = reserved,
	};

	FreeRTOS_sendto(socket, &reply, sizeof(reply), 0, &address, sizeof(address));
}

void
printResult(const char *label,
		uint32_t packetCount,
		uint64_t payloadBytes,
		uint32_t durationMs,
		uint32_t sequenceErrors)
{
	const uint64_t bytesPerSecond = durationMs ? (payloadBytes * 1000ULL) / durationMs : 0;
	const uint64_t bitsPerSecond = bytesPerSecond * 8ULL;
	const uint32_t wholeMbps = uint32_t(bitsPerSecond / 1'000'000ULL);
	const uint32_t fracMbps = uint32_t((bitsPerSecond % 1'000'000ULL) / 1'000ULL);

	MODM_LOG_INFO << label
			<< ": packets=" << packetCount
			<< " payload=" << payloadBytes
			<< "B duration=" << durationMs
			<< "ms throughput=" << wholeMbps
			<< ".";
	if (fracMbps < 100) {
		MODM_LOG_INFO << "0";
	}
	if (fracMbps < 10) {
		MODM_LOG_INFO << "0";
	}
	MODM_LOG_INFO << fracMbps
			<< "Mbps seqerr=" << sequenceErrors
			<< modm::endl;
}

void
runReceiveTest(Socket_t controlSocket, Socket_t dataSocket,
		const freertos_sockaddr &controlPeer, const Message &request)
{
	DatagramBuffer buffer {};
	uint32_t packetCount { 0 };
	uint64_t payloadTotalBytes { 0 };
	uint32_t sequenceErrors { 0 };
	uint32_t expectedSequence { 0 };
	bool started { false };

	const TickType_t duration = pdMS_TO_TICKS(request.durationMs);
	const TickType_t commandStart = xTaskGetTickCount();
	TickType_t firstPacketTick { 0 };
	TickType_t lastPacketTick { commandStart };

	for (;;) {
		freertos_sockaddr sender {};
		socklen_t senderLength = sizeof(sender);
		const int32_t received = FreeRTOS_recvfrom(dataSocket, buffer.data(), buffer.size(), 0,
				&sender, &senderLength);
		const TickType_t now = xTaskGetTickCount();

		if (received >= int32_t(sizeof(DataHeader))) {
			const auto *header = reinterpret_cast<const DataHeader *>(buffer.data());
			if (header->magic == Magic and header->runId == request.runId) {
				if (not started) {
					started = true;
					firstPacketTick = now;
					expectedSequence = header->sequence;
				}

				if (header->sequence != expectedSequence) {
					if (header->sequence > expectedSequence)
						sequenceErrors += header->sequence - expectedSequence;
					else
						sequenceErrors++;
				}
				expectedSequence = header->sequence + 1;

				const std::size_t payloadBytes = std::min<std::size_t>(
						header->payloadBytes,
						std::size_t(received - sizeof(DataHeader)));
				payloadTotalBytes += payloadBytes;
				packetCount++;
				lastPacketTick = now;
			}
		}

		if (started) {
			if ((now - firstPacketTick) >= duration and (now - lastPacketTick) >= ReceiveGraceTime)
				break;
		}
		else if ((now - commandStart) >= (duration + pdMS_TO_TICKS(1000)))
			break;
	}

	const uint32_t durationMs = started ? ticksToMilliseconds(lastPacketTick - firstPacketTick) : 0;
	printResult("UDP RX", packetCount, payloadTotalBytes, durationMs, sequenceErrors);
	sendReply(controlSocket, controlPeer, Command::Result, request.runId, request.payloadBytes,
			request.dataPort, durationMs, packetCount, payloadTotalBytes, sequenceErrors);
}

void
runTransmitTest(Socket_t controlSocket, Socket_t dataSocket,
		const freertos_sockaddr &controlPeer, const Message &request)
{
	DatagramBuffer buffer {};
	auto *header = reinterpret_cast<DataHeader *>(buffer.data());
	auto *payload = buffer.data() + sizeof(DataHeader);
	fillPayload(payload, request.payloadBytes);

	header->magic = Magic;
	header->runId = request.runId;
	header->payloadBytes = request.payloadBytes;
	header->reserved = 0;

	freertos_sockaddr dataPeer = controlPeer;
	dataPeer.sin_port = FreeRTOS_htons(request.dataPort);

	const TickType_t duration = pdMS_TO_TICKS(request.durationMs);
	const TickType_t start = xTaskGetTickCount();

	uint32_t packetCount { 0 };
	uint64_t payloadTotalBytes { 0 };
	uint32_t sequenceErrors { 0 };

	while ((xTaskGetTickCount() - start) < duration and
			(request.packetCount == 0 or packetCount < request.packetCount)) {
		header->sequence = packetCount;

		const int32_t sent = FreeRTOS_sendto(dataSocket, buffer.data(),
				sizeof(DataHeader) + request.payloadBytes, 0, &dataPeer, sizeof(dataPeer));
		if (sent < 0) {
			sequenceErrors++;
			break;
		}
		if (sent != int32_t(sizeof(DataHeader) + request.payloadBytes)) {
			sequenceErrors++;
			continue;
		}

		packetCount++;
		payloadTotalBytes += request.payloadBytes;

		if ((packetCount & 0x3f) == 0)
			taskYIELD();
	}

	const uint32_t durationMs = ticksToMilliseconds(xTaskGetTickCount() - start);
	printResult("UDP TX", packetCount, payloadTotalBytes, durationMs, sequenceErrors);
	sendReply(controlSocket, controlPeer, Command::Result, request.runId, request.payloadBytes,
			request.dataPort, durationMs, packetCount, payloadTotalBytes, sequenceErrors);
}

class UdpBenchmarkServer
{
	static constexpr TickType_t SocketTimeout { pdMS_TO_TICKS(250) };

public:
	static constexpr char name[] { "UdpBenchmark" };

	static void
	run(void *)
	{
		const TickType_t timeout = SocketTimeout;
		Socket_t controlSocket = FreeRTOS_socket(FREERTOS_AF_INET, FREERTOS_SOCK_DGRAM,
				FREERTOS_IPPROTO_UDP);
		Socket_t dataSocket = FreeRTOS_socket(FREERTOS_AF_INET, FREERTOS_SOCK_DGRAM,
				FREERTOS_IPPROTO_UDP);

		configASSERT(controlSocket != FREERTOS_INVALID_SOCKET);
		configASSERT(dataSocket != FREERTOS_INVALID_SOCKET);

		FreeRTOS_setsockopt(controlSocket, 0, FREERTOS_SO_RCVTIMEO, &timeout, sizeof(timeout));
		FreeRTOS_setsockopt(dataSocket, 0, FREERTOS_SO_RCVTIMEO, &timeout, sizeof(timeout));

		freertos_sockaddr controlAddress {};
		controlAddress.sin_port = FreeRTOS_htons(ControlPort);
		FreeRTOS_bind(controlSocket, &controlAddress, sizeof(controlAddress));

		freertos_sockaddr dataAddress {};
		dataAddress.sin_port = FreeRTOS_htons(DataPort);
		FreeRTOS_bind(dataSocket, &dataAddress, sizeof(dataAddress));

		MODM_LOG_INFO << "UDP throughput control port " << ControlPort
				<< ", data port " << DataPort << modm::endl;

		for (;;) {
			Message request {};
			freertos_sockaddr controlPeer {};
			socklen_t controlPeerLength = sizeof(controlPeer);
			const int32_t received = FreeRTOS_recvfrom(controlSocket, &request, sizeof(request), 0,
					&controlPeer, &controlPeerLength);

			if (received < int32_t(sizeof(request)))
				continue;
			if (not isValidRequest(request)) {
				sendReply(controlSocket, controlPeer, Command::Error, request.runId,
						request.payloadBytes, request.dataPort, 0, 0, 0, 0, 1);
				continue;
			}

			sendReply(controlSocket, controlPeer, Command::Ack, request.runId,
					request.payloadBytes, request.dataPort, request.durationMs, 0, 0, 0);

			switch (Command(request.command)) {
			case Command::StartRx:
				runReceiveTest(controlSocket, dataSocket, controlPeer, request);
				break;
			case Command::StartTx:
				runTransmitTest(controlSocket, dataSocket, controlPeer, request);
				break;
			default:
				sendReply(controlSocket, controlPeer, Command::Error, request.runId,
						request.payloadBytes, request.dataPort, 0, 0, 0, 0, 2);
				break;
			}
		}
	}
};

} // namespace Benchmark

UBaseType_t ulNextRand;

void
vApplicationIPNetworkEventHook(eIPCallbackEvent_t eNetworkEvent);

class NetworkInitTask : modm::rtos::Thread
{
public:
	NetworkInitTask()
	: Thread(configMAX_PRIORITIES - 1, 2048, "network_init")
	{}

	void
	run() override
	{
		uint8_t ipAddress[4] { 192, 168, 10, 50 };
		uint8_t netmask[4] { 255, 255, 255, 0 };
		uint8_t gatewayAddress[4] { 0, 0, 0, 0 };
		uint8_t dnsAddress[4] { 0, 0, 0, 0 };
		uint8_t macAddress[] { 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 };

		time_t now;
		time(&now);
		ulNextRand = uint32_t(now);

		FreeRTOS_IPInit(ipAddress, netmask, gatewayAddress, dnsAddress, &macAddress[0]);
		vTaskDelete(nullptr);
	}
};

NetworkInitTask networkInit;

int
main()
{
	Board::initialize();
	Leds::setOutput();
	MODM_LOG_INFO << "\n\nReboot: Ethernet UDP Throughput" << modm::endl;

	Ethernet::Port::connect<Ethernet::RMII_Ref_Clk::Refclk,
		Ethernet::RMII_Mdc::Mdc,
		Ethernet::RMII_Mdio::Mdio,
		Ethernet::RMII_Crs_Dv::Rcccrsdv,
		Ethernet::RMII_Tx_En::Txen,
		Ethernet::RMII_Tx_D0::Txd0,
		Ethernet::RMII_Tx_D1::Txd1,
		Ethernet::RMII_Rx_D0::Rxd0,
		Ethernet::RMII_Rx_D1::Rxd1>();

	modm::rtos::Scheduler::schedule();
	return 0;
}

void
vApplicationIPNetworkEventHook(eIPCallbackEvent_t eNetworkEvent)
{
	static bool taskCreated = false;

	if (eNetworkEvent != eNetworkUp)
		return;

	if (not taskCreated) {
		xTaskCreate(Benchmark::UdpBenchmarkServer::run, Benchmark::UdpBenchmarkServer::name,
				configMINIMAL_STACK_SIZE * 6, nullptr, configMAX_PRIORITIES - 2, nullptr);
		taskCreated = true;
	}

	uint32_t ipAddress;
	uint32_t netmask;
	uint32_t gateway;
	uint32_t dns;
	char buffer[16];

	FreeRTOS_GetAddressConfiguration(&ipAddress, &netmask, &gateway, &dns);
	FreeRTOS_inet_ntoa(ipAddress, buffer);
	MODM_LOG_DEBUG << "IP address: " << buffer << modm::endl;
	FreeRTOS_inet_ntoa(netmask, buffer);
	MODM_LOG_DEBUG << "Netmask   : " << buffer << modm::endl;
	FreeRTOS_inet_ntoa(gateway, buffer);
	MODM_LOG_DEBUG << "Gateway   : " << buffer << modm::endl;
	FreeRTOS_inet_ntoa(dns, buffer);
	MODM_LOG_DEBUG << "DNS       : " << buffer << modm::endl;
}

UBaseType_t
uxRand()
{
	static constexpr uint32_t ulMultiplier = 0x015a4e35UL;
	static constexpr uint32_t ulIncrement = 1UL;

	ulNextRand = (ulMultiplier * ulNextRand) + ulIncrement;
	return (int(ulNextRand >> 16UL) & 0x7fffUL);
}

BaseType_t
xApplicationGetRandomNumber(uint32_t *pulNumber)
{
	*pulNumber = uxRand();
	return pdTRUE;
}

uint32_t
ulApplicationGetNextSequenceNumber(uint32_t, uint16_t, uint32_t, uint16_t)
{
	return uxRand();
}
