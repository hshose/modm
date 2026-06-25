#pragma once

#include "packet.hpp"

#include "lwip/ip_addr.h"

#include <cstddef>
#include <cstdint>

struct udp_pcb;

namespace modm::lwip
{

class UdpEndpoint
{
public:
	UdpEndpoint() = default;

	UdpEndpoint(const ip_addr_t &address, uint16_t port) :
		address_(address), port_(port)
	{
	}

	const ip_addr_t &address() const
	{
		return address_;
	}

	uint16_t port() const
	{
		return port_;
	}

private:
	ip_addr_t address_ {};
	uint16_t port_ = 0;
};

class UdpSocket
{
public:
	using ReceiveCallback = void (*)(UdpSocket &socket,
			const UdpEndpoint &remote,
			PacketView packet,
			void *userData);

	UdpSocket() = default;
	~UdpSocket();

	UdpSocket(const UdpSocket &) = delete;
	UdpSocket &operator=(const UdpSocket &) = delete;

	bool bind(uint16_t port);
	void close();

	bool isBound() const;

	void onReceive(ReceiveCallback callback, void *userData = nullptr);

	bool sendTo(const UdpEndpoint &endpoint, const void *data, std::size_t length);
	bool sendTo(const UdpEndpoint &endpoint, PacketBuffer &&packet);

	uint32_t rxCount() const;
	uint32_t txCount() const;
	uint32_t errorCount() const;

private:
	static void receiveTrampoline(void *arg,
			struct udp_pcb *pcb,
			struct pbuf *p,
			const ip_addr_t *addr,
			u16_t port);

	struct udp_pcb *pcb_ = nullptr;
	ReceiveCallback callback_ = nullptr;
	void *userData_ = nullptr;

	uint32_t rxCount_ = 0;
	uint32_t txCount_ = 0;
	uint32_t errorCount_ = 0;
};

}
