#pragma once

#include <cstdint>

#include <modm_lwip/packet.hpp>
#include <modm_lwip/tcp_connection.hpp>
#include <modm_lwip/tcp_server.hpp>
#include <modm_lwip/udp_socket.hpp>

struct netif;

namespace modm::lwip
{

struct Ipv4Address
{
	uint8_t octets[4];

	constexpr Ipv4Address(uint8_t a, uint8_t b, uint8_t c, uint8_t d) :
		octets{ a, b, c, d }
	{
	}
};

struct MacAddress
{
	uint8_t octets[6];

	constexpr MacAddress(uint8_t a, uint8_t b, uint8_t c,
			uint8_t d, uint8_t e, uint8_t f) :
		octets{ a, b, c, d, e, f }
	{
	}
};

struct EthernetConfig
{
	Ipv4Address address;
	Ipv4Address netmask;
	Ipv4Address gateway;
	MacAddress mac { 0x02, 0x00, 0x00, 0x00, 0x00, 0x01 };
};

class Ethernet
{
public:
	static bool initialize(const EthernetConfig &config);
	static void poll();
	static void pollStatus();

	static bool isInitialized();
	static bool isLinkUp();
	static struct netif *netif();

private:
	Ethernet() = delete;
};

using Config = EthernetConfig;

void initialize(const Config &config);
void poll();

}
