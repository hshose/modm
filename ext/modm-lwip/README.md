# modm lwIP Bridge

This module integrates lwIP with modm and provides an STM32 Ethernet backend.

Applications using Ethernet normally include only:

```cpp
#include <modm_lwip.hpp>
```

Minimal static IPv4 setup:

```cpp
Board::initialize();

modm::lwip::Ethernet::initialize({
	{ 192, 168, 1, 50 },
	{ 255, 255, 255, 0 },
	{ 192, 168, 1, 1 },
});

while (true) {
	modm::lwip::Ethernet::poll();
}
```

`modm::lwip::Ethernet::initialize()` owns the Ethernet/lwIP bring-up sequence:
Ethernet memory setup, backend initialization, lwIP initialization, static IPv4
configuration, MAC address configuration, netif creation, default netif
selection, netif up state, and initial link state.

`modm::lwip::Ethernet::poll()` owns regular no-OS processing: completed TX
descriptor reclaim, RX frame input into lwIP, and lwIP timeout handling.

`modm::lwip::Ethernet::pollStatus()` owns periodic backend and PHY/link status
polling and updates the lwIP netif link state.

Application protocols remain explicit. The UDP speed-test example calls
`udp_speedtest_init()` after Ethernet initialization and calls
`udp_speedtest_poll()` from the main loop. Global lwIP application hooks are not
required.

Advanced diagnostics and backend-specific functions remain available through
`modm_lwip_ethernet.hpp` and the `modm::lwip::ethernet` namespace.

## Raw API transport wrappers

The module also provides thin no-OS wrappers over lwIP's raw UDP and TCP APIs:

```cpp
#include <modm_lwip/packet.hpp>
#include <modm_lwip/udp_socket.hpp>
#include <modm_lwip/tcp_connection.hpp>
#include <modm_lwip/tcp_server.hpp>
```

`<modm_lwip.hpp>` includes these commonly used headers as well.

UDP remains datagram-oriented:

```cpp
static modm::lwip::UdpSocket socket;

static void
onReceive(modm::lwip::UdpSocket& socket,
          const modm::lwip::UdpEndpoint& remote,
          modm::lwip::PacketView packet,
          void*)
{
    socket.sendTo(remote, packet.data(), packet.size());
}

socket.onReceive(onReceive);
socket.bind(5005);
```

TCP remains byte-stream and connection-oriented:

```cpp
static modm::lwip::TcpConnection connections[2];
static modm::lwip::TcpServer server{connections, 2};

static void
onReceive(modm::lwip::TcpConnection& connection,
          modm::lwip::PacketView data,
          void*)
{
    connection.write(data);
    connection.output();
}

static void
onAccept(modm::lwip::TcpConnection& connection, void*)
{
    connection.onReceive(onReceive);
}

server.onAccept(onAccept);
server.listen(5007);
```

The wrappers hide raw lwIP callback trampolines, PCB setup, endpoint conversion,
and common pbuf ownership rules. Callbacks are non-blocking and run from the
regular `Ethernet::poll()` path. `PacketView` is a non-owning receive view and
is only valid during the callback. `PacketBuffer` owns an outgoing pbuf until it
is passed to `UdpSocket::sendTo()` or destroyed. TCP writes use
`TCP_WRITE_FLAG_COPY`; UDP `sendTo(const void*, size)` copies into a temporary
pbuf, while `PacketBuffer` allows applications to fill the outgoing payload
directly.

The wrapper classes do not dynamically allocate their own state. lwIP still
allocates PCBs and pbufs internally. The wrappers do not use sockets, netconn,
RTOS primitives, exceptions, or blocking calls.

The STM32 Ethernet backend always uses hardware TX checksum insertion. lwIP TX
checksum generation for IPv4, UDP, TCP, and ICMP is disabled in the generated
`lwipopts.h`; RX checksum checking remains software-enabled unless a future
backend implements RX hardware validation.

Supported hardware must provide working TX checksum insertion. Invalid
board-originated checksums usually indicate a descriptor checksum insertion
configuration issue, not a software checksum fallback configuration issue. This
is not configurable in the current backend.

When porting the backend to another STM32 variant, validate board-originated UDP
and TCP checksums with packet capture in addition to ping, UDP, and TCP
functional tests.
