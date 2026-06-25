# STM32H743 lwIP UDP speed test

This example runs the modm lwIP Ethernet bridge on the Nucleo-H743ZI and exposes
only the UDP speed-test server on port `5006`.

The application uses the high-level `modm::lwip::Ethernet` API from
`modm_lwip.hpp`. Ethernet/lwIP bring-up, RX polling, TX descriptor reclaim, lwIP
timeouts, and link-state updates are handled by that API. The UDP speed-test
protocol remains explicit in the example through `udp_speedtest_init()` and
`udp_speedtest_poll()`. The speed-test implementation uses
`modm::lwip::UdpSocket`, `modm::lwip::UdpEndpoint`, `modm::lwip::PacketView`,
and `modm::lwip::PacketBuffer` instead of direct raw lwIP UDP PCB and pbuf
management.

Board IPv4 configuration:

- IP: `192.168.1.50`
- Netmask: `255.255.255.0`
- Gateway: `192.168.1.1`

To test from a directly connected PC, configure the PC Ethernet interface as
`192.168.1.10/24`.

Build and flash:

```sh
scons -C examples/nucleo_h743zi/lwip_speedtest -j32
scons -C examples/nucleo_h743zi/lwip_speedtest program
```

Run a PC-to-board RX throughput test:

```sh
python3 tools/udp_speedtest.py rx --ip 192.168.1.50 --port 5006 --size 1472 --count 10000
```

Run board-to-PC TX throughput tests:

```sh
python3 tools/udp_speedtest.py tx --ip 192.168.1.50 --port 5006 --size 1472 --count 10000 --budget 64 --fill-mode full
python3 tools/udp_speedtest.py tx --ip 192.168.1.50 --port 5006 --size 1472 --count 10000 --budget 64 --fill-mode header-only
```
