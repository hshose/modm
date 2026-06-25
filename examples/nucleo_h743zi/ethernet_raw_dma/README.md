# STM32H743 raw Ethernet DMA + lwIP ping/UDP echo

This example runs lwIP in bare-metal `NO_SYS=1` mode on top of the existing
STM32 Ethernet DMA descriptor driver.

Default board IPv4 configuration is in `network.hpp`:

- IP: `192.168.1.50`
- Netmask: `255.255.255.0`
- Gateway: `192.168.1.1`

To test from a directly connected PC, configure the PC Ethernet interface as:

- IP: `192.168.1.10`
- Netmask: `255.255.255.0`

Then run:

```sh
ping 192.168.1.50
```

Expected traffic in Wireshark is ARP resolution followed by ICMP echo
request/reply.

The board also runs a UDP echo server on port `5005`. A quick `netcat` test is:

```sh
echo -n "hello stm32" | nc -u -w1 192.168.1.50 5005
```

Some `nc` implementations are easier to test interactively:

```sh
nc -u 192.168.1.50 5005
```

Then type `hello stm32`; the same text should be echoed back.

This Python test is more reliable for automated UDP echo checks:

```python
import socket

BOARD_IP = "192.168.1.50"
BOARD_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(1.0)

payload = b"hello stm32 udp echo"
sock.sendto(payload, (BOARD_IP, BOARD_PORT))

data, addr = sock.recvfrom(2048)

print("received:", data)
print("from:", addr)

assert data == payload
print("UDP echo test passed")
```

Wireshark should show ARP if needed, a UDP packet from the PC to the board, and
a UDP packet from the board back to the PC source address and port. DHCP, TCP,
DNS, sockets, and netconn are intentionally disabled.

The UDP speed test server listens on port `5006`. Its binary protocol uses
little-endian fields and a 20-byte header:

```c
uint32_t magic;        // 0x53504454
uint16_t version;      // 1
uint16_t type;
uint32_t seq;
uint32_t payload_len;  // full UDP payload size, including this header
uint32_t timestamp_us;
```

Run a PC-to-board RX throughput test:

```sh
python3 tools/udp_speedtest.py rx --ip 192.168.1.50 --port 5006 --size 1472 --count 10000
```

Run a board-to-PC TX throughput test:

```sh
python3 tools/udp_speedtest.py tx --ip 192.168.1.50 --port 5006 --size 1472 --count 10000
```

The default `--size 1472` is the maximum UDP payload that fits in a standard
1500-byte Ethernet MTU without IPv4 fragmentation. The board uses millisecond
system time converted to microseconds for benchmark timestamps, so short tests
have reduced timing precision.

The TCP echo server listens on port `5007`. Manual test with netcat:

```sh
nc 192.168.1.50 5007
```

Then type `hello stm32 tcp`; the board should echo the same bytes. TCP is a byte
stream, so terminal buffering and line endings can affect what appears on
screen.

Automated TCP echo test:

```sh
python3 tools/tcp_echo_test.py --ip 192.168.1.50 --port 5007
```

The script opens one TCP connection, sends small, 1 kB, and 4 kB payloads, and
verifies that every echoed byte matches.

The lwIP usable heap is 10 kB (`MEM_SIZE`) and starts at linker symbol
`__lwip_heap_start`, directly after the TX DMA buffer area in D2 SRAM3. The
linker section reserves that heap plus lwIP allocator metadata. The MPU region
covers the whole 32 kB Ethernet/lwIP window at `0x30040000`.
