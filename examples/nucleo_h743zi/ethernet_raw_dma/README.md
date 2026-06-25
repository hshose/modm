# STM32H743 raw Ethernet DMA + lwIP ping/UDP/TCP echo

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
a UDP packet from the board back to the PC source address and port. DHCP, DNS,
sockets, and netconn are intentionally disabled.

The UDP speed test app lives in `../lwip_speedtest`.

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

## Ethernet pbuf ownership

The lwIP Ethernet bridge in `../../../ext/modm-lwip` is pbuf-only. TX maps each
non-empty segment of the lwIP pbuf chain directly to one STM32 Ethernet TX
descriptor. The driver first verifies that the full frame fits in the available
descriptor ring, then calls `pbuf_ref()` once for the whole frame before handing
descriptors to DMA. The driver stores that referenced pbuf on the last descriptor
of the frame. `modm::lwip::ethernet::reclaimTxDescriptors()` scans completed TX
descriptors from `modm::lwip::poll()` and releases the driver-held reference with
`pbuf_free()` when the last descriptor completes.

RX hands DMA buffers to lwIP as custom pbufs. Each RX descriptor has a matching
custom pbuf context, and the descriptor remains owned by lwIP until the pbuf free
callback returns it to DMA. The bridge exposes diagnostics counters for descriptor
starvation, custom pbuf allocation failures, and input errors.

The STM32 lwIP backend always uses hardware TX checksum insertion. lwIP TX
checksum generation for IPv4, UDP, TCP, and ICMP is disabled; RX checksum
checking remains software-enabled.

The lwIP heap starts at `__lwip_heap_start` inside the 128 kB Ethernet/lwIP MPU
window in D2 SRAM2 at `0x30020000`. That region is configured non-cacheable for DMA, so pbuf
payloads allocated by lwIP do not need D-cache cleaning. The TX path still
contains a cache-clean helper for payload pointers outside that MPU window, used
only if D-cache is enabled.

The lwIP usable heap is 10 kB (`MEM_SIZE`) and starts at linker symbol
`__lwip_heap_start`, directly after the RX DMA buffer area in D2 SRAM2. The
linker section reserves that heap plus lwIP allocator metadata. The MPU region
covers all 128 kB of D2 SRAM2 at `0x30020000`. The Ethernet ring uses
32 RX descriptors and 32 TX descriptors.
