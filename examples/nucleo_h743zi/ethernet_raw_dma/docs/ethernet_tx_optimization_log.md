# Ethernet TX Optimization Log

## Baseline / Experiment 1

Goal: add diagnostics before changing TX performance behavior.

Modified files:

- `ethernet_dma.hpp`
- `ethernet_dma.cpp`
- `ethernet_dma_diag.h`
- `udp_speedtest.c`
- `tools/udp_speedtest.py`

Configuration:

- Ethernet/lwIP memory: D2 SRAM2, `0x30020000..0x30040000`
- RX descriptors: 32
- TX descriptors: 32
- UDP speed-test TX budget: 4 packets per `udp_speedtest_poll()`
- TX zero-copy: enabled
- Copy fallback: enabled

Diagnostics added:

- Ethernet TX zero-copy/copy frame and byte counters
- descriptor starvation and ring-full counters
- reclaim call, completed descriptor, completed frame counters
- pbuf reference acquired/released counters
- current/max descriptors in use
- minimum free descriptors observed
- max pbuf chain length and descriptors per frame
- UDP speed-test `udp_sendto()` calls/errors
- UDP speed-test blocked events, poll calls, max packets per poll

Results:

| Test | Result |
| --- | --- |
| Build | pass, `scons -C examples/nucleo_h743zi/ethernet_raw_dma -j32` |
| Flash | pass, `scons -C examples/nucleo_h743zi/ethernet_raw_dma program` |
| Ping | pass, 0% loss, ~0.6 ms average |
| UDP echo | pass |
| TCP echo | pass |
| UDP RX speed | 93.982 Mbit/s board-side, 0 missing packets |
| UDP TX speed | 59.028 Mbit/s board-side, 59.588 Mbit/s PC-side, 0 missing packets |

TX diagnostics from 10,000 packets of 1472-byte UDP payload:

| Counter | Value |
| --- | ---: |
| udp_sendto calls | 10000 |
| udp_sendto errors | 0 |
| tx blocked events | 0 |
| tx poll calls | 2501 |
| tx max packets/poll | 4 |
| eth zero-copy frames | 10016 |
| eth zero-copy bytes | 15146243 |
| eth copy frames | 0 |
| eth copy bytes | 0 |
| eth descriptor starvation | 0 |
| eth ring full | 0 |
| eth reclaim calls | 655667 |
| eth completed descriptors | 10015 |
| eth completed frames | 10015 |
| eth pbuf refs acquired | 10016 |
| eth pbuf refs released | 10015 |
| eth descriptors in use at TX_DONE | 1 |
| eth max descriptors in use | 1 |
| eth min free descriptors | 32 |
| eth max pbuf chain length | 1 |
| eth max descriptors/frame | 1 |
| eth tx errors | 0 |
| eth tx busy | 0 |

Conclusion:

- Zero-copy TX is active.
- Copy fallback is not being used.
- Descriptor starvation is not occurring.
- Each UDP TX packet uses one pbuf segment and one TX descriptor.
- The 32-descriptor ring is not filling; max observed descriptor use is 1.
- Current throughput is limited above descriptor availability, most likely by software pacing, checksum generation, packet fill, pbuf allocation/free, or `network::poll()`/`udp_speedtest_poll()` scheduling.

Next action: Experiment 2/3 should focus on TX poll budget and pacing, since the fixed budget of 4 packets per poll produced 2501 poll calls for 10000 packets.

## Experiment 2: Runtime TX Poll Budget Sweep

Goal: determine whether the fixed 4-packet TX budget in `udp_speedtest_poll()` is limiting board-to-PC throughput.

Modified files:

- `udp_speedtest.h`
- `udp_speedtest.c`
- `tools/udp_speedtest.py`

Configuration:

- Ethernet/lwIP memory: D2 SRAM2, `0x30020000..0x30040000`
- RX descriptors: 32
- TX descriptors: 32
- UDP TX payload size: 1472 bytes
- UDP TX packet count: 10000
- TX zero-copy: enabled
- Copy fallback: enabled
- Runtime TX budget command field added to the UDP speed-test TX start packet
- Legacy TX start and TX done packet formats remain accepted by the host tool

Validation before sweep:

| Test | Result |
| --- | --- |
| Build | pass, `scons -C examples/nucleo_h743zi/ethernet_raw_dma -j32` |
| Flash | pass, `scons -C examples/nucleo_h743zi/ethernet_raw_dma program` |
| Ping | pass, 0% loss, ~0.6 ms average |
| UDP echo | pass |
| TCP echo | pass |

Budget sweep results:

| TX budget | PC throughput | Board throughput | Missing packets | UDP send errors | Descriptor starvation | Ring full | TX poll calls | Max packets/poll | Max descriptors in use | Min free descriptors |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 4 | 59.831 Mbit/s | 59.206 Mbit/s | 0 | 0 | 0 | 0 | 2501 | 4 | 1 | 32 |
| 8 | 61.928 Mbit/s | 61.301 Mbit/s | 0 | 0 | 0 | 0 | 1251 | 8 | 2 | 31 |
| 16 | 63.026 Mbit/s | 62.406 Mbit/s | 0 | 0 | 0 | 0 | 626 | 16 | 2 | 31 |
| 32 | 63.576 Mbit/s | 62.973 Mbit/s | 0 | 0 | 0 | 0 | 313 | 32 | 2 | 31 |
| 64 | 63.834 Mbit/s | 63.278 Mbit/s | 0 | 0 | 0 | 0 | 157 | 64 | 2 | 31 |
| 128 | 63.988 Mbit/s | 63.414 Mbit/s | 0 | 0 | 0 | 0 | 79 | 128 | 2 | 31 |

RX regression check after the TX sweep:

| Test | Result |
| --- | --- |
| UDP RX speed | 94.283 Mbit/s board-side, 95.594 Mbit/s PC-send-side, 0 missing packets |

Conclusion:

- Increasing the TX budget improves throughput from about 59 Mbit/s to about 63.4 Mbit/s.
- Throughput saturates quickly after a budget of 32 to 64 packets per poll.
- There were no missing packets, UDP send errors, descriptor starvation events, or TX ring-full events in the sweep.
- The TX descriptor ring is not the bottleneck in this test. Even with a 128-packet software budget, the maximum observed descriptor use was 2 out of 32.
- A budget of 64 is a practical default candidate for the current speed-test workload; it is near the measured maximum without relying on an unnecessarily large burst.
- Remaining TX throughput loss is likely above descriptor availability: checksum work, packet fill, pbuf allocation/free, lwIP UDP path cost, or application poll scheduling.

## Experiment 3A: TX timing attribution, software checksum baseline

Goal: add TX timing attribution without intentionally changing TX behavior.

Configuration:

- TX descriptors: 32
- TX zero-copy: enabled
- TX budget: 64
- UDP payload size: 1472
- UDP packet count: 10000
- Checksum mode: software TX checksum generation
- Timing source: Cortex-M DWT cycle counter at 400 MHz

Validation:

| Test | Result |
| --- | --- |
| Build | pass, `scons -C examples/nucleo_h743zi/ethernet_raw_dma -j32` |
| Flash | pass, `scons -C examples/nucleo_h743zi/ethernet_raw_dma program` |
| Ping | pass, 0% loss |
| UDP echo | pass |
| TCP echo | pass when run without overlapping RX speed traffic |
| UDP RX speed | 91.216 Mbit/s board-side, 0 missing packets |

Results:

| Metric | Value |
| --- | ---: |
| Board TX throughput | 53.625 Mbit/s |
| PC RX throughput | 54.226 Mbit/s |
| Missing packets | 0 |
| UDP send errors | 0 |
| Descriptor starvation | 0 |
| Ring full | 0 |
| Max descriptors in use | 2 |
| pbuf alloc avg cycles/us | 956.1 / 2.390 |
| payload fill avg cycles/us | 40987.6 / 102.469 |
| udp_sendto avg cycles/us | 39441.9 / 98.605 |
| pbuf free avg cycles/us | 160.0 / 0.400 |
| low_level_output avg cycles/us | 6665.6 / 16.664 |
| descriptor setup avg cycles/us | 1251.4 / 3.128 |
| cache clean avg cycles/us | 124.0 / 0.310 |
| reclaim avg cycles/us | 2166.2 / 5.415 |
| send_one_packet avg cycles/us | 86961.6 / 217.404 |
| poll avg cycles/us | 5576691.5 / 13941.729 |

Conclusion:

- The largest measured software-checksum TX costs are payload fill and `udp_sendto()`.
- `udp_sendto()` scales with payload size in the software-checksum build, consistent with UDP checksum generation being a major cost.
- Timing instrumentation is useful for attribution but is not performance-neutral: the 1472-byte timing-enabled baseline is lower than the earlier non-timed Experiment 2 result.

## Experiment 3B/3C: TX checksum offload

Goal: enable STM32H743 hardware TX checksum insertion under a compile-time switch and compare it against the software-checksum baseline.

Implementation:

- Added `ETH_TX_CHECKSUM_OFFLOAD_ENABLE` in `ethernet_config.h`.
- When enabled, `lwipopts.h` sets `CHECKSUM_GEN_IP`, `CHECKSUM_GEN_UDP`, `CHECKSUM_GEN_TCP`, and `CHECKSUM_GEN_ICMP` to `0`.
- RX checksum checking remains software-enabled.
- TX descriptors set full checksum insertion with `DESC3` bits 17 and 16 on the first descriptor of each transmitted frame.
- The final checked-in default is `ETH_TX_CHECKSUM_OFFLOAD_ENABLE = 1`.

Correctness:

| Test | Result |
| --- | --- |
| Build | pass |
| Flash | pass |
| Ping | pass, 0% loss |
| UDP echo | pass |
| TCP echo | pass |
| UDP RX speed | 93.535 Mbit/s board-side, 94.645 Mbit/s PC-send-side, 0 missing packets |
| UDP/TCP checksum capture | not captured; `tcpdump` is installed but capture permission is denied for `enp0s31f6` |

Software checksum baseline:

| Payload size | Board Mbit/s | PC Mbit/s | Packet rate | Missing | udp_sendto avg | fill avg | cache clean avg | low_level_output avg |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 64 | 12.518 | 12.623 | 19559/s | 0 | 28.193 us | 3.906 us | 0.310 us | 16.638 us |
| 256 | 31.267 | 31.579 | 15267/s | 0 | 37.778 us | 17.347 us | 0.310 us | 16.640 us |
| 512 | 41.796 | 42.239 | 10204/s | 0 | 50.578 us | 35.267 us | 0.310 us | 16.639 us |
| 1024 | 50.319 | 50.805 | 6143/s | 0 | 76.204 us | 71.108 us | 0.310 us | 16.664 us |
| 1472 | 53.625 | 54.166 | 4554/s | 0 | 98.605 us | 102.469 us | 0.310 us | 16.664 us |

Hardware TX checksum offload:

| Payload size | Board Mbit/s | PC Mbit/s | Packet rate | Missing | udp_sendto avg | fill avg | cache clean avg | low_level_output avg |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 64 | 13.838 | 13.961 | 21622/s | 0 | 23.960 us | 4.270 us | 0.345 us | 17.583 us |
| 256 | 37.996 | 38.356 | 18738/s | 0 | 23.952 us | 19.631 us | 0.325 us | 17.586 us |
| 512 | 53.824 | 54.349 | 10508/s | 0 | 23.952 us | 40.112 us | 0.325 us | 17.586 us |
| 1024 | 67.983 | 68.659 | 8299/s | 0 | 23.979 us | 81.075 us | 0.325 us | 17.613 us |
| 1472 | 73.877 | 74.581 | 6274/s | 0 | 23.978 us | 116.916 us | 0.325 us | 17.613 us |

Conclusion:

- Hardware TX checksum offload is a major improvement.
- At 1472-byte UDP payloads, timing-enabled board TX throughput improved from 53.625 Mbit/s to 73.877 Mbit/s.
- `udp_sendto()` dropped from 98.605 us to about 23.978 us at 1472 bytes and became nearly payload-size independent.
- Payload fill is now the largest remaining per-byte TX cost.
- Descriptor availability is still not limiting: no descriptor starvation, no ring-full events, and maximum descriptor use stayed at 2.
- Keep checksum offload enabled by default. The next TX experiment should focus on payload fill/copy strategy and pbuf allocation/free overhead.

## Experiment 4: UDP TX payload fill modes

### Motivation

Experiment 3 showed that hardware checksum offload reduced `udp_sendto()` cost from payload-size-dependent ~98 us at 1472 bytes to payload-size-independent ~24 us. Payload fill is now the largest measured cost.

### Configuration

- TX descriptors: 32
- TX zero-copy: enabled
- Hardware TX checksum offload: enabled
- TX budget: 64
- UDP packet count: 10000
- Timing source: DWT cycle counter at 400 MHz
- Fill modes: `full` and `header-only`

Implementation notes:

- `full` mode preserves deterministic payload generation.
- `header-only` mode only writes the speed-test header into each allocated pbuf; body bytes are intentionally diagnostic-only.
- The host validates packet header, sequence, length, count, and ordering. It does not validate body pattern in TX mode.
- Payload fill timing now includes the body write/copy work for `full` mode and the header write for `header-only` mode.
- Allocation misses before a pbuf exists are counted as blocked/pacing events, not send errors.

Validation:

| Test | Result |
| --- | --- |
| Build | pass, `scons -C examples/nucleo_h743zi/ethernet_raw_dma -j32` |
| Flash | pass, `scons -C examples/nucleo_h743zi/ethernet_raw_dma program` |
| Ping | pass, 0% loss |
| UDP echo | pass |
| TCP echo | pass |

### Results: full-fill mode

| Payload size | Board Mbit/s | PC Mbit/s | Packet rate | Missing | Send errors | Fill avg us | pbuf alloc avg us | udp_sendto avg us | low_level_output avg us | send_one_packet avg us | Max desc in use |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 64 | 13.763 | 13.900 | 21505/s | 0 | 0 | 5.717 | 2.165 | 23.983 | 17.607 | 35.107 | 1 |
| 256 | 39.234 | 39.623 | 19157/s | 0 | 0 | 20.437 | 2.432 | 23.988 | 17.610 | 50.099 | 1 |
| 512 | 56.968 | 57.564 | 13908/s | 0 | 0 | 40.080 | 2.432 | 23.988 | 17.611 | 69.742 | 1 |
| 1024 | 73.735 | 74.379 | 9001/s | 0 | 0 | 79.360 | 2.433 | 24.015 | 17.638 | 109.050 | 2 |
| 1472 | 80.935 | 81.645 | 6873/s | 0 | 0 | 113.732 | 2.433 | 24.015 | 17.638 | 143.422 | 2 |

### Results: header-only mode

| Payload size | Board Mbit/s | PC Mbit/s | Packet rate | Missing | Send errors | Blocked events | Fill avg us | pbuf alloc avg us | udp_sendto avg us | low_level_output avg us | send_one_packet avg us | Max desc in use |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 64 | 15.422 | 15.526 | 30120/s | 0 | 0 | 0 | 1.827 | 2.165 | 23.968 | 17.608 | 31.202 | 2 |
| 256 | 60.413 | 60.896 | 29499/s | 0 | 0 | 0 | 1.827 | 2.497 | 24.290 | 17.924 | 31.856 | 2 |
| 512 | 85.870 | 86.651 | 20964/s | 0 | 0 | 2222 | 1.817 | 3.412 | 24.464 | 18.076 | 27.671 | 16 |
| 1024 | 91.327 | 92.024 | 11148/s | 0 | 0 | 10013 | 1.776 | 2.833 | 23.149 | 16.786 | 17.194 | 16 |
| 1472 | 92.944 | 93.815 | 7893/s | 0 | 0 | 16845 | 1.771 | 2.633 | 23.059 | 16.697 | 13.453 | 16 |

### RX regression

| Test | Result |
| --- | --- |
| UDP RX speed | 93.683 Mbit/s board-side, 94.965 Mbit/s PC-send-side, 0 missing packets |

### Conclusion

- Payload fill/copy is confirmed as the dominant remaining bottleneck for large UDP TX payloads.
- At 1472 bytes, `header-only` improved board TX throughput from 80.935 Mbit/s to 92.944 Mbit/s.
- Payload fill timing dropped from 113.732 us to 1.771 us at 1472 bytes.
- `udp_sendto()` stayed near 23-24 us, confirming checksum offload remains effective and payload-size independent.
- Header-only mode reaches near the practical 100BASE-TX UDP payload limit with zero missing packets and zero send errors.
- At higher packet rates the app sees pbuf allocation backpressure and up to 16 descriptors in use, but still no descriptor starvation or ring-full events.
- Next experiment should use preinitialized/preallocated TX buffers or custom pbufs so full deterministic payloads can approach header-only throughput without rewriting/copying the full body each packet.
