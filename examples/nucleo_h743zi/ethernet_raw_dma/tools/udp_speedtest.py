#!/usr/bin/env python3

import argparse
import socket
import struct
import time

MAGIC = 0x53504454
VERSION = 1

RX_DATA = 1
RX_DONE = 2
RX_STATS = 3
TX_START = 4
TX_DATA = 5
TX_DONE = 6
RESET = 7

HEADER = struct.Struct("<IHHIII")
RX_STATS_PACKET = struct.Struct("<IHHIIIIIIIIII")
TX_START_PACKET = struct.Struct("<IHHIIIIII")
TX_DONE_PACKET = struct.Struct("<IHHIIIIIIIII")
HEADER_SIZE = HEADER.size
MAX_UDP_PAYLOAD = 1472


def time_us():
    return time.monotonic_ns() // 1000


def mbps(byte_count, elapsed_s):
    if elapsed_s <= 0:
        return 0.0
    return (byte_count * 8.0) / elapsed_s / 1_000_000.0


def pack_header(packet_type, seq, payload_len):
    return HEADER.pack(MAGIC, VERSION, packet_type, seq, payload_len, time_us() & 0xFFFFFFFF)


def validate_header(data, expected_type=None):
    if len(data) < HEADER_SIZE:
        raise ValueError("packet too short")
    magic, version, packet_type, seq, payload_len, timestamp_us = HEADER.unpack_from(data)
    if magic != MAGIC:
        raise ValueError(f"bad magic 0x{magic:08x}")
    if version != VERSION:
        raise ValueError(f"bad version {version}")
    if payload_len != len(data):
        raise ValueError(f"bad payload_len {payload_len}, actual {len(data)}")
    if expected_type is not None and packet_type != expected_type:
        raise ValueError(f"unexpected type {packet_type}, expected {expected_type}")
    return packet_type, seq, timestamp_us


def make_data_packet(seq, size):
    header = pack_header(RX_DATA, seq, size)
    pattern = bytes(((seq + index) & 0xFF) for index in range(HEADER_SIZE, size))
    return header + pattern


def rx_mode(args):
    if args.size < HEADER_SIZE or args.size > MAX_UDP_PAYLOAD:
        raise SystemExit(f"--size must be between {HEADER_SIZE} and {MAX_UDP_PAYLOAD}")

    addr = (args.ip, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(args.timeout)

    start = time.perf_counter()
    for seq in range(args.count):
        sock.sendto(make_data_packet(seq, args.size), addr)
        if args.delay_us:
            time.sleep(args.delay_us / 1_000_000.0)
    send_elapsed = time.perf_counter() - start

    sock.sendto(pack_header(RX_DONE, args.count, HEADER_SIZE), addr)

    try:
        data, source = sock.recvfrom(2048)
    except socket.timeout:
        raise SystemExit("timed out waiting for RX_STATS")

    validate_header(data, RX_STATS)
    fields = RX_STATS_PACKET.unpack(data)
    rx_packets = fields[6]
    rx_bytes = fields[7]
    missing = fields[8]
    malformed = fields[9]
    out_of_order = fields[10]
    elapsed_us = fields[11]
    board_bps = fields[12]

    print(f"RX stats from {source[0]}:{source[1]}")
    print(f"  packets sent:              {args.count}")
    print(f"  packets received by board: {rx_packets}")
    print(f"  missing packets:           {missing}")
    print(f"  malformed packets:         {malformed}")
    print(f"  out-of-order packets:      {out_of_order}")
    print(f"  bytes received by board:   {rx_bytes}")
    print(f"  board elapsed:             {elapsed_us / 1_000_000.0:.6f} s")
    print(f"  board throughput:          {board_bps * 8.0 / 1_000_000.0:.3f} Mbit/s")
    print(f"  PC send elapsed:           {send_elapsed:.6f} s")
    print(f"  PC send throughput:        {mbps(args.count * args.size, send_elapsed):.3f} Mbit/s")


def tx_mode(args):
    if args.size < HEADER_SIZE or args.size > MAX_UDP_PAYLOAD:
        raise SystemExit(f"--size must be between {HEADER_SIZE} and {MAX_UDP_PAYLOAD}")

    addr = (args.ip, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", 0))
    sock.settimeout(args.timeout)

    command = TX_START_PACKET.pack(
        MAGIC,
        VERSION,
        TX_START,
        0,
        TX_START_PACKET.size,
        time_us() & 0xFFFFFFFF,
        args.count,
        args.size,
        args.delay_us,
    )
    sock.sendto(command, addr)

    received = 0
    received_bytes = 0
    missing = 0
    out_of_order = 0
    expected_seq = 0
    start = None
    end = None
    done = None

    deadline = time.perf_counter() + args.timeout
    while time.perf_counter() < deadline:
        try:
            data, source = sock.recvfrom(2048)
        except socket.timeout:
            break

        packet_type, seq, _ = validate_header(data)
        now = time.perf_counter()

        if packet_type == TX_DATA:
            if start is None:
                start = now
            end = now
            received += 1
            received_bytes += len(data)
            if seq == expected_seq:
                expected_seq += 1
            elif seq > expected_seq:
                missing += seq - expected_seq
                expected_seq = seq + 1
            else:
                out_of_order += 1
            deadline = time.perf_counter() + args.timeout
        elif packet_type == TX_DONE:
            done = TX_DONE_PACKET.unpack(data)
            break

    elapsed = (end - start) if start is not None and end is not None else 0.0
    print("TX receive stats")
    print(f"  packets requested:    {args.count}")
    print(f"  packets received:     {received}")
    print(f"  missing packets:      {missing}")
    print(f"  out-of-order packets: {out_of_order}")
    print(f"  bytes received:       {received_bytes}")
    print(f"  PC elapsed:           {elapsed:.6f} s")
    print(f"  PC throughput:        {mbps(received_bytes, elapsed):.3f} Mbit/s")

    if done is None:
        print("  board TX_DONE:        not received")
        return

    attempted = done[6]
    sent = done[7]
    errors = done[8]
    sent_bytes = done[9]
    elapsed_us = done[10]
    board_bps = done[11]
    print("  board TX_DONE:")
    print(f"    attempted packets:  {attempted}")
    print(f"    sent packets:       {sent}")
    print(f"    send errors:        {errors}")
    print(f"    sent bytes:         {sent_bytes}")
    print(f"    board elapsed:      {elapsed_us / 1_000_000.0:.6f} s")
    print(f"    board throughput:   {board_bps * 8.0 / 1_000_000.0:.3f} Mbit/s")


def reset_mode(args):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(pack_header(RESET, 0, HEADER_SIZE), (args.ip, args.port))


def main():
    parser = argparse.ArgumentParser(description="STM32 UDP speed test client")
    parser.add_argument("mode", choices=("rx", "tx", "reset"))
    parser.add_argument("--ip", default="192.168.1.50")
    parser.add_argument("--port", type=int, default=5006)
    parser.add_argument("--size", type=int, default=1472,
                        help="UDP payload size including the 20-byte speedtest header")
    parser.add_argument("--count", type=int, default=10000)
    parser.add_argument("--delay-us", type=int, default=0)
    parser.add_argument("--timeout", type=float, default=2.0)
    args = parser.parse_args()

    if args.mode == "rx":
        rx_mode(args)
    elif args.mode == "tx":
        tx_mode(args)
    else:
        reset_mode(args)


if __name__ == "__main__":
    main()
