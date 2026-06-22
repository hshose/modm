#!/usr/bin/env python3

import argparse
import random
import select
import socket
import struct
import sys
import time

MAGIC = 0x4D42544D
VERSION = 1
CONTROL_PORT = 5000
DATA_PORT = 5001
MAX_PAYLOAD = 1456

CMD_START_RX = 1
CMD_START_TX = 2
CMD_ACK = 0x8000
CMD_RESULT = 0x8001
CMD_ERROR = 0xFFFF

MESSAGE_STRUCT = struct.Struct("<IHHIHHIIQII")
DATA_HEADER_STRUCT = struct.Struct("<IIIHH")


def build_message(command, run_id, payload_bytes, data_port, duration_ms, packet_count=0,
                  payload_total_bytes=0, sequence_errors=0, reserved=0):
    return MESSAGE_STRUCT.pack(
        MAGIC,
        VERSION,
        command,
        run_id,
        payload_bytes,
        data_port,
        duration_ms,
        packet_count,
        payload_total_bytes,
        sequence_errors,
        reserved,
    )


def parse_message(data):
    if len(data) < MESSAGE_STRUCT.size:
        raise ValueError("short message")

    fields = MESSAGE_STRUCT.unpack_from(data)
    return {
        "magic": fields[0],
        "version": fields[1],
        "command": fields[2],
        "run_id": fields[3],
        "payload_bytes": fields[4],
        "data_port": fields[5],
        "duration_ms": fields[6],
        "packet_count": fields[7],
        "payload_total_bytes": fields[8],
        "sequence_errors": fields[9],
        "reserved": fields[10],
    }


def recv_matching_message(sock, run_id, expected_commands):
    data, address = sock.recvfrom(2048)
    message = parse_message(data)
    if message["magic"] != MAGIC or message["version"] != VERSION:
        return None, None
    if message["run_id"] != run_id:
        return None, None
    if message["command"] not in expected_commands:
        return None, None
    return message, address


def human_mbps(payload_bytes, duration_seconds):
    if duration_seconds <= 0:
        return 0.0
    return (payload_bytes * 8.0) / duration_seconds / 1_000_000.0


def wait_for_message(sock, run_id, expected_commands, timeout):
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("timed out waiting for benchmark reply")
        readable, _, _ = select.select([sock], [], [], remaining)
        if not readable:
            continue

        message, address = recv_matching_message(sock, run_id, expected_commands)
        if message is not None:
            return message, address


def run_upload(args):
    run_id = random.getrandbits(32)
    control = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    control.bind(("", 0))
    control.setblocking(False)

    request = build_message(
        CMD_START_RX,
        run_id,
        args.payload_size,
        args.data_port,
        int(args.duration * 1000),
    )
    control.sendto(request, (args.board_ip, args.control_port))
    wait_for_message(control, run_id, {CMD_ACK}, args.timeout)

    data = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    payload = bytes((index & 0xFF for index in range(args.payload_size)))

    sent_packets = 0
    sent_payload_bytes = 0
    start = time.monotonic()
    deadline = start + args.duration

    while True:
        now = time.monotonic()
        if now >= deadline:
            break

        header = DATA_HEADER_STRUCT.pack(MAGIC, run_id, sent_packets, args.payload_size, 0)
        data.sendto(header + payload, (args.board_ip, args.data_port))
        sent_packets += 1
        sent_payload_bytes += args.payload_size

    elapsed = time.monotonic() - start
    result, _ = wait_for_message(control, run_id, {CMD_RESULT, CMD_ERROR}, args.timeout)
    if result["command"] == CMD_ERROR:
        raise RuntimeError(f"board returned error code {result['reserved']}")

    host_mbps = human_mbps(sent_payload_bytes, elapsed)
    board_mbps = human_mbps(result["payload_total_bytes"], result["duration_ms"] / 1000.0)

    print(f"host->board run_id={run_id}")
    print(f"host sent    : packets={sent_packets} payload={sent_payload_bytes}B time={elapsed:.3f}s throughput={host_mbps:.3f}Mbps")
    print(f"board saw    : packets={result['packet_count']} payload={result['payload_total_bytes']}B time={result['duration_ms'] / 1000.0:.3f}s throughput={board_mbps:.3f}Mbps seqerr={result['sequence_errors']}")
    if sent_packets:
        dropped = sent_packets - result["packet_count"]
        print(f"packet delta : {dropped}")


def run_download(args):
    run_id = random.getrandbits(32)
    control = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    control.bind(("", 0))
    control.setblocking(False)

    data = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    data.bind(("", args.data_port))
    data.setblocking(False)

    request = build_message(
        CMD_START_TX,
        run_id,
        args.payload_size,
        args.data_port,
        int(args.duration * 1000),
    )
    control.sendto(request, (args.board_ip, args.control_port))
    wait_for_message(control, run_id, {CMD_ACK}, args.timeout)

    received_packets = 0
    received_payload_bytes = 0
    sequence_errors = 0
    expected_sequence = 0
    started = False
    start = 0.0
    end = 0.0
    result = None

    deadline = time.monotonic() + args.duration + args.timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break

        readable, _, _ = select.select([control, data], [], [], remaining)
        if not readable:
            continue

        if control in readable:
            message, _ = recv_matching_message(control, run_id, {CMD_RESULT, CMD_ERROR})
            if message is None:
                continue
            if message["command"] == CMD_ERROR:
                raise RuntimeError(f"board returned error code {message['reserved']}")
            result = message
            break

        if data in readable:
            datagram, _ = data.recvfrom(4096)
            if len(datagram) < DATA_HEADER_STRUCT.size:
                continue
            header = DATA_HEADER_STRUCT.unpack_from(datagram)
            if header[0] != MAGIC or header[1] != run_id:
                continue

            if not started:
                started = True
                start = time.monotonic()
                expected_sequence = header[2]

            if header[2] != expected_sequence:
                if header[2] > expected_sequence:
                    sequence_errors += header[2] - expected_sequence
                else:
                    sequence_errors += 1
            expected_sequence = header[2] + 1

            payload_bytes = min(header[3], len(datagram) - DATA_HEADER_STRUCT.size)
            received_packets += 1
            received_payload_bytes += payload_bytes
            end = time.monotonic()

    if result is None:
        raise TimeoutError("timed out waiting for benchmark result")

    elapsed = max(0.0, end - start) if started else 0.0
    host_mbps = human_mbps(received_payload_bytes, elapsed) if started else 0.0
    board_mbps = human_mbps(result["payload_total_bytes"], result["duration_ms"] / 1000.0)

    print(f"board->host run_id={run_id}")
    print(f"host received: packets={received_packets} payload={received_payload_bytes}B time={elapsed:.3f}s throughput={host_mbps:.3f}Mbps seqerr={sequence_errors}")
    print(f"board sent   : packets={result['packet_count']} payload={result['payload_total_bytes']}B time={result['duration_ms'] / 1000.0:.3f}s throughput={board_mbps:.3f}Mbps seqerr={result['sequence_errors']}")
    if result["packet_count"]:
        dropped = result["packet_count"] - received_packets
        print(f"packet delta : {dropped}")


def main():
    parser = argparse.ArgumentParser(description="UDP throughput tool for the nucleo_h723zg ethernet_throughput example")
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--board-ip", default="192.168.10.50")
    common.add_argument("--control-port", type=int, default=CONTROL_PORT)
    common.add_argument("--data-port", type=int, default=DATA_PORT)
    common.add_argument("--payload-size", type=int, default=1400)
    common.add_argument("--duration", type=float, default=5.0)
    common.add_argument("--timeout", type=float, default=3.0)

    subparsers = parser.add_subparsers(dest="mode", required=True)
    subparsers.add_parser("upload", parents=[common], help="send UDP payload from host to board")
    subparsers.add_parser("download", parents=[common], help="receive UDP payload from board")

    args = parser.parse_args()
    if not 1 <= args.payload_size <= MAX_PAYLOAD:
        parser.error(f"payload size must be between 1 and {MAX_PAYLOAD}")

    if args.mode == "upload":
        run_upload(args)
    elif args.mode == "download":
        run_download(args)
    else:
        parser.error("unknown mode")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
