#!/usr/bin/env python3

import argparse
import csv
import socket
import struct
import time

MAGIC = 0x45535452
VERSION = 1
CONTROL_PORT = 5010

CMD_START = 1
CMD_STOP = 2
CMD_STATUS = 3
DATA = 10
STATUS = 11

HEADER = struct.Struct("<IHHII")
START = struct.Struct("<IHHIIIII")
STATUS_PACKET = struct.Struct("<IHHIIIIIIIIIII")
DATA_PREFIX = struct.Struct("<IHHIIIIII")
SAMPLE = struct.Struct("<IIII")


def make_header(packet_type, sequence, payload_len):
	return HEADER.pack(MAGIC, VERSION, packet_type, sequence, payload_len)


def make_start(sequence, decimation, max_samples, flags):
	return START.pack(
		MAGIC,
		VERSION,
		CMD_START,
		sequence,
		START.size,
		decimation,
		max_samples,
		flags,
	)


def send_command(sock, ip, port, packet_type, sequence=0):
	packet = make_header(packet_type, sequence, HEADER.size)
	sock.sendto(packet, (ip, port))


def recv_status(sock, timeout=2.0):
	sock.settimeout(timeout)
	while True:
		data, remote = sock.recvfrom(2048)
		if len(data) < HEADER.size:
			continue
		magic, version, packet_type, sequence, payload_len = HEADER.unpack_from(data)
		if magic != MAGIC or version != VERSION or payload_len != len(data):
			continue
		if packet_type != STATUS or len(data) < STATUS_PACKET.size:
			continue
		values = STATUS_PACKET.unpack_from(data)
		return {
			"remote": remote,
			"sequence": sequence,
			"streaming_enabled": values[5],
			"samples_produced": values[6],
			"samples_sent": values[7],
			"ring_overruns": values[8],
			"ring_fill_level": values[9],
			"ring_max_fill_level": values[10],
			"udp_packets_sent": values[11],
			"udp_send_errors": values[12],
			"last_encoder_position": values[13],
		}


def print_status(status):
	for key in (
		"streaming_enabled",
		"samples_produced",
		"samples_sent",
		"ring_overruns",
		"ring_fill_level",
		"ring_max_fill_level",
		"udp_packets_sent",
		"udp_send_errors",
		"last_encoder_position",
	):
		print(f"{key}: {status[key]}")


def command_start(args):
	with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
		sock.sendto(make_start(0, args.decimation, args.max_samples, 0), (args.ip, args.port))
		print_status(recv_status(sock))


def command_stop(args):
	with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
		send_command(sock, args.ip, args.port, CMD_STOP)
		print_status(recv_status(sock))


def command_status(args):
	with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
		send_command(sock, args.ip, args.port, CMD_STATUS)
		print_status(recv_status(sock))


def parse_samples(data):
	if len(data) < DATA_PREFIX.size:
		return None, []
	values = DATA_PREFIX.unpack_from(data)
	magic, version, packet_type, packet_sequence, payload_len = values[:5]
	if magic != MAGIC or version != VERSION or packet_type != DATA or payload_len != len(data):
		return None, []
	first_sequence, sample_count, stream_drops, ring_overruns = values[5:9]
	expected_len = DATA_PREFIX.size + sample_count * SAMPLE.size
	if expected_len != len(data):
		return None, []
	samples = []
	offset = DATA_PREFIX.size
	for _ in range(sample_count):
		samples.append(SAMPLE.unpack_from(data, offset))
		offset += SAMPLE.size
	return {
		"packet_sequence": packet_sequence,
		"first_sequence": first_sequence,
		"sample_count": sample_count,
		"stream_drops": stream_drops,
		"ring_overruns": ring_overruns,
	}, samples


def command_record(args):
	rows = []
	packets = 0
	samples_received = 0
	missing_samples = 0
	expected_sample_sequence = None
	max_ring_overruns = 0
	max_stream_drops = 0

	with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
		sock.bind(("", 0))
		sock.sendto(make_start(0, args.decimation, args.max_samples, 0), (args.ip, args.port))
		print("START status:")
		print_status(recv_status(sock))

		start_time = time.monotonic()
		end_time = start_time + args.duration
		sock.settimeout(0.25)
		try:
			while time.monotonic() < end_time:
				try:
					data, _ = sock.recvfrom(2048)
				except socket.timeout:
					continue

				host_time = time.time()
				info, packet_samples = parse_samples(data)
				if info is None:
					continue

				packets += 1
				max_ring_overruns = max(max_ring_overruns, info["ring_overruns"])
				max_stream_drops = max(max_stream_drops, info["stream_drops"])
				for sequence, timestamp_us, position_raw, status in packet_samples:
					if expected_sample_sequence is not None and sequence != expected_sample_sequence:
						if sequence > expected_sample_sequence:
							missing_samples += sequence - expected_sample_sequence
					expected_sample_sequence = sequence + 1
					samples_received += 1
					if args.csv:
						rows.append((host_time, sequence, timestamp_us, position_raw, status))
		finally:
			send_command(sock, args.ip, args.port, CMD_STOP)
			try:
				print("STOP status:")
				print_status(recv_status(sock))
			except socket.timeout:
				print("STOP status: no response")

	elapsed = max(time.monotonic() - start_time, 1e-9)
	print(f"packets_received: {packets}")
	print(f"samples_received: {samples_received}")
	print(f"sample_rate: {samples_received / elapsed:.1f}")
	print(f"missing_sample_sequences: {missing_samples}")
	print(f"board_ring_overruns: {max_ring_overruns}")
	print(f"stream_drop_count: {max_stream_drops}")

	if args.csv:
		with open(args.csv, "w", newline="") as output:
			writer = csv.writer(output)
			writer.writerow(
				["host_receive_time", "sample_sequence", "timestamp_us", "position_raw", "status"]
			)
			writer.writerows(rows)


def main():
	parser = argparse.ArgumentParser()
	subparsers = parser.add_subparsers(dest="command", required=True)

	for name, handler in (
		("start", command_start),
		("stop", command_stop),
		("status", command_status),
	):
		sub = subparsers.add_parser(name)
		sub.add_argument("--ip", required=True)
		sub.add_argument("--port", type=int, default=CONTROL_PORT)
		if name == "start":
			sub.add_argument("--decimation", type=int, default=1)
			sub.add_argument("--max-samples", type=int, default=64)
		sub.set_defaults(func=handler)

	record = subparsers.add_parser("record")
	record.add_argument("--ip", required=True)
	record.add_argument("--port", type=int, default=CONTROL_PORT)
	record.add_argument("--duration", type=float, required=True)
	record.add_argument("--csv")
	record.add_argument("--decimation", type=int, default=1)
	record.add_argument("--max-samples", type=int, default=64)
	record.set_defaults(func=command_record)

	args = parser.parse_args()
	args.func(args)


if __name__ == "__main__":
	main()
