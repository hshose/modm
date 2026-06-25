#!/usr/bin/env python3

import argparse
import socket
import time


def receive_exact(sock, length):
    data = bytearray()
    while len(data) < length:
        chunk = sock.recv(length - len(data))
        if not chunk:
            raise ConnectionError("connection closed before full echo was received")
        data.extend(chunk)
    return bytes(data)


def run_test(ip, port, timeout):
    payloads = [
        b"hello stm32 tcp",
        bytes((index & 0xFF) for index in range(1024)),
        bytes(((index * 7) & 0xFF) for index in range(4096)),
    ]

    with socket.create_connection((ip, port), timeout=timeout) as sock:
        sock.settimeout(timeout)

        for index, payload in enumerate(payloads, start=1):
            start = time.perf_counter()
            sock.sendall(payload)
            echoed = receive_exact(sock, len(payload))
            elapsed_ms = (time.perf_counter() - start) * 1000.0

            if echoed != payload:
                raise AssertionError(
                    f"payload {index} mismatch: sent {len(payload)} bytes, "
                    f"received {len(echoed)} bytes"
                )

            print(f"payload {index}: {len(payload)} bytes echoed in {elapsed_ms:.2f} ms")

    print("TCP echo test passed")


def main():
    parser = argparse.ArgumentParser(description="STM32 TCP echo test")
    parser.add_argument("--ip", default="192.168.1.50")
    parser.add_argument("--port", type=int, default=5007)
    parser.add_argument("--timeout", type=float, default=2.0)
    args = parser.parse_args()

    run_test(args.ip, args.port, args.timeout)


if __name__ == "__main__":
    main()
