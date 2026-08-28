#!/usr/bin/env python3
"""Poll MSP_RAW_IMU and report serial gaps without changing FC configuration."""

import argparse
import struct
import time

import serial


MSP_RAW_IMU = 102
MSP_PAYLOAD_SIZE = 18
MSP_REQUEST = bytes((0x24, 0x4D, 0x3C, 0, MSP_RAW_IMU, MSP_RAW_IMU))
MSP_RESPONSE_HEADER = bytes((0x24, 0x4D, 0x3E, MSP_PAYLOAD_SIZE, MSP_RAW_IMU))
MSP_RESPONSE_SIZE = len(MSP_RESPONSE_HEADER) + MSP_PAYLOAD_SIZE + 1


def valid_response(response):
    if len(response) != MSP_RESPONSE_SIZE or not response.startswith(MSP_RESPONSE_HEADER):
        return False
    checksum = 0
    for value in response[3:-1]:
        checksum ^= value
    return checksum == response[-1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM6")
    parser.add_argument("--duration", type=float, default=90.0)
    parser.add_argument("--rate", type=float, default=20.0)
    args = parser.parse_args()

    timeout = max(0.12, 2.0 / args.rate)
    with serial.Serial(args.port, 115200, timeout=timeout, write_timeout=timeout) as stream:
        stream.dtr = False
        stream.rts = False
        time.sleep(3.0)
        stream.reset_input_buffer()

        start = time.monotonic()
        last_valid = start
        next_poll = start
        max_gap = 0.0
        max_latency = 0.0
        valid = 0
        invalid = 0
        changed = 0
        previous = None

        while time.monotonic() - start < args.duration:
            next_poll += 1.0 / args.rate
            sent = time.monotonic()
            stream.write(MSP_REQUEST)
            response = stream.read(MSP_RESPONSE_SIZE)
            now = time.monotonic()
            max_latency = max(max_latency, now - sent)

            if valid_response(response):
                values = struct.unpack("<9h", response[5:-1])
                valid += 1
                if previous is not None and values != previous:
                    changed += 1
                previous = values
                max_gap = max(max_gap, now - last_valid)
                last_valid = now
            else:
                invalid += 1

            time.sleep(max(0.0, next_poll - time.monotonic()))

    elapsed = time.monotonic() - start
    print(
        f"elapsed={elapsed:.1f}s valid={valid} invalid={invalid} changed={changed} "
        f"max_gap_ms={max_gap * 1000:.1f} max_latency_ms={max_latency * 1000:.1f} "
        f"last={previous}"
    )


if __name__ == "__main__":
    main()
