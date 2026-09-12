#!/usr/bin/env python3
"""Reference stop-and-wait STM32 updater for Linux SocketCAN."""

from __future__ import annotations

import argparse
import pathlib
import struct
import sys
import time
import zlib

try:
    import can
except ImportError as exc:
    raise SystemExit("error: install python-can first: python3 -m pip install python-can") from exc

APP_MAX_SIZE = 0x5C00
DEVICE_ID_MASK = 0x03FFFFFF
BOOT_CTRL_BASE = 0x0C000000
BOOT_DATA_BASE = 0x10000000
BOOT_RESP_BASE = 0x14000000
PROTOCOL_VERSION = 1

CTRL_ENTER = 0x01
CTRL_BEGIN = 0x02
CTRL_END = 0x03
RESP_READY = 0x80
RESP_ACK = 0x81
RESP_NACK = 0x82
RESP_SUCCESS = 0x83
RESP_CRC_ERROR = 0x84
RESP_ERROR = 0x85
ENTER_MAGIC = bytes((0x07, 0xB0, 0xAD, 0x10))


def parse_int(value: str) -> int:
    return int(value, 0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--device-id", required=True, type=parse_int)
    parser.add_argument("--bin", required=True, type=pathlib.Path)
    parser.add_argument("--version", type=parse_int, default=1)
    parser.add_argument("--timeout", type=float, default=0.5)
    parser.add_argument("--retries", type=int, default=5)
    return parser.parse_args()


class Updater:
    def __init__(self, bus: can.BusABC, device_id: int, timeout: float, retries: int):
        self.bus = bus
        self.ctrl_id = BOOT_CTRL_BASE | device_id
        self.data_id = BOOT_DATA_BASE | device_id
        self.resp_id = BOOT_RESP_BASE | device_id
        self.timeout = timeout
        self.retries = retries

    def send(self, arbitration_id: int, payload: bytes) -> None:
        self.bus.send(
            can.Message(
                arbitration_id=arbitration_id,
                is_extended_id=True,
                data=payload,
            ),
            timeout=self.timeout,
        )

    def receive(self, expected_opcode: int) -> bytes | None:
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            message = self.bus.recv(max(0.0, deadline - time.monotonic()))
            if message is None or message.arbitration_id != self.resp_id:
                continue
            payload = bytes(message.data)
            if payload and payload[0] in (RESP_ERROR, RESP_CRC_ERROR):
                raise RuntimeError(f"bootloader rejected request: {payload.hex(' ')}")
            if payload and payload[0] == expected_opcode:
                return payload
        return None

    def exchange(self, arbitration_id: int, payload: bytes,
                 expected_opcode: int, sequence: int | None = None) -> bytes:
        for _ in range(self.retries):
            self.send(arbitration_id, payload)
            response = self.receive(expected_opcode)
            if response is None:
                continue
            if sequence is None:
                return response
            if len(response) >= 3 and int.from_bytes(response[1:3], "little") == sequence:
                return response
        raise TimeoutError(f"no valid response for opcode 0x{payload[0]:02X}")

    def flash(self, image: bytes, version: int) -> None:
        crc32 = zlib.crc32(image) & 0xFFFFFFFF
        enter = bytes((CTRL_ENTER, PROTOCOL_VERSION)) + ENTER_MAGIC
        self.send(self.ctrl_id, enter)
        time.sleep(0.25)

        begin = struct.pack("<BBHI", CTRL_BEGIN, PROTOCOL_VERSION, len(image), crc32)
        self.exchange(self.ctrl_id, begin, RESP_READY)
        print(f"READY: {len(image)} bytes, CRC32=0x{crc32:08X}")

        frame_count = (len(image) + 5) // 6
        for sequence, offset in enumerate(range(0, len(image), 6)):
            payload = struct.pack("<H", sequence) + image[offset : offset + 6]
            self.exchange(self.data_id, payload, RESP_ACK, sequence)
            if sequence % 128 == 0 or sequence + 1 == frame_count:
                print(f"DATA: {sequence + 1}/{frame_count}", end="\r", flush=True)
        print()

        end = struct.pack("<BI", CTRL_END, version)
        self.exchange(self.ctrl_id, end, RESP_SUCCESS)
        print("SUCCESS: metadata committed; target is resetting")


def main() -> int:
    args = parse_args()
    if not 1 <= args.device_id <= DEVICE_ID_MASK:
        raise SystemExit("error: --device-id must be in 1..0x03FFFFFF")
    if not 0 <= args.version <= 0xFFFFFFFF:
        raise SystemExit("error: --version must fit uint32")
    if args.timeout <= 0 or args.retries <= 0:
        raise SystemExit("error: --timeout and --retries must be positive")
    image = args.bin.read_bytes()
    if not image or len(image) > APP_MAX_SIZE:
        raise SystemExit(f"error: image size must be 1..{APP_MAX_SIZE} bytes")

    try:
        with can.Bus(interface="socketcan", channel=args.channel) as bus:
            Updater(bus, args.device_id, args.timeout, args.retries).flash(
                image, args.version
            )
    except (can.CanError, OSError, RuntimeError, TimeoutError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
