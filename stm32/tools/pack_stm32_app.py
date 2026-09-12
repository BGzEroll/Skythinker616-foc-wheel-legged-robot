#!/usr/bin/env python3
"""Create the fixed 1 KiB STM32 application metadata page."""

from __future__ import annotations

import argparse
import pathlib
import struct
import zlib

APP_MAX_SIZE = 0x5C00
META_SIZE = 0x400
METADATA_MAGIC = 0x31505041


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("bin", type=pathlib.Path, help="application project.bin")
    parser.add_argument("output", type=pathlib.Path, help="output metadata binary")
    parser.add_argument("--version", type=lambda value: int(value, 0), default=1)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    image = args.bin.read_bytes()
    if not image:
        raise SystemExit("error: application image is empty")
    if len(image) > APP_MAX_SIZE:
        raise SystemExit(
            f"error: application is {len(image)} bytes, limit is {APP_MAX_SIZE} bytes"
        )
    if not 0 <= args.version <= 0xFFFFFFFF:
        raise SystemExit("error: version must fit uint32")

    crc32 = zlib.crc32(image) & 0xFFFFFFFF
    record = struct.pack("<IIII", METADATA_MAGIC, len(image), crc32, args.version)
    metadata = record + bytes([0xFF]) * (META_SIZE - len(record))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(metadata)
    print(
        f"metadata: size={len(image)} crc32=0x{crc32:08X} "
        f"version={args.version} output={args.output} ({len(metadata)} bytes)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
