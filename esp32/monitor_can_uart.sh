#!/usr/bin/env sh

set -eu

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
    echo "Usage: $0 [serial_port] [baudrate] [display_interval_seconds]"
    echo "Defaults: /dev/ttyUSB0 921600 0.1"
    echo ""
    echo "Parses: A5 5A | CAN ID (little-endian) | DLC | payload | XOR checksum"
    echo "CAN ID 0x100 with 8 data bytes is decoded as full_angle and speed."
    exit 0
fi

serial_port=${1:-/dev/ttyUSB0}
baudrate=${2:-921600}
display_interval=${3:-0.1}

# Prefer the Python environment installed with PlatformIO because it normally
# already contains pyserial. Fall back to the system Python when available.
if [ -x /home/bgzerol/.platformio/penv/bin/python ]; then
    python_bin=/home/bgzerol/.platformio/penv/bin/python
else
    python_bin=$(command -v python3 2>/dev/null || true)
fi

if [ -z "$python_bin" ]; then
    echo "error: python3 was not found" >&2
    exit 1
fi

if ! "$python_bin" -c 'import serial' 2>/dev/null; then
    echo "error: pyserial is not available in $python_bin" >&2
    echo "       install it with: $python_bin -m pip install pyserial" >&2
    exit 1
fi

exec "$python_bin" - "$serial_port" "$baudrate" "$display_interval" <<'PY'
import datetime
import struct
import sys
import time

import serial


SOF = b"\xA5\x5A"
MAX_DLC = 8
HEADER_SIZE = 2 + 4 + 1


def extract_frames(buffer):
    """Extract valid UART frames and resynchronize after noise or boot text."""
    frames = []

    while True:
        start = buffer.find(SOF)
        if start < 0:
            # Keep a possible first byte of the next SOF sequence.
            if buffer and buffer[-1] == SOF[0]:
                del buffer[:-1]
            else:
                buffer.clear()
            break

        if start:
            del buffer[:start]

        if len(buffer) < HEADER_SIZE:
            break

        can_id = struct.unpack_from("<I", buffer, 2)[0]
        dlc = buffer[6]
        if dlc > MAX_DLC:
            del buffer[0]
            continue

        frame_size = HEADER_SIZE + dlc + 1
        if len(buffer) < frame_size:
            break

        checksum = 0
        for value in buffer[2:frame_size - 1]:
            checksum ^= value
        if checksum != buffer[frame_size - 1]:
            # The SOF was a false match; try the next byte as a new start.
            del buffer[0]
            continue

        data = bytes(buffer[7:7 + dlc])
        del buffer[:frame_size]
        frames.append((can_id, data))

    return frames


def format_frame(can_id, data):
    raw = data.hex(" ")
    if can_id == 0x100 and len(data) == 8:
        full_angle, speed = struct.unpack("<ff", data)
        return (
            f"id=0x{can_id:08X} full_angle={full_angle: .6f} "
            f"speed={speed: .6f} raw={raw}"
        )
    return f"id=0x{can_id:08X} dlc={len(data)} data={raw}"


port = sys.argv[1]
baud = int(sys.argv[2])
interval = float(sys.argv[3])
if interval <= 0:
    raise SystemExit("display interval must be greater than zero")

try:
    # Set control lines before opening so the monitor does not intentionally
    # reset an ESP32 connected through USB-UART.
    uart = serial.Serial(
        port=None,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.05,
        rtscts=False,
        dsrdtr=False,
    )
    uart.port = port
    uart.dtr = False
    uart.rts = False
    uart.open()
    uart.reset_input_buffer()
except (serial.SerialException, ValueError) as exc:
    raise SystemExit(f"error: cannot open {port} at {baud} baud: {exc}")


print(
    f"Listening on {port} at {baud} baud; displaying valid CAN frames "
    f"every {interval:g}s. Ctrl+C to stop.",
    flush=True,
)

buffer = bytearray()
latest_frames = {}
next_display = time.monotonic() + interval
next_waiting = time.monotonic() + 1.0

try:
    while True:
        available = uart.in_waiting
        chunk = uart.read(available if available else 1)
        if chunk:
            buffer.extend(chunk)
            for can_id, data in extract_frames(buffer):
                latest_frames[can_id] = data

        now = time.monotonic()
        if now >= next_display:
            if latest_frames:
                stamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                for can_id in sorted(latest_frames):
                    print(f"[{stamp}] {format_frame(can_id, latest_frames[can_id])}", flush=True)
                latest_frames.clear()
            elif now >= next_waiting:
                print("[waiting] no valid CAN UART frame received", flush=True)
                next_waiting = now + 1.0

            next_display = now + interval
except KeyboardInterrupt:
    print("\nStopped.", file=sys.stderr)
finally:
    uart.close()
PY
