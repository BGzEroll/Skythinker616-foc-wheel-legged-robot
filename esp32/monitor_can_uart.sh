#!/usr/bin/env sh

set -eu

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
    echo "Usage: $0 [serial_port] [baudrate]"
    echo "Defaults: /dev/ttyUSB0 921600"
    echo ""
    echo "Parses: A5 5A | CAN ID (little-endian) | DLC | payload | XOR checksum"
    echo "Every valid CAN UART frame is displayed immediately."
    echo "Extended feedback IDs (0x04000000 | device_id) are decoded as sequence, timestamp_us, full_count, full_angle, and derived speed."
    echo "Legacy standard ID 0x100 with 8 data bytes is also supported."
    exit 0
fi

serial_port=${1:-/dev/ttyUSB0}
baudrate=${2:-921600}

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

exec "$python_bin" - "$serial_port" "$baudrate" <<'PY'
import datetime
import struct
import sys
import time

import serial


SOF = b"\xA5\x5A"
MAX_DLC = 8
HEADER_SIZE = 2 + 4 + 1
FEEDBACK_ID_BASE = 0x04000000
FEEDBACK_ID_TYPE_MASK = 0x1C000000
DEVICE_ID_MASK = 0x03FFFFFF
ENCODER_RESOLUTION = 4096
COUNT_TO_RAD = 2.0 * 3.141592653589793 / ENCODER_RESOLUTION
FEEDBACK_HISTORY = {}


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


def signed_delta32(current, previous):
    delta = (current - previous) & 0xFFFFFFFF
    if delta & 0x80000000:
        delta -= 0x100000000
    return delta


def format_frame(can_id, data):
    raw = data.hex()
    is_feedback = (
        (can_id & FEEDBACK_ID_TYPE_MASK) == FEEDBACK_ID_BASE
        and len(data) == 8
    )
    is_legacy_feedback = can_id == 0x100 and len(data) == 8
    if is_feedback:
        sequence, timestamp_us, full_count = struct.unpack("<HHi", data)
        full_angle = full_count * COUNT_TO_RAD
        previous = FEEDBACK_HISTORY.get(can_id)
        speed = None
        if previous is not None:
            previous_timestamp, previous_count, previous_speed, previous_sequence = previous
            sequence_delta = (sequence - previous_sequence) & 0xFFFF
            timestamp_delta = (timestamp_us - previous_timestamp) & 0xFFFF

            # timestamp_us is uint16_t in the STM32 packet. Modular subtraction
            # handles its wraparound; reject an implausibly large gap after a
            # restart instead of producing a false speed spike.
            if 0 < sequence_delta <= 1000 and 0 < timestamp_delta <= 0x8000:
                speed = (
                    signed_delta32(full_count, previous_count)
                    * COUNT_TO_RAD
                    * 1000000.0
                    / timestamp_delta
                )
            elif timestamp_delta == 0:
                # The STM32 can send the same encoder sample in multiple CAN
                # frames. Keep the last calculated speed for those repeats.
                speed = previous_speed

        FEEDBACK_HISTORY[can_id] = (
            timestamp_us,
            full_count,
            speed,
            sequence,
        )
        speed_text = "UNAVAILABLE" if speed is None else f"{speed:+012.6f}"
        return (
            f"id=0x{can_id:08X}|type=feedback|"
            f"device_id=0x{can_id & DEVICE_ID_MASK:06X}|"
            f"sequence={sequence:05d}|timestamp_us={timestamp_us:05d}|"
            f"full_count={full_count:+011d}|full_angle={full_angle:+012.6f}|"
            f"speed={speed_text}|raw={raw}"
        )
    if is_legacy_feedback:
        full_angle, speed = struct.unpack("<ff", data)
        return (
            f"id=0x{can_id:08X}|type=legacy_feedback|"
            f"full_angle={full_angle:+012.6f}|speed={speed:+012.6f}|raw={raw}"
        )
    return f"id=0x{can_id:08X}|type=can|dlc={len(data):02d}|data={raw}"


port = sys.argv[1]
baud = int(sys.argv[2])

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
    f"Listening on {port} at {baud} baud; displaying every valid CAN frame "
    "immediately. Ctrl+C to stop.",
    flush=True,
)

buffer = bytearray()
last_frame_time = time.monotonic()
next_waiting = last_frame_time + 1.0

try:
    while True:
        available = uart.in_waiting
        chunk = uart.read(available if available else 1)
        if chunk:
            buffer.extend(chunk)
            for can_id, data in extract_frames(buffer):
                stamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                print(f"[{stamp}] {format_frame(can_id, data)}", flush=True)
                last_frame_time = time.monotonic()

        now = time.monotonic()
        if now >= next_waiting and now - last_frame_time >= 1.0:
            print("[waiting] no valid CAN UART frame received", flush=True)
            next_waiting = now + 1.0
except KeyboardInterrupt:
    print("\nStopped.", file=sys.stderr)
finally:
    uart.close()
PY
