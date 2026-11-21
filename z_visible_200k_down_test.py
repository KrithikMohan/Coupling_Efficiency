"""Preposition Z up 200000, then command a 200000-count downward leg."""

import csv
import time
from datetime import datetime, timezone
from pathlib import Path

import serial

from verify_one_step import query, read_number, send


START = {1: -1, 2: 2, 3: -483998}
UP_TARGET = -283998
DOWN_TARGET = -483998
CHUNK = 4000
LOG = Path(__file__).with_name("z_visible_200k_down_test_log.csv")


def move_leg(ser: serial.Serial, writer, file, current: int, target: int, phase: str) -> int:
    while current != target:
        remaining = target - current
        delta = min(abs(remaining), CHUNK) * (1 if remaining > 0 else -1)
        if delta % 16:
            raise RuntimeError(f"Chunk is not a full-step multiple: {delta}")
        send(ser, f"3PR{delta}")
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            time.sleep(0.2)
            status = query(ser, "3TS?")
            if status == "3TS? Q":
                break
            if status != "3TS? P":
                raise RuntimeError(f"Unexpected Z status: {status}")
        else:
            raise TimeoutError("Z motion timed out")
        actual = read_number(ser, "3TP?")
        error = read_number(ser, "3TE?")
        writer.writerow((datetime.now(timezone.utc).isoformat(timespec="milliseconds"), phase, current, delta, actual, error))
        file.flush()
        if actual != current + delta or error not in (0, 6):
            raise RuntimeError(f"Z reported {actual}, expected {current + delta}; error {error}")
        current = actual
        print(f"{phase}: Z={current}/{target}; error={error}", flush=True)
    return current


def main() -> None:
    if LOG.exists():
        raise RuntimeError(f"Test log exists; refusing duplicate movement: {LOG}")
    with serial.Serial("COM4", 19200, timeout=1.0) as ser:
        time.sleep(0.2)
        before = {axis: read_number(ser, f"{axis}TP?") for axis in START}
        print(f"Before: {before}", flush=True)
        if before != START:
            raise RuntimeError("Starting counts changed; no movement attempted")
        low, high = read_number(ser, "3SL?"), read_number(ser, "3SR?")
        if not low <= DOWN_TARGET < UP_TARGET <= high:
            raise RuntimeError(f"Test exceeds controller limits {low}..{high}")
        status = query(ser, "3TS?")
        if status not in ("3TS? @", "3TS? Q"):
            raise RuntimeError(f"Z is not idle: {status}")
        enabled_here = status == "3TS? @"
        if enabled_here:
            send(ser, "3MO")
            time.sleep(0.2)
            if query(ser, "3TS?") != "3TS? Q":
                raise RuntimeError("Z motor did not enable")

        current = START[3]
        try:
            with LOG.open("w", newline="", encoding="utf-8") as file:
                writer = csv.writer(file)
                writer.writerow(("timestamp_utc", "phase", "before_z", "delta", "after_z", "error_code"))
                file.flush()
                print("PREPOSITIONING UPWARD 200000 counts", flush=True)
                current = move_leg(ser, writer, file, current, UP_TARGET, "up")
                print("DOWNWARD 200000-COUNT LEG BEGINS IN 5 SECONDS; WATCH Z NOW", flush=True)
                time.sleep(5.0)
                current = move_leg(ser, writer, file, current, DOWN_TARGET, "down")
        except BaseException:
            send(ser, "3ST")
            raise
        finally:
            if enabled_here:
                send(ser, "3MF")

        after = {axis: read_number(ser, f"{axis}TP?") for axis in START}
        if after != {1: START[1], 2: START[2], 3: DOWN_TARGET}:
            raise RuntimeError(f"Unexpected final counts: {after}")
        print(f"Final controller counts: {after}", flush=True)


if __name__ == "__main__":
    main()
