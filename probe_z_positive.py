"""One bounded +4992-count Z probe from the observed manual position."""

import csv
import time
from datetime import datetime, timezone
from pathlib import Path

import serial

from verify_one_step import query, read_number, send


START = {1: -1, 2: 2, 3: -260161}
DELTA = 192
COUNT = 26
TARGET_Z = START[3] + DELTA * COUNT
LOG = Path(__file__).with_name("z_positive_probe_log.csv")


def main() -> None:
    if LOG.exists():
        raise RuntimeError(f"Probe log exists; refusing a second probe: {LOG}")
    with serial.Serial("COM4", 19200, timeout=1.0) as ser:
        time.sleep(0.2)
        before = {axis: read_number(ser, f"{axis}TP?") for axis in START}
        print(f"Before: {before}", flush=True)
        if before != START:
            raise RuntimeError("Starting counts changed; no movement attempted")
        low, high = read_number(ser, "3SL?"), read_number(ser, "3SR?")
        if not low <= TARGET_Z <= high:
            raise RuntimeError(f"Probe target {TARGET_Z} exceeds controller limits {low}..{high}")
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
                writer.writerow(("timestamp_utc", "before_z", "commanded_delta", "after_z", "error_code"))
                file.flush()
                for _ in range(COUNT):
                    send(ser, f"3PR{DELTA}")
                    deadline = time.monotonic() + 5.0
                    while time.monotonic() < deadline:
                        time.sleep(0.15)
                        moving = query(ser, "3TS?")
                        if moving == "3TS? Q":
                            break
                        if moving != "3TS? P":
                            raise RuntimeError(f"Unexpected Z status: {moving}")
                    else:
                        raise TimeoutError("Z probe timed out")
                    actual = read_number(ser, "3TP?")
                    error = read_number(ser, "3TE?")
                    writer.writerow((datetime.now(timezone.utc).isoformat(timespec="milliseconds"), current, DELTA, actual, error))
                    file.flush()
                    if actual != current + DELTA or error not in (0, 6):
                        raise RuntimeError(f"Z probe stopped: expected {current + DELTA}, got {actual}, error {error}")
                    current = actual
                print(f"Z controller count: {START[3]} -> {current}", flush=True)
        except BaseException:
            send(ser, "3ST")
            raise
        finally:
            if enabled_here:
                send(ser, "3MF")

        after = {axis: read_number(ser, f"{axis}TP?") for axis in START}
        if after != {1: START[1], 2: START[2], 3: TARGET_Z}:
            raise RuntimeError(f"Unexpected final counts: {after}")
        print(f"After: {after}", flush=True)


if __name__ == "__main__":
    main()
