"""One bounded +16000-count Z lift to check physical displacement."""

import csv
import time
from datetime import datetime, timezone
from pathlib import Path

import serial

from verify_one_step import query, read_number, send


EXPECTED = {1: -1, 2: 2, 3: -483998}
CHUNK = 4000
CHUNKS = 4
TARGET_Z = EXPECTED[3] + CHUNK * CHUNKS
LOG = Path(__file__).with_name("z_measurable_lift_log.csv")


def main() -> None:
    if LOG.exists():
        raise RuntimeError("Lift log exists; refusing duplicate movement")
    with serial.Serial("COM4", 19200, timeout=1.0) as ser:
        time.sleep(0.2)
        before = {axis: read_number(ser, f"{axis}TP?") for axis in EXPECTED}
        print(f"Before: {before}", flush=True)
        if before != EXPECTED:
            raise RuntimeError("Starting counts changed; no move attempted")
        low, high = read_number(ser, "3SL?"), read_number(ser, "3SR?")
        if not low <= TARGET_Z <= high:
            raise RuntimeError(f"Target {TARGET_Z} exceeds hardware limits {low}..{high}")
        status = query(ser, "3TS?")
        if status not in ("3TS? @", "3TS? Q"):
            raise RuntimeError(f"Z is not idle: {status}")
        enabled_here = status == "3TS? @"
        if enabled_here:
            send(ser, "3MO")
            time.sleep(0.2)
            if query(ser, "3TS?") != "3TS? Q":
                raise RuntimeError("Z motor did not enable")

        current = EXPECTED[3]
        try:
            with LOG.open("w", newline="", encoding="utf-8") as file:
                writer = csv.writer(file)
                writer.writerow(("timestamp_utc", "before_z", "commanded_delta", "after_z", "error_code"))
                file.flush()
                for _ in range(CHUNKS):
                    send(ser, f"3PR{CHUNK}")
                    deadline = time.monotonic() + 10.0
                    while time.monotonic() < deadline:
                        time.sleep(0.2)
                        motion_status = query(ser, "3TS?")
                        if motion_status == "3TS? Q":
                            break
                        if motion_status != "3TS? P":
                            raise RuntimeError(f"Unexpected Z status: {motion_status}")
                    else:
                        raise TimeoutError("Z lift timed out")
                    after = read_number(ser, "3TP?")
                    error = read_number(ser, "3TE?")
                    writer.writerow((datetime.now(timezone.utc).isoformat(timespec="milliseconds"), current, CHUNK, after, error))
                    file.flush()
                    if after != current + CHUNK or error not in (0, 6):
                        raise RuntimeError(f"Unexpected Z result: {after}, error {error}")
                    print(f"Z: {current} -> {after}; error {error}", flush=True)
                    current = after
        except BaseException:
            send(ser, "3ST")
            raise
        finally:
            if enabled_here:
                send(ser, "3MF")

        final = {axis: read_number(ser, f"{axis}TP?") for axis in EXPECTED}
        if final != {1: EXPECTED[1], 2: EXPECTED[2], 3: TARGET_Z}:
            raise RuntimeError(f"Unexpected final counts: {final}")
        print(f"Final controller counts: {final}", flush=True)


if __name__ == "__main__":
    main()
