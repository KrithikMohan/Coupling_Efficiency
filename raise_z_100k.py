"""One authorized Z raise from -384000 to -484000 controller microsteps."""

import csv
import json
import time
from datetime import datetime, timezone
from pathlib import Path

import serial

from verify_one_step import check_error, query, read_number, send


START_Z = -384000
TARGET_Z = -484000
LOG = Path(__file__).with_name("z_raise_100k_log.csv")


def main() -> None:
    config = json.loads(Path(__file__).with_name("stage_travel_config.json").read_text(encoding="utf-8"))
    if (not config.get("z_direction_confirmed", False)
            or not config.get("z_extended_clearance_verified", False)
            or config["modes"]["fiber_at_coupling"]["Z"][0] != TARGET_Z):
        raise RuntimeError("This negative-Z raise script is disabled: physical Z direction has changed")

    with serial.Serial("COM4", 19200, timeout=1.0) as ser:
        time.sleep(0.2)
        x = read_number(ser, "1TP?")
        y = read_number(ser, "2TP?")
        z = read_number(ser, "3TP?")
        print(f"Before: X={x}, Y={y}, Z={z}", flush=True)
        if (x, y) != (-1, 2) or not TARGET_Z <= z <= START_Z:
            raise RuntimeError("Unexpected starting counts; no limit write or move")
        if query(ser, "3TS?") not in ("3TS? @", "3TS? Q"):
            raise RuntimeError("Z is not idle")

        send(ser, f"3SL{TARGET_Z}")
        time.sleep(0.2)
        check_error(ser, 3, "set Z upward limit")
        if read_number(ser, "3SL?") != TARGET_Z:
            raise RuntimeError("Z upward hardware limit readback failed")
        print(f"Z hardware lower limit verified: {TARGET_Z}", flush=True)

        status = query(ser, "3TS?")
        enabled_here = status == "3TS? @"
        if enabled_here:
            send(ser, "3MO")
            time.sleep(0.2)
            check_error(ser, 3, "enable Z")
        if query(ser, "3TS?") != "3TS? Q":
            raise RuntimeError("Z motor is not ready")

        try:
            with LOG.open("a", newline="", encoding="utf-8") as file:
                writer = csv.writer(file)
                if file.tell() == 0:
                    writer.writerow(("timestamp_utc", "before_z", "delta", "after_z", "error_code"))
                    file.flush()
                while z != TARGET_Z:
                    remaining = z - TARGET_Z
                    size = min(4992, remaining)
                    if size > 192:
                        size = (size // 16) * 16
                    delta = -size
                    send(ser, f"3PR{delta}")
                    deadline = time.monotonic() + 15.0
                    while time.monotonic() < deadline:
                        time.sleep(0.2)
                        motion_status = query(ser, "3TS?")
                        if motion_status == "3TS? Q":
                            break
                        if motion_status != "3TS? P":
                            raise RuntimeError(f"Unexpected Z status: {motion_status}")
                    else:
                        raise TimeoutError("Z motion timed out")
                    actual = read_number(ser, "3TP?")
                    error_code = read_number(ser, "3TE?")
                    if actual != z + delta or read_number(ser, "3TP?") != actual:
                        raise RuntimeError(f"Z count did not settle at {z + delta}; read {actual}")
                    writer.writerow((datetime.now(timezone.utc).isoformat(timespec="milliseconds"), z, delta, actual, error_code))
                    file.flush()
                    print(f"Z: {z} -> {actual}; error {error_code}", flush=True)
                    z = actual
                    if error_code not in (0, 6):
                        raise RuntimeError(f"Z controller reported error {error_code}")
        except BaseException:
            send(ser, "3ST")
            raise
        finally:
            if enabled_here:
                send(ser, "3MF")

        final = {axis: read_number(ser, f"{axis}TP?") for axis in (1, 2, 3)}
        if final != {1: x, 2: y, 3: TARGET_Z}:
            raise RuntimeError(f"Unexpected final counts: {final}")
        print(f"Final controller counts: {final}", flush=True)


if __name__ == "__main__":
    main()
