"""Raise Z to -300,000 controller counts after the user-corrected target."""

import csv
import json
import time
from datetime import datetime, timezone
from pathlib import Path

import serial

from verify_one_step import check_error, query, read_number, send


EXPECTED_XY = {1: 61450, 2: 250000}
START_Z = -210000
TARGET_Z = -300000
LOG = Path(__file__).with_name("z_adjustment_log.csv")


def main() -> None:
    config = json.loads(Path(__file__).with_name("stage_travel_config.json").read_text(encoding="utf-8"))
    if (not config.get("z_direction_confirmed", False)
            or not config.get("z_physical_motion_verified", False)
            or not config.get("z_extended_clearance_verified", False)):
        raise RuntimeError("Live motion disabled: extended Z clearance must be verified")
    lower, upper = config["modes"]["fiber_at_coupling"]["Z"]
    if not lower <= TARGET_Z <= upper:
        raise RuntimeError(f"Z target {TARGET_Z} exceeds configured bounds {lower}..{upper}")
    with serial.Serial("COM4", 19200, timeout=1.0) as ser:
        time.sleep(0.2)
        before = {axis: read_number(ser, f"{axis}TP?") for axis in (1, 2, 3)}
        if any(before[axis] != value for axis, value in EXPECTED_XY.items()) or not TARGET_Z <= before[3] <= START_Z:
            raise RuntimeError(f"Controller counts changed: {before}; no move attempted")
        hardware_lower = read_number(ser, "3SL?")
        hardware_upper = read_number(ser, "3SR?")
        if not hardware_lower <= TARGET_Z <= hardware_upper:
            raise RuntimeError(f"Z hardware limits {hardware_lower}..{hardware_upper} exclude {TARGET_Z}")
        status = query(ser, "3TS?")
        if status not in ("3TS? @", "3TS? Q"):
            raise RuntimeError(f"Z is not idle: {status}")
        enabled_here = status == "3TS? @"
        if enabled_here:
            send(ser, "3MO")
            time.sleep(0.2)
            check_error(ser, 3, "enable Z")
        if query(ser, "3TS?") != "3TS? Q":
            raise RuntimeError("Z motor is not ready")

        current = before[3]
        try:
            with LOG.open("a" if LOG.exists() else "w", newline="", encoding="utf-8") as file:
                writer = csv.writer(file)
                if file.tell() == 0:
                    writer.writerow(("timestamp_utc", "before_z_steps", "commanded_delta_steps", "after_z_steps", "controller_error"))
                file.flush()
                while current != TARGET_Z:
                    delta = -min(4992, current - TARGET_Z)
                    send(ser, f"3PR{delta}")
                    deadline = time.monotonic() + 15.0
                    while time.monotonic() < deadline:
                        time.sleep(0.2)
                        moving = query(ser, "3TS?")
                        if moving == "3TS? Q":
                            break
                        if moving != "3TS? P":
                            raise RuntimeError(f"Unexpected Z status: {moving}")
                    else:
                        raise TimeoutError("Z adjustment timed out")
                    actual = read_number(ser, "3TP?")
                    error_code = read_number(ser, "3TE?")
                    if actual != current + delta:
                        raise RuntimeError(f"Z reported {actual}, expected {current + delta}")
                    if read_number(ser, "3TP?") != actual:
                        raise RuntimeError("Z count was not stable on a second read")
                    writer.writerow((datetime.now(timezone.utc).isoformat(timespec="milliseconds"), current, delta, actual, error_code))
                    file.flush()
                    print(f"Z: {current} -> {actual}; error register {error_code}", flush=True)
                    current = actual
                    if error_code not in (0, 6):
                        raise RuntimeError(f"Z controller reported error {error_code}")
        except BaseException:
            send(ser, "3ST")
            raise
        finally:
            if enabled_here:
                send(ser, "3MF")

        after = {axis: read_number(ser, f"{axis}TP?") for axis in (1, 2, 3)}
        if after != {**EXPECTED_XY, 3: TARGET_Z}:
            raise RuntimeError(f"Unexpected final controller counts: {after}")
        print(f"Final controller counts: {after}", flush=True)


if __name__ == "__main__":
    main()
