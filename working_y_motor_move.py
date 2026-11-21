"""Finish the user-requested Y move from 274320 to 300000 counts.

Single bounded PR move. Controller counts are not physical position feedback.
"""

import json
import time
from datetime import datetime, timezone
from pathlib import Path

import serial

from verify_one_step import check_error, query, read_number, send


START = {1: 0, 2: 274320, 3: -135519}
TARGET_Y = 300000
LIMITS = (-1, 416700)
LOG = Path(__file__).with_name("finish_y_right_to_300k_log.json")


def main() -> None:
    if LOG.exists():
        raise RuntimeError(f"This finishing move was already logged: {LOG}")

    with serial.Serial("COM4", 19200, timeout=1.0) as controller:
        time.sleep(0.2)
        before = {axis: read_number(controller, f"{axis}TP?") for axis in START}
        if before != START:
            raise RuntimeError(f"Counts changed: {before}; no motion sent")
        limits = (read_number(controller, "2SL?"), read_number(controller, "2SR?"))
        if limits != LIMITS or not limits[0] < TARGET_Y < limits[1]:
            raise RuntimeError(f"Y limits changed: {limits}; no motion sent")
        status = query(controller, "2TS?")
        if status not in ("2TS? @", "2TS? Q"):
            raise RuntimeError(f"Y is not idle: {status}; no motion sent")
        previous_error = read_number(controller, "2TE?")
        if previous_error != 0:
            raise RuntimeError(f"Y has pending error {previous_error}; no motion sent")
        enabled_here = status == "2TS? @"
        if enabled_here:
            send(controller, "2MO")
            check_error(controller, 2, "enable Y")
            if query(controller, "2TS?") != "2TS? Q":
                raise RuntimeError("Y did not enable; no motion sent")

        print("Final +25680-count Y move begins in 5 seconds. Watch Y now.", flush=True)
        time.sleep(5.0)
        delta = TARGET_Y - START[2]
        try:
            send(controller, f"2PR{delta}")
            # PR is self-limiting. Leave the bus quiet while the motor runs.
            time.sleep(5.0)
            for attempt in range(5):
                try:
                    motion_status = query(controller, "2TS?")
                except RuntimeError:
                    motion_status = "[no reply]"
                if motion_status == "2TS? Q":
                    break
                if motion_status not in ("2TS? P", "[no reply]"):
                    raise RuntimeError(f"Unexpected Y status: {motion_status}")
                time.sleep(1.0)
            else:
                raise TimeoutError("Y status remained unavailable after finishing move")
        except BaseException:
            send(controller, "2ST")
            raise
        finally:
            if enabled_here:
                send(controller, "2MF")

        after = {axis: read_number(controller, f"{axis}TP?") for axis in START}
        error = read_number(controller, "2TE?")
        result = {
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "controller_counts_before": before,
            "controller_counts_after": after,
            "y_software_limits": limits,
            "command": f"2PR{delta}",
            "error_code": error,
            "physical_displacement": "observer to report",
        }
        LOG.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print(f"Finished: {before} -> {after}; Y error {error}; log {LOG}", flush=True)
        if after != {1: START[1], 2: TARGET_Y, 3: START[3]} or error != 0:
            raise RuntimeError("Final controller readback differs from target")


if __name__ == "__main__":
    main()
