"""Single-use Y +300000-count move from the user-confirmed physical left stop.

TP? is an open-loop controller count, not measured physical displacement.
"""

import json
import time
from datetime import datetime, timezone
from pathlib import Path

import serial

from verify_one_step import check_error, query, read_number, send


START = {1: 0, 2: 0, 3: -135519}
Y_LEFT_LIMIT = -1
Y_RIGHT_LIMIT = 416700
Y_TARGET = 300000
LOG = Path(__file__).with_name("y_from_left_stop_right_300k_log.json")


def wait_idle(controller: serial.Serial, seconds: float) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        status = query(controller, "2TS?")
        if status == "2TS? Q":
            return
        if status != "2TS? P":
            raise RuntimeError(f"Unexpected Y status: {status}")
    raise TimeoutError("Y move exceeded time limit")


def main() -> None:
    if LOG.exists():
        raise RuntimeError(f"Single-use move already logged: {LOG}")

    with serial.Serial("COM4", 19200, timeout=1.0) as controller:
        time.sleep(0.2)
        before = {axis: read_number(controller, f"{axis}TP?") for axis in START}
        if before != START:
            raise RuntimeError(f"Controller counts changed: {before}; no move sent")
        if query(controller, "2ID?") != "2ID? PZA12":
            raise RuntimeError("Y actuator identity not confirmed; no move sent")
        status = query(controller, "2TS?")
        if status not in ("2TS? @", "2TS? Q"):
            raise RuntimeError(f"Y not idle: {status}; no move sent")
        old_limits = (read_number(controller, "2SL?"), read_number(controller, "2SR?"))
        if old_limits not in ((-10000000, 10000000), (Y_LEFT_LIMIT, Y_RIGHT_LIMIT)):
            raise RuntimeError(f"Unexpected Y limits {old_limits}; no move sent")

        # The user has physically identified this location as the left stop.
        send(controller, "2OR")
        check_error(controller, 2, "zero Y at left stop")
        if read_number(controller, "2TP?") != 0:
            raise RuntimeError("Y zero did not read back")
        for command in (f"2SL{Y_LEFT_LIMIT}", f"2SR{Y_RIGHT_LIMIT}"):
            send(controller, command)
            check_error(controller, 2, command)
        limits = (read_number(controller, "2SL?"), read_number(controller, "2SR?"))
        if limits != (Y_LEFT_LIMIT, Y_RIGHT_LIMIT):
            raise RuntimeError(f"Y limit readback failed: {limits}")
        send(controller, "2SM")  # Preserve the restored limits across power cycles.
        check_error(controller, 2, "save Y limits")

        enabled_here = status == "2TS? @"
        if enabled_here:
            send(controller, "2MO")
            check_error(controller, 2, "enable Y")
            if query(controller, "2TS?") != "2TS? Q":
                raise RuntimeError("Y did not enable; no move sent")

        print("Y +300000-count RIGHT move begins in 5 seconds. Watch the stage now.", flush=True)
        time.sleep(5.0)
        started = time.monotonic()
        try:
            send(controller, f"2PR{Y_TARGET}")
            wait_idle(controller, 120.0)
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
            "user_confirmed_physical_start": "Y left mechanical stop",
            "controller_counts_before": before,
            "controller_counts_after": after,
            "old_y_software_limits": old_limits,
            "restored_y_software_limits": limits,
            "command": f"2PR{Y_TARGET}",
            "elapsed_seconds": round(time.monotonic() - started, 3),
            "error_code": error,
            "physical_displacement": "not measured by controller; observer to report",
        }
        LOG.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print(f"Finished: {before} -> {after}; Y error {error}; log {LOG}", flush=True)
        if after != {1: START[1], 2: Y_TARGET, 3: START[3]} or error != 0:
            raise RuntimeError("Controller did not report the requested final counts")


if __name__ == "__main__":
    main()
