"""Single-use, observed Y -200000 count move on a PZC200.

JA-7 is the fastest documented PZC200 jog. TP? is an open-loop count,
not a measurement of physical stage displacement.
"""

import json
import time
from datetime import datetime, timezone
from pathlib import Path

import serial

from verify_one_step import query, read_number, send


START = {1: 0, 2: 307293, 3: -135519}
Y_LIMITS = (-1, 416700)
TARGET_Y = START[2] - 200000
JOG_STOP_Y = TARGET_Y + 40000  # Allow for stop latency seen during Z jog.
LOG = Path(__file__).with_name("fast_y_back_200k_log.json")


def wait_idle(controller: serial.Serial, timeout_seconds: float) -> None:
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        status = query(controller, "2TS?")
        if status == "2TS? Q":
            return
        if status != "2TS? P":
            raise RuntimeError(f"Unexpected Y status: {status}")
    raise TimeoutError("Y motion timed out")


def main() -> None:
    if LOG.exists():
        raise RuntimeError(f"Run already logged; refusing repeat: {LOG}")

    with serial.Serial("COM4", 19200, timeout=1.0) as controller:
        time.sleep(0.2)
        before = {axis: read_number(controller, f"{axis}TP?") for axis in START}
        if before != START:
            raise RuntimeError(f"Controller counts changed: {before}; no move sent")
        limits = read_number(controller, "2SL?"), read_number(controller, "2SR?")
        if limits != Y_LIMITS or not limits[0] + 50000 < TARGET_Y < limits[1]:
            raise RuntimeError(f"Y limits changed or target too close to limit: {limits}; no move sent")
        status = query(controller, "2TS?")
        if status not in ("2TS? @", "2TS? Q"):
            raise RuntimeError(f"Y is not idle: {status}; no move sent")

        enabled_here = status == "2TS? @"
        if enabled_here:
            send(controller, "2MO")
            if query(controller, "2TS?") != "2TS? Q":
                raise RuntimeError("Y did not enable; no jog sent")

        print("Y backward jog at JA-7 starts in 5 seconds. Watch Y now.", flush=True)
        time.sleep(5.0)
        samples = []
        jog_started = False
        after_jog = None
        trim_delta = None
        try:
            try:
                send(controller, "2JA-7")
                jog_started = True
                started = time.monotonic()
                while time.monotonic() - started < 3.6:
                    count = read_number(controller, "2TP?")
                    elapsed = time.monotonic() - started
                    samples.append({"elapsed_seconds": round(elapsed, 3), "y_count": count})
                    print(f"Y count {count}; elapsed {elapsed:.2f}s", flush=True)
                    if count <= JOG_STOP_Y:
                        break
            finally:
                if jog_started:
                    send(controller, "2ST")

            wait_idle(controller, 5.0)
            after_jog = read_number(controller, "2TP?")
            jog_error = read_number(controller, "2TE?")
            print(f"Fast jog stopped at Y={after_jog}; error={jog_error}", flush=True)
            if jog_error != 0 or not TARGET_Y <= after_jog < before[2]:
                raise RuntimeError("Jog overshot target or reported an error; no trim sent")

            if after_jog != TARGET_Y:
                trim_delta = TARGET_Y - after_jog
                send(controller, f"2PR{trim_delta}")
                wait_idle(controller, 20.0)
                trim_error = read_number(controller, "2TE?")
                if trim_error != 0:
                    raise RuntimeError(f"Y trim reported error {trim_error}")
        except BaseException:
            send(controller, "2ST")
            raise
        finally:
            if enabled_here:
                send(controller, "2MF")

        after = {axis: read_number(controller, f"{axis}TP?") for axis in START}
        result = {
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "controller_counts_before": before,
            "controller_counts_after": after,
            "y_software_limits": limits,
            "command": "2JA-7; 2ST; optional 2PR trim",
            "jog_samples": samples,
            "y_after_jog": after_jog,
            "trim_delta": trim_delta,
            "physical_displacement": "not measured by controller; observer to report",
        }
        LOG.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print(f"Finished: {before} -> {after}; log {LOG}", flush=True)
        if (after[1] != START[1] or after[3] != START[3]
                or abs(after[2] - TARGET_Y) > 8):
            raise RuntimeError("Final controller counts differ beyond the documented full-step rounding")


if __name__ == "__main__":
    main()
