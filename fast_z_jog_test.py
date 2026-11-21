"""One bounded, fast Z-down jog for direct visual observation.

The PZC200 TP? value is an open-loop count, not measured displacement.
This test is intentionally separate from the read-only motor_check.py.
"""

import json
import time
from datetime import datetime, timezone
from pathlib import Path

import serial

from verify_one_step import query, read_number, send


PORT = "COM4"
EXPECTED_START = {1: 0, 2: 307293, 3: 1}
EXPECTED_Z_LIMITS = (-484000, 1)
MAX_COUNTS = 100000
# A prior JA-7 run advanced another 20,000 counts after the stop query began.
# Stop early; a timed jog cannot enforce an exact displacement limit.
STOP_EARLY_COUNTS = 40000
MAX_JOG_SECONDS = 1.8
LOG = Path(__file__).with_name("fast_z_jog_test_log.json")


def main() -> None:
    if LOG.exists():
        raise RuntimeError(f"Test already logged; refusing repeat: {LOG}")

    with serial.Serial(PORT, 19200, timeout=1.0) as controller:
        time.sleep(0.2)
        start = {axis: read_number(controller, f"{axis}TP?") for axis in (1, 2, 3)}
        if start != EXPECTED_START:
            raise RuntimeError(f"Position changed: {start}; expected {EXPECTED_START}. No motion sent.")

        limits = (read_number(controller, "3SL?"), read_number(controller, "3SR?"))
        if limits != EXPECTED_Z_LIMITS:
            raise RuntimeError(f"Z limits changed: {limits}; expected {EXPECTED_Z_LIMITS}. No motion sent.")
        target = start[3] - MAX_COUNTS + STOP_EARLY_COUNTS
        if not limits[0] + 10000 <= target < start[3] <= limits[1]:
            raise RuntimeError("Insufficient count margin for the bounded jog. No motion sent.")

        status = query(controller, "3TS?")
        if status not in ("3TS? @", "3TS? Q"):
            raise RuntimeError(f"Z not idle: {status}. No motion sent.")
        enabled_here = status == "3TS? @"
        if enabled_here:
            send(controller, "3MO")
            if query(controller, "3TS?") != "3TS? Q":
                raise RuntimeError("Z did not enable. No jog sent.")

        print("Z fast DOWN jog starts in 5 seconds; watch the stage now.", flush=True)
        time.sleep(5.0)
        samples = []
        jog_started = False
        started = time.monotonic()
        try:
            send(controller, "3JA-7")
            jog_started = True
            started = time.monotonic()
            while time.monotonic() - started < MAX_JOG_SECONDS:
                count = read_number(controller, "3TP?")
                elapsed = time.monotonic() - started
                samples.append({"seconds": round(elapsed, 3), "controller_count": count})
                print(f"Z count {count}; elapsed {elapsed:.2f}s", flush=True)
                if count <= target:
                    break
                if count <= limits[0] + 10000:
                    raise RuntimeError("Approaching Z controller limit; stopping")
        finally:
            if jog_started:
                send(controller, "3ST")
                time.sleep(0.2)
            if enabled_here:
                send(controller, "3MF")

        finish = {axis: read_number(controller, f"{axis}TP?") for axis in (1, 2, 3)}
        error = read_number(controller, "3TE?")
        result = {
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "port": PORT,
            "command": "3JA-7 then 3ST",
            "start_controller_counts": start,
            "finish_controller_counts": finish,
            "z_software_limits": limits,
            "target_count": target,
            "planned_max_counts": MAX_COUNTS,
            "stop_early_counts": STOP_EARLY_COUNTS,
            "max_jog_seconds": MAX_JOG_SECONDS,
            "samples": samples,
            "error_code": error,
            "physical_displacement": "not measured by controller; requires direct observation",
        }
        LOG.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print(f"Jog stopped. Controller counts: {start} -> {finish}; Z error {error}", flush=True)
        print(f"Log: {LOG}", flush=True)
        if finish[1] != start[1] or finish[2] != start[2] or error != 0:
            raise RuntimeError("Unexpected controller result; inspect log")


if __name__ == "__main__":
    main()
