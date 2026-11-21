"""Move PZC200 X/Y/Z from confirmed physical stops to the old coupling area.

Targets are controller counts: X +61450, Y +250000, Z +200000.
Positive Z is the user-confirmed upward direction. TP? is open-loop feedback.
"""

import json
import time
from datetime import datetime, timezone
from pathlib import Path

import serial

from verify_one_step import check_error, query, read_number, send


PORT = "COM4"
AXIS_IDS = {"X": 1, "Y": 2, "Z": 3}
TARGETS = {"X": 61450, "Y": 250000, "Z": 200000}
ORDER = ("Z", "Y", "X")
LIMITS = {"X": (-1, 61450), "Y": (-1, 416700), "Z": (-1, 250000)}
LOG = Path(__file__).with_name("motor_check_move_log.jsonl")


def log(event: str, **fields: object) -> None:
    record = {"time_utc": datetime.now(timezone.utc).isoformat(), "event": event, **fields}
    with LOG.open("a", encoding="utf-8") as file:
        file.write(json.dumps(record) + "\n")
        file.flush()


def counts(controller: serial.Serial) -> dict[str, int]:
    return {axis: read_number(controller, f"{axis_id}TP?")
            for axis, axis_id in AXIS_IDS.items()}


def wait_idle(controller: serial.Serial, axis_id: int, timeout_seconds: float = 120.0) -> None:
    deadline = time.monotonic() + timeout_seconds
    last = "[no response]"
    while time.monotonic() < deadline:
        try:
            last = query(controller, f"{axis_id}TS?")
        except RuntimeError as error:
            last = str(error)
        if last == f"{axis_id}TS? Q":
            return
        if last != f"{axis_id}TS? P" and "unexpected response" not in last:
            raise RuntimeError(f"Unexpected axis {axis_id} status: {last}")
        time.sleep(1.0)
    raise TimeoutError(f"Axis {axis_id} did not become idle: {last}")


def preflight(controller: serial.Serial) -> dict[str, int]:
    initial = counts(controller)
    print(f"Controller counts: {initial} (not physical position)", flush=True)
    if any(abs(value) > 1 for value in initial.values()):
        raise RuntimeError("All three controller counts must be within one count of origin; no move sent")
    for axis, axis_id in AXIS_IDS.items():
        if query(controller, f"{axis_id}ID?") != f"{axis_id}ID? PZA12":
            raise RuntimeError(f"{axis} PZA12 identity not confirmed; no move sent")
        status = query(controller, f"{axis_id}TS?")
        if status not in (f"{axis_id}TS? @", f"{axis_id}TS? Q"):
            raise RuntimeError(f"{axis} not idle: {status}; no move sent")
        check_error(controller, axis_id, f"preflight {axis}")
    return initial


def install_limits(controller: serial.Serial) -> None:
    for axis, axis_id in AXIS_IDS.items():
        low, high = LIMITS[axis]
        for name, expected in (("SL", low), ("SR", high)):
            send(controller, f"{axis_id}{name}{expected}")
            check_error(controller, axis_id, f"set {axis} {name}")
            actual = read_number(controller, f"{axis_id}{name}?")
            if actual != expected:
                raise RuntimeError(f"{axis} {name} readback {actual}, expected {expected}")
        send(controller, f"{axis_id}SM")
        check_error(controller, axis_id, f"save {axis} limits")


def move_axis(controller: serial.Serial, axis: str) -> None:
    axis_id = AXIS_IDS[axis]
    status = query(controller, f"{axis_id}TS?")
    if status not in (f"{axis_id}TS? @", f"{axis_id}TS? Q"):
        raise RuntimeError(f"{axis} not idle: {status}")
    enabled_here = status == f"{axis_id}TS? @"
    if enabled_here:
        send(controller, f"{axis_id}MO")
        check_error(controller, axis_id, f"enable {axis}")
        if query(controller, f"{axis_id}TS?") != f"{axis_id}TS? Q":
            raise RuntimeError(f"{axis} did not enable")

    # Long PR moves round to multiples of 16; trim the final remainder.
    current = read_number(controller, f"{axis_id}TP?")
    try:
        while current < TARGETS[axis]:
            remaining = TARGETS[axis] - current
            delta = (remaining // 16) * 16 if remaining > 16 else remaining
            if delta <= 0:
                raise RuntimeError(f"{axis} cannot advance from {current}")
            expected = current + delta
            log("move_started", axis=axis, before_count=current, commanded_delta=delta)
            print(f"{axis}: +{delta} counts toward {TARGETS[axis]}", flush=True)
            send(controller, f"{axis_id}PR{delta}")
            # Leave the bus quiet for the expected travel time; avoid the
            # communication overflow seen with rapid status polling.
            time.sleep(max(0.3, delta / 48000.0 + 0.3))
            wait_idle(controller, axis_id)
            actual = read_number(controller, f"{axis_id}TP?")
            check_error(controller, axis_id, f"move {axis}")
            log("move_finished", axis=axis, after_count=actual)
            if not current < actual <= TARGETS[axis] or abs(actual - expected) > 16:
                raise RuntimeError(f"{axis} controller stopped at {actual}, expected near {expected}")
            current = actual
    except BaseException as error:
        send(controller, f"{axis_id}ST")
        log("move_failed", axis=axis, error=str(error))
        raise
    finally:
        if enabled_here:
            send(controller, f"{axis_id}MF")


def main() -> None:
    with serial.Serial(PORT, 19200, timeout=1.0) as controller:
        time.sleep(0.2)
        initial = preflight(controller)
        install_limits(controller)
        log("run_started", targets=TARGETS, limits=LIMITS)
        print("Motion starts now: Z up, Y right, X forward. Watch the stage.", flush=True)
        for index, axis in enumerate(ORDER):
            expected = {name: TARGETS[name] if name in ORDER[:index] else initial[name]
                        for name in AXIS_IDS}
            actual = counts(controller)
            if actual != expected:
                raise RuntimeError(f"Counts before {axis}: {actual}; expected {expected}")
            move_axis(controller, axis)
        final = counts(controller)
        log("run_finished", final_counts=final)
        if final != TARGETS:
            raise RuntimeError(f"Final counts {final} differ from targets {TARGETS}")
        print(f"Controller targets reached: {final}. Verify physical position separately.", flush=True)


if __name__ == "__main__":
    main()
