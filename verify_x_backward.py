"""Command exactly one negative X micro-step from the verified forward limit."""

import time

import serial

from verify_one_step import check_error, query, read_number, send


EXPECTED_X = 825738


def main() -> None:
    with serial.Serial("COM4", 19200, timeout=1.0) as ser:
        time.sleep(0.2)
        before = {axis: read_number(ser, f"{number}TP?") for axis, number in (("X", 1), ("Y", 2), ("Z", 3))}
        positive_limit = read_number(ser, "1SR?")
        negative_limit = read_number(ser, "1SL?")
        print(f"Before: {before}; X limits: {negative_limit}..{positive_limit}", flush=True)
        if before["X"] != EXPECTED_X or positive_limit != EXPECTED_X:
            raise RuntimeError("X start or positive limit changed; no movement attempted")
        if not negative_limit <= EXPECTED_X - 1 < positive_limit:
            raise RuntimeError("One negative X step is outside controller limits")

        status = query(ser, "1TS?")
        if status not in ("1TS? @", "1TS? Q"):
            raise RuntimeError(f"X controller is not idle: {status}")
        enabled_here = status == "1TS? @"
        completed = False
        try:
            if enabled_here:
                send(ser, "1MO")
                time.sleep(0.2)
                check_error(ser, 1, "enable X")
                if query(ser, "1TS?") != "1TS? Q":
                    raise RuntimeError("X motor did not enable")

            send(ser, "1PR-1")
            time.sleep(0.3)
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                try:
                    x = read_number(ser, "1TP?")
                except RuntimeError:
                    time.sleep(0.2)
                    continue
                if x == EXPECTED_X - 1:
                    break
                if x != EXPECTED_X:
                    raise RuntimeError(f"Unexpected X count: {x}")
            else:
                raise TimeoutError("X did not report the requested step")
            check_error(ser, 1, "X relative move")
            completed = True
        finally:
            if not completed:
                send(ser, "1ST")
                time.sleep(0.2)
            if enabled_here:
                send(ser, "1MF")

        after = {axis: read_number(ser, f"{number}TP?") for axis, number in (("X", 1), ("Y", 2), ("Z", 3))}
        if after != {**before, "X": EXPECTED_X - 1}:
            raise RuntimeError(f"Unexpected final counts: {after}")
        print(f"After: {after}; one X negative step accepted", flush=True)


if __name__ == "__main__":
    main()
