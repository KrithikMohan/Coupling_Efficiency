"""Move from physical zero stops to the original coupling area.

Targets: X +61450, Y +250000, Z +200000 controller microsteps.
The file refuses to move unless all three controllers report zero.
Use --positions for a read-only count check.
"""

import argparse
import re
import time

import serial


PORT = "COM4"
BAUDRATE = 19200
AXES = {"X": 1, "Y": 2, "Z": 3}


def read_position(controller: serial.Serial, axis_id: int) -> int | None:
    for _ in range(5):
        controller.reset_input_buffer()
        controller.write(f"{axis_id}TP?\r".encode("ascii"))
        controller.flush()
        time.sleep(0.05)
        reply = controller.read_until(b"\r").decode("ascii", errors="replace").strip()
        match = re.fullmatch(rf"{axis_id}TP\?\s+(-?\d+)", reply)
        if match:
            return int(match.group(1))
    return None


def show_positions() -> None:
    with serial.Serial(PORT, BAUDRATE, timeout=1.0) as controller:
        time.sleep(0.2)
        controller.reset_input_buffer()
        controller.reset_output_buffer()
        print(f"Reading controller-reported counts on {PORT} (no movement; not measured physical displacement):")
        for axis, axis_id in AXES.items():
            position = read_position(controller, axis_id)
            if position is None:
                print(f"  {axis} (axis {axis_id}): position unavailable")
            else:
                print(f"  {axis} (axis {axis_id}): {position} controller microsteps")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--positions", action="store_true", help="read counts without moving")
    args = parser.parse_args()
    if args.positions:
        show_positions()
    else:
        from return_to_original_from_zero import main as move_from_zero
        move_from_zero()


if __name__ == "__main__":
    main()
