"""Read the controller-reported position of each motor without moving it."""

import re
import time

import serial


PORT = "COM4"
BAUDRATE = 19200
AXES = {"X": 1, "Y": 2, "Z": 3}


def read_position(controller: serial.Serial, axis_id: int) -> int:
    command = f"{axis_id}TP?"
    for _ in range(3):
        time.sleep(0.12)
        controller.write(f"{command}\r".encode("ascii"))
        controller.flush()
        reply = controller.read_until(b"\r").decode("ascii", errors="replace").strip()
        match = re.fullmatch(rf"{re.escape(command)}\s+(-?\d+)", reply)
        if match:
            return int(match.group(1))
        if reply and "Err." in reply:
            raise RuntimeError(f"Axis {axis_id} returned {reply!r}")
    raise RuntimeError(f"Axis {axis_id} position unavailable; last reply: {reply!r}")


def main() -> None:
    with serial.Serial(PORT, BAUDRATE, timeout=1.0) as controller:
        time.sleep(0.2)
        controller.reset_input_buffer()
        print(f"Controller positions on {PORT} (microsteps):", flush=True)
        for axis, axis_id in AXES.items():
            try:
                position = read_position(controller, axis_id)
            except RuntimeError as error:
                print(f"  {axis}: {error}", flush=True)
            else:
                print(f"  {axis}: {position}", flush=True)


if __name__ == "__main__":
    main()
