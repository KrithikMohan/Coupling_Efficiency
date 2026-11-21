"""One bounded live jog: lock X at its present count, then move Y +1."""

import re
import time

import serial


PORT = "COM4"
X_LIMIT = 825738
Y_START = -1310


def query(ser: serial.Serial, command: str) -> str:
    # The RS-485 converter needs breathing room between complete commands.
    # Never flush received bytes here: they may contain a controller error.
    time.sleep(0.12)
    ser.write(f"{command}\r".encode("ascii"))
    ser.flush()
    reply = ser.read_until(b"\r").decode("ascii", errors="replace").strip()
    if not reply or "Err." in reply or not reply.startswith(command + " "):
        raise RuntimeError(f"{command}: unexpected response {reply!r}")
    return reply


def read_number(ser: serial.Serial, command: str) -> int:
    reply = query(ser, command)
    match = re.fullmatch(rf"{re.escape(command)}\s+(-?\d+)", reply)
    if not match:
        raise RuntimeError(f"{command}: unexpected response {reply!r}")
    return int(match.group(1))


def send(ser: serial.Serial, command: str) -> None:
    time.sleep(0.12)
    ser.write(f"{command}\r".encode("ascii"))
    ser.flush()


def check_error(ser: serial.Serial, axis_id: int, action: str) -> None:
    """Read and report the stored error immediately after a write.

    TE? clears a stored error on this controller, so never use this as a
    generic retry mechanism for a motion command.
    """
    code = read_number(ser, f"{axis_id}TE?")
    if code:
        raise RuntimeError(f"{action}: controller {axis_id} reported error {code}")


def main() -> None:
    with serial.Serial(PORT, 19200, timeout=1.0) as ser:
        time.sleep(0.2)
        x = read_number(ser, "1TP?")
        y = read_number(ser, "2TP?")
        z = read_number(ser, "3TP?")
        print(f"Before: X={x}, Y={y}, Z={z}", flush=True)
        if (x, y) != (X_LIMIT, Y_START):
            raise RuntimeError("Positions changed; no limit write or move attempted")

        send(ser, f"1SR{X_LIMIT}")
        time.sleep(0.1)
        check_error(ser, 1, "set X limit")
        limit = read_number(ser, "1SR?")
        if limit != X_LIMIT:
            raise RuntimeError(f"X positive limit readback was {limit}; no move attempted")
        print(f"X positive software limit verified: {limit}", flush=True)

        status = query(ser, "2TS?")
        if status not in ("2TS? @", "2TS? Q"):
            raise RuntimeError(f"Y not idle: {status}")
        completed = False
        try:
            if status == "2TS? @":
                send(ser, "2MO")
                time.sleep(0.2)
                check_error(ser, 2, "enable Y")
                if query(ser, "2TS?") != "2TS? Q":
                    raise RuntimeError("Y motor did not enable")

            # Only Y is commanded. The starting-count guard prevents a second move on rerun.
            send(ser, "2PR1")
            time.sleep(0.3)
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                try:
                    actual_y = read_number(ser, "2TP?")
                except RuntimeError:
                    time.sleep(0.2)
                    continue
                if actual_y == Y_START + 1:
                    break
                if actual_y != Y_START:
                    raise RuntimeError(f"Unexpected Y count: {actual_y}")
            else:
                raise TimeoutError("Y did not report the requested single step")
            check_error(ser, 2, "Y relative move")
            completed = True
        finally:
            if not completed:
                send(ser, "2ST")
                time.sleep(0.2)
            send(ser, "2MF")

        final_x = read_number(ser, "1TP?")
        final_z = read_number(ser, "3TP?")
        if (final_x, final_z) != (x, z):
            raise RuntimeError(f"Other axis counts changed: X={final_x}, Z={final_z}")
        print(f"After: X={final_x}, Y={actual_y}, Z={final_z}", flush=True)
        print("One Y step commanded; controller counts verified. Physical motion requires independent observation.")


if __name__ == "__main__":
    main()
