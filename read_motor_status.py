"""Read controller status and last error without commanding movement.

TE? may clear a stored error; save its output before rerunning this script.
"""
import time
import serial

with serial.Serial("COM4", 19200, timeout=1.0) as ser:
    time.sleep(0.2)
    for axis in (1, 2, 3):
        for command in ("ID?", "SL?", "SR?", "TS?", "TP?", "PH?", "TE?"):
            cmd = f"{axis}{command}"
            time.sleep(0.12)
            ser.write(f"{cmd}\r".encode("ascii"))
            ser.flush()
            reply = ser.read_until(b"\r").decode("ascii", errors="replace").strip()
            print(f"{cmd}: {reply or '[timeout]'}")
