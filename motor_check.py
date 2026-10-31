import time
import serial

# Connect to COM4
ser = serial.Serial("COM4", 19200, timeout=1.0)
time.sleep(0.2)
ser.reset_input_buffer()
ser.reset_output_buffer()

print("Pinging controller on COM4...")

for axis in [1, 2, 3]:
    # 1. Query position with required '?'
    ser.reset_input_buffer()
    ser.write(f"{axis}TP?\r".encode("ascii"))
    ser.flush()
    time.sleep(0.05)

    resp = ser.read_until(b"\r").decode("ascii", errors="replace").strip()

    # 2. Fallback query firmware version if position is blank
    if not resp:
        ser.reset_input_buffer()
        ser.write(f"{axis}VE?\r".encode("ascii"))
        ser.flush()
        time.sleep(0.05)
        resp = ser.read_until(b"\r").decode("ascii", errors="replace").strip()

    if resp:
        print(f"SUCCESS -> Axis {axis} answered: {
        resp!r}")
    else:

        print(f"Axis {axis}: No response")


ser.close()

