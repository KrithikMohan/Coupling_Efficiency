import serial
import time

# Hardware Configuration
SERIAL_PORT = "COM3"
BAUD_RATE = 19200
TIMEOUT = 2

print(f"Attempting to connect to Newport controller on {SERIAL_PORT}...")

try:
    motor = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
    
    # 1. Check Communication (Tell Version)
    motor.write(b"1VE\r\n")
    time.sleep(0.1)
    version_response = motor.readline().decode('utf-8').strip()
    
    if version_response:
        print(f"[SUCCESS] Controller responded: {version_response}")
        
        # 2. Check Current Position (Tell Position)
        motor.write(b"1TP\r\n")
        time.sleep(0.1)
        position_response = motor.readline().decode('utf-8').strip()
        print(f"[STATUS] Current Axis 1 Position: {position_response}")
    else:
        print("[WARNING] Port opened, but received no data back. Check wiring and power.")
        
except Exception as e:
    print(f"[ERROR] Serial connection failed: {e}")
    print("Ensure the COM port is correct and no other software is holding it open.")
finally:
    if 'motor' in locals() and motor.is_open:
        motor.close()
        print("Serial port closed.")