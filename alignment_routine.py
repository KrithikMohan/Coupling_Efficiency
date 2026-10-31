import ctypes
import os
import sys
import time
from ctypes import byref, c_bool, c_char_p, c_double, c_uint32, c_void_p, create_string_buffer
import serial

# ==============================================================================
# CONFIGURATION
# ==============================================================================
STAGE_PORT: str = os.getenv("STAGE_PORT", "COM4")
STAGE_BAUD: int = 19200

AXIS_MAP: dict[str, int] = {
    "X": 1,
    "Y": 2,
    "Z": 3,
}

THORLABS_DLL: str = r"C:\Program Files\IVI Foundation\VISA\Win64\Bin\TLPM_64.dll"
POLL_INTERVAL_SEC: float = 0.5


# ==============================================================================
# THORLABS PM100USB INTERFACE
# ==============================================================================
class ThorlabsPM100USB:
    def __init__(self, dll_path: str = THORLABS_DLL):
        if not os.path.isfile(dll_path):
            raise FileNotFoundError(f"Thorlabs DLL not found at: {dll_path}")

        self.dll = ctypes.cdll.LoadLibrary(dll_path)
        self.handle = c_void_p(0)
        self._setup_signatures()

    def _setup_signatures(self):
        self.dll.TLPM_findRsrc.argtypes = [c_void_p, ctypes.POINTER(c_uint32)]
        self.dll.TLPM_findRsrc.restype = c_uint32

        self.dll.TLPM_getRsrcName.argtypes = [c_void_p, c_uint32, c_char_p]
        self.dll.TLPM_getRsrcName.restype = c_uint32

        # ViStatus TLPM_init(ViRsrc, ViBoolean idQuery, ViBoolean resetDevice, ViPSession)
        self.dll.TLPM_init.argtypes = [c_char_p, c_bool, c_bool, ctypes.POINTER(c_void_p)]
        self.dll.TLPM_init.restype = c_uint32

        self.dll.TLPM_measPower.argtypes = [c_void_p, ctypes.POINTER(c_double)]
        self.dll.TLPM_measPower.restype = c_uint32

        self.dll.TLPM_close.argtypes = [c_void_p]
        self.dll.TLPM_close.restype = c_uint32

    def connect(self) -> str:
        count = c_uint32(0)
        self.dll.TLPM_findRsrc(c_void_p(0), byref(count))

        # Build list of potential resource descriptors
        candidates = []
        if count.value > 0:
            name_buf = create_string_buffer(256)
            self.dll.TLPM_getRsrcName(c_void_p(0), 0, name_buf)
            raw = name_buf.value
            if raw:
                candidates.append(raw)
                candidates.append(raw.replace(b"::::INSTR", b"::0::INSTR"))

        candidates.extend([
            b"USB0::0x1313::0x8072::M01339668::INSTR",
            b"USB0::0x1313::0x8072::261005409::INSTR",
            b"USB0::0x1313::0x8072::1916864::INSTR",
        ])

        for rsrc in candidates:
            h = c_void_p(0)
            # resetDevice=True kicks the meter out of any lingering measurement loops
            status = self.dll.TLPM_init(rsrc, False, True, byref(h))
            if status == 0 and h.value:
                self.handle = h
                time.sleep(0.5)  # Allow reset to settle
                return rsrc.decode("ascii", errors="replace")

        raise ConnectionError(
            "Failed to open PM100USB. Make sure Thorlabs Optical Parameter Monitor is closed "
            "and replug the meter's USB cable."
        )

    def read_power(self, retries: int = 3) -> float:
        power = c_double(0.0)
        for attempt in range(retries):
            status = self.dll.TLPM_measPower(self.handle, byref(power))
            if status == 0:
                return power.value
            time.sleep(0.05)

        raise RuntimeError(f"Measurement failed with status {hex(status)}")

    def close(self):
        if self.handle and self.handle.value:
            self.dll.TLPM_close(self.handle)
            self.handle = c_void_p(0)


# ==============================================================================
# NEWPORT PZC200 STAGE INTERFACE
# ==============================================================================
class PZC200Stage:
    def __init__(self, port: str = STAGE_PORT, baudrate: int = STAGE_BAUD):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0,
        )
        time.sleep(0.2)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def _query(self, cmd: str) -> str:
        time.sleep(0.06)  # RS-485 transceiver bus pacing
        self.ser.reset_input_buffer()
        self.ser.write(f"{cmd}\r".encode("ascii"))
        self.ser.flush()
        time.sleep(0.06)
        return self.ser.read_until(b"\r").decode("ascii", errors="replace").strip()

    def get_position(self, axis_label: str) -> int:
        axis_id = AXIS_MAP[axis_label]
        for _ in range(3):
            resp = self._query(f"{axis_id}TP?")
            if not resp:
                resp = self._query(f"{axis_id}TP")

            parts = resp.split()
            if parts:
                cleaned = parts[-1].lstrip("?").strip()
                try:
                    return int(cleaned)
                except ValueError:
                    try:
                        return int(float(cleaned))
                    except ValueError:
                        pass
            time.sleep(0.06)

        raise TimeoutError(f"Stage axis {axis_label} (ID {axis_id}) timed out.")

    def close(self):
        if self.ser.is_open:
            self.ser.close()


def format_power(watts: float) -> str:
    abs_w = abs(watts)
    if abs_w >= 1e-3:
        return f"{watts * 1e3:8.3f} mW"
    elif abs_w >= 1e-6:
        return f"{watts * 1e6:8.3f} uW"
    else:
        return f"{watts * 1e9:8.2f} nW"


# ==============================================================================
# PASSIVE MONITOR EXECUTION
# ==============================================================================
def main():
    print("=== OPTICAL COUPLING MONITOR (PASSIVE MODE) ===")
    print("Connecting instruments...\n")

    meter = ThorlabsPM100USB()
    rsrc_name = meter.connect()
    print(f"  Power Meter Active: {rsrc_name}")

    stage = PZC200Stage()
    _ = stage._query("1TP?")  # RS-485 line bias warm-up
    print(f"  Stage Controllers : Online on {STAGE_PORT}\n")

    print("-" * 75)
    print("Streaming live telemetry. Zero motor commands sent. Press Ctrl+C to halt.")
    print("-" * 75)

    sample = 0
    try:
        while True:
            sample += 1

            pos_x = stage.get_position("X")
            pos_y = stage.get_position("Y")
            pos_z = stage.get_position("Z")

            try:
                power_w = meter.read_power()
                power_str = format_power(power_w)
                raw_str = f"({power_w:.4e} W)"
            except Exception as e:
                power_str = "  ERR SAMPLE  "
                raw_str = f"[{e}]"

            print(
                f"[{sample:04d}] "
                f"Positions -> X: {pos_x:>6d} | Y: {pos_y:>6d} | Z: {pos_z:>6d}  ||  "
                f"Optical Power: {power_str} {raw_str}"
            )

            time.sleep(POLL_INTERVAL_SEC)

    except KeyboardInterrupt:
        print("\n\nMonitoring stopped by user.")
    finally:
        stage.close()
        meter.close()
        print("Hardware handles closed safely.")


if __name__ == "__main__":
    main()