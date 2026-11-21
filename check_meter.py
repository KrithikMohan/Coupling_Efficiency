"""Reusable PM100USB voltage/power reader and a five-sample live check."""

import math
import os
import sys
import time
from pathlib import Path

INTEROP_DIR = Path(r"C:\Program Files\IVI Foundation\VISA\VisaCom64\Primary Interop Assemblies")
NATIVE_DIR = Path(r"C:\Program Files\IVI Foundation\VISA\Win64\Bin")
RESOURCE = "USB0::0x1313::0x8072::M01339668::INSTR"
WAVELENGTH_NM = 1310.0


class ThorlabsPM100:
    def __init__(self, dll_path: str | None = None):
        if dll_path is not None:
            raise ValueError("This meter uses the installed Thorlabs .NET interop driver")
        self.instrument = None
        self._dll_directory = None

    def connect(self) -> str:
        os.environ["PATH"] = str(NATIVE_DIR) + os.pathsep + os.environ["PATH"]
        self._dll_directory = os.add_dll_directory(str(NATIVE_DIR))
        sys.path.insert(0, str(INTEROP_DIR))
        import clr
        clr.AddReference("Thorlabs.TLPM_64.Interop")
        from Thorlabs.TLPM_64.Interop import TLPM

        self.instrument = TLPM(RESOURCE, True, False)
        status, unit = self.instrument.getPowerUnit()
        if status != 0 or unit != 0:
            self.close()
            raise RuntimeError(f"Expected watts from meter; status={hex(status)}, unit={unit}")
        return RESOURCE

    def set_wavelength(self, wavelength_nm: float) -> None:
        status = self.instrument.setWavelength(wavelength_nm)
        if status != 0:
            raise RuntimeError(f"Setting wavelength failed with status {hex(status)}")

    def read_voltage(self) -> float:
        status, value = self.instrument.measVoltage()
        if status != 0 or not math.isfinite(value):
            raise RuntimeError(f"Voltage measurement failed with status {hex(status)}")
        return value

    def read_power(self) -> float:
        status, value = self.instrument.measPower()
        if status != 0 or not math.isfinite(value):
            raise RuntimeError(f"Power measurement failed with status {hex(status)}")
        return value

    def close(self) -> None:
        if self.instrument is not None:
            self.instrument.Dispose()
            self.instrument = None
        if self._dll_directory is not None:
            self._dll_directory.close()
            self._dll_directory = None


def main() -> None:
    print("=== THORLABS PM100USB LIVE VOLTAGE AND POWER CHECK ===")
    meter = ThorlabsPM100()
    try:
        print(f"Connected to: {meter.connect()}")
        meter.set_wavelength(WAVELENGTH_NM)
        print(f"Wavelength: {WAVELENGTH_NM:g} nm")
        for index in range(5):
            voltage = meter.read_voltage()
            power = meter.read_power()
            print(f"Sample {index + 1}: {voltage:.6e} V, {power:.6e} W", flush=True)
            time.sleep(0.5)
    finally:
        meter.close()


if __name__ == "__main__":
    main()