import time
import sys
import os
from pathlib import Path

INTEROP_DIR = Path(r"C:\Program Files\IVI Foundation\VISA\VisaCom64\Primary Interop Assemblies")
NATIVE_DIR = Path(r"C:\Program Files\IVI Foundation\VISA\Win64\Bin")
RESOURCE = "USB0::0x1313::0x8072::M01339668::INSTR"
WAVELENGTH_NM = 1310.0

print("=== THORLABS PM100USB LIVE POWER CHECK ===")

print("Using installed Thorlabs .NET interop driver")
inst = None

try:
    os.environ["PATH"] = str(NATIVE_DIR) + os.pathsep + os.environ["PATH"]
    os.add_dll_directory(str(NATIVE_DIR))
    sys.path.insert(0, str(INTEROP_DIR))
    import clr
    clr.AddReference("Thorlabs.TLPM_64.Interop")
    from Thorlabs.TLPM_64.Interop import TLPM

    inst = TLPM(RESOURCE, True, False)
    print(f"Connected to: {RESOURCE}")
    status = inst.setWavelength(WAVELENGTH_NM)
    if status != 0:
        raise RuntimeError(f"Setting wavelength failed with status {hex(status)}")
    print(f"Wavelength: {WAVELENGTH_NM:g} nm")
    status, unit = inst.getPowerUnit()
    if status != 0 or unit != 0:
        raise RuntimeError(f"Expected watts from meter; power unit status={hex(status)}, unit={unit}")
    time.sleep(0.3)

    print("\nStreaming 5 live readings:")
    for i in range(5):
        status, raw_val = inst.measPower()
        if status != 0:
            raise RuntimeError(f"Measurement failed with status {hex(status)}")
        abs_v = abs(raw_val)

        if abs_v >= 1e-3:
            scaled = f"{raw_val * 1e3:8.3f} mW"
        elif abs_v >= 1e-6:
            scaled = f"{raw_val * 1e6:8.3f} uW"
        else:
            scaled = f"{raw_val * 1e9:8.2f} nW"

        print(f"  Sample {i+1}: {scaled} ({raw_val:.6e} W)")
        time.sleep(0.5)

    print("\nSession closed successfully.")

except Exception as e:
    print(f"\nCommunication error: {e}")
    print("Check that Optical Parameter Monitor is closed.")
    sys.exit(1)
finally:
    if inst is not None:
        inst.Dispose()
