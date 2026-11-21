"""Record PM100USB voltage until Ctrl+C; no motor connection or movement."""

import argparse
import csv
import math
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


RESOURCE = "USB0::0x1313::0x8072::M01339668::INSTR"
WAVELENGTH_NM = 1310.0


def open_meter():
    native_dir = Path(r"C:\Program Files\IVI Foundation\VISA\Win64\Bin")
    interop_dir = Path(r"C:\Program Files\IVI Foundation\VISA\VisaCom64\Primary Interop Assemblies")
    os.environ["PATH"] = str(native_dir) + os.pathsep + os.environ["PATH"]
    os.add_dll_directory(str(native_dir))
    sys.path.insert(0, str(interop_dir))
    import clr
    clr.AddReference("Thorlabs.TLPM_64.Interop")
    from Thorlabs.TLPM_64.Interop import TLPM

    meter = TLPM(RESOURCE, True, False)
    try:
        status = meter.setWavelength(WAVELENGTH_NM)
        if status != 0:
            raise RuntimeError(f"Cannot set wavelength: {hex(status)}")
        return meter
    except BaseException:
        meter.Dispose()
        raise


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interval", type=float, default=1.0, help="seconds between readings")
    parser.add_argument("--output", type=Path, help="CSV path; defaults to a new timestamped file")
    args = parser.parse_args()
    if not math.isfinite(args.interval) or args.interval < 0.1:
        parser.error("--interval must be at least 0.1 seconds")

    started = datetime.now(timezone.utc)
    output = args.output or Path(__file__).with_name(
        f"voltage_{started:%Y%m%d_%H%M%S}.csv"
    )
    meter = open_meter()
    saved = 0
    try:
        with output.open("w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(["timestamp_utc", "voltage_v"])
            file.flush()
            print(f"Recording voltage to {output}. Press Ctrl+C to stop and save.")
            try:
                while True:
                    status, voltage = meter.measVoltage()
                    if status != 0 or not math.isfinite(voltage):
                        raise RuntimeError(f"Voltage measurement failed: {hex(status)}")
                    writer.writerow([
                        datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
                        f"{voltage:.12g}",
                    ])
                    file.flush()
                    saved += 1
                    print(f"{voltage:.6g} V")
                    time.sleep(args.interval)
            except KeyboardInterrupt:
                print("\nStopped by Ctrl+C.")
        print(f"Saved {saved} readings to {output}")
    finally:
        meter.Dispose()


if __name__ == "__main__":
    main()
