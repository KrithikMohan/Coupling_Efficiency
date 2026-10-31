"""Phase 3: acquire position and power with both interfaces open; no movement."""
import argparse
import math
import time

from check_meter import ThorlabsPM100
from motor_check import MockPZC200, NewportPZC200


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--axes", type=int, nargs="+", default=[1, 2])
    parser.add_argument("--mock-stage", action="store_true")
    parser.add_argument("--dll-path")
    parser.add_argument("--wavelength", type=float, default=1550.0)
    parser.add_argument("--samples", type=int, default=5)
    args = parser.parse_args()
    if args.samples < 1 or not math.isfinite(args.wavelength) or args.wavelength <= 0:
        parser.error("Samples and wavelength must be positive and finite.")
    if any(axis < 1 or axis > 3 for axis in args.axes):
        parser.error("Choose axes 1, 2, or 3 for this setup.")
    stage = MockPZC200() if args.mock_stage else NewportPZC200(port=args.port)
    try:
        meter = ThorlabsPM100(args.dll_path)
        try:
            print(f"Meter resource: {meter.connect()}")
            meter.set_wavelength(args.wavelength)
            print("MOCK stage + REAL meter" if args.mock_stage else "REAL stage + REAL meter")
            print("Read-only check: no stage movement will be commanded.")
            time.sleep(0.2)
            for index in range(args.samples):
                positions = {axis: stage.get_position(axis) for axis in args.axes}
                power = meter.read_power()
                if not math.isfinite(power):
                    raise ValueError(f"Non-finite power reading: {power}")
                print(f"Sample {index + 1}: positions={positions} steps; power={power:.6e} W")
                time.sleep(0.1)
            print("PASS: simultaneous interface access. Motion and optical response remain unverified.")
        finally:
            meter.close()
    finally:
        stage.close()


if __name__ == "__main__":
    main()
