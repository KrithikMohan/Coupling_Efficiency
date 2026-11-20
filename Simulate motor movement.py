"""Return live motors to the supplied coordinates, or simulate a local search."""

import argparse
import csv
import json
import math
import os
import re
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import serial

from verify_one_step import check_error, query, read_number, send

AXES = ("X", "Y", "Z")
MAX_OFFSET = 6
BASELINE_V = 0.025
DATA_FILE = Path(__file__).with_name("simulated_voltage.csv")
TRAVEL_CONFIG = Path(__file__).with_name("stage_travel_config.json")
DEFAULT_LOG = Path(__file__).with_name("position_signal_log.csv")
SIMULATION_START = {"X": 61446, "Y": 249995, "Z": -200003}
SIMULATION_PEAK = {"X": 61450, "Y": 250000, "Z": -200000}
AXIS_IDS = {"X": 1, "Y": 2, "Z": 3}
WAVELENGTH_NM = 1310.0
LIVE_TARGET = {"X": 61450, "Y": 250000, "Z": -200000}
LIVE_ORDER = ("Z", "Y", "X")
LIVE_LOG = Path(__file__).with_name("live_return_log.csv")
LOG_HEADER = ["timestamp_utc", "stage_mode", "x_position_steps", "y_position_steps", "z_position_steps", "voltage_v", "power_w", "signal_source"]


def load_profiles(path: Path) -> dict[str, dict[int, float]]:
    profiles = {axis: {} for axis in AXES}
    with path.open(newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        if reader.fieldnames != ["axis", "controller_position_steps", "voltage_v"]:
            raise ValueError("Expected axis,controller_position_steps,voltage_v columns")
        for row in reader:
            axis = row["axis"]
            if axis not in profiles:
                raise ValueError(f"Unknown axis: {axis}")
            offset = int(row["controller_position_steps"]) - SIMULATION_START[axis]
            voltage = float(row["voltage_v"])
            if not 0 <= offset <= MAX_OFFSET or not math.isfinite(voltage) or voltage < 0:
                raise ValueError(f"Invalid voltage row: {row}")
            if offset in profiles[axis]:
                raise ValueError(f"Duplicate sample for {axis} at step {offset}")
            profiles[axis][offset] = voltage
    required = set(range(MAX_OFFSET + 1))
    if any(set(profiles[axis]) != required for axis in AXES):
        raise ValueError(f"Each axis needs offsets 0 through {MAX_OFFSET}")
    return profiles


class SimulatedStage:
    def __init__(self, profiles: dict[str, dict[int, float]]):
        self.profiles = profiles
        self.position = {axis: 0 for axis in AXES}
        self.moves = 0

    def move_one_step(self, axis: str, direction: int) -> None:
        if axis not in AXES or direction not in (-1, 1):
            raise ValueError("Only one-step moves on X, Y, or Z are allowed")
        target = self.position[axis] + direction
        if not 0 <= target <= MAX_OFFSET:
            raise ValueError(f"{axis} travel limit reached at {self.position[axis]}")
        self.position[axis] = target
        self.moves += 1

    def read_voltage(self) -> float:
        return BASELINE_V + sum(
            self.profiles[axis][self.position[axis]] for axis in AXES
        )

    def coordinates(self) -> dict[str, int]:
        return {axis: SIMULATION_START[axis] + self.position[axis] for axis in AXES}


class RealStage(SimulatedStage):
    """Move real axes one controller step at a time."""

    def __init__(self, profiles, port: str, limits: dict[str, tuple[int, int]]):
        super().__init__(profiles)
        self.ser = serial.Serial(port, 19200, timeout=1.0)
        try:
            time.sleep(0.2)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.start = {axis: self.get_position(axis) for axis in AXES}
            self.limits = limits
            for axis in AXES:
                low, high = limits[axis]
                if low > high or not (low <= self.start[axis] <= high):
                    raise ValueError(f"{axis} starts outside its approved limits")
                if self.start[axis] + MAX_OFFSET > high:
                    raise ValueError(f"{axis} needs room for {MAX_OFFSET} forward steps")
            print(f"Verified starting controller positions: {self.start}")
        except BaseException:
            self.close()
            raise

    def get_position(self, axis: str) -> int:
        axis_id = AXIS_IDS[axis]
        for _ in range(3):
            self.ser.reset_input_buffer()
            self.ser.write(f"{axis_id}TP?\r".encode("ascii"))
            self.ser.flush()
            time.sleep(0.05)
            reply = self.ser.read_until(b"\r").decode("ascii", errors="replace").strip()
            match = re.fullmatch(rf"{axis_id}TP\?\s+(-?\d+)", reply)
            if match:
                return int(match.group(1))
        raise TimeoutError(f"{axis} did not return a valid position")

    def move_one_step(self, axis: str, direction: int) -> None:
        if axis not in AXES or direction not in (-1, 1):
            raise ValueError("Only one-step moves on X, Y, or Z are allowed")
        next_offset = self.position[axis] + direction
        if not 0 <= next_offset <= MAX_OFFSET:
            raise ValueError(f"{axis} simulated travel limit reached")
        current = self.start[axis] + self.position[axis]
        actual = self.get_position(axis)
        if actual != current:
            raise RuntimeError(f"{axis} moved unexpectedly: expected {current}, got {actual}")
        target = current + direction
        low, high = self.limits[axis]
        if not low <= target <= high:
            raise ValueError(f"{axis} target {target} exceeds approved limits")
        time.sleep(0.12)
        self.ser.write(f"{AXIS_IDS[axis]}PR{direction}\r".encode("ascii"))
        self.ser.flush()
        time.sleep(0.3)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if self.get_position(axis) == target:
                self.position[axis] = next_offset
                self.moves += 1
                return
            time.sleep(0.05)
        raise TimeoutError(f"{axis} did not reach {target}; motion stopped")

    def close(self) -> None:
        if self.ser.is_open:
            self.ser.close()


class RealMeter:
    """Use the same installed TLPM interop driver as check_meter.py."""

    def __init__(self):
        native_dir = Path(r"C:\Program Files\IVI Foundation\VISA\Win64\Bin")
        interop_dir = Path(r"C:\Program Files\IVI Foundation\VISA\VisaCom64\Primary Interop Assemblies")
        os.environ["PATH"] = str(native_dir) + os.pathsep + os.environ["PATH"]
        os.add_dll_directory(str(native_dir))
        sys.path.insert(0, str(interop_dir))
        import clr
        clr.AddReference("Thorlabs.TLPM_64.Interop")
        from Thorlabs.TLPM_64.Interop import TLPM
        self.instrument = TLPM("USB0::0x1313::0x8072::M01339668::INSTR", True, False)
        try:
            status = self.instrument.setWavelength(WAVELENGTH_NM)
            unit_status, unit = self.instrument.getPowerUnit()
            if status != 0 or unit_status != 0 or unit != 0:
                raise RuntimeError("Failed to configure meter for 1310 nm and watts")
        except BaseException:
            self.close()
            raise

    def read(self) -> tuple[float, float]:
        voltage_status, voltage = self.instrument.measVoltage()
        power_status, power = self.instrument.measPower()
        if voltage_status != 0 or power_status != 0:
            raise RuntimeError(f"Meter error: voltage={hex(voltage_status)}, power={hex(power_status)}")
        if not math.isfinite(voltage) or not math.isfinite(power):
            raise RuntimeError("Meter returned a non-finite measurement")
        return voltage, power

    def close(self) -> None:
        self.instrument.Dispose()


def record_sample(writer, file, stage: SimulatedStage, mode: str, meter: RealMeter | None) -> float:
    if isinstance(stage, RealStage):
        positions = {axis: stage.get_position(axis) for axis in AXES}
        for axis in AXES:
            expected = stage.start[axis] + stage.position[axis]
            if positions[axis] != expected:
                raise RuntimeError(f"{axis} position changed unexpectedly: {positions[axis]} != {expected}")
        voltage, power = meter.read()
        score = power
    else:
        positions = stage.coordinates()
        voltage, power = stage.read_voltage(), None
        score = voltage
    writer.writerow([
        datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
        mode,
        *(positions[axis] for axis in AXES),
        f"{voltage:.12g}",
        f"{power:.12g}" if power is not None else "",
        "measured" if meter is not None else "simulated",
    ])
    file.flush()
    return score


def record_without_movement(port: str, interval: float, output: Path) -> None:
    """Log real stage positions and meter values; issue no move commands."""
    ser = serial.Serial(port, 19200, timeout=1.0)
    meter = None
    try:
        time.sleep(0.2)
        meter = RealMeter()
        new_file = not output.exists() or output.stat().st_size == 0
        with output.open("a", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            if new_file:
                writer.writerow(LOG_HEADER)
            print("Recording real positions, voltage, and power. Press Ctrl+C to stop.")
            try:
                while True:
                    positions = {}
                    for axis in AXES:
                        axis_id = AXIS_IDS[axis]
                        for _ in range(6):
                            ser.reset_input_buffer()
                            ser.write(f"{axis_id}TP?\r".encode("ascii"))
                            ser.flush()
                            time.sleep(0.05)
                            reply = ser.read_until(b"\r").decode("ascii", errors="replace").strip()
                            match = re.fullmatch(rf"{axis_id}TP\?\s+(-?\d+)", reply)
                            if match:
                                positions[axis] = int(match.group(1))
                                break
                        if axis not in positions:
                            print(f"{axis} did not return a valid position; retrying next cycle")
                            break
                    if len(positions) != len(AXES):
                        time.sleep(interval)
                        continue
                    voltage, power = meter.read()
                    writer.writerow([datetime.now(timezone.utc).isoformat(timespec="milliseconds"), "real_stage_read_only", *(positions[axis] for axis in AXES), f"{voltage:.12g}", f"{power:.12g}", "measured"])
                    file.flush()
                    print(f"{positions} | {voltage:.6g} V | {power:.6g} W")
                    time.sleep(interval)
            except KeyboardInterrupt:
                print("\nStopped by Ctrl+C.")
            print(f"Saved table: {output}")
    finally:
        if meter is not None:
            meter.close()
        ser.close()


def live_return(port: str) -> None:
    """Move from the freshly zeroed hard-stop origin to the supplied position."""
    config = json.loads(TRAVEL_CONFIG.read_text(encoding="utf-8"))
    if (not config.get("z_direction_confirmed", False)
            or not config.get("z_physical_motion_verified", False)
            or not config.get("z_extended_clearance_verified", False)):
        raise RuntimeError("Live motion disabled: extended Z clearance must be verified")
    bounds = config["modes"]["fiber_at_coupling"]
    if any(not bounds[axis][0] <= LIVE_TARGET[axis] <= bounds[axis][1] for axis in AXES):
        raise ValueError("A live target is outside the configured travel bounds")
    resuming = LIVE_LOG.exists()

    with serial.Serial(port, 19200, timeout=1.0) as ser:
        time.sleep(0.2)
        start = {axis: read_number(ser, f"{AXIS_IDS[axis]}TP?") for axis in AXES}
        print(f"Controller start counts: {start}", flush=True)
        if not resuming and start != {axis: 0 for axis in AXES}:
            raise RuntimeError("First live return requires the recorded 0,0,0 origin; no move sent")
        if resuming:
            if (start["Z"] > 0 or start["Z"] < LIVE_TARGET["Z"]
                    or not 0 <= start["Y"] <= LIVE_TARGET["Y"]
                    or not 0 <= start["X"] <= LIVE_TARGET["X"]
                    or (start["Y"] != 0 and start["Z"] != LIVE_TARGET["Z"])
                    or (start["X"] != 0 and (start["Y"], start["Z"]) != (LIVE_TARGET["Y"], LIVE_TARGET["Z"]))):
                raise RuntimeError("Current counts do not match the staged return path; no move sent")
            print(f"Resuming from controller counts: {start}", flush=True)
        for axis in AXES:
            status = query(ser, f"{AXIS_IDS[axis]}TS?")
            if status not in (f"{AXIS_IDS[axis]}TS? @", f"{AXIS_IDS[axis]}TS? Q"):
                raise RuntimeError(f"{axis} is not idle: {status}")

        # Controller limits are volatile. Set and verify each before motion.
        for axis in AXES:
            number = AXIS_IDS[axis]
            low, high = bounds[axis]
            for command, value in (("SL", min(low, -1)), ("SR", max(high, 1))):
                send(ser, f"{number}{command}{value}")
                time.sleep(0.2)
                check_error(ser, number, f"set {axis} {command}")
                if read_number(ser, f"{number}{command}?") != value:
                    raise RuntimeError(f"{axis} {command} limit did not read back")

        with LIVE_LOG.open("a" if resuming else "w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            if not resuming:
                writer.writerow(("timestamp_utc", "axis", "before_steps", "commanded_delta_steps", "after_steps"))
            file.flush()
            for axis in LIVE_ORDER:
                number = AXIS_IDS[axis]
                target = LIVE_TARGET[axis]
                current = read_number(ser, f"{number}TP?")
                if current == target:
                    continue
                initial_status = query(ser, f"{number}TS?")
                enabled_here = initial_status == f"{number}TS? @"
                if enabled_here:
                    send(ser, f"{number}MO")
                    time.sleep(0.2)
                    check_error(ser, number, f"enable {axis}")
                if query(ser, f"{number}TS?") != f"{number}TS? Q":
                    raise RuntimeError(f"{axis} motor is not ready")
                try:
                    while current != target:
                        remaining = target - current
                        chunk = min(abs(remaining), 5000 if axis != "X" else 1000)
                        if chunk > 192:
                            chunk = (chunk // 16) * 16
                        delta = chunk if remaining > 0 else -chunk
                        next_count = current + delta
                        low, high = bounds[axis]
                        if not low <= next_count <= high:
                            raise RuntimeError(f"{axis} target {next_count} exceeds {low}..{high}")
                        send(ser, f"{number}PR{delta}")
                        deadline = time.monotonic() + 15.0
                        while time.monotonic() < deadline:
                            time.sleep(0.2)
                            status = query(ser, f"{number}TS?")
                            if status == f"{number}TS? Q":
                                break
                            if status != f"{number}TS? P":
                                raise RuntimeError(f"{axis} unexpected status: {status}")
                        else:
                            raise TimeoutError(f"{axis} motion timed out")
                        actual = read_number(ser, f"{number}TP?")
                        check_error(ser, number, f"move {axis}")
                        if actual != next_count:
                            raise RuntimeError(f"{axis} reported {actual}, expected {next_count}")
                        writer.writerow((datetime.now(timezone.utc).isoformat(timespec="milliseconds"), axis, current, delta, actual))
                        file.flush()
                        current = actual
                        print(f"{axis}: {current}/{target}", flush=True)
                except BaseException:
                    send(ser, f"{number}ST")
                    raise
                finally:
                    if enabled_here:
                        send(ser, f"{number}MF")

        final = {axis: read_number(ser, f"{AXIS_IDS[axis]}TP?") for axis in AXES}
        if final != LIVE_TARGET:
            raise RuntimeError(f"Unexpected final controller counts: {final}")
        print(f"Controller target counts reached: {final}; physical displacement not measured", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true", help="alias for the default live return")
    parser.add_argument("--simulate", action="store_true", help="use synthetic voltage and no hardware")
    parser.add_argument("--record-only", action="store_true", help="record real positions and meter values without moving")
    parser.add_argument("--port", default="COM4")
    parser.add_argument("--output", type=Path, default=DEFAULT_LOG)
    parser.add_argument("--interval", type=float, default=5.0, help="seconds between axis searches")
    parser.add_argument("--cycles", type=int, default=1, help="number of X/Y/Z search passes; default is one")
    for axis in AXES:
        parser.add_argument(f"--{axis.lower()}-limits", type=int, nargs=2, metavar=("MIN", "MAX"))
    args = parser.parse_args()
    if not math.isfinite(args.interval) or args.interval < 0.1:
        parser.error("--interval must be at least 0.1 seconds")
    if args.cycles < 1:
        parser.error("--cycles must be at least 1")
    if args.run and args.record_only:
        parser.error("Choose either --run or --record-only")
    if args.simulate and (args.run or args.record_only):
        parser.error("--simulate cannot be combined with live modes")
    if args.record_only:
        record_without_movement(args.port, args.interval, args.output)
        return
    if not args.simulate:
        live_return(args.port)
        return
    limits = {axis: getattr(args, f"{axis.lower()}_limits") for axis in AXES}
    if args.run and any(value is None for value in limits.values()):
        parser.error("Real movement requires --x-limits, --y-limits, and --z-limits")
    profiles = load_profiles(DATA_FILE)
    if args.run:
        approved = json.loads(TRAVEL_CONFIG.read_text(encoding="utf-8"))["modes"]["fiber_at_coupling"]
        for axis in AXES:
            low, high = limits[axis]
            safe_low, safe_high = approved[axis]
            if not safe_low <= low <= high <= safe_high:
                parser.error(f"{axis} limits {low}..{high} exceed configured fiber-present bounds {safe_low}..{safe_high}")
    stage = RealStage(profiles, args.port, limits) if args.run else SimulatedStage(profiles)
    try:
        meter = RealMeter() if args.run else None
    except BaseException:
        if args.run:
            stage.close()
        raise
    if not args.run:
        print("SIMULATION ONLY: no motor or meter connection")
        print(f"Synthetic coordinates: {SIMULATION_START} toward {SIMULATION_PEAK}")
    print(f"One controller count per move; local search offsets 0..{MAX_OFFSET} per axis")
    if args.run:
        print(f"Real PM100USB measurements at {WAVELENGTH_NM:g} nm; search uses optical power")
    else:
        print(f"Starting voltage: {stage.read_voltage():.3f} V")
    mode = "real_stage" if args.run else "simulated_stage"
    try:
        new_file = not args.output.exists() or args.output.stat().st_size == 0
        with args.output.open("a", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            if new_file:
                writer.writerow(LOG_HEADER)
            current_score = record_sample(writer, file, stage, mode, meter)
            print(f"Searching for {args.cycles} X/Y/Z pass(es). Press Ctrl+C to stop.")
            try:
                for _ in range(args.cycles):
                    for axis in AXES:
                        best_score = current_score
                        best_position = stage.position[axis]
                        print(f"\nSearching {axis}:")
                        for _ in range(MAX_OFFSET - stage.position[axis]):
                            stage.move_one_step(axis, 1)
                            score = record_sample(writer, file, stage, mode, meter)
                            unit = "W" if args.run else "V"
                            display_position = stage.start[axis] + stage.position[axis] if args.run else stage.coordinates()[axis]
                            print(f"  {axis}={display_position}: {score:.6g} {unit}")
                            if score > best_score:
                                best_score = score
                                best_position = stage.position[axis]
                                current_score = score
                            else:
                                while stage.position[axis] > best_position:
                                    stage.move_one_step(axis, -1)
                                    current_score = record_sample(writer, file, stage, mode, meter)
                                best_coordinate = stage.start[axis] + best_position if args.run else SIMULATION_START[axis] + best_position
                                print(f"  Signal fell; returned to {axis}={best_coordinate}")
                                break
                        best_coordinate = stage.start[axis] + best_position if args.run else SIMULATION_START[axis] + best_position
                        print(f"  Best {axis}={best_coordinate}: {best_score:.6g} {'W' if args.run else 'V'}")
                        time.sleep(args.interval)
            except KeyboardInterrupt:
                print("\nStopped by Ctrl+C.")
            print(f"Final coordinates: { {axis: stage.start[axis] + stage.position[axis] for axis in AXES} if args.run else stage.coordinates() }")
            print(f"One-step moves: {stage.moves}")
            print(f"Saved table: {args.output}")
    finally:
        if meter is not None:
            meter.close()
        if args.run:
            stage.close()


if __name__ == "__main__":
    main()
