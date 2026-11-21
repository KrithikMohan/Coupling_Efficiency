import argparse
import csv
import math
import os
import statistics
import time
from datetime import datetime, timezone
from pathlib import Path
import serial

from check_meter import ThorlabsPM100
from verify_one_step import check_error, query, read_number, send

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

POLL_INTERVAL_SEC: float = 0.5
BOUNDS = {"X": (-1, 61450), "Y": (-1, None), "Z": (-1, 150000)}
INITIAL_STEPS = {"X": 30000, "Y": 100000, "Z": 100000}
MAX_MOVE_COUNTS = {"X": 50000, "Y": 100000, "Z": 100000}
SEARCH_ORDER = ("Y", "X")
SEARCH_BOUNDS = {"X": (1000, 61450)}
LOG_COLUMNS = ("time_utc", "event", "axis", "delta", "x", "y", "z", "voltage_v", "power_w", "efficiency_percent")


# ==============================================================================
# THORLABS PM100USB INTERFACE
# ==============================================================================
ThorlabsPM100USB = ThorlabsPM100


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
        return query(self.ser, cmd)

    def get_position(self, axis_label: str) -> int:
        command = f"{AXIS_MAP[axis_label]}TP?"
        for _ in range(3):
            try:
                return read_number(self.ser, command)
            except RuntimeError as error:
                last_error = error
        raise last_error

    def positions(self) -> dict[str, int]:
        return {axis: self.get_position(axis) for axis in AXIS_MAP}

    def get_limits(self, axis: str) -> tuple[int, int]:
        number = AXIS_MAP[axis]
        controller_low = read_number(self.ser, f"{number}SL?")
        controller_high = read_number(self.ser, f"{number}SR?")
        if axis in ("X", "Z") and (controller_low, controller_high) != BOUNDS[axis]:
            raise RuntimeError(f"{axis} controller limits changed to {controller_low}..{controller_high}")
        if controller_low > -1 or controller_high <= 0:
            raise RuntimeError(f"{axis} controller limits {controller_low}..{controller_high} exclude origin travel")
        return max(-1, controller_low), controller_high

    def origin_candidate(self) -> dict[str, int]:
        """Read positions and verify every controller is idle before re-referencing."""
        positions = self.positions()
        for axis, number in AXIS_MAP.items():
            status = query(self.ser, f"{number}TS?")
            identity = query(self.ser, f"{number}ID?")
            if status not in (f"{number}TS? @", f"{number}TS? Q") or identity != f"{number}ID? PZA12":
                raise RuntimeError(f"{axis}: status={status}, identity={identity}; origin unchanged")
        return positions

    def establish_origin(self, expected: dict[str, int]) -> dict[str, int]:
        """Set the user-confirmed current physical position to zero without motion."""
        if self.origin_candidate() != expected:
            raise RuntimeError("Controller positions changed before origin selection; no origin command sent")
        for axis, number in AXIS_MAP.items():
            if self.get_position(axis) != expected[axis]:
                raise RuntimeError(f"{axis} changed before origin selection")
            send(self.ser, f"{number}OR")
            time.sleep(0.2)
            actual = self.get_position(axis)
            error = read_number(self.ser, f"{number}TE?")
            if actual != 0 or error not in (0, 6):
                raise RuntimeError(f"{axis} origin command sent; readback={actual}, error={error}")
            print(f"ORIGIN {axis}: {expected[axis]} -> 0 (no motion)", flush=True)
        for axis, number in AXIS_MAP.items():
            if axis in ("X", "Z"):
                for command, value in (("SL", -1), ("SR", BOUNDS[axis][1])):
                    send(self.ser, f"{number}{command}{value}")
                    check_error(self.ser, number, f"set {axis} {command}")
                    if read_number(self.ser, f"{number}{command}?") != value:
                        raise RuntimeError(f"{axis} {command} limit readback failed")
            low, high = self.get_limits(axis)
            print(f"LIMITS {axis}: {low}..{high} (controller readback)", flush=True)
        after = self.preflight()
        if any(value != 0 for value in after.values()):
            raise RuntimeError(f"Origin selection did not hold: {after}")
        return after

    def preflight(self) -> dict[str, int]:
        positions = self.positions()
        for axis, number in AXIS_MAP.items():
            low, high = self.get_limits(axis)
            status = query(self.ser, f"{number}TS?")
            if not low <= positions[axis] <= high:
                raise RuntimeError(f"{axis} count {positions[axis]} is outside {low}..{high}")
            if status not in (f"{number}TS? @", f"{number}TS? Q"):
                raise RuntimeError(f"{axis} is not idle: {status}")
        return positions

    def move_to(self, axis: str, target: int) -> int:
        low, high = self.get_limits(axis)
        if not low <= target <= high:
            raise ValueError(f"{axis} target {target} exceeds {low}..{high}")
        number = AXIS_MAP[axis]
        current = self.get_position(axis)
        if not low <= current <= high:
            raise RuntimeError(f"{axis} starts outside its bounds at {current}")
        if current == target:
            return current
        status = query(self.ser, f"{number}TS?")
        if status not in (f"{number}TS? @", f"{number}TS? Q"):
            raise RuntimeError(f"{axis} is not idle: {status}")
        enabled_here = status == f"{number}TS? @"
        try:
            if enabled_here:
                send(self.ser, f"{number}MO")
                check_error(self.ser, number, f"enable {axis}")
            if query(self.ser, f"{number}TS?") != f"{number}TS? Q":
                raise RuntimeError(f"{axis} did not become ready")
            for _ in range(20):
                remaining = target - current
                if abs(remaining) <= 16:
                    return current
                controller_limits = self.get_limits(axis)
                if controller_limits != (low, high):
                    raise RuntimeError(f"{axis} limits changed to {controller_limits}; no further move sent")
                size = min(abs(remaining), MAX_MOVE_COUNTS[axis])
                if size > 16:
                    size = (size // 16) * 16
                delta = size if remaining > 0 else -size
                expected = current + delta
                if not low <= expected <= high:
                    raise RuntimeError(f"{axis} next count {expected} exceeds {low}..{high}")
                print(f"MOVING {axis}: initial={current}, commanded_target={expected}, "
                      f"delta={delta}", flush=True)
                send(self.ser, f"{number}PR{delta}")
                # Leave the RS-485 bus quiet during travel; rapid polling caused error 50.
                time.sleep(max(0.3, abs(delta) / 48000 + 0.3))
                deadline = time.monotonic() + 30.0
                while time.monotonic() < deadline:
                    try:
                        motion_status = query(self.ser, f"{number}TS?")
                    except RuntimeError:
                        time.sleep(1.0)
                        continue
                    if motion_status == f"{number}TS? Q":
                        break
                    if motion_status != f"{number}TS? P":
                        raise RuntimeError(f"{axis} unexpected status: {motion_status}")
                    time.sleep(1.0)
                else:
                    raise TimeoutError(f"{axis} did not become idle")
                actual = self.get_position(axis)
                check_error(self.ser, number, f"move {axis}")
                if (not low <= actual <= high or abs(actual - expected) > 16
                        or (actual - current) * delta <= 0):
                    raise RuntimeError(f"{axis} reported {actual}, expected near {expected}")
                print(f"MOVED {axis}: initial={current}, final={actual}", flush=True)
                current = actual
            raise RuntimeError(f"{axis} did not reach target {target} within 20 bounded moves")
        except BaseException:
            send(self.ser, f"{number}ST")
            raise
        finally:
            if enabled_here:
                send(self.ser, f"{number}MF")

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


def efficiency_percent(power_w: float, dark_w: float) -> float:
    """Project calibration: 10 uW input and 86 percent path throughput."""
    return 100.0 * math.sqrt(max(0.0, power_w - dark_w) / (10e-6 * 0.86))


def search_axis(stage, axis: str, measure, first_step: int, min_step: int,
                min_improvement_w: float, target_percent: float,
                move_budget: list[int], best_power: float,
                search_bounds: dict[str, tuple[int, int]] | None = None) -> tuple[float, bool]:
    """Walk toward rising power; recover the last peak after a drop."""
    step = first_step
    while step >= min_step:
        improved_this_step = False
        for direction in (1, -1):
            while True:
                current = stage.get_position(axis)
                low, high = (search_bounds or BOUNDS)[axis]
                candidate = max(low, min(high, current + direction * step))
                if candidate == current:
                    break
                if move_budget[0] <= 0:
                    raise RuntimeError("Move budget exhausted; stopping at the last measured position")
                move_budget[0] -= 1
                actual = stage.move_to(axis, candidate)
                power, efficiency = measure("probe", axis, actual - current)
                if efficiency is not None and efficiency >= target_percent:
                    return power, True
                if power > best_power + min_improvement_w:
                    best_power = power
                    improved_this_step = True
                    continue
                if move_budget[0] <= 0:
                    raise RuntimeError("Move budget exhausted before peak recovery")
                move_budget[0] -= 1
                recovered = stage.move_to(axis, current)
                best_power, efficiency = measure("recover_peak", axis, recovered - actual)
                if efficiency is not None and efficiency >= target_percent:
                    return best_power, True
                break
            if improved_this_step:
                break
        step //= 2
    return best_power, False


def run_alignment(args) -> None:
    if args.dark_power_w is not None and (args.dark_power_w < 0 or not math.isfinite(args.dark_power_w)):
        raise ValueError("--dark-power-w must be measured, finite, and nonnegative")
    if args.samples < 1 or args.min_step < 1 or args.max_moves < 1:
        raise ValueError("Samples, minimum step, and move budget must be positive")
    if not 0 < args.z_sweep_step <= MAX_MOVE_COUNTS["Z"]:
        raise ValueError(f"--z-sweep-step must be within 1..{MAX_MOVE_COUNTS['Z']}")
    if not math.isfinite(args.signal_rise_v) or args.signal_rise_v <= 0:
        raise ValueError("--signal-rise-v must be finite and positive")
    if not 0 < args.target_efficiency <= 100:
        raise ValueError("Target efficiency must be within 0..100 percent")
    if args.min_improvement_v < 0 or not math.isfinite(args.min_improvement_v):
        raise ValueError("Minimum voltage improvement must be finite and nonnegative")
    steps = {"X": args.x_step, "Y": args.y_step, "Z": args.z_step}
    if any(step < args.min_step for step in steps.values()):
        raise ValueError("Initial steps must be at least --min-step")

    meter = ThorlabsPM100USB()
    stage = None
    try:
        resource = meter.connect()
        meter.set_wavelength(1310.0)
        stage = PZC200Stage()
        positions = stage.origin_candidate()
        print(f"Meter: {resource}; 1310 nm; measured voltage drives the search")
        if args.dark_power_w is None:
            print("No blocked-light baseline supplied: efficiency will not be calculated")
        else:
            print(f"Dark baseline: {args.dark_power_w:.6e} W")
        print(f"INITIAL CONTROLLER POSITIONS: {positions}")
        print("Selecting the current physical stage placement as X/Y/Z origin")
        print(f"X cap: {BOUNDS['X'][1]}; Z cap: {BOUNDS['Z'][1]}; "
              "Y uses its live controller limits")
        print(f"Z acquisition: move upward in {args.z_sweep_step}-count steps "
              "to the Z cap, then search Y and X")
        print(f"At the Z cap: scan Y, step X, and scan Y back; "
              f"then refine {SEARCH_ORDER}; first moves: {steps}")
        print("Controller counts do not measure physical displacement.")

        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(LOG_COLUMNS)
            file.flush()
            writer.writerow((datetime.now(timezone.utc).isoformat(), "origin_before", "", "",
                             positions["X"], positions["Y"], positions["Z"], "", "", ""))
            file.flush()
            zeroed = stage.establish_origin(positions)
            live_bounds = {axis: stage.get_limits(axis) for axis in AXIS_MAP}
            print(f"Live controller travel bounds: {live_bounds}", flush=True)
            writer.writerow((datetime.now(timezone.utc).isoformat(), "origin_selected", "", "",
                             zeroed["X"], zeroed["Y"], zeroed["Z"], "", "", ""))
            file.flush()

            def measure(event: str, axis: str = "", delta: int = 0) -> tuple[float, float]:
                voltages = []
                powers = []
                for _ in range(args.samples):
                    voltage = meter.read_voltage()
                    power = meter.read_power()
                    if not math.isfinite(voltage) or not math.isfinite(power):
                        raise RuntimeError(f"Non-finite meter readings: voltage={voltage}, power={power}")
                    voltages.append(voltage)
                    powers.append(power)
                    time.sleep(0.05)
                voltage = statistics.median(voltages)
                power = statistics.median(powers)
                efficiency = (efficiency_percent(power, args.dark_power_w)
                              if args.dark_power_w is not None else None)
                efficiency_text = f"{efficiency:.2f}%" if efficiency is not None else "uncalibrated"
                counts = stage.positions()
                writer.writerow((datetime.now(timezone.utc).isoformat(), event, axis, delta,
                                 counts["X"], counts["Y"], counts["Z"], voltage, power, efficiency))
                file.flush()
                if axis:
                    before = counts[axis] - delta
                    print(f"MEASURED {axis}: initial={before}, final={counts[axis]}, "
                          f"voltage={voltage:.6g} V, power={format_power(power)}, "
                          f"efficiency={efficiency_text}", flush=True)
                else:
                    print(f"MEASURED {event}: positions={counts}, voltage={voltage:.6g} V, "
                          f"power={format_power(power)}, efficiency={efficiency_text}", flush=True)
                return voltage, efficiency

            baseline_voltage, _ = measure("origin_baseline")
            threshold = baseline_voltage + args.signal_rise_v
            print(f"Substantial signal threshold: {threshold:.6g} V "
                  f"(baseline {baseline_voltage:.6g} V + rise {args.signal_rise_v:.6g} V)", flush=True)
            best_voltage = baseline_voltage
            budget = [args.max_moves]
            signal_found = False
            while stage.get_position("Z") < stage.get_limits("Z")[1] - 16:
                before = stage.get_position("Z")
                target = min(stage.get_limits("Z")[1], before + args.z_sweep_step)
                actual = stage.move_to("Z", target)
                if actual <= before:
                    raise RuntimeError(f"Z made no progress from {before}")
                voltage, efficiency = measure("z_acquire", "Z", actual - before)
                best_voltage = voltage
                if efficiency is not None and efficiency >= args.target_efficiency:
                    print("Efficiency target reached during Z acquisition.", flush=True)
                    return
                signal_found = voltage >= threshold
            if not signal_found:
                print(f"No substantial signal through Z={stage.get_position('Z')} "
                      f"(controller limit {stage.get_limits('Z')[1]}); "
                      "scanning Y and X.", flush=True)
            else:
                print(f"Substantial signal at Z={stage.get_position('Z')}; "
                      "refining Y and X.", flush=True)
            search_bounds = {"X": SEARCH_BOUNDS["X"],
                             "Y": stage.get_limits("Y"),
                             "Z": stage.get_limits("Z")}
            if not signal_found:
                y_direction = 1
                while budget[0] > 0:
                    y_before = stage.get_position("Y")
                    y_low, y_high = stage.get_limits("Y")
                    y_target = max(y_low, min(y_high, y_before + y_direction * steps["Y"]))
                    if abs(y_target - y_before) > 16:
                        budget[0] -= 1
                        y_actual = stage.move_to("Y", y_target)
                        if y_actual == y_before:
                            raise RuntimeError("Y scan made no progress")
                        voltage, efficiency = measure("y_scan", "Y", y_actual - y_before)
                        best_voltage = max(best_voltage, voltage)
                        if efficiency is not None and efficiency >= args.target_efficiency:
                            print(f"Target reached on Y. Final controller counts: {stage.positions()}")
                            return
                        if voltage >= threshold:
                            signal_found = True
                            print(f"Substantial signal found at Y={y_actual}; refining Y and X.", flush=True)
                            break
                        continue
                    x_before = stage.get_position("X")
                    x_target = min(search_bounds["X"][1], x_before + steps["X"])
                    if x_target - x_before <= 16:
                        break
                    budget[0] -= 1
                    x_actual = stage.move_to("X", x_target)
                    if x_actual == x_before:
                        raise RuntimeError("X scan made no progress")
                    voltage, efficiency = measure("x_scan", "X", x_actual - x_before)
                    best_voltage = max(best_voltage, voltage)
                    if efficiency is not None and efficiency >= args.target_efficiency:
                        print(f"Target reached on X. Final controller counts: {stage.positions()}")
                        return
                    if voltage >= threshold:
                        signal_found = True
                        print(f"Substantial signal found at X={x_actual}; refining Y and X.", flush=True)
                        break
                    y_direction *= -1
            if signal_found:
                min_improvement = max(args.min_improvement_v, args.signal_rise_v / 4)
                while budget[0] > 0:
                    cycle_start = best_voltage
                    for axis in SEARCH_ORDER:
                        best_voltage, reached = search_axis(stage, axis, measure, steps[axis],
                                                             args.min_step, min_improvement,
                                                             args.target_efficiency, budget, best_voltage,
                                                             search_bounds)
                        if reached:
                            print(f"Target reached on {axis}. Final controller counts: {stage.positions()}")
                            return
                    if best_voltage <= cycle_start + min_improvement:
                        break
            print(f"Search finished. Final controller counts: {stage.positions()}; "
                  f"best voltage={best_voltage:.6g} V; moves remaining={budget[0]}")
            print(f"Log: {args.output}")
    finally:
        if stage is not None:
            try:
                print(f"FINAL POSITIONS: {stage.positions()}", flush=True)
            except Exception as error:
                print(f"FINAL POSITIONS unavailable: {error}", flush=True)
            stage.close()
        meter.close()


# ==============================================================================
# PASSIVE MONITOR EXECUTION
# ==============================================================================
def monitor():
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
    parser = argparse.ArgumentParser(description="Run meter-guided alignment, or explicitly select passive monitoring")
    parser.add_argument("--run", action="store_true", help="run alignment (the default)")
    parser.add_argument("--monitor", action="store_true", help="read positions and meter continuously without moving")
    parser.add_argument("--dark-power-w", type=float, help="measured blocked-light power in watts; enables efficiency calculation")
    parser.add_argument("--z-sweep-step", type=int, default=50000,
                        help="positive Z counts per acquisition move from origin")
    parser.add_argument("--signal-rise-v", type=float, default=1e-8,
                        help="voltage rise above origin baseline required before peak search")
    parser.add_argument("--target-efficiency", type=float, default=65.0)
    parser.add_argument("--x-step", type=int, default=INITIAL_STEPS["X"])
    parser.add_argument("--y-step", type=int, default=INITIAL_STEPS["Y"])
    parser.add_argument("--z-step", type=int, default=INITIAL_STEPS["Z"])
    parser.add_argument("--min-step", type=int, default=16000)
    parser.add_argument("--min-improvement-v", type=float, default=2e-10)
    parser.add_argument("--samples", type=int, default=3)
    parser.add_argument("--max-moves", type=int, default=60)
    parser.add_argument("--output", type=Path, default=Path(__file__).with_name(
        f"coupling_search_{datetime.now(timezone.utc):%Y%m%d_%H%M%S}.csv"))
    arguments = parser.parse_args()
    if arguments.run and arguments.monitor:
        parser.error("Choose --run or --monitor")
    if arguments.monitor:
        monitor()
    else:
        run_alignment(arguments)
