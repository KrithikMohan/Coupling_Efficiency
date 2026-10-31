import math
import os
import random
import sys
import time
import serial

# ==============================================================================
# CONFIGURATION & PARAMETERS
# ==============================================================================
PORT: str = os.getenv("STAGE_PORT", "COM4")
BAUDRATE: int = 19200

AXIS_MAP: dict[str, int] = {
    "X": 1,
    "Y": 2,
    "Z": 3,
}

# Software safety travel limits relative to starting origin (+/- steps)
MAX_DISPLACEMENT: int = 1500

# Optimization parameters
STEP_SIZE: int = 25            # Steps per evaluation move (~0.8 to 1.0 µm)
CONVERGENCE_VOLTAGE: float = 4.65  # Target threshold (V)


# ==============================================================================
# SYNTHETIC VOLTAGE GENERATOR
# ==============================================================================
class SyntheticOpticalDetector:
    """Simulates a photodiode response over a 3D Gaussian profile."""

    def __init__(self, start_pos: dict[str, int]):
        # Offset target peak slightly from start positions (+5 to +9 steps away)
        self.target = {
            "X": start_pos["X"] + 175,
            "Y": start_pos["Y"] + 200,
            "Z": start_pos["Z"] + 150,
        }
        self.v_baseline = 0.025   # Dark noise floor (~25 mV)
        self.v_peak = 4.850       # Maximum coupled signal (~4.85 V)
        self.waist = 280.0        # Beam waist parameter in steps

    def read_voltage(self, pos: dict[str, int]) -> float:
        dist_sq = sum((pos[ax] - self.target[ax]) ** 2 for ax in AXIS_MAP)
        signal = self.v_peak * math.exp(-dist_sq / (2.0 * (self.waist ** 2)))
        noise = random.gauss(0.0, 0.012)
        return max(0.001, self.v_baseline + signal + noise)


# ==============================================================================
# HARDWARE CONTROLLER (SAFE ABSOLUTE POSITIONING)
# ==============================================================================
class StageController:
    def __init__(self, port: str = PORT, baudrate: int = BAUDRATE):
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
        """Sends an ASCII command and reads back with RS-485 bus pacing."""
        time.sleep(0.06)
        self.ser.reset_input_buffer()
        self.ser.write(f"{cmd}\r".encode("ascii"))
        self.ser.flush()
        time.sleep(0.06)
        return self.ser.read_until(b"\r").decode("ascii", errors="replace").strip()

    def _send(self, cmd: str):
        """Sends command without blocking for an ASCII reply."""
        time.sleep(0.06)
        self.ser.reset_input_buffer()
        self.ser.write(f"{cmd}\r".encode("ascii"))
        self.ser.flush()
        time.sleep(0.06)

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
            time.sleep(0.08)

        raise TimeoutError(f"Axis {axis_label} (ID {axis_id}) failed to report position.")

    def move_to(self, axis_label: str, target_steps: int):
        """Moves stage using PA (Position Absolute) to avoid negative PR firmware parser bugs."""
        axis_id = AXIS_MAP[axis_label]
        self._send(f"{axis_id}PA{target_steps}")
        # Allow piezo pulse train to complete (~15 ms per 25 steps)
        time.sleep(0.12)

    def close(self):
        if self.ser.is_open:
            self.ser.close()


# ==============================================================================
# ALIGNMENT ROUTINE
# ==============================================================================
def run_alignment():
    print(f"Connecting to Newport PZC200 on {PORT}...")
    stage = StageController()

    try:
        # RS-485 line warm-up
        _ = stage._query("1TP?")

        # 1. Read hardware baseline coordinates
        start_pos: dict[str, int] = {}
        for ax in ["X", "Y", "Z"]:
            start_pos[ax] = stage.get_position(ax)
            print(f"  {ax}-Axis Initial Position: {start_pos[ax]} steps")

        current_pos = dict(start_pos)
        detector = SyntheticOpticalDetector(start_pos)

        # Baseline voltage measurement
        v_current = detector.read_voltage(current_pos)
        best_v = v_current

        print("\nStarting hill-climbing search with synthetic voltage feedback.")
        print(f"Safety travel envelope: +/- {MAX_DISPLACEMENT} steps.")
        print("Press Ctrl+C at any time to halt safely.\n")

        # Direction tracking per axis (+1 = forward, -1 = backward)
        direction = {"X": 1, "Y": 1, "Z": 1}
        iteration = 0

        while iteration < 200:
            iteration += 1

            for ax in ["X", "Y", "Z"]:
                # Boundary safety check
                disp = current_pos[ax] - start_pos[ax]
                if disp >= MAX_DISPLACEMENT:
                    direction[ax] = -1
                elif disp <= -MAX_DISPLACEMENT:
                    direction[ax] = 1

                step = direction[ax] * STEP_SIZE
                target_pos = current_pos[ax] + step

                # 1. Physically step the actuator
                stage.move_to(ax, target_pos)
                current_pos[ax] = target_pos

                # 2. Evaluate simulated optical power
                v_new = detector.read_voltage(current_pos)

                # 3. Gradient Ascent Logic
                if v_new > v_current:
                    # Improvement: continue moving in this direction
                    v_current = v_new
                    if v_current > best_v:
                        best_v = v_current
                else:
                    # Signal dropped: revert step and reverse search direction
                    retreat_pos = current_pos[ax] - step
                    stage.move_to(ax, retreat_pos)
                    current_pos[ax] = retreat_pos

                    direction[ax] *= -1
                    v_current = detector.read_voltage(current_pos)

                # Console telemetry
                pos_str = " | ".join(
                    f"{a}: {current_pos[a]:>6d} (Δ {current_pos[a] - start_pos[a]:+5d})"
                    for a in ["X", "Y", "Z"]
                )
                print(
                    f"[{iteration:03d}] {pos_str} -> "
                    f"V: {v_current:6.4f} V | Peak: {best_v:6.4f} V"
                )

            # Check convergence
            if best_v >= CONVERGENCE_VOLTAGE:
                print(f"\nAlignment converged! Reached target peak: {best_v:.4f} V")
                break

    except KeyboardInterrupt:
        print("\nInterrupted by user (Ctrl+C). Halting motion safely...")
    finally:
        stage.close()
        print("COM port closed safely.")


if __name__ == "__main__":
    run_alignment()