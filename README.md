# Automated Fiber-to-Stage Alignment System

Thorlabs PM100USB + S132C power acquisition and Newport PZC200 stage control. Run each phase separately and review its result before proceeding. Nothing automatically advances to the next phase.

## Setup

Use Windows 10/11 and 64-bit Python 3.10+. Install Python packages with `pip install -r requirements.txt`. Install Thorlabs OPM for the USB driver and TLPM_64.dll, and close the OPM desktop application before Python accesses the meter.

## Four phases

| Phase | Script | Behavior |
| --- | --- | --- |
| 1 | `check_meter.py` | Connects only to the meter; prints five measured voltage and power readings. |
| 2 | `motor_check.py` | Connects only to the controller; queries X/Y/Z counts without movement. |
| 3 | `alignment_routine.py --monitor` | Passive position-and-power monitor with no movement. |
| 4 | `alignment_routine.py` | Live meter-guided coupling search. |

```powershell
python check_meter.py
python motor_check.py
python alignment_routine.py --monitor
```

`motor_check.py` reads all three controller positions on COM4. `alignment_routine.py` moves motors by default; use `--monitor` for a passive position-and-power display. Commands report failures through exceptions and nonzero exit status.

## Real-motor fake-voltage demonstration

`python "Simulate motor movement.py" --run` moves the **real** X/Y/Z stages using a deterministic **fake voltage** response. It does not open the power meter. It requires all three controller counts within 16 of the physical origin, and verifies the saved controller limits before any move. The search starts with Z and Y moves of 100000 counts and an X move of 50000 counts, then probes and reverses according to the synthetic signal. Every reported count and fake voltage is recorded in `fake_voltage_live_log.csv`. `--simulate` remains a software-only stage simulation and sends no motor commands. Neither mode measures coupling efficiency.

For physical coupling, place all three stages at their confirmed physical origin, then run:

```powershell
python alignment_routine.py
```

The routine reads all three controller counts and idle states. It sets that physical placement as the X/Y/Z controller origin without moving, installs and reads back the X software limits (`-1..61450`) and Z software limits (`-1..150000`), and leaves Y controller limits unchanged. It logs the before/after origin counts. It measures the origin voltage, then moves Z upward in 50000-count increments (configurable with `--z-sweep-step`, with at most 100000 counts per command). At Z 150000, it scans Y in both directions, stepping X between Y passes if no substantial signal appears. Y uses its live controller limits without a fixed alignment cap. Once a signal is found, it alternates Y and X refinements using real voltage changes until it reaches the calibrated efficiency target, finds no further improvement, or uses the move budget. Every command is limited to 50000 X counts or 100000 Y/Z counts. It prints each moving axis's initial and final counts and the measured voltage and power. The CSV log contains all measured voltage, power, and positions.

To calculate efficiency and stop at the default 65 percent target, supply a measured blocked-light baseline: `--dark-power-w DARK_POWER_IN_WATTS`. Without it, the routine still searches measured voltage but labels efficiency uncalibrated. Controller counts are open-loop and do not measure physical displacement. A controller or meter error aborts the run without an automatic return.

## Four-file layout

`check_meter.py` provides the shared PM100USB voltage/power reader and its five-sample diagnostic. `motor_check.py` reads stage positions. `alignment_routine.py` imports the meter reader from `check_meter.py`, reads live controller counts, and prints each axis's commanded start and final count plus the measured voltage and power after the move.

## DLL discovery

The meter reader uses the installed Thorlabs .NET interop driver and the configured PM100USB resource in `check_meter.py`. Close the Thorlabs desktop monitor before Python opens the meter.

## Hardware state and verification

The real-motor fake-voltage demonstration completed X/Y/Z probe and recovery moves and logged all controller counts. The live meter diagnostic has returned both voltage and power. These checks do not establish optical coupling or measure physical displacement independently of the open-loop controller.

## Optical Coupling & Alignment Specifications

### 1. Positioning Tolerances & Step Resolution
* **Working Clearance:** The X-axis must maintain a clearance of at most 14 nm from the lensed fiber facet to avoid physical collision while staying within the coupling waist.
* **Search move size:** One search move is a substantial relative `PR` command, not `PR1`. The current defaults start at 30000 X counts and 100000 Y/Z counts, then reduce the size near a measured peak.
* **Target Efficiency:** Alignment terminates once coupling efficiency reaches **65% to 70%**.

---

### 2. Coupling Efficiency Calculation
Coupling efficiency ($\eta$) is calculated from the measured optical power relative to the calibrated input power and splitter throughput:

$$\eta = \sqrt{\frac{P_{\text{measured}} - P_{\text{dark}}}{10\,\mu\text{W} \times 0.86}} \times 100\%$$

Where:
* $P_{\text{measured}}$: Real-time optical power sampled from the Thorlabs PM100USB.
* $P_{\text{dark}}$: Calibrated dark noise floor / electronic offset (baseline reading with beam blocked).
* $10\,\mu\text{W} \times 0.86$: Input reference power adjusted for the 86% transmission factor of the optical path / coupler split.

---

### 3. Peak Tracking & Direction Reversal Logic
* **Asymmetric Signal Profile:** Optical coupling curves exhibit non-linear rises and sharp drop-offs due to spatial mode overlap and mechanical hysteresis.
* **Peak Detection:**
  1. The stage makes one bounded search move and samples the power.
  2. If efficiency increases, the optimizer continues in that direction.
  3. When the signal falls, it reverses direction.
  4. It measures again at the recovered peak before changing axis or step size.

### Current stage origin and coupling-neighborhood simulation

The alignment code caps X at `61450` and Z at `150000`. Y uses the controller limits already present on the hardware; the program does not enlarge them. Z was rezeroed at the user-confirmed low stop, and positive Z commands raise it. Historical negative-Z coordinates in `stage_travel_config.json` use an obsolete origin and direction assumption. Do not use those historical targets for live coupling.
