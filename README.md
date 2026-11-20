# Automated Fiber-to-Stage Alignment System

Thorlabs PM100USB + S132C power acquisition and Newport PZC200 stage control. Run each phase separately and review its result before proceeding. Nothing automatically advances to the next phase.

## Setup

Use Windows 10/11 and 64-bit Python 3.10+. Install Python packages with `pip install -r requirements.txt`. Install Thorlabs OPM for the USB driver and TLPM_64.dll, and close the OPM desktop application before Python accesses the meter.

## Four phases

| Phase | Script | Behavior |
| --- | --- | --- |
| 1 | `check_meter.py` | Connects only to the meter; configures wavelength and prints five finite power readings. |
| 2 | `motor_check.py` | Connects only to the controller; queries selected axis positions without movement. |
| 3 | `combined_check.py` | Opens both interfaces and samples position and power without movement. |
| 4 | `alignment_routine.py` | Explicitly enabled raster scan, then return to the highest measured power position. |

```powershell
python check_meter.py
python motor_check.py --port COM3 --axes 1
python combined_check.py --port COM3 --axes 1 2
```

Use `--axes 1 2 3` only once the multi-axis cables are ready. Phase 1 accepts `--dll-path`, `--wavelength`, and `--samples`. Phase 3 accepts those options too. Commands report failures through exceptions and nonzero exit status; a diagnostic PASS means the stated checks passed, not that optical calibration or motion has been verified.

During the cable wait:

```powershell
python motor_check.py --mock
python combined_check.py --mock-stage
python alignment_routine.py --run
```

The last two still use a **real meter**. A mock stage does not produce a simulated optical response and cannot verify physical alignment. Running alignment without arguments opens no hardware and performs no scan.

For physical alignment, first review successful physical diagnostics, verify the controller's absolute-move and motion-status commands, and establish suitable axis travel bounds in controller steps. Start with `python alignment_routine.py --run --live-stage --port COM3 --x-limits MIN MAX --y-limits MIN MAX`, replacing MIN/MAX with actual integer bounds for each axis. Physical movement also requires typing `ALIGN`. The default scan spans 80 steps with spacing of 20; use `--span` and `--step-size` to change it. Values are controller steps, not calibrated micrometres.

The three diagnostics deliberately issue no movement commands. They do not validate motion, optical coupling, or safe travel bounds. Absolute movement (`PA`) and motion completion (`MD?`) remain unverified on the physical multi-axis setup. Motion polling is bounded and unexpected responses abort the run. Closing a serial connection is not an emergency stop; on a communication failure, no automatic return move is attempted.

## Four-file layout

`check_meter.py` contains the meter driver and its phase-1 diagnostic. `motor_check.py` contains the stage driver, mock stage, and its phase-2 diagnostic. The combined check and alignment import those same classes, so hardware communication is implemented only once. There is no separate optimizer or dependency on the earlier thorlabs_tlpm_interface wrapper. The previous threshold scan and live plot are not part of this raster workflow.

## DLL discovery

The meter driver checks an explicit constructor path, `THORLABS_TLPM_DLL`, saved `hardware_config.local.json`, the standard IVI installation location, vendor folders under Program Files, and PATH. After loading a discovered DLL, it reports `meter.dll_path` and asks whether to save it locally. Only `y` or `yes` saves it; noninteractive runs do not save. The local file is ignored by Git. Discovery does not install drivers or search every drive.

## Hardware state and verification

The owner reports validated PM100USB dark-current readings and PZC200 relative movement and position queries. Multi-axis operation awaits verified straight-through 6P6C RJ12 data cables. See AGENT_CONTEXT.md for hardware details. Software-only checks do not constitute physical phase completion. No physical hardware was operated during this restructuring.

## Optical Coupling & Alignment Specifications

### 1. Positioning Tolerances & Step Resolution
* **Working Clearance:** The X-axis must maintain a clearance of at most 14 nm from the lensed fiber facet to avoid physical collision while staying within the coupling waist.
* **Actuation Resolution:** Fine motion uses relative `PR1` or `PR-1` commands; one controller count is one commanded microstep.
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
  1. The stage steps forward by 1 count and samples the power.
  2. If efficiency increases, the optimizer continues in that direction.
  3. When the signal falls, it reverses direction.
  4. It measures again at the recovered peak before changing axis or step size.

### Current stage origin and coupling-neighborhood simulation

The controller origin was set at the user-identified back/left/bottom endpoints; see `stage_origin_log.json`. The user measured direction-dependent physical travel per controller count. Those calibration values, travel envelopes, and the proposed Z-then-Y-then-X return sequence are recorded in `stage_travel_config.json`.

The original coupling neighborhood is estimated at X `+61450`, Y `+250000`, Z `-200000` controller counts. The `200000`–`245800` Y range describes a roughly 6 mm chip scan, not the Y hard travel maximum. The current fiber-present application envelope in the configuration is X `0..61450`, Y `0..416700`, Z `-484000..0` after the user supplied a revised measured upward clearance.

Run `Simulate motor movement.py --simulate` for the software-only six-count search around the estimated coupling coordinates. Its synthetic voltages are in `simulated_voltage.csv`; the output log records absolute controller coordinates. Synthetic voltage does not verify physical motor movement or measured optical response.

Running `Simulate motor movement.py` without `--simulate` now commands the live Z → Y → X return from controller origin `(0,0,0)` to `(X +61450, Y +250000, Z -200000)`. It verifies idle controllers, writes and reads back hardware travel limits, moves in bounded relative chunks, checks controller status and counts after each chunk, and writes `live_return_log.csv`. It can resume a partial run only when reported counts match the staged path. The lensed fiber must be out of the stage's path during this return. The X positive limit remains at `+61450` after the return. This is an open-loop controller count check; external observation is needed to verify physical position.

**Z direction correction:** A manual upward move changed the reported Z count from `-484000` to `-260161`. Increasing Z count therefore tracked upward motion in that observation, contrary to the negative-up assumption used in the historical return. The live return and old negative-Z adjustment scripts are disabled in `stage_travel_config.json`. Re-establish the Z origin, physical direction, and clearances before reusing those commands.

The current live-motion interlock in `stage_travel_config.json` blocks further Z movement: controller counts reached `-300000`, but physical height change was not observed. The user-supplied upward ceiling remains `-384000`; a further `-100000` command would exceed it.
