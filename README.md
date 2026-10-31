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
