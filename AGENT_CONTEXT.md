# Project Context: Automated Fiber-to-Stage Alignment System

## 1. Project Progress & Hardware State

The following hardware status was supplied by the project owner.

* **Optical Power Meter (Thorlabs PM100USB + S132C): Fully operational.** The 64-bit C-DLL interface communicates with the console and reads dark-current power values (~10^-10 W) via the Germanium photodiode head.
* **Motion Control Stage (Newport PZC200): Hardware validated.** Commands (`1PR`, `1TP?`) successfully step and query the motor over RS-485. Multi-axis bus communication is temporarily on hold pending delivery of verified straight-through 6P6C RJ12 data cables (previous Amazon Hoqqf cables were reverse-pinned voice cords). A software mock stage handles testing in the interim.
* **Current Software Capability:** The system can read optical power values, issue motor move commands, and run closed-loop feedback routines.

## 2. Dependencies & Environment

Install via pip:

```bash
pip install pyserial numpy
```

| Package | Version | Purpose |
| --- | --- | --- |
| pyserial | >=3.5 | RS-485 serial communication with the Newport PZC200 stage |
| numpy | >=1.24 | Coordinate grid math and power array analysis |

Optional visualization/analysis libraries: matplotlib, scipy.

Built-in standard libraries used:
* ctypes (low-level C-DLL binding for Thorlabs driver)
* time (settling delays between stage steps)
* os, sys, platform (path and environment validation)

## 3. Driver & OS Requirements

* **Operating System:** Windows 10/11 (64-bit).
* **Python Architecture:** Must use **64-bit Python**. A 32-bit interpreter will fail to load 64-bit DLLs with `[WinError 193]`.
* **Required Thorlabs Software:** **Thorlabs Optical Power Monitor (OPM)** must be installed on the machine.
* **Driver Purpose:** Installing Thorlabs OPM installs the proprietary kernel USB driver (`Thorlabs USB Devices -> PM100USB (Interface 0 TMC)`) and places the required C-library on disk:

```text
C:\Program Files\IVI Foundation\VISA\Win64\Bin\TLPM_64.dll
```

* **Critical Rule:** The desktop Thorlabs OPM application **must be completely closed** before executing Python code. The PM100USB does not allow concurrent access, and active background sessions will cause `TLPM_init` to fail.

## 4. Current architecture and deliberate phase order

There are exactly four Python files. `check_meter.py` contains DLL discovery, the meter driver, and its diagnostic. `motor_check.py` contains the serial driver, stage mock, and its diagnostic. The remaining two scripts import those classes. Importing either diagnostic file does not run its main function.

1. check_meter.py: meter-only diagnostic, repeated finite readings.
2. motor_check.py: stage-only diagnostic, position queries, no movement.
3. combined_check.py: simultaneous stage position and meter acquisition, no movement.
4. alignment_routine.py: raster alignment, idle unless --run is supplied. Physical stage requires --live-stage, explicit axis bounds, and typed ALIGN confirmation after reviewing phases 1-3.

See README.md for commands. Phase advancement is manual; no physical phase has been certified by the software changes. Mock results do not establish hardware readiness. The PA and MD? command behavior must be verified before a live raster. Motion polling times out and rejects unexpected status replies. Position parse failures raise errors. Cleanup closes connections, but does not command an emergency stop or a recovery move.

The earlier optimizer.py was removed. Each diagnostic file contains its matching shared driver; the combined check and alignment import those classes. Source files are authoritative; no duplicated code snapshots are kept here.

## 5. Environment and DLL discovery

Configured interpreter observed during the prior update: 64-bit Python 3.14.5. Recheck the current environment before running Python commands.

The DLL driver accepts an explicit path, THORLABS_TLPM_DLL, or a saved hardware_config.local.json path; otherwise searches the standard IVI location, vendor installation folders, and PATH. After successful loading, meter.dll_path reports the selected location. Interactive confirmation is required to save a newly discovered path. The local config is excluded from Git.

When verified straight-through RJ12 cables arrive, connect the axes through the splitter and run physical diagnostics on each axis before considering alignment. All coordinates are controller steps, not calibrated micrometres.
