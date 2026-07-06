Automated Optical Alignment System

This repository contains Python scripts to automate fiber-to-chip alignment using a motion stage and a digital power meter. The system moves the translation stage incrementally based on real-time optical feedback until the reflection signal reaches a set target threshold.
Features

    Alignment Loop: Moves a Newport stage via serial commands until the reflection power reaches 70% of the target value.

    Instrument Control: Interfaces directly with the Thorlabs TLPM driver over USB to read digital optical power in Watts.

    Live Plotting & Logging: Tracks position and power data, generating a live graph with a target limit line and displaying a summary table at the end.

    Diagnostics: Includes standalone scripts (check_motor.py and check_meter.py) to test hardware connections independently.