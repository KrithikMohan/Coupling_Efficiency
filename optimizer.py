import time
import serial
import numpy as np
import matplotlib.pyplot as plt
from thorlabs_tlpm_interface import TLPM  # Official Thorlabs Python Wrapper

# ==========================================
# 1. HARDWARE CONFIGURATION & INITIALIZATION
# ==========================================

# Serial parameters for the Newport NSC-485-232-I Converter
SERIAL_PORT = "COM3"  
BAUD_RATE = 19200     
TIMEOUT = 1

# Target threshold configuration
TARGET_FRACTION = 0.70  
MAX_STEPS = 500         
STEP_SIZE_UM = 0.5      

# Initialize Serial Connection
try:
    motor = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
    print(f"[SUCCESS] Connected to Newport Motion Controller on {SERIAL_PORT}")
except Exception as e:
    print(f"[ERROR] Failed to open serial port {SERIAL_PORT}: {e}")
    exit()

# Initialize Thorlabs Meter
try:
    tlpm_device = TLPM()
    resource_name = tlpm_device.find_resources()[0] 
    meter = tlpm_device.open_resource(resource_name)
    
    meter.set_wavelength(1550.0)  
    meter.set_power_unit(0)       # Watts (W)
    print(f"[SUCCESS] Connected to Thorlabs Meter: {resource_name}")
except Exception as e:
    print(f"[ERROR] Failed to initialize Thorlabs sensor: {e}")
    motor.close()
    exit()

# ==========================================
# 2. DATA LOGGING & PLOT SETUP
# ==========================================

# Data storage arrays
positions_um = []
power_readings = []

# Initialize live plot
plt.ion()
fig, ax = plt.subplots(figsize=(8, 5))
line_plot, = ax.plot([], [], 'b-', marker='o', markersize=4, label='Measured Power')
ax.set_xlabel('Stage Displacement (µm)')
ax.set_ylabel('Optical Power (W)')
ax.set_title('Live Reflection Power Tracking')
ax.grid(True)

# ==========================================
# 3. CORE AUTOMATION & ALIGNMENT LOOP
# ==========================================

def get_live_power():
    time.sleep(0.05)
    return meter.get_power()

def step_newport_motor(distance_um):
    command = f"1PR{distance_um}\r\n"
    motor.write(command.encode('utf-8'))
    motor.flush()

try:
    print("\n--- Starting Baseline Calibration ---")
    baseline_power = get_live_power()
    print(f"Calculated Ambient Noise Floor: {baseline_power:.6e} W")
    
    ideal_peak_power = 1.0e-3  # Replace with your actual known peak power
    stop_threshold = ideal_peak_power * TARGET_FRACTION
    
    # Add target limit line to the graph
    limit_line = ax.axhline(y=stop_threshold, color='r', linestyle='--', label=f'70% Limit ({stop_threshold:.2e} W)')
    ax.legend(loc="upper left")
    
    print(f"Tracking Target Goal: {stop_threshold:.6e} W")
    print("\n--- Initiating Active Search Loop ---")

    current_position = 0.0

    for step in range(MAX_STEPS):
        current_reading = get_live_power()
        
        # Log data
        positions_um.append(current_position)
        power_readings.append(current_reading)
        
        # Update live plot
        line_plot.set_data(positions_um, power_readings)
        ax.relim()
        ax.autoscale_view()
        fig.canvas.draw()
        fig.canvas.flush_events()

        if current_reading >= stop_threshold:
            break

        step_newport_motor(STEP_SIZE_UM)
        current_position += STEP_SIZE_UM
        time.sleep(0.2) 

    # ==========================================
    # 4. CONSOLE SUMMARY TABLE
    # ==========================================
    print("\n" + "="*50)
    print(f"{'ALIGNMENT RUN SUMMARY':^50}")
    print("="*50)
    print(f"{'Total Steps Taken:':<30} {len(positions_um)}")
    print(f"{'Total Displacement (µm):':<30} {positions_um[-1]:.1f}")
    print(f"{'Peak Power Reached (W):':<30} {power_readings[-1]:.6e}")
    print(f"{'Target Status:':<30} {'REACHED' if power_readings[-1] >= stop_threshold else 'FAILED'}")
    print("="*50 + "\n")

finally:
    print("Closing hardware interfaces safely...")
    motor.close()
    meter.close()
    
    # Keep final plot open until user closes it
    plt.ioff()
    plt.show()
    print("[FINISHED] System disengaged cleanly.")