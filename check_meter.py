import time
from thorlabs_tlpm_interface import TLPM

print("Scanning USB bus for Thorlabs instruments...")

try:
    tlpm_device = TLPM()
    resources = tlpm_device.find_resources()
    
    if not resources:
        print("[ERROR] No Thorlabs devices detected. Check the mini-USB cable.")
        exit()
        
    target_resource = resources[0]
    print(f"[SUCCESS] Found hardware at address: {target_resource}")
    
    # Connect and configure
    meter = tlpm_device.open_resource(target_resource)
    meter.set_wavelength(1550.0)
    meter.set_power_unit(0)  # 0 = Watts
    
    # Allow sensor filter to settle, then read
    time.sleep(0.2)
    power_reading = meter.get_power()
    
    print(f"[STATUS] Hardware initialized. Live optical power at 1550 nm: {power_reading:.6e} W")

except Exception as e:
    print(f"[ERROR] Communication with Thorlabs driver failed: {e}")
finally:
    if 'meter' in locals():
        meter.close()
        print("USB resource released.")