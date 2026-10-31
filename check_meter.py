import ctypes
import os
import sys
from ctypes import byref, c_bool, c_char_p, c_double, c_uint32, c_void_p, create_string_buffer

DLL_PATH = r"C:\Program Files\IVI Foundation\VISA\Win64\Bin\TLPM_64.dll"

if not os.path.isfile(DLL_PATH):
    print(f"ERROR: DLL not found at {DLL_PATH}")
    sys.exit(1)

dll = ctypes.cdll.LoadLibrary(DLL_PATH)

# Function signatures (64-bit handles)
dll.TLPM_findRsrc.argtypes = [c_void_p, ctypes.POINTER(c_uint32)]
dll.TLPM_findRsrc.restype = c_uint32

dll.TLPM_getRsrcName.argtypes = [c_void_p, c_uint32, c_char_p]
dll.TLPM_getRsrcName.restype = c_uint32

dll.TLPM_init.argtypes = [c_char_p, c_bool, c_bool, ctypes.POINTER(c_void_p)]
dll.TLPM_init.restype = c_uint32

dll.TLPM_measPower.argtypes = [c_void_p, ctypes.POINTER(c_double)]
dll.TLPM_measPower.restype = c_uint32

dll.TLPM_close.argtypes = [c_void_p]
dll.TLPM_close.restype = c_uint32

print("=== THORLABS PM100USB ADAPTER TEST ===")

# 1. Scan for raw VISA descriptors
count = c_uint32(0)
dll.TLPM_findRsrc(c_void_p(0), byref(count))
print(f"Devices detected by scan: {count.value}")

raw_scanned = None
if count.value > 0:
    buf = create_string_buffer(256)
    dll.TLPM_getRsrcName(c_void_p(0), 0, buf)
    raw_scanned = buf.value
    print(f"Raw scanned string: {raw_scanned.decode('ascii', errors='replace')}")

# 2. Candidate addresses resolving the empty serial number / composite interface
candidates = []

# If scanned string had the '::::' issue, patch it with interface 0
if raw_scanned:
    candidates.append(raw_scanned)
    fixed = raw_scanned.replace(b"::::INSTR", b"::0::INSTR")
    candidates.append(fixed)

# Known serial numbers from your device & Thorlabs monitor
candidates.extend([
    b"USB0::0x1313::0x8072::0::INSTR",
    b"USB0::0x1313::0x8072::1916864::INSTR",
    b"USB0::0x1313::0x8072::1916864::0::INSTR",
    b"USB0::0x1313::0x8072::261005409::INSTR",
    b"USB0::0x1313::0x8072::261005409::0::INSTR",
    b"USB0::0x1313::0x8072::?*::INSTR",
])

handle = c_void_p(0)
connected_address = None

print("\nTesting descriptor candidates...")
for rsrc in candidates:
    h = c_void_p(0)
    status = dll.TLPM_init(rsrc, False, False, byref(h))
    status_hex = hex(status)
    rsrc_str = rsrc.decode('ascii', errors='replace')

    if status == 0:
        print(f"  [SUCCESS] {rsrc_str} -> Connected! (Status: {status_hex})")
        handle = h
        connected_address = rsrc
        break
    else:
        print(f"  [FAILED]  {rsrc_str} -> Error: {status_hex}")

# 3. Read live power if connected
if handle.value:
    power = c_double(0.0)
    dll.TLPM_measPower(handle, byref(power))
    power_w = power.value

    if abs(power_w) >= 1e-6:
        val_str = f"{power_w * 1e6:8.3f} uW"
    else:
        val_str = f"{power_w * 1e9:8.2f} nW"

    print(f"\nLive Optical Power Reading: {val_str} ({power_w:.6e} W)")
    dll.TLPM_close(handle)
    print("Session closed successfully.")
else:
    print("\nFAILED: None of the descriptors could open the device.")