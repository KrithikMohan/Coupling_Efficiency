import ctypes
import os
import sys
from ctypes import byref, c_bool, c_char_p, c_double, c_uint32, create_string_buffer

DLL_PATH = r"C:\Program Files\IVI Foundation\VISA\Win64\Bin\TLPM_64.dll"

if not os.path.isfile(DLL_PATH):
    print(f"ERROR: DLL not found at {DLL_PATH}")
    sys.exit(1)

dll = ctypes.cdll.LoadLibrary(DLL_PATH)

# Function signatures
dll.TLPM_findRsrc.argtypes = [c_uint32, ctypes.POINTER(c_uint32)]
dll.TLPM_findRsrc.restype = c_uint32

dll.TLPM_getRsrcName.argtypes = [c_uint32, c_uint32, c_char_p]
dll.TLPM_getRsrcName.restype = c_uint32

dll.TLPM_init.argtypes = [c_char_p, c_bool, c_bool, ctypes.POINTER(c_uint32)]
dll.TLPM_init.restype = c_uint32

dll.TLPM_measPower.argtypes = [c_uint32, ctypes.POINTER(c_double)]
dll.TLPM_measPower.restype = c_uint32

dll.TLPM_close.argtypes = [c_uint32]
dll.TLPM_close.restype = c_uint32

print("=== THORLABS PM100USB DIAGNOSTIC ===")

# Test 1: Resource Finder
count = c_uint32(0)
status_find = dll.TLPM_findRsrc(0, byref(count))
print(f"TLPM_findRsrc status : {hex(status_find)} (0x0 = VI_SUCCESS)")
print(f"Devices detected     : {count.value}")

handle = c_uint32(0)
connected_rsrc = None

if status_find == 0 and count.value > 0:
    buf = create_string_buffer(256)
    dll.TLPM_getRsrcName(0, 0, buf)
    connected_rsrc = buf.value
    print(f"Found via scan       : {connected_rsrc.decode('ascii')}")
    init_stat = dll.TLPM_init(connected_rsrc, False, False, byref(handle))
    print(f"TLPM_init status     : {hex(init_stat)}")
else:
    # Test 2: Direct Resource Address Bypass using known serial number
    print("\nScan returned 0. Testing direct VISA resource descriptors...")
    candidates = [
        b"USB0::0x1313::0x8072::261005409::0::INSTR",
        b"USB0::0x1313::0x8072::PM 261005409::0::INSTR",
        b"USB0::0x1313::0x8072::P261005409::0::INSTR",
        b"USB0::0x1313::0x8072::261005409::INSTR",
    ]
    for rsrc in candidates:
        h = c_uint32(0)
        st = dll.TLPM_init(rsrc, False, False, byref(h))
        print(f"  Trying {rsrc.decode('ascii')} -> Status: {hex(st)}")
        if st == 0:
            handle = h
            connected_rsrc = rsrc
            print(f"  -> SUCCESS: Direct connection established!")
            break

# Test 3: Sample Power
if handle.value != 0:
    power = c_double(0.0)
    dll.TLPM_measPower(handle, byref(power))
    print(f"\nLive Optical Power Reading: {power.value * 1e9:.2f} nW ({power.value:.6e} W)")
    dll.TLPM_close(handle)
    print("Session closed successfully.")
else:
    print("\nFAILED: Meter not accessible via VISA. Ensure driver is set to USBTMC and Task Manager has no Thorlabs processes running.")