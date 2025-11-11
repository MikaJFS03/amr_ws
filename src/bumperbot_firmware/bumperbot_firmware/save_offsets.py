#!/usr/bin/env python3
import smbus, time, os

BUS = 1        # <--- set to your I2C bus number (e.g. 1, 5, etc.)
ADDR = 0x28    # <--- set to your BNO address (0x28 or 0x29)
OPR_MODE = 0x3D
CONFIG_MODE = 0x00
NDOF_MODE = 0x0C
OFFSET_START = 0x55
OFFSET_END = 0x6A
OUT_PATH = os.path.expanduser("~/.ros/bno055_offsets.bin")

bus = smbus.SMBus(BUS)

def read_byte(reg):
    return bus.read_byte_data(ADDR, reg)

def write_byte(reg, val):
    bus.write_byte_data(ADDR, reg, val)

# store current mode, enter config mode
prev_mode = read_byte(OPR_MODE)
write_byte(OPR_MODE, CONFIG_MODE)
time.sleep(0.03)

# read offset block
offsets = bytearray()
for r in range(OFFSET_START, OFFSET_END+1):
    offsets.append(read_byte(r))
time.sleep(0.01)

# restore previous mode
write_byte(OPR_MODE, prev_mode)
time.sleep(0.02)

# save to file
os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
with open(OUT_PATH, "wb") as f:
    f.write(offsets)

print(f"Saved {len(offsets)} bytes to {OUT_PATH}")
