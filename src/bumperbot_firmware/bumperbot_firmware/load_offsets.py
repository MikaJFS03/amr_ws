#!/usr/bin/env python3
import smbus, time, os, sys

BUS = 1         # <--- set to your I2C bus
ADDR = 0x28     # <--- set to your BNO address
OPR_MODE = 0x3D
CONFIG_MODE = 0x00
OFFSET_START = 0x55
OFFSET_END = 0x6A
IN_PATH = os.path.expanduser("~/.ros/bno055_offsets.bin")

if not os.path.exists(IN_PATH):
    print("Offsets file not found:", IN_PATH)
    sys.exit(1)

data = open(IN_PATH, "rb").read()
if len(data) != (OFFSET_END - OFFSET_START + 1):
    print("Offsets file size mismatch:", len(data))
    sys.exit(1)

bus = smbus.SMBus(BUS)

def read_byte(reg): return bus.read_byte_data(ADDR, reg)
def write_byte(reg, val): bus.write_byte_data(ADDR, reg, val)

# save current mode, enter config mode
prev_mode = read_byte(OPR_MODE)
write_byte(OPR_MODE, CONFIG_MODE)
time.sleep(0.03)

# write offsets sequentially
for i, b in enumerate(data):
    write_byte(OFFSET_START + i, b)
    time.sleep(0.002)   # tiny pause between writes

time.sleep(0.02)
# restore previous op mode
write_byte(OPR_MODE, prev_mode)
time.sleep(0.02)

print("Offsets loaded to sensor from", IN_PATH)
