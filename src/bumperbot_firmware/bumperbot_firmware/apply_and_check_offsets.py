#!/usr/bin/env python3
"""
apply_and_check_offsets.py
Writes ~/.ros/bno055_offsets.bin to BNO055 registers 0x55..0x6A, then verifies.
Adjust BUS and ADDR if your I2C bus/address differ.
If you run with sudo, use: sudo HOME=/home/mf python3 apply_and_check_offsets.py
"""
import os, time, sys
try:
    import smbus
except Exception as e:
    print("Missing smbus library:", e); sys.exit(2)

BUS = 1        # change to your bus if needed
ADDR = 0x28    # change to 0x29 if your ADR pin is high
OFF_START = 0x55
OFF_END = 0x6A
NUM = OFF_END - OFF_START + 1
OPR_MODE = 0x3D
CONFIG_MODE = 0x00
NDOF_MODE = 0x0C
CALIB_REG = 0x35
PATH = os.path.expanduser("~/.ros/bno055_offsets.bin")

def read_file_bytes(path):
    if not os.path.exists(path):
        print("Offsets file not found:", path)
        sys.exit(1)
    data = open(path, "rb").read()
    if len(data) != NUM:
        print(f"Offsets file size mismatch: {len(data)} != {NUM}")
        sys.exit(1)
    return data

def format_bytes(b):
    return " ".join(f"{x:02x}" for x in b)

def main():
    data = read_file_bytes(PATH)
    print("Loaded offsets file:", PATH)
    print("File bytes:   ", format_bytes(data))

    bus = smbus.SMBus(BUS)
    try:
        # read current op mode
        prev = bus.read_byte_data(ADDR, OPR_MODE)
    except Exception as e:
        print("Failed to read OPR_MODE (is device present on bus?):", e)
        sys.exit(2)

    try:
        print("Switching to CONFIG_MODE...")
        bus.write_byte_data(ADDR, OPR_MODE, CONFIG_MODE)
        time.sleep(0.03)

        print("Writing offsets to sensor registers 0x55..0x6A ...")
        for i, b in enumerate(data):
            reg = OFF_START + i
            bus.write_byte_data(ADDR, reg, b)
            # tiny pause helps reliability
            time.sleep(0.002)

        time.sleep(0.02)

        # restore previous op mode (or set to NDOF)
        try:
            print("Restoring previous op mode...")
            bus.write_byte_data(ADDR, OPR_MODE, prev)
        except Exception:
            # fallback: ensure in NDOF
            bus.write_byte_data(ADDR, OPR_MODE, NDOF_MODE)
        time.sleep(0.03)

        # read back offsets from sensor
        sensor_vals = []
        for reg in range(OFF_START, OFF_END + 1):
            sensor_vals.append(bus.read_byte_data(ADDR, reg))
        print("Sensor bytes: ", format_bytes(sensor_vals))

        # compare and show differences
        diffs = [(i, data[i], sensor_vals[i]) for i in range(NUM) if data[i] != sensor_vals[i]]
        if not diffs:
            print("All bytes match! Offsets successfully written.")
        else:
            print("Differences found at indices (index, file, sensor):")
            for i,f,s in diffs:
                print(f"  {i:02d}: file {f:02x}  sensor {s:02x}")

        # print calibration status
        cal = bus.read_byte_data(ADDR, CALIB_REG)
        sysc = (cal >> 6) & 0x03
        gyro = (cal >> 4) & 0x03
        accel = (cal >> 2) & 0x03
        mag = cal & 0x03
        print(f"CALIB_REG: 0x{cal:02x} -> sys:{sysc} gyro:{gyro} accel:{accel} mag:{mag}")

    except Exception as e:
        print("Error during write/verify:", e)
        raise
    finally:
        bus.close()

if __name__ == "__main__":
    main()
