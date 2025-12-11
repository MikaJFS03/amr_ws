#!/usr/bin/env python3
import struct
import time
import os
import json  # Added for saving/loading calibration

import smbus
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

# BNO055 I2C defaults / registers (page 0)
BNO055_ADDRESS = 0x28        # common default; change to 0x29 if ADR pin high
PAGE_ID = 0x07
OPR_MODE = 0x3D
PWR_MODE = 0x3E
SYS_TRIGGER = 0x3F
CALIB_STAT = 0x35

# Data registers (page 0)
QUATERNION_DATA_W_LSB = 0x20  # W LSB (W, X, Y, Z each 16-bit little-endian)
GYRO_DATA_X_LSB = 0x14        # gyro registers start here (6 bytes)
LINEAR_ACC_X_LSB = 0x28       # linear acceleration registers (6 bytes)

# --- NEW: Calibration Registers (Page 0) ---
# Start of 22-byte calibration data
ACCEL_OFFSET_X_LSB = 0x55
ACCEL_OFFSET_X_MSB = 0x56
ACCEL_OFFSET_Y_LSB = 0x57
ACCEL_OFFSET_Y_MSB = 0x58
ACCEL_OFFSET_Z_LSB = 0x59
ACCEL_OFFSET_Z_MSB = 0x5A

MAG_OFFSET_X_LSB = 0x5B
MAG_OFFSET_X_MSB = 0x5C
MAG_OFFSET_Y_LSB = 0x5D
MAG_OFFSET_Y_MSB = 0x5E
MAG_OFFSET_Z_LSB = 0x5F
MAG_OFFSET_Z_MSB = 0x60

GYRO_OFFSET_X_LSB = 0x61
GYRO_OFFSET_X_MSB = 0x62
GYRO_OFFSET_Y_LSB = 0x63
GYRO_OFFSET_Y_MSB = 0x64
GYRO_OFFSET_Z_LSB = 0x65
GYRO_OFFSET_Z_MSB = 0x66

ACCEL_RADIUS_LSB = 0x67
ACCEL_RADIUS_MSB = 0x68
MAG_RADIUS_LSB = 0x69
MAG_RADIUS_MSB = 0x6A

CALIB_DATA_LEN = 22  # 22 bytes total
# --- End NEW ---

# Operation modes
CONFIG_MODE = 0x00
NDOF_MODE = 0x0C

class BNO055_Driver(Node):
    def __init__(self, i2c_bus=1, address=BNO055_ADDRESS):
        super().__init__("bno055_driver")
        self.address = address
        self.bus = None
        self.is_connected_ = False
        
        # --- NEW: Calibration file path ---
        # Saves calibration data in the same directory the script is run from
        self.calib_file = "bno055_calib.json" 
        self.calibration_saved = False # Flag to prevent saving multiple times
        # --- End NEW ---

        self.init_i2c(i2c_bus)

        # ROS interface
        self.imu_pub_ = self.create_publisher(Imu, "/imu/out", qos_profile=qos_profile_sensor_data)
        self.imu_msg_ = Imu()
        self.imu_msg_.header.frame_id = "base_footprint"

        # 100 Hz timer
        self.timer_period_s = 0.01
        self.timer_ = self.create_timer(self.timer_period_s, self.timerCallback)

        # calibration info log once
        self.reported_cal_complete = False # Renamed from reported_cal_ok

    def init_i2c(self, bus_num=1):
        try:
            self.bus = smbus.SMBus(bus_num)

            # Per datasheet: after POR wait >= 650 ms (using 10ms is risky, but part of original code)
            time.sleep(0.01)

            # Enter config mode before making configuration writes
            self.bus.write_byte_data(self.address, OPR_MODE, CONFIG_MODE)
            time.sleep(0.02) # Datasheet: 19ms to switch to CONFIG_MODE

            # Set power mode to normal
            self.bus.write_byte_data(self.address, PWR_MODE, 0x00)
            time.sleep(0.01)

            # Ensure register page 0
            self.bus.write_byte_data(self.address, PAGE_ID, 0x00)
            time.sleep(0.01)

            # (Optional) clear SYS_TRIGGER RST_SYS (not doing full reset here)
            # self.bus.write_byte_data(self.address, SYS_TRIGGER, 0x00)

            # --- NEW: Load calibration data ---
            # This must be done in CONFIG_MODE
            self.load_calibration()
            # --- End NEW ---

            # Set the operation mode to NDOF (fusion).
            self.bus.write_byte_data(self.address, OPR_MODE, NDOF_MODE)
            time.sleep(0.02) # Datasheet: 7ms to switch from CONFIG to other mode

            self.is_connected_ = True
            self.get_logger().info(f"BNO055 at 0x{self.address:02x} initialized (NDOF).")
        except OSError as e:
            self.is_connected_ = False
            self.get_logger().error(f"Failed to init BNO055 on bus {bus_num} addr 0x{self.address:02x}: {e}")

    # --- NEW: Function to load calibration data ---
    def load_calibration(self):
        """Checks for calibration file and writes it to the sensor."""
        if not os.path.exists(self.calib_file):
            self.get_logger().warn(f"Calibration file {self.calib_file} not found. Sensor will calibrate from scratch.")
            return

        try:
            with open(self.calib_file, 'r') as f:
                calib_data_list = json.load(f)

            if len(calib_data_list) != CALIB_DATA_LEN:
                self.get_logger().error(f"Calibration file {self.calib_file} is corrupt (wrong length). Calibrating from scratch.")
                return
            
            # We must be in CONFIG_MODE to write calibration data (checked in init_i2c)
            
            # Write the 22 bytes of data
            # write_i2c_block_data handles writing a list/bytes block
            self.bus.write_i2c_block_data(self.address, ACCEL_OFFSET_X_LSB, calib_data_list)
            time.sleep(0.02) # Give it a moment to process
            
            self.get_logger().info(f"Successfully loaded calibration data from {self.calib_file}.")
            self.calibration_saved = True # We have loaded data, no need to save it again
            self.reported_cal_complete = True # Assume loaded data is good
        except Exception as e:
            self.get_logger().error(f"Failed to read/write calibration data: {e}. Calibrating from scratch.")
    # --- End NEW ---

    # --- NEW: Function to save calibration data ---
    def save_calibration(self):
        """Reads 22 bytes of calibration data and saves to a JSON file."""
        if self.calibration_saved:
            return # Already saved (or loaded)

        try:
            # Must be in CONFIG_MODE to read calibration
            # Get current mode to restore it later
            original_mode = self.bus.read_byte_data(self.address, OPR_MODE)
            
            # Switch to CONFIG_MODE
            self.bus.write_byte_data(self.address, OPR_MODE, CONFIG_MODE)
            time.sleep(0.02) # Wait for mode switch

            # Read the 22 bytes of calibration data
            # read_i2c_block_data returns a list of bytes
            calib_data_list = self.bus.read_i2c_block_data(self.address, ACCEL_OFFSET_X_LSB, CALIB_DATA_LEN)

            # Restore original mode
            self.bus.write_byte_data(self.address, OPR_MODE, original_mode)
            time.sleep(0.02)
            
            # Save data to file
            with open(self.calib_file, 'w') as f:
                json.dump(calib_data_list, f)
            
            self.calibration_saved = True
            self.get_logger().info(f"Successfully saved calibration data to {self.calib_file}.")

        except Exception as e:
            self.get_logger().error(f"Failed to save calibration data: {e}")
    # --- End NEW ---

    def timerCallback(self):
        try:
            if not self.is_connected_:
                self.init_i2c()
                if not self.is_connected_: # Still not connected, skip this cycle
                    return

            # Read 8 bytes quaternion (W, X, Y, Z)
            q_bytes = self.bus.read_i2c_block_data(self.address, QUATERNION_DATA_W_LSB, 8)
            qw, qx, qy, qz = struct.unpack_from('<hhhh', bytes(q_bytes), 0)
            scale_q = 1.0 / (1 << 14)  
            qw_f = qw * scale_q
            qx_f = qx * scale_q
            qy_f = qy * scale_q
            qz_f = qz * scale_q

            # Read linear acceleration
            lia_bytes = self.bus.read_i2c_block_data(self.address, LINEAR_ACC_X_LSB, 6)
            lia_x_raw, lia_y_raw, lia_z_raw = struct.unpack_from('<hhh', bytes(lia_bytes), 0)
            scale_lia = 1.0 / 100.0
            lia_x = lia_x_raw * scale_lia
            lia_y = lia_y_raw * scale_lia
            lia_z = lia_z_raw * scale_lia

            # Read gyro (angular velocity)
            gyr_bytes = self.bus.read_i2c_block_data(self.address, GYRO_DATA_X_LSB, 6)
            gyr_x_raw, gyr_y_raw, gyr_z_raw = struct.unpack_from('<hhh', bytes(gyr_bytes), 0)
            gyr_scale = 0.001090830782496456 # rad/s
            gyr_x = gyr_x_raw * gyr_scale
            gyr_y = gyr_y_raw * gyr_scale
            gyr_z = gyr_z_raw * gyr_scale

            # Fill IMU message
            self.imu_msg_.orientation.x = qx_f
            self.imu_msg_.orientation.y = qy_f
            self.imu_msg_.orientation.z = qz_f
            self.imu_msg_.orientation.w = qw_f

            self.imu_msg_.angular_velocity.x = gyr_x
            self.imu_msg_.angular_velocity.y = gyr_y
            self.imu_msg_.angular_velocity.z = gyr_z

            self.imu_msg_.linear_acceleration.x = lia_x
            self.imu_msg_.linear_acceleration.y = lia_y
            self.imu_msg_.linear_acceleration.z = lia_z

            # Read calibration status
            calib = self.bus.read_byte_data(self.address, CALIB_STAT)
            sys_cal = (calib >> 6) & 0x03
            
            # --- MODIFIED: Calibration check and save ---
            if not self.reported_cal_complete and sys_cal < 3:
                # Log a warning, but only if we didn't load a profile
                if not self.calibration_saved:
                    self.get_logger().warn(f"BNO055 system calibration not complete (sys={sys_cal}). Orientation may be relative. Please calibrate.")
            elif sys_cal == 3 and not self.reported_cal_complete:
                self.get_logger().info("BNO055 fully calibrated (system = 3).")
                self.reported_cal_complete = True
                
                # *** Save calibration data now that it's good ***
                self.save_calibration()
            # --- End MODIFIED ---

            self.imu_msg_.header.stamp = self.get_clock().now().to_msg()
            self.imu_pub_.publish(self.imu_msg_)
        except OSError as e:
            self.get_logger().error(f"I2C read error: {e}")
            self.is_connected_ = False

def main():
    rclpy.init()
    node = BNO055_Driver(i2c_bus=7)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()