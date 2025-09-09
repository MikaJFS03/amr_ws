#!/usr/bin/env python3
# teleop_key.py
import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def getch():
    """Read a single character from stdin (no Enter). Unix only."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

class TeleopKey(Node):
    def __init__(self):
        super().__init__('teleop_key')
        self.pub = self.create_publisher(String, 'serial_transmitter', 10)
        self.left_speed = 0
        self.right_speed = 0
        self.step = 10
        self.max_speed = 255
        self.min_speed = -255
        # small info printed to stdout immediately (helps when rclpy logging is quiet)
        print("Teleop started — use i/k (inc/dec), p (zero), w/s/a/d (directions), q (quit).")
        # also log via rclpy logger after initialization
        self.get_logger().info("teleop node initialized")
        # publish initial 0,0 so Arduino has a known value (optional)
        self.publish_speeds()

    def publish_speeds(self):
        # IMPORTANT: append newline so the Arduino will parse the command immediately
        payload = f"{int(self.left_speed)},{int(self.right_speed)}\n"
        msg = String()
        msg.data = payload
        self.pub.publish(msg)
        # print to stdout too so you can see activity in plain terminal
        print(f"Sent: {payload.strip()}")

    def run(self):
        try:
            while rclpy.ok():
                ch = getch()
                if ch in ('i', 'I'):
                    self.left_speed  = clamp(self.left_speed  + self.step, self.min_speed, self.max_speed)
                    self.right_speed = clamp(self.right_speed + self.step, self.min_speed, self.max_speed)
                    self.publish_speeds()

                elif ch in ('k', 'K'):
                    self.left_speed  = clamp(self.left_speed  - self.step, self.min_speed, self.max_speed)
                    self.right_speed = clamp(self.right_speed - self.step, self.min_speed, self.max_speed)
                    self.publish_speeds()

                elif ch in ('p', 'P'):
                    self.left_speed = 0
                    self.right_speed = 0
                    self.publish_speeds()

                elif ch in ('w', 'W'):
                    self.left_speed  = abs(self.left_speed)  if self.left_speed  != 0 else self.step
                    self.right_speed = abs(self.right_speed) if self.right_speed != 0 else self.step
                    self.publish_speeds()

                elif ch in ('s', 'S'):
                    self.left_speed  = -abs(self.left_speed)  if self.left_speed  != 0 else -self.step
                    self.right_speed = -abs(self.right_speed) if self.right_speed != 0 else -self.step
                    self.publish_speeds()

                elif ch in ('a', 'A'):
                    self.left_speed  = -abs(self.left_speed)  if self.left_speed  != 0 else -self.step
                    self.right_speed =  abs(self.right_speed) if self.right_speed != 0 else  self.step
                    self.publish_speeds()

                elif ch in ('d', 'D'):
                    self.left_speed  =  abs(self.left_speed) if self.left_speed  != 0 else  self.step
                    self.right_speed = -abs(self.right_speed) if self.right_speed != 0 else -self.step
                    self.publish_speeds()

                elif ch in ('q', 'Q'):
                    print("Quitting teleop.")
                    break
                else:
                    # ignore other keys
                    continue

        except KeyboardInterrupt:
            pass

def main():
    rclpy.init()
    node = TeleopKey()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
