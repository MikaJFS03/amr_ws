#!/usr/bin/env python3
# teleop_vw.py
import sys
import termios
import tty
import select
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TeleopVW(Node):
    """
    Publishes 'v_mmps,w_mradps\n' to 'serial_transmitter'.
    - v in mm/s (int), w in mrad/s (int, CCW +)
    Has a heartbeat so the MCU (500 ms failsafe) won’t stop mid-drive.
    """

    def __init__(self):
        super().__init__('teleop_vw')

        # Params
        self.topic = self.declare_parameter('topic', 'serial_transmitter').get_parameter_value().string_value
        self.linear_step = self.declare_parameter('linear_step_mmps', 100).get_parameter_value().integer_value
        self.angular_step = self.declare_parameter('angular_step_mradps', 100).get_parameter_value().integer_value
        self.linear_max = self.declare_parameter('linear_max_mmps', 1200).get_parameter_value().integer_value
        self.angular_max = self.declare_parameter('angular_max_mradps', 3000).get_parameter_value().integer_value
        self.heartbeat_hz = float(self.declare_parameter('heartbeat_hz', 10.0).get_parameter_value().double_value)

        # Publisher
        self.pub = self.create_publisher(String, self.topic, 10)

        # State
        self.v_mmps = 0
        self.w_mradps = 0
        self.last_send = 0.0
        # --- NEW: Staging state (starts equal to live values) ---
        self.staged_v_mmps = self.v_mmps
        self.staged_w_mradps = self.w_mradps
        # --- NEW: Staging increments (per your request: fixed ±100) ---
        self.stage_linear_increment = 100
        self.stage_angular_increment = 100

        # Log FIRST (don’t touch terminal until this is fine)
        self.get_logger().info(f"teleop_vw started — publishing to '{self.topic}'")

        # Terminal raw mode
        self.fd = sys.stdin.fileno()
        self.old_term = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)  # non-canonical, char-by-char

        # Help + initial stop
        self.print_help()
        self.send_command()

    def __del__(self):
        try:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_term)
        except Exception:
            pass

    def clamp(self, x, lo, hi):
        return max(lo, min(hi, x))

    def print_help(self):
        print(
            "\nTeleop for v,w (integers):\n"
            "  w: +v (forward)      s: -v (backward)\n"
            "  a: +w (turn left)    d: -w (turn right)\n"
            "  SPACE: stop (v=0, w=0)\n"
            "  z: reset encoders/PID on MCU\n"
            "  h: help,  q: quit\n"
            f"Steps: linear={self.linear_step} mm/s, angular={self.angular_step} mrad/s\n"
            f"Limits: |v| <= {self.linear_max} mm/s, |w| <= {self.angular_max} mrad/s\n"
            # --- NEW: Help for staging keys ---
            "Staging (won't publish until 'm'):\n"
            "  u: stage +v (+100)   i: stage -v (-100)\n"
            "  j: stage +w (+100)   k: stage -w (-100)\n"
            "  m: publish staged values\n"
        )

    def send_command(self):
        msg = String()
        msg.data = f"{int(self.v_mmps)},{int(self.w_mradps)}\n"
        self.pub.publish(msg)
        self.last_send = time.time()
        print(f"CMD v,w: {self.v_mmps},{self.w_mradps}  future v,w: {self.staged_v_mmps},{self.staged_w_mradps}")

    def send_reset(self):
        msg = String(); msg.data = "z\n"
        self.pub.publish(msg)
        self.v_mmps = 0; self.w_mradps = 0
        self.last_send = time.time()
        print("CMD: z (reset)")

    def step_v(self, dv):
        self.v_mmps = self.clamp(self.v_mmps + dv, -self.linear_max, self.linear_max)
        self.send_command()

    def step_w(self, dw):
        self.w_mradps = self.clamp(self.w_mradps + dw, -self.angular_max, self.angular_max)
        self.send_command()

    # --- NEW: staging helpers (no publish) ---
    def _print_staged(self):
        print(f"STAGE v,w: {int(self.staged_v_mmps)},{int(self.staged_w_mradps)} (pending)")

    def stage_v(self, dv):
        self.staged_v_mmps = self.clamp(self.staged_v_mmps + dv, -self.linear_max, self.linear_max)
        self._print_staged()

    def stage_w(self, dw):
        self.staged_w_mradps = self.clamp(self.staged_w_mradps + dw, -self.angular_max, self.angular_max)
        self._print_staged()

    def publish_staged(self):
        self.v_mmps = int(self.staged_v_mmps)
        self.w_mradps = int(self.staged_w_mradps)
        self.send_command()

    def run(self):
        try:
            period = 1.0 / self.heartbeat_hz if self.heartbeat_hz > 0 else 0.1
            while rclpy.ok():
                # Heartbeat so MCU doesn't time out
                if (time.time() - self.last_send) >= period:
                    self.send_command()

                # Non-blocking key poll
                rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
                if not rlist:
                    continue

                ch = sys.stdin.read(1)
                if ch in ('w', 'W'):
                    self.step_v(+self.linear_step)
                elif ch in ('s', 'S'):
                    self.step_v(-self.linear_step)
                elif ch in ('a', 'A'):
                    self.step_w(+self.angular_step)   # CCW positive
                elif ch in ('d', 'D'):
                    self.step_w(-self.angular_step)   # CW negative
                # --- NEW: staging keys (no publish) ---
                elif ch in ('u', 'U'):
                    self.stage_v(+self.stage_linear_increment)
                elif ch in ('i', 'I'):
                    self.stage_v(-self.stage_linear_increment)
                elif ch in ('j', 'J'):
                    self.stage_w(+self.stage_angular_increment)   # CCW positive (like 'a')
                elif ch in ('k', 'K'):
                    self.stage_w(-self.stage_angular_increment)   # CW negative (like 'd')
                elif ch in ('m', 'M'):
                    self.publish_staged()
                elif ch == ' ':
                    self.v_mmps = 0; self.w_mradps = 0; self.send_command()
                elif ch in ('z', 'Z'):
                    self.send_reset()
                elif ch in ('h', 'H'):
                    self.print_help()
                elif ch in ('q', 'Q'):
                    print("Quitting teleop; sending stop.")
                    self.v_mmps = 0; self.w_mradps = 0; self.send_command()
                    break
                else:
                    pass

        except KeyboardInterrupt:
            print("\nKeyboardInterrupt — stopping.")
            self.v_mmps = 0; self.w_mradps = 0; self.send_command()
        finally:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_term)


def main():
    rclpy.init()
    node = TeleopVW()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
