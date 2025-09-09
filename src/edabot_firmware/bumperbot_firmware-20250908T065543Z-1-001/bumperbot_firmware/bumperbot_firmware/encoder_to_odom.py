#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64
from geometry_msgs.msg import Quaternion, Pose, Twist, Point, Vector3
import math
import time

class EncoderToOdom(Node):
    def __init__(self):
        super().__init__('encoder_to_odom')
        # Parameters (set these to match your robot)
        self.wheel_radius = 0.075  # meters
        self.wheel_separation = 0.4174  # meters
        self.ticks_per_rev = 400  # set to your encoder's value

        self.last_left = None
        self.last_right = None
        self.last_time = self.get_clock().now().nanoseconds / 1e9
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.left_sub = self.create_subscription(Int64, 'left_encoder', self.left_cb, 10)
        self.right_sub = self.create_subscription(Int64, 'right_encoder', self.right_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        self.left_ticks = 0
        self.right_ticks = 0

        self.timer = self.create_timer(0.05, self.update)  # 20 Hz

    def left_cb(self, msg):
        self.left_ticks = msg.data

    def right_cb(self, msg):
        self.right_ticks = msg.data

    def update(self):
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.last_time
        if dt == 0:
            return

        if self.last_left is None or self.last_right is None:
            self.last_left = self.left_ticks
            self.last_right = self.right_ticks
            self.last_time = now
            return

        d_left = (self.left_ticks - self.last_left) * (2 * math.pi * self.wheel_radius) / self.ticks_per_rev
        d_right = (self.right_ticks - self.last_right) * (2 * math.pi * self.wheel_radius) / self.ticks_per_rev

        self.last_left = self.left_ticks
        self.last_right = self.right_ticks

        d = (d_left + d_right) / 2.0
        th = (d_right - d_left) / self.wheel_separation

        self.x += d * math.cos(self.th + th / 2.0)
        self.y += d * math.sin(self.th + th / 2.0)
        self.th += th

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        qx, qy, qz, qw = self.euler_to_quaternion(0, 0, self.th)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        linear = Vector3()
        linear.x = d / dt
        linear.y = 0.0
        linear.z = 0.0
        angular = Vector3()
        angular.x = 0.0
        angular.y = 0.0
        angular.z = th / dt
        twist = Twist()
        twist.linear = linear
        twist.angular = angular
        odom.twist.twist = twist
        self.odom_pub.publish(odom)

        # Publish joint states
        js = JointState()
        js.header.stamp = odom.header.stamp
        js.name = ['base_to_left_wheel_joint', 'base_to_right_wheel_joint']
        js.position = [self.left_ticks * (2 * math.pi) / self.ticks_per_rev,
                   self.right_ticks * (2 * math.pi) / self.ticks_per_rev]
        self.joint_pub.publish(js)

        self.last_time = now

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch     /2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = EncoderToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()