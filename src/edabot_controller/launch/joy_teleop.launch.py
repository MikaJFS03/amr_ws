import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    edabot_controller_dir = get_package_share_directory("edabot_controller")

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[os.path.join(edabot_controller_dir, "config", "joy_config.yaml")]
    )

    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[os.path.join(edabot_controller_dir, "config", "joy_teleop.yaml")]
    )

    return LaunchDescription([
        joy_node,
        joy_teleop
    ])