import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("edabot_description"), "launch", "gazebo.launch.py"),
        launch_arguments={"world_name": "small_warehouse"}.items()
    )

    controller = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("edabot_controller"), "launch", "controller.launch.py")
    )

    joystick = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("edabot_controller"), "launch", "joy_teleop.launch.py")
    )

    slam = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("edabot_mapping"), "launch", "slam.launch.py")
    )

    navigation = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("edabot_navigation"), "launch", "navigation.launch.py")
    )

    return LaunchDescription([
        gazebo,
        controller,
        # joystick,
        # slam,
        # navigation
    ])