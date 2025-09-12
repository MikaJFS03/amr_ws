import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    edabot_description_dir = get_package_share_directory("edabot_description")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(edabot_description_dir, "urdf", "edabot.urdf.xacro"),
        description="Absolute path to robot URDF file."
    )

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    hardware_interface = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("edabot_firmware"), "launch", "hardware_interface.launch.py")
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
        model_arg,
        robot_state_publisher,
        hardware_interface,
        # controller,
        # joystick,
        slam,
        # navigation
    ])