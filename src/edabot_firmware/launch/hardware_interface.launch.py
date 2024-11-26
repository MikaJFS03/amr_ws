import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():
    edabot_description_dir = get_package_share_directory("edabot_description")
    edabot_controller_dir = get_package_share_directory("edabot_controller")

    robot_description = ParameterValue(
        Command(["xacro ", os.path.join(edabot_description_dir, "urdf", "edabot.urdf.xacro")]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, os.path.join(edabot_controller_dir, "config", "edabot_controller.yaml")],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager,
    ])