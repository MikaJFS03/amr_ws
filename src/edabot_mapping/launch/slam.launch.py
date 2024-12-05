import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    zed_wrapper_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("zed_wrapper"), "launch", "zed_camera.launch.py"),
        launch_arguments={"camera_model": "zed2i"}.items()
    )

    rtabmap_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("edabot_mapping"), "launch", "rtabmap_rgbd.launch.py")
    )

    return LaunchDescription([
        zed_wrapper_launch,
        rtabmap_launch,
    ])