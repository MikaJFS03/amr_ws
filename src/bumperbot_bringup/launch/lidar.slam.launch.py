# lidar.slam.launch.py (edited)
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration,Command
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")
    use_sim_time = LaunchConfiguration("use_sim_time")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="true"   # run slam by default
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false"  # change default as you prefer
    )

    edabot_description_dir = get_package_share_directory("bumperbot_description")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(edabot_description_dir, "urdf", "bumperbot.urdf.xacro"),
        description="Absolute path to robot URDF file."
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', model_arg]),
            'use_sim_time': use_sim_time
        }]
    )
    # keep the laser driver
    laser_driver = Node(
        package="rplidar_ros",
        executable="rplidar_composition",
        name="rplidar_composition",
        parameters=[os.path.join(
            get_package_share_directory("bumperbot_bringup"),
            "config",
            "rplidar_a1.yaml"
        )],
        output="screen"
    )

    slam_launch_path = os.path.join(
        get_package_share_directory("bumperbot_mapping"),
        "launch",
        "slam.launch.py"
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={
            "use_sim_time": use_sim_time,
            # optionally pass slam_config override if you want a different file:
            # "slam_config": os.path.join(get_package_share_directory("bumperbot_mapping"), "config", "slam_toolbox.yaml")
        }.items(),
        condition=IfCondition(use_slam)
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        use_slam_arg,
        use_sim_time_arg,
        laser_driver,
        slam
    ])
