from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():
    # Static TF base_footprint -> base_link (z=0, no rotation)
    static_basefootprint_baselink = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='basefootprint_baselink_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    )
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
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("edabot_mapping"), "launch", "slam.launch.py")
        )
    )

    navigation = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("edabot_navigation"), "launch", "navigation.launch.py")
    )
    
    # Path ke config SLAM Toolbox (mapping mode)
    slam_config = os.path.join(
        get_package_share_directory("edabot_bringup"),
        "config", "mapper_params_online_async.yaml"
    )

    # Path ke launch file bawaan slam_toolbox
    slam_launch = os.path.join(
        get_package_share_directory("slam_toolbox"),
        "launch",
        "online_async_launch.py"
    )

    return LaunchDescription([
        static_basefootprint_baselink,
        model_arg,
        robot_state_publisher,
        hardware_interface,
        #controller,
        # joystick,
        #slam,
        # navigation

        # RPLIDAR node
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB1',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'queue_size': 20
            }]
        ),
#
#        # Static TF base_link -> laser
#        Node(
#            package='tf2_ros',
#            executable='static_transform_publisher',
#            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
#            name='laser_broadcaster'
#        ),
#
#        # Static TF odom -> base_link
#        Node(
#            package='tf2_ros',
#            executable='static_transform_publisher',
#            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
#            name='odom_broadcaster'
#        ),
#       

        # Include SLAM Toolbox default launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={'slam_params_file': slam_config}.items()
        ),
        # Static TF base_link -> laser  (adjust translation/rotation as needed)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_broadcaster',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
        ),

#        # Static TF odom -> base_link
#        Node(
#        package='tf2_ros',
#        executable='static_transform_publisher',
#        name='odom_broadcaster',
#        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
#            ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/edabot_bringup/config/slam_config2.rviz'],
            output='screen'
        ),
    ])
