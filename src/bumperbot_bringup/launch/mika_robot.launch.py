# file: 2ndmika_robot.launch.py   (contoh nama)
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # -------------------
    # Declare launch arguments (configurable dari CLI)
    # -------------------
    default_map = os.path.join(
        get_package_share_directory("bumperbot_mapping"),
        "maps",
        "small_house",
        "map.yaml"
    )
    declare_map_yaml = DeclareLaunchArgument(
        "map_yaml",
        default_value=default_map,
        description="Path to map yaml file"
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulated clock"
    )

    declare_rviz = DeclareLaunchArgument(
        "rviz",
        default_value="True",
        description="Launch RViz2"
    )

    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value="True",
        description="Automatically startup nav2 lifecycle nodes"
    )

    # LaunchConfigurations
    map_yaml = LaunchConfiguration("map_yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_enabled = LaunchConfiguration("rviz")
    autostart = LaunchConfiguration("autostart")

    # -------------------
    # Hardware interface (include)
    # -------------------
    hardware_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bumperbot_firmware"),
                "launch",
                "hardware_interface.launch.py"
            )
        ),
    )

    # -------------------
    # RPLIDAR node
    # -------------------
    laser_driver = Node(
        package="rplidar_ros",
        executable="rplidar_composition",
        name="rplidar_node",
        parameters=[os.path.join(
            get_package_share_directory("bumperbot_bringup"),
            "config",
            "rplidar_a1.yaml"
        )],
        output="screen"
    )

    # -------------------
    # Controller include
    # -------------------
    controller_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bumperbot_controller"),
                "launch",
                "controller.launch.py"
            )
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False"
        }.items()
    )

    # -------------------
    # Joystick teleop include
    # -------------------
    joystick_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bumperbot_controller"),
                "launch",
                "joystick_teleop.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time
        }.items()
    )

    # -------------------
    # Navigation include (nav2)
    # -------------------
    navigation_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bumperbot_navigation"),
                "launch",
                "navigation.launch.py"
            )
        ),
    )

    # -------------------
    # Teleop keyboard (optionally open xterm)
    # -------------------
    keyboard_teleop = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        prefix='xterm -e',  # Opens in new terminal window
        remappings=[('cmd_vel', 'key_vel')],
        output='screen'
    )

    imu_driver_node = Node(
        package="bumperbot_firmware",
        executable="mpu6050_driver.py"
    )


    # -------------------
    # RViz (optional)
    # -------------------
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    rviz_launch_path = os.path.join(nav2_bringup_share, 'launch', 'rviz_launch.py')
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch_path),
        condition=IfCondition(rviz_enabled)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_mapping"),
            "launch",
            "slam.launch.py"
        ),
    )
    # -------------------
    # Build LaunchDescription
    # -------------------
    ld = LaunchDescription()

    # Add declared args
    ld.add_action(declare_map_yaml)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_rviz)
    ld.add_action(declare_autostart)

    # Add nodes/includes
    ld.add_action(keyboard_teleop)
    ld.add_action(hardware_interface_launch)
    ld.add_action(laser_driver)
    ld.add_action(controller_include)
    ld.add_action(joystick_include)
    ld.add_action(navigation_include)
    ld.add_action(rviz)
    ld.add_action(slam)
    ld.add_action(imu_driver_node)
    # optional: log the chosen map path at startup
    ld.add_action(LogInfo(msg=["Using map: ", map_yaml]))

    return ld
