from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    edabot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "edabot_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=['./src/edabot_controller/config/twist_mux.yaml'],
        remappings=[('cmd_vel_out', 'edabot_controller/cmd_vel_unstamped')]
    )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        edabot_controller_spawner,
        twist_mux
    ])