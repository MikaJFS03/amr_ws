from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    frame_id = LaunchConfiguration('frame_id')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('frame_id', default_value='lidar_link'),

        # Driver LiDAR (ganti dengan urg_node/sick_scan_xd bila perlu)
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            parameters=[{
                'serial_port': serial_port,
                'serial_baudrate': 115200,  # A1:115200, A2/A3:256000
                'frame_id': frame_id,
                'inverted': False,
                'angle_compensate': True
            }],
            output='screen'
        ),

        # SLAM Toolbox (online mapping)
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            parameters=['/home/ubuntu/amar_ws/src/amar_lidar_mapping/config/slam_toolbox.yaml'],
            output='screen'
        ),
    ])
