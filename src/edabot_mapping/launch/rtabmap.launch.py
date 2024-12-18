import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    edabot_mapping_dir = get_package_share_directory("edabot_mapping");
    rtabmap_launch_dir = get_package_share_directory("rtabmap_launch");

    rtabmap_launch = IncludeLaunchDescription(
        os.path.join(rtabmap_launch_dir, "launch", "rtabmap.launch.py"),
        launch_arguments={
            # 'args': '--delete_db_on_start',
            'args': '--database_path ~/.ros/rtabmap.db',
            'namespace': '',
            'rtabmap_viz': 'false',
            'rviz': 'false',

            # define mode
            'localization': 'true',
            'icp_odometry': 'false',
            'visual_odometry': 'true',
            'publish_tf_map': 'true',
            'publish_tf_odom': 'true',
            "wait_imu_to_init": "true",

            # based on rtabmap forum
            'approx_sync': 'false',
            'rgbd_sync': 'true',
            'approx_rgbd_sync': 'false',
            # 'topic_queue_size': '2',

            # define subscribe parameter
            'depth': 'true',
            'stereo': 'false',
            # 'subscribe_rgbd': 'true',
            # 'subscribe_rgb': 'true',
            'subscribe_scan': 'false',
            'subscribe_scan_cloud': 'false',

            # define frame
            'frame_id': 'zed_camera_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',

            # define topics
            'rgb_topic': '/zed/zed_node/rgb/image_rect_color',
            'depth_topic': '/zed/zed_node/depth/depth_registered',
            'camera_info_topic': '/zed/zed_node/rgb/camera_info',
            'odom_topic': '/odom',
            'imu_topic': '/zed/zed_node/imu/data',
        }.items()
    )

    return LaunchDescription([
        rtabmap_launch
    ])