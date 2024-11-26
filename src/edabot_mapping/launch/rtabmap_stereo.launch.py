from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    parameters=[{
          'frame_id':'zed_camera_link',
          'subscribe_stereo':True,
          'subscribe_odom_info':True,
          'wait_imu_to_init':False}]

    remappings=[
          ('imu', '/imu/data'),
          ('left/image_rect', '/zed/zed_node/left/image_rect_color'),
          ('left/camera_info', '/zed/zed_node/left/camera_info'),
          ('right/image_rect', '/zed/zed_node/right/image_rect_color'),
          ('right/camera_info', '/zed/zed_node/right/camera_info')]

    return LaunchDescription([

        # Nodes to launch       
        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings),
                
        # Compute quaternion of the IMU
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 
                         'world_frame': 'enu', 
                         'publish_tf': True}],
            remappings=[('imu/data_raw', '/zed/zed_node/imu/data_raw')]),
        
        # The IMU frame is missing in TF tree, add it:
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'zed_imu_link', 'camera_imu_optical_frame']),
    ])