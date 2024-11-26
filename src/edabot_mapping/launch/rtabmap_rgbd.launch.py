from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    parameters=[{
          'frame_id':'zed_camera_link',
          'subscribe_depth':True,
          'subscribe_odom_info':True,
          'approx_sync':False,
          'wait_imu_to_init':False}]

    remappings=[
          ('imu', '/imu/data'),
          ('rgb/image', '/zed/zed_node/rgb/image_rect_color'),
          ('rgb/camera_info', '/zed/zed_node/rgb/camera_info'),
          ('depth/image', '/zed/zed_node/depth/depth_registered')]

    return LaunchDescription([

        # Nodes to launch
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            # arguments=['-d']),
            arguments=['--database_path ~/.ros/rtabmap.db']), # load saved map

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings),
        
        # Because of this issue: https://github.com/IntelRealSense/realsense-ros/issues/2564
        # Generate point cloud from not aligned depth
        Node(
            package='rtabmap_util', executable='point_cloud_xyz', output='screen',
            parameters=[{'approx_sync':False}],
            remappings=[('depth/image',       '/zed/zed_node/depth/depth_registered'),
                        ('depth/camera_info', '/zed/zed_node/depth/camera_info'),
                        ('cloud',             '/zed/zed_node/point_cloud/cloud_registered')]),
        
        # Generate aligned depth to color camera from the point cloud above       
        Node(
            package='rtabmap_util', executable='pointcloud_to_depthimage', output='screen',
            parameters=[{ 'decimation':2,
                          'fixed_frame_id':'zed_camera_link',
                          'fill_holes_size':1}],
            remappings=[('camera_info', '/zed/zed_node/rgb/camera_info'),
                        ('cloud',       '/zed/zed_node/point_cloud/cloud_registered'),
                        ('image_raw',   '/zed/zed_node/depth/depth_registered')]),
        
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