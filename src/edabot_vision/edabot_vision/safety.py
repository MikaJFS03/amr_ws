import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import time


class SafetyPointCloud(Node):
    def __init__(self):
        super().__init__('safety_point_cloud')
        
        # Parameters
        self.declare_parameter('image_topic', '/zed/zed_node/left/image_rect_color')
        self.declare_parameter('depth_topic', '/zed/zed_node/depth/depth_registered')
        self.declare_parameter('camera_info_topic', '/zed/zed_node/left/camera_info')
        self.image_topic = self.get_parameter('image_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value

        # ROS2 communication
        self.bridge = CvBridge()
        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )
        self.depth_subscription = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )
        self.point_cloud_publisher = self.create_publisher(
            PointCloud2,
            '/point_cloud',
            10
        )

        # Variables
        self.depth_image = None
        self.camera_info = None
        self.pointcloud_buffer = []
        self.pointcloud_lifetime = 0.6  # Lifetime in seconds
        self.get_logger().info(f'Subscribed to image topic: {self.image_topic}')
        self.get_logger().info(f'Subscribed to depth topic: {self.depth_topic}')
        self.get_logger().info(f'Subscribed to camera info topic: {self.camera_info_topic}')

    def camera_info_callback(self, msg):
        try:
            # Extract intrinsic camera parameters
            self.camera_info = {
                'fx': msg.k[0],  # Focal length x
                'fy': msg.k[4],  # Focal length y
                'cx': msg.k[2],  # Principal point x
                'cy': msg.k[5]   # Principal point y
            }
            self.get_logger().info(f"Camera info received: {self.camera_info}")
        except Exception as e:
            self.get_logger().error(f"Error processing camera info: {e}")

    def image_callback(self, msg):
        if self.camera_info is None:
            self.get_logger().warn("Camera info not received yet.")
            return
        
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            height, width, _ = cv_image.shape

            # Calculate spacing and offset for points
            horizontal_spacing = width // 64
            vertical_spacing = height // 10
            horizontal_offset = 32
            vertical_offset = 36

            # Collect 2D points
            points_2d = []
            for y in range(vertical_offset, height, vertical_spacing):
                for x in range(horizontal_offset, width, horizontal_spacing):
                    points_2d.append((x, y))

            # Check depth and calculate 3D coordinates
            if self.depth_image is not None:
                fx, fy = self.camera_info['fx'], self.camera_info['fy']
                cx, cy = self.camera_info['cx'], self.camera_info['cy']

                points_3d = []
                current_time = time.time()
                for x, y in points_2d:
                    depth = self.depth_image[y, x]
                    if 0.35 < depth < 2.1:  # Ensure valid depth
                        X = depth
                        Y = -((x - cx) * depth / fx) 
                        Z = -((y - cy) * depth / fy)
                        points_3d.append((X, Y, Z))

                # Add points to buffer with timestamp
                self.pointcloud_buffer.append((current_time, points_3d))

                # Clean up points older than 2 seconds
                self.clean_pointcloud_buffer(current_time)

                # Publish PointCloud2
                all_points = []
                for _, points in self.pointcloud_buffer:
                    all_points.extend(points)

                self.publish_pointcloud(all_points)

            # Display the image with points
            for x, y in points_2d:
                cv2.circle(cv_image, (x, y), 3, (0, 255, 0), -1)
            cv2.imshow('Processed Image', cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def depth_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV depth image
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {e}")

    def clean_pointcloud_buffer(self, current_time):
        """Menghapus titik-titik dari buffer yang sudah melewati waktu tinggal"""
        self.pointcloud_buffer = [
            (timestamp, points)
            for timestamp, points in self.pointcloud_buffer
            if current_time - timestamp <= self.pointcloud_lifetime
        ]

    def publish_pointcloud(self, points):
        """Mempublikasikan PointCloud2 berdasarkan daftar titik 3D"""
        # Header untuk PointCloud2
        header = self.create_header("zed_left_camera_frame")

        # Definisi field PointCloud2
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # Packing data
        if len(points) > 0:
            cloud_data = np.asarray(points, dtype=np.float32).tobytes()
            pointcloud = PointCloud2(
                header=header,
                height=1,
                width=len(points),
                is_dense=True,
                is_bigendian=False,
                point_step=12,
                row_step=12 * len(points),
                fields=fields,
                data=cloud_data
            )
        else:
            # Jika tidak ada titik, kirim PointCloud kosong
            pointcloud = PointCloud2(
                header=header,
                height=0,
                width=0,
                is_dense=True,
                is_bigendian=False,
                point_step=12,
                row_step=0,
                fields=fields,
                data=b''
            )

        # Publikasi PointCloud2
        self.point_cloud_publisher.publish(pointcloud)

    def create_header(self, frame_id):
        """Membuat header standar untuk PointCloud2"""
        from std_msgs.msg import Header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        return header


def main(args=None):
    rclpy.init(args=args)
    node = SafetyPointCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()