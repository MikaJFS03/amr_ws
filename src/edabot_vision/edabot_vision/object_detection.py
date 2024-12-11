import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
from ultralytics import YOLO
import tf2_ros
import tf2_geometry_msgs
import cv2


class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('object_detector')

        # Publisher untuk koordinat objek sebagai string
        self.coord_publisher = self.create_publisher(String, '/detected_object', 10)

        # Publisher untuk visualisasi di RViz
        self.marker_array_publisher = self.create_publisher(MarkerArray, '/detected_object/visual', 10)

        # Subscription untuk gambar dan kedalaman
        self.image_subscription = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.image_callback,
            10
        )
        self.depth_subscription = self.create_subscription(
            Image,
            '/zed/zed_node/depth/depth_registered',
            self.depth_callback,
            10
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/zed/zed_node/left/camera_info',
            self.camera_info_callback,
            10
        )

        # TF Listener untuk transformasi
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.bridge = CvBridge()
        self.depth_image = None
        self.camera_info = None
        self.model = YOLO('yolov8m-seg.pt')  # Gunakan model segmentasi

    def image_callback(self, msg):
        # Konversi gambar ke format OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Periksa apakah depth_image tersedia
        if self.depth_image is None:
            self.get_logger().warn("Depth image is not yet available. Skipping this frame.")
            return  # Abaikan jika depth_image belum tersedia

        # Dapatkan dimensi asli gambar
        height_orig, width_orig = cv_image.shape[:2]

        # Jalankan deteksi menggunakan YOLO segmentasi
        results = self.model(cv_image)

        # Buat array marker
        marker_array = MarkerArray()

        for result in results:
            for i, score in enumerate(result.boxes.conf):
                if score >= 0.7:  # Filter berdasarkan confidence score
                    cls_id = int(result.boxes.cls[i])
                    label = self.model.names[cls_id]  # Ambil nama kelas

                    # Ambil mask segmentasi
                    mask = result.masks.data[i].cpu().numpy()  # YOLOv8 mask dalam format numpy
                    mask = (mask * 255).astype(np.uint8)  # Konversi ke format 8-bit untuk OpenCV

                    # **Resize mask ke resolusi asli gambar**
                    mask_resized = cv2.resize(mask, (width_orig, height_orig), interpolation=cv2.INTER_NEAREST)

                    # Temukan kontur dari mask
                    contours, _ = cv2.findContours(mask_resized, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if contours:
                        # Ambil kontur terbesar
                        largest_contour = max(contours, key=cv2.contourArea)

                        # Hitung centroid kontur
                        M = cv2.moments(largest_contour)
                        if M['m00'] != 0:
                            center_x = int(M['m10'] / M['m00'])
                            center_y = int(M['m01'] / M['m00'])

                            # Validasi indeks agar tetap dalam batas dimensi citra depth
                            if (0 <= center_y < self.depth_image.shape[0] and 
                                0 <= center_x < self.depth_image.shape[1]):
                                depth_value = self.depth_image[center_y, center_x]
                                if not np.isnan(depth_value):
                                    x3d, y3d, z3d = self.get_3d_position(center_x, center_y, depth_value)
                                    transformed_point = self.transform_to_map(x3d, y3d, z3d)
                                    if transformed_point:
                                        # Publikasikan koordinat objek
                                        tx, ty, tz = transformed_point
                                        message = f"{label}, {tx:.2f}, {ty:.2f}, {tz:.2f}"
                                        self.coord_publisher.publish(String(data=message))

                                        # Tambahkan marker ke array
                                        marker = self.create_marker(tx, ty, tz, label, i)
                                        marker_array.markers.append(marker)

                            # Gambar kontur pada gambar
                            cv2.drawContours(cv_image, [largest_contour], -1, (0, 255, 0), 2)  # Hijau

                            # Tampilkan label dan confidence pada centroid
                            text = f"{label} {score:.2f}"
                            cv2.putText(cv_image, text, (center_x, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        else:
                            self.get_logger().warn(
                                f"Centroid ({center_x}, {center_y}) out of bounds for depth image size {self.depth_image.shape}"
                            )

        # Publikasikan semua marker sekaligus
        self.marker_array_publisher.publish(marker_array)

        # Tampilkan frame dengan deteksi dan segmentasi secara live
        cv2.imshow("YOLOv8 Segmentation", cv_image)
        cv2.waitKey(1)



    def depth_callback(self, msg):
        # Simpan data depth image
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

    def camera_info_callback(self, msg):
        # Simpan informasi kamera
        self.camera_info = msg

    def get_3d_position(self, x, y, depth):
        # Hitung koordinat 3D dari koordinat piksel
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        X = float(depth)
        Y = float(-((x - cx) * depth / fx))
        Z = float((y - cy) * depth / fy)

        return X, Y, Z

    def transform_to_map(self, x, y, z):
        try:
            # Membuat PointStamped di kerangka kamera
            point_camera = PointStamped()
            point_camera.header.frame_id = "zed_camera_link"
            point_camera.header.stamp = self.get_clock().now().to_msg()
            point_camera.point.x = x
            point_camera.point.y = y
            point_camera.point.z = z

            # Transformasi ke kerangka 'map'
            transform = self.tf_buffer.lookup_transform(
                "map",  # Target frame
                "zed_camera_link",  # Source frame
                rclpy.time.Time()  # Waktu
            )

            point_map = tf2_geometry_msgs.do_transform_point(point_camera, transform)
            return point_map.point.x, point_map.point.y, point_map.point.z
        except Exception as e:
            self.get_logger().warn(f"Transform error: {e}")
            return None

    def create_marker(self, x, y, z, label, marker_id):
        # Membuat marker untuk RViz
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "detected_objects"
        marker.id = marker_id  # ID unik untuk setiap objek
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2  # Ukuran marker
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Transparansi
        marker.text = label

        return marker


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()