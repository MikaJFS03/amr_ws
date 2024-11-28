# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, CameraInfo
# from std_msgs.msg import String
# from cv_bridge import CvBridge
# import numpy as np
# from ultralytics import YOLO

# class ObjectDetectorNode(Node):
#     def __init__(self):
#         super().__init__('object_detector')

#         # Publisher untuk koordinat objek
#         self.coord_publisher = self.create_publisher(String, '/detected_object', 10)

#         # Subscription untuk gambar
#         self.image_subscription = self.create_subscription(
#             Image,
#             '/zed/zed_node/rgb/image_rect_color',
#             self.image_callback,
#             10
#         )
        
#         self.depth_subscription = self.create_subscription(
#             Image,
#             '/zed/zed_node/depth/depth_registered',
#             self.depth_callback,
#             10
#         )

#         self.camera_info_subscription = self.create_subscription(
#             CameraInfo,
#             '/zed/zed_node/left/camera_info',
#             self.camera_info_callback,
#             10
#         )

#         self.bridge = CvBridge()
#         self.depth_image = None
#         self.camera_info = None
#         self.model = YOLO('yolov8n.pt')

#     def image_callback(self, msg):
#         cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         results = self.model(cv_image)

#         for result in results:
#             for i, score in enumerate(result.boxes.conf):
#                 if score >= 0.8:  # Filter berdasarkan confidence score
#                     cls_id = int(result.boxes.cls[i])
#                     u, v, _, _ = result.boxes.xyxy[i].int().tolist()
#                     if self.depth_image is not None:
#                         depth_value = self.depth_image[v, u]
#                         if np.isnan(depth_value):
#                             continue
#                         x3d, y3d, z3d = self.get_3d_position(u, v, depth_value)
#                         message = f"{cls_id},{x3d},{y3d},{z3d}"
#                         self.coord_publisher.publish(String(data=message))

#     def depth_callback(self, msg):
#         self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

#     def camera_info_callback(self, msg):
#         self.camera_info = msg

#     def get_3d_position(self, x, y, depth):
#         fx = self.camera_info.k[0]
#         fy = self.camera_info.k[4]
#         cx = self.camera_info.k[2]
#         cy = self.camera_info.k[5]

#         X = depth
#         Y = (x - cx) * depth / fx
#         Z = (y - cy) * depth / fy

#         return X, Y, Z

# def main(args=None):
#     rclpy.init(args=args)
#     node = ObjectDetectorNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, CameraInfo
# from geometry_msgs.msg import PointStamped
# from std_msgs.msg import String
# from cv_bridge import CvBridge
# import numpy as np
# from ultralytics import YOLO
# import tf2_ros
# import tf2_geometry_msgs

# class ObjectDetectorNode(Node):
#     def __init__(self):
#         super().__init__('object_detector')

#         # Publisher untuk koordinat objek
#         self.coord_publisher = self.create_publisher(String, '/detected_object', 10)

#         # Subscription untuk gambar dan kedalaman
#         self.image_subscription = self.create_subscription(
#             Image,
#             '/zed/zed_node/rgb/image_rect_color',
#             self.image_callback,
#             10
#         )
#         self.depth_subscription = self.create_subscription(
#             Image,
#             '/zed/zed_node/depth/depth_registered',
#             self.depth_callback,
#             10
#         )
#         self.camera_info_subscription = self.create_subscription(
#             CameraInfo,
#             '/zed/zed_node/left/camera_info',
#             self.camera_info_callback,
#             10
#         )

#         # TF Listener untuk transformasi
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

#         self.bridge = CvBridge()
#         self.depth_image = None
#         self.camera_info = None
#         self.model = YOLO('yolov8n.pt')

#     def image_callback(self, msg):
#         cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         results = self.model(cv_image)

#         for result in results:
#             for i, score in enumerate(result.boxes.conf):
#                 if score >= 0.8:  # Filter berdasarkan confidence score
#                     cls_id = int(result.boxes.cls[i])
#                     u, v, _, _ = result.boxes.xyxy[i].int().tolist()

#                     if self.depth_image is not None:
#                         depth_value = self.depth_image[v, u]
#                         if np.isnan(depth_value):
#                             continue

#                         x3d, y3d, z3d = self.get_3d_position(u, v, depth_value)
#                         transformed_point = self.transform_to_map(x3d, y3d, z3d)

#                         if transformed_point:
#                             tx, ty, tz = transformed_point
#                             message = f"{cls_id},{tx},{ty},{tz}"
#                             self.coord_publisher.publish(String(data=message))

#     def depth_callback(self, msg):
#         self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

#     def camera_info_callback(self, msg):
#         self.camera_info = msg

#     def get_3d_position(self, x, y, depth):
#         fx = self.camera_info.k[0]
#         fy = self.camera_info.k[4]
#         cx = self.camera_info.k[2]
#         cy = self.camera_info.k[5]

#         X = float(depth)
#         Y = float(-((x - cx) * depth / fx))
#         Z = float((y - cy) * depth / fy)

#         return X, Y, Z

#     def transform_to_map(self, x, y, z):
#         try:
#             # Membuat PointStamped di kerangka kamera
#             point_camera = PointStamped()
#             point_camera.header.frame_id = "zed_camera_link"
#             point_camera.header.stamp = self.get_clock().now().to_msg()
#             point_camera.point.x = x
#             point_camera.point.y = y
#             point_camera.point.z = z

#             # Transformasi ke kerangka 'map'
#             transform = self.tf_buffer.lookup_transform(
#                 "map",  # Target frame
#                 "zed_camera",  # Source frame
#                 rclpy.time.Time()  # Waktu
#             )

#             point_map = tf2_geometry_msgs.do_transform_point(point_camera, transform)
#             return point_map.point.x, point_map.point.y, point_map.point.z
#         except Exception as e:
#             self.get_logger().warn(f"Transform error: {e}")
#             return None

# def main(args=None):
#     rclpy.init(args=args)
#     node = ObjectDetectorNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, CameraInfo
# from geometry_msgs.msg import PointStamped
# from std_msgs.msg import String
# from cv_bridge import CvBridge
# import numpy as np
# from ultralytics import YOLO
# import tf2_ros
# import tf2_geometry_msgs
# import cv2

# class ObjectDetectorNode(Node):
#     def __init__(self):
#         super().__init__('object_detector')

#         # Publisher untuk koordinat objek
#         self.coord_publisher = self.create_publisher(String, '/detected_object', 10)

#         # Subscription untuk gambar dan kedalaman
#         self.image_subscription = self.create_subscription(
#             Image,
#             '/zed/zed_node/rgb/image_rect_color',
#             self.image_callback,
#             10
#         )
#         self.depth_subscription = self.create_subscription(
#             Image,
#             '/zed/zed_node/depth/depth_registered',
#             self.depth_callback,
#             10
#         )
#         self.camera_info_subscription = self.create_subscription(
#             CameraInfo,
#             '/zed/zed_node/left/camera_info',
#             self.camera_info_callback,
#             10
#         )

#         # TF Listener untuk transformasi
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

#         self.bridge = CvBridge()
#         self.depth_image = None
#         self.camera_info = None
#         self.model = YOLO('yolov8n-seg.pt')  # Gunakan model segmentasi

#     def image_callback(self, msg):
#         # Konversi gambar ke format OpenCV
#         cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#         # Jalankan deteksi menggunakan YOLO segmentasi
#         results = self.model(cv_image)

#         for result in results:
#             for i, score in enumerate(result.boxes.conf):
#                 if score >= 0.8:  # Filter berdasarkan confidence score
#                     cls_id = int(result.boxes.cls[i])
#                     u, v, w, h = result.boxes.xyxy[i].int().tolist()

#                     # Visualisasi: gambar bounding box pada frame
#                     label = f"{self.model.names[cls_id]}: {score:.2f}"
#                     cv2.rectangle(cv_image, (u, v), (w, h), (0, 255, 0), 2)
#                     cv2.putText(cv_image, label, (u, v - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

#                     # Gambar segmentasi sebagai poligon
#                     if result.masks is not None:  # Pastikan ada mask yang terdeteksi
#                         mask = result.masks.data[i].cpu().numpy()  # Ambil mask sebagai numpy array
#                         mask = (mask * 255).astype(np.uint8)  # Ubah ke format uint8 untuk OpenCV

#                         # Buat kontur dari mask
#                         contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#                         for contour in contours:
#                             cv2.polylines(cv_image, [contour], isClosed=True, color=(0, 0, 255), thickness=2)

#                     # Hitung koordinat 3D jika depth image tersedia
#                     if self.depth_image is not None:
#                         depth_value = self.depth_image[v, u]
#                         if np.isnan(depth_value):
#                             continue

#                         x3d, y3d, z3d = self.get_3d_position(u, v, depth_value)
#                         transformed_point = self.transform_to_map(x3d, y3d, z3d)

#                         if transformed_point:
#                             tx, ty, tz = transformed_point
#                             message = f"{cls_id},{tx},{ty},{tz}"
#                             self.coord_publisher.publish(String(data=message))

#         # Tampilkan frame dengan deteksi dan segmentasi secara live
#         cv2.imshow("YOLOv8 Segmentation", cv_image)
#         cv2.waitKey(1)

#     def depth_callback(self, msg):
#         # Simpan data depth image
#         self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

#     def camera_info_callback(self, msg):
#         # Simpan informasi kamera
#         self.camera_info = msg

#     def get_3d_position(self, x, y, depth):
#         # Hitung koordinat 3D dari koordinat piksel
#         fx = self.camera_info.k[0]
#         fy = self.camera_info.k[4]
#         cx = self.camera_info.k[2]
#         cy = self.camera_info.k[5]

#         X = float(depth)
#         Y = float(-((x - cx) * depth / fx))
#         Z = float((y - cy) * depth / fy)

#         return X, Y, Z

#     def transform_to_map(self, x, y, z):
#         try:
#             # Membuat PointStamped di kerangka kamera
#             point_camera = PointStamped()
#             point_camera.header.frame_id = "zed_camera_link"
#             point_camera.header.stamp = self.get_clock().now().to_msg()
#             point_camera.point.x = x
#             point_camera.point.y = y
#             point_camera.point.z = z

#             # Transformasi ke kerangka 'map'
#             transform = self.tf_buffer.lookup_transform(
#                 "map",  # Target frame
#                 "zed_camera",  # Source frame
#                 rclpy.time.Time()  # Waktu
#             )

#             point_map = tf2_geometry_msgs.do_transform_point(point_camera, transform)
#             return point_map.point.x, point_map.point.y, point_map.point.z
#         except Exception as e:
#             self.get_logger().warn(f"Transform error: {e}")
#             return None

# def main(args=None):
#     rclpy.init(args=args)
#     node = ObjectDetectorNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
#     cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main()

# #yang perlu diperbaiki: pesan yang dikirim kordinat dan kelasnya
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, CameraInfo
# from geometry_msgs.msg import PointStamped
# from std_msgs.msg import String
# from cv_bridge import CvBridge
# import numpy as np
# from ultralytics import YOLO
# import tf2_ros
# import tf2_geometry_msgs
# import cv2

# class ObjectDetectorNode(Node):
#     def __init__(self):
#         super().__init__('object_detector')

#         # Publisher untuk koordinat objek
#         self.coord_publisher = self.create_publisher(String, '/detected_object', 10)

#         # Subscription untuk gambar dan kedalaman
#         self.image_subscription = self.create_subscription(
#             Image,
#             '/zed/zed_node/rgb/image_rect_color',
#             self.image_callback,
#             10
#         )
#         self.depth_subscription = self.create_subscription(
#             Image,
#             '/zed/zed_node/depth/depth_registered',
#             self.depth_callback,
#             10
#         )
#         self.camera_info_subscription = self.create_subscription(
#             CameraInfo,
#             '/zed/zed_node/left/camera_info',
#             self.camera_info_callback,
#             10
#         )

#         # TF Listener untuk transformasi
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

#         self.bridge = CvBridge()
#         self.depth_image = None
#         self.camera_info = None
#         self.model = YOLO('yolov8n-seg.pt')  # Gunakan model segmentasi

#     def image_callback(self, msg):
#         # Konversi gambar ke format OpenCV
#         cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#         # Jalankan deteksi menggunakan YOLO segmentasi
#         results = self.model(cv_image)

#         for result in results:
#             for i, score in enumerate(result.boxes.conf):
#                 if score >= 0.2:  # Filter berdasarkan confidence score
#                     cls_id = int(result.boxes.cls[i])
#                     label = f"{self.model.names[cls_id]}: {score:.2f}"

#                     # Ambil bounding box koordinat
#                     x1, y1, x2, y2 = result.boxes.xyxy[i].int().tolist()

#                     # Visualisasi: gambar bounding box pada frame
#                     cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
#                     cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

#                     # Gambar segmentasi sebagai poligon
#                     if result.masks is not None:  # Pastikan ada mask yang terdeteksi
#                         mask = result.masks.data[i].cpu().numpy()  # Ambil mask sebagai numpy array
#                         mask_resized = cv2.resize(mask, (cv_image.shape[1], cv_image.shape[0]))  # Sesuaikan ukuran mask
#                         mask_binary = (mask_resized * 255).astype(np.uint8)  # Konversi ke uint8

#                         # Cari kontur dari mask
#                         contours, _ = cv2.findContours(mask_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#                         # Gambar kontur pada frame
#                         cv2.drawContours(cv_image, contours, -1, (0, 0, 255), 2)

#                     # Hitung koordinat 3D jika depth image tersedia
#                     if self.depth_image is not None:
#                         center_x = (x1 + x2) // 2
#                         center_y = (y1 + y2) // 2
#                         depth_value = self.depth_image[center_y, center_x]
#                         if np.isnan(depth_value):
#                             continue

#                         x3d, y3d, z3d = self.get_3d_position(center_x, center_y, depth_value)
#                         transformed_point = self.transform_to_map(x3d, y3d, z3d)

#                         if transformed_point:
#                             tx, ty, tz = transformed_point
#                             message = f"{cls_id},{tx},{ty},{tz}"
#                             self.coord_publisher.publish(String(data=message))

#         # Tampilkan frame dengan deteksi dan segmentasi secara live
#         cv2.imshow("YOLOv8 Segmentation", cv_image)
#         cv2.waitKey(1)

#     def depth_callback(self, msg):
#         # Simpan data depth image
#         self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

#     def camera_info_callback(self, msg):
#         # Simpan informasi kamera
#         self.camera_info = msg

#     def get_3d_position(self, x, y, depth):
#         # Hitung koordinat 3D dari koordinat piksel
#         fx = self.camera_info.k[0]
#         fy = self.camera_info.k[4]
#         cx = self.camera_info.k[2]
#         cy = self.camera_info.k[5]

#         X = float(depth)
#         Y = float(-((x - cx) * depth / fx))
#         Z = float((y - cy) * depth / fy)

#         return X, Y, Z

#     def transform_to_map(self, x, y, z):
#         try:
#             # Membuat PointStamped di kerangka kamera
#             point_camera = PointStamped()
#             point_camera.header.frame_id = "zed_camera_link"
#             point_camera.header.stamp = self.get_clock().now().to_msg()
#             point_camera.point.x = x
#             point_camera.point.y = y
#             point_camera.point.z = z

#             # Transformasi ke kerangka 'map'
#             transform = self.tf_buffer.lookup_transform(
#                 "map",  # Target frame
#                 "zed_camera_link",  # Source frame
#                 rclpy.time.Time()  # Waktu
#             )

#             point_map = tf2_geometry_msgs.do_transform_point(point_camera, transform)
#             return point_map.point.x, point_map.point.y, point_map.point.z
#         except Exception as e:
#             self.get_logger().warn(f"Transform error: {e}")
#             return None

# def main(args=None):
#     rclpy.init(args=args)
#     node = ObjectDetectorNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
#     cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main()


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, CameraInfo
# from geometry_msgs.msg import PointStamped
# from std_msgs.msg import String
# from cv_bridge import CvBridge
# import numpy as np
# from ultralytics import YOLO
# import tf2_ros
# import tf2_geometry_msgs
# import cv2


# class ObjectDetectorNode(Node):
#     def __init__(self):
#         super().__init__('object_detector')

#         # Publisher untuk koordinat objek
#         self.coord_publisher = self.create_publisher(String, '/detected_object', 10)

#         # Subscription untuk gambar dan kedalaman
#         self.image_subscription = self.create_subscription(
#             Image,
#             '/zed/zed_node/rgb/image_rect_color',
#             self.image_callback,
#             10
#         )
#         self.depth_subscription = self.create_subscription(
#             Image,
#             '/zed/zed_node/depth/depth_registered',
#             self.depth_callback,
#             10
#         )
#         self.camera_info_subscription = self.create_subscription(
#             CameraInfo,
#             '/zed/zed_node/left/camera_info',
#             self.camera_info_callback,
#             10
#         )

#         # TF Listener untuk transformasi
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

#         self.bridge = CvBridge()
#         self.depth_image = None
#         self.camera_info = None
#         self.model = YOLO('yolov8n-seg.pt')  # Gunakan model segmentasi

#     def image_callback(self, msg):
#         # Konversi gambar ke format OpenCV
#         cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#         # Jalankan deteksi menggunakan YOLO segmentasi
#         results = self.model(cv_image)

#         for result in results:
#             for i, score in enumerate(result.boxes.conf):
#                 if score >= 0.8:  # Filter berdasarkan confidence score
#                     cls_id = int(result.boxes.cls[i])
#                     label = self.model.names[cls_id]  # Ambil nama kelas

#                     # Ambil bounding box koordinat
#                     x1, y1, x2, y2 = result.boxes.xyxy[i].int().tolist()

#                     # Visualisasi: gambar bounding box pada frame
#                     # cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
#                     # cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

#                     # Gambar segmentasi sebagai poligon
#                     if result.masks is not None:  # Pastikan ada mask yang terdeteksi
#                         mask = result.masks.data[i].cpu().numpy()  # Mask dalam format array [H, W]
#                         mask_resized = cv2.resize(mask, (cv_image.shape[1], cv_image.shape[0]))  
#                         mask_binary = (mask_resized > 0.5).astype(np.uint8)  # Threshold untuk binary mask

#                         # Cari kontur dari mask
#                         contours, _ = cv2.findContours(mask_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#                         # Gambar kontur pada frame
#                         cv2.drawContours(cv_image, contours, -1, (0, 0, 255), 2)  # Warna merah untuk kontur

#                     # Hitung koordinat 3D jika depth image tersedia
#                     if self.depth_image is not None:
#                         center_x = (x1 + x2) // 2
#                         center_y = (y1 + y2) // 2
#                         depth_value = self.depth_image[center_y, center_x]
#                         if np.isnan(depth_value):
#                             continue

#                         x3d, y3d, z3d = self.get_3d_position(center_x, center_y, depth_value)
#                         transformed_point = self.transform_to_map(x3d, y3d, z3d)

#                         if transformed_point:
#                             tx, ty, tz = transformed_point
#                             # Membuat pesan dengan kelas dan koordinat
#                             message = f"{label}, {tx:.2f}, {ty:.2f}, {tz:.2f}"
#                             self.coord_publisher.publish(String(data=message))

#         # Tampilkan frame dengan deteksi dan segmentasi secara live
#         cv2.imshow("YOLOv8 Segmentation", cv_image)
#         cv2.waitKey(1)

#     def depth_callback(self, msg):
#         # Simpan data depth image
#         self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

#     def camera_info_callback(self, msg):
#         # Simpan informasi kamera
#         self.camera_info = msg

#     def get_3d_position(self, x, y, depth):
#         # Hitung koordinat 3D dari koordinat piksel
#         fx = self.camera_info.k[0]
#         fy = self.camera_info.k[4]
#         cx = self.camera_info.k[2]
#         cy = self.camera_info.k[5]

#         X = float(depth)
#         Y = float(-((x - cx) * depth / fx))
#         Z = float((y - cy) * depth / fy)

#         return X, Y, Z

#     def transform_to_map(self, x, y, z):
#         try:
#             # Membuat PointStamped di kerangka kamera
#             point_camera = PointStamped()
#             point_camera.header.frame_id = "zed_camera_link"
#             point_camera.header.stamp = self.get_clock().now().to_msg()
#             point_camera.point.x = x
#             point_camera.point.y = y
#             point_camera.point.z = z

#             # Transformasi ke kerangka 'map'
#             transform = self.tf_buffer.lookup_transform(
#                 "map",  # Target frame
#                 "zed_camera_link",  # Source frame
#                 rclpy.time.Time()  # Waktu
#             )

#             point_map = tf2_geometry_msgs.do_transform_point(point_camera, transform)
#             return point_map.point.x, point_map.point.y, point_map.point.z
#         except Exception as e:
#             self.get_logger().warn(f"Transform error: {e}")
#             return None


# def main(args=None):
#     rclpy.init(args=args)
#     node = ObjectDetectorNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
#     cv2.destroyAllWindows()


# if __name__ == '__main__':
#     main()

# #sudah jadi namun belum multi objek
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, CameraInfo
# from geometry_msgs.msg import PointStamped
# from visualization_msgs.msg import Marker
# from std_msgs.msg import String
# from cv_bridge import CvBridge
# import numpy as np
# from ultralytics import YOLO
# import tf2_ros
# import tf2_geometry_msgs
# import cv2


# class ObjectDetectorNode(Node):
#     def __init__(self):
#         super().__init__('object_detector')

#         # Publisher untuk koordinat objek sebagai string
#         self.coord_publisher = self.create_publisher(String, '/detected_object', 10)

#         # Publisher untuk visualisasi di RViz
#         self.marker_publisher = self.create_publisher(Marker, '/detected_object/visual', 10)

#         # Subscription untuk gambar dan kedalaman
#         self.image_subscription = self.create_subscription(
#             Image,
#             '/zed/zed_node/rgb/image_rect_color',
#             self.image_callback,
#             10
#         )
#         self.depth_subscription = self.create_subscription(
#             Image,
#             '/zed/zed_node/depth/depth_registered',
#             self.depth_callback,
#             10
#         )
#         self.camera_info_subscription = self.create_subscription(
#             CameraInfo,
#             '/zed/zed_node/left/camera_info',
#             self.camera_info_callback,
#             10
#         )

#         # TF Listener untuk transformasi
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

#         self.bridge = CvBridge()
#         self.depth_image = None
#         self.camera_info = None
#         self.model = YOLO('yolov8n-seg.pt')  # Gunakan model segmentasi

#     def image_callback(self, msg):
#         # Konversi gambar ke format OpenCV
#         cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#         # Jalankan deteksi menggunakan YOLO segmentasi
#         results = self.model(cv_image)

#         for result in results:
#             for i, score in enumerate(result.boxes.conf):
#                 if score >= 0.2:  # Filter berdasarkan confidence score
#                     cls_id = int(result.boxes.cls[i])
#                     label = self.model.names[cls_id]  # Ambil nama kelas

#                     # Ambil bounding box koordinat
#                     x1, y1, x2, y2 = result.boxes.xyxy[i].int().tolist()

#                     # Visualisasi: gambar bounding box pada frame
#                     cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
#                     cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

#                     # Gambar segmentasi sebagai poligon
#                     if result.masks is not None:  # Pastikan ada mask yang terdeteksi
#                         mask = result.masks.data[i].cpu().numpy()  # Mask dalam format array [H, W]
#                         mask_resized = cv2.resize(mask, (cv_image.shape[1], cv_image.shape[0]))  
#                         mask_binary = (mask_resized > 0.5).astype(np.uint8)  # Threshold untuk binary mask

#                         # Cari kontur dari mask
#                         contours, _ = cv2.findContours(mask_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#                         # Gambar kontur pada frame
#                         cv2.drawContours(cv_image, contours, -1, (0, 0, 255), 2)  # Warna merah untuk kontur

#                     # Hitung koordinat 3D jika depth image tersedia
#                     if self.depth_image is not None:
#                         center_x = (x1 + x2) // 2
#                         center_y = (y1 + y2) // 2
#                         depth_value = self.depth_image[center_y, center_x]
#                         if np.isnan(depth_value):
#                             continue

#                         x3d, y3d, z3d = self.get_3d_position(center_x, center_y, depth_value)
#                         transformed_point = self.transform_to_map(x3d, y3d, z3d)

#                         if transformed_point:
#                             tx, ty, tz = transformed_point
#                             # Membuat pesan dengan kelas dan koordinat
#                             message = f"{label}, {tx:.2f}, {ty:.2f}, {tz:.2f}"
#                             self.coord_publisher.publish(String(data=message))

#                             # Publikasi marker untuk visualisasi di RViz
#                             self.publish_marker(tx, ty, tz, label)

#         # Tampilkan frame dengan deteksi dan segmentasi secara live
#         cv2.imshow("YOLOv8 Segmentation", cv_image)
#         cv2.waitKey(1)

#     def depth_callback(self, msg):
#         # Simpan data depth image
#         self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

#     def camera_info_callback(self, msg):
#         # Simpan informasi kamera
#         self.camera_info = msg

#     def get_3d_position(self, x, y, depth):
#         # Hitung koordinat 3D dari koordinat piksel
#         fx = self.camera_info.k[0]
#         fy = self.camera_info.k[4]
#         cx = self.camera_info.k[2]
#         cy = self.camera_info.k[5]

#         X = float(depth)
#         Y = float(-((x - cx) * depth / fx))
#         Z = float((y - cy) * depth / fy)

#         return X, Y, Z

#     def transform_to_map(self, x, y, z):
#         try:
#             # Membuat PointStamped di kerangka kamera
#             point_camera = PointStamped()
#             point_camera.header.frame_id = "zed_camera_link"
#             point_camera.header.stamp = self.get_clock().now().to_msg()
#             point_camera.point.x = x
#             point_camera.point.y = y
#             point_camera.point.z = z

#             # Transformasi ke kerangka 'map'
#             transform = self.tf_buffer.lookup_transform(
#                 "map",  # Target frame
#                 "zed_camera_link",  # Source frame
#                 rclpy.time.Time()  # Waktu
#             )

#             point_map = tf2_geometry_msgs.do_transform_point(point_camera, transform)
#             return point_map.point.x, point_map.point.y, point_map.point.z
#         except Exception as e:
#             self.get_logger().warn(f"Transform error: {e}")
#             return None

#     def publish_marker(self, x, y, z, label):
#         # Membuat marker untuk RViz
#         marker = Marker()
#         marker.header.frame_id = "map"
#         marker.header.stamp = self.get_clock().now().to_msg()
#         marker.ns = "detected_objects"
#         marker.id = int(hash(label) % 1000)  # ID unik untuk setiap objek
#         marker.type = Marker.SPHERE
#         marker.action = Marker.ADD
#         marker.pose.position.x = x
#         marker.pose.position.y = y
#         marker.pose.position.z = z
#         marker.pose.orientation.x = 0.0
#         marker.pose.orientation.y = 0.0
#         marker.pose.orientation.z = 0.0
#         marker.pose.orientation.w = 1.0
#         marker.scale.x = 0.2  # Ukuran marker
#         marker.scale.y = 0.2
#         marker.scale.z = 0.2
#         marker.color.r = 0.0
#         marker.color.g = 1.0
#         marker.color.b = 0.0
#         marker.color.a = 1.0  # Transparansi
#         marker.text = label

#         # Publikasi marker
#         self.marker_publisher.publish(marker)


# def main(args=None):
#     rclpy.init(args=args)
#     node = ObjectDetectorNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
#     cv2.destroyAllWindows()


# if __name__ == '__main__':
#     main()

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
        self.model = YOLO('yolov8n-seg.pt')  # Gunakan model segmentasi

    def image_callback(self, msg):
        # Konversi gambar ke format OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Jalankan deteksi menggunakan YOLO segmentasi
        results = self.model(cv_image)

        # Buat array marker
        marker_array = MarkerArray()

        for result in results:
            for i, score in enumerate(result.boxes.conf):
                if score >= 0.2:  # Filter berdasarkan confidence score
                    cls_id = int(result.boxes.cls[i])
                    label = self.model.names[cls_id]  # Ambil nama kelas

                    # Ambil bounding box koordinat
                    x1, y1, x2, y2 = result.boxes.xyxy[i].int().tolist()

                    # Hitung koordinat 3D jika depth image tersedia
                    if self.depth_image is not None:
                        # Hitung koordinat tengah bounding box
                        center_x = (x1 + x2) // 2
                        center_y = (y1 + y2) // 2

                        # Validasi indeks agar tetap dalam batas dimensi citra
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
                        else:
                            self.get_logger().warn(f"Center coordinate ({center_x}, {center_y}) out of bounds for depth image size {self.depth_image.shape}")

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
