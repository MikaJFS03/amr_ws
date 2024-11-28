# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import PointCloud2, PointField
# from std_msgs.msg import String
# import struct
# import numpy as np

# class HumanPointCloudNode(Node):
#     def __init__(self):
#         super().__init__('person')

#         self.coord_subscription = self.create_subscription(
#             String,
#             '/detected_object',
#             self.coord_callback,
#             10
#         )

#         self.human_publisher = self.create_publisher(PointCloud2, '/detected_object/person', 10)

#     def coord_callback(self, msg):
#         data = msg.data.split(',')
#         cls_id = int(data[0])
#         if cls_id == 0:  # Filter manusia
#             x, y, z = map(float, data[1:])
#             points = self.generate_cylinder(x, y, z)
#             self.publish_pointcloud(points)
#         else:
#             self.publish_pointcloud([])

#     def generate_cylinder(self, x, y, z):
#         radius = 0.4
#         height = 1.0
#         resolution = 0.05
#         points = []

#         for h in np.arange(-height / 2, height / 2, resolution):
#             for angle in np.arange(0, 2 * np.pi, resolution / radius):
#                 px = x + radius * np.cos(angle)
#                 py = y + radius * np.sin(angle)
#                 pz = z + h
#                 points.append((px, py, pz))
#         return points

#     def publish_pointcloud(self, points):
#         header = self.create_header("map")
#         fields = [
#             PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
#             PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
#             PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
#         ]

#         cloud_data = [struct.pack('fff', *p) for p in points]
#         pointcloud = PointCloud2(
#             header=header,
#             height=1,
#             width=len(points),
#             is_dense=True,
#             is_bigendian=False,
#             point_step=12,
#             row_step=12 * len(points),
#             fields=fields,
#             data=b''.join(cloud_data)
#         )
#         self.human_publisher.publish(pointcloud)

#     def create_header(self, frame_id):
#         from std_msgs.msg import Header
#         header = Header()
#         header.stamp = self.get_clock().now().to_msg()
#         header.frame_id = frame_id
#         return header

# def main(args=None):
#     rclpy.init(args=args)
#     node = HumanPointCloudNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import PointCloud2, PointField
# from std_msgs.msg import String
# import struct
# import numpy as np

# class HumanPointCloudNode(Node):
#     def __init__(self):
#         super().__init__('human_pointcloud')

#         # Subscription ke topik /detected_object
#         self.coord_subscription = self.create_subscription(
#             String,
#             '/detected_object',
#             self.coord_callback,
#             10
#         )

#         # Publisher ke topik /detected_object/person
#         self.human_publisher = self.create_publisher(PointCloud2, '/detected_object/person', 10)

#     def coord_callback(self, msg):
#         try:
#             # Parsing data dari string
#             data = msg.data.split(',')
#             label = data[0]  # Ambil label objek (string)

#             if label == "person":  # Filter hanya untuk manusia (label = "person")
#                 x, y, z = map(float, data[1:])
#                 points = self.generate_cylinder(x, y, z)
#                 self.publish_pointcloud(points)
#             else:
#                 self.publish_pointcloud([])  # Tidak ada objek manusia, kirim data kosong
#         except ValueError as e:
#             self.get_logger().error(f"Error parsing message: {e}")
#         except Exception as e:
#             self.get_logger().error(f"Unexpected error: {e}")

#     def generate_cylinder(self, x, y, z, radius=0.4, height=1.0, resolution=0.05):
#         """Menghasilkan titik-titik 3D dalam bentuk silinder"""
#         # Menghitung titik-titik silinder menggunakan Numpy
#         angles = np.arange(0, 2 * np.pi, resolution / radius)
#         heights = np.arange(-height / 2, height / 2, resolution)
#         angles_grid, heights_grid = np.meshgrid(angles, heights)

#         px = x + radius * np.cos(angles_grid)
#         py = y + radius * np.sin(angles_grid)
#         pz = z + heights_grid

#         # Flatten array menjadi daftar koordinat 3D
#         points = np.stack((px.flatten(), py.flatten(), pz.flatten()), axis=-1)
#         return points

#     def publish_pointcloud(self, points):
#         """Mempublikasikan PointCloud2 berdasarkan daftar titik 3D"""
#         # Header untuk PointCloud2
#         header = self.create_header("map")

#         # Definisi field PointCloud2
#         fields = [
#             PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
#             PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
#             PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
#         ]

#         # Packing data
#         if len(points) > 0:
#             cloud_data = np.asarray(points, dtype=np.float32).tobytes()
#             pointcloud = PointCloud2(
#                 header=header,
#                 height=1,
#                 width=len(points),
#                 is_dense=True,
#                 is_bigendian=False,
#                 point_step=12,
#                 row_step=12 * len(points),
#                 fields=fields,
#                 data=cloud_data
#             )
#         else:
#             # Jika tidak ada titik, kirim PointCloud kosong
#             pointcloud = PointCloud2(
#                 header=header,
#                 height=0,
#                 width=0,
#                 is_dense=True,
#                 is_bigendian=False,
#                 point_step=12,
#                 row_step=0,
#                 fields=fields,
#                 data=b''
#             )

#         # Publikasi PointCloud2
#         self.human_publisher.publish(pointcloud)

#     def create_header(self, frame_id):
#         """Membuat header standar untuk PointCloud2"""
#         from std_msgs.msg import Header
#         header = Header()
#         header.stamp = self.get_clock().now().to_msg()
#         header.frame_id = frame_id
#         return header

# def main(args=None):
#     rclpy.init(args=args)
#     node = HumanPointCloudNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
# #sudah bisa publish tabung tapi belum multi
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import PointCloud2, PointField
# from std_msgs.msg import String
# import struct
# import numpy as np

# class MultiObjectPointCloudNode(Node):
#     def __init__(self):
#         super().__init__('multi_object_pointcloud')

#         # Subscription ke topik /detected_object
#         self.coord_subscription = self.create_subscription(
#             String,
#             '/detected_object',
#             self.coord_callback,
#             10
#         )

#         # Dictionary untuk menyimpan publisher per label objek
#         self.publishers = {}

#     def coord_callback(self, msg):
#         try:
#             # Parsing data dari string
#             data = msg.data.split(',')
#             label = data[0]  # Ambil label objek (string)

#             # Ambil koordinat 3D
#             x, y, z = map(float, data[1:])
#             points = self.generate_cylinder(x, y, z)

#             # Publikasikan PointCloud untuk label objek ini
#             self.publish_pointcloud(label, points)
#         except ValueError as e:
#             self.get_logger().error(f"Error parsing message: {e}")
#         except Exception as e:
#             self.get_logger().error(f"Unexpected error: {e}")

#     def generate_cylinder(self, x, y, z, radius=0.4, height=1.0, resolution=0.05):
#         """Menghasilkan titik-titik 3D dalam bentuk silinder"""
#         # Menghitung titik-titik silinder menggunakan Numpy
#         angles = np.arange(0, 2 * np.pi, resolution / radius)
#         heights = np.arange(-height / 2, height / 2, resolution)
#         angles_grid, heights_grid = np.meshgrid(angles, heights)

#         px = x + radius * np.cos(angles_grid)
#         py = y + radius * np.sin(angles_grid)
#         pz = z + heights_grid

#         # Flatten array menjadi daftar koordinat 3D
#         points = np.stack((px.flatten(), py.flatten(), pz.flatten()), axis=-1)
#         return points

#     def publish_pointcloud(self, label, points):
#         """Mempublikasikan PointCloud2 untuk label tertentu"""
#         if label not in self.publishers:
#             # Buat publisher baru untuk label jika belum ada
#             self.publishers[label] = self.create_publisher(PointCloud2, f'/detected_object/{label}', 10)

#         # Header untuk PointCloud2
#         header = self.create_header("map")

#         # Definisi field PointCloud2
#         fields = [
#             PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
#             PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
#             PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
#         ]

#         # Packing data
#         if len(points) > 0:
#             cloud_data = np.asarray(points, dtype=np.float32).tobytes()
#             pointcloud = PointCloud2(
#                 header=header,
#                 height=1,
#                 width=len(points),
#                 is_dense=True,
#                 is_bigendian=False,
#                 point_step=12,
#                 row_step=12 * len(points),
#                 fields=fields,
#                 data=cloud_data
#             )
#         else:
#             # Jika tidak ada titik, kirim PointCloud kosong
#             pointcloud = PointCloud2(
#                 header=header,
#                 height=0,
#                 width=0,
#                 is_dense=True,
#                 is_bigendian=False,
#                 point_step=12,
#                 row_step=0,
#                 fields=fields,
#                 data=b''
#             )

#         # Publikasi PointCloud2 ke topik label
#         self.publishers[label].publish(pointcloud)

#     def create_header(self, frame_id):
#         """Membuat header standar untuk PointCloud2"""
#         from std_msgs.msg import Header
#         header = Header()
#         header.stamp = self.get_clock().now().to_msg()
#         header.frame_id = frame_id
#         return header

# def main(args=None):
#     rclpy.init(args=args)
#     node = MultiObjectPointCloudNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import MarkerArray
import numpy as np


class MultiObjectPointCloudNode(Node):
    def __init__(self):
        super().__init__('person_pointcloud')

        # Subscription ke topik /detected_visual
        self.marker_subscription = self.create_subscription(
            MarkerArray,
            '/detected_object/visual',
            self.marker_callback,
            10
        )

        # Publisher untuk PointCloud2
        self.pointcloud_publisher = self.create_publisher(PointCloud2, '/detected_object/person', 10)

    def marker_callback(self, msg):
        try:
            all_points = []

            # Loop melalui semua marker
            for marker in msg.markers:
                # Filter hanya objek dengan label "person"
                if marker.text == "person":
                    x = marker.pose.position.x
                    y = marker.pose.position.y
                    z = marker.pose.position.z

                    # Generate silinder untuk objek "person"
                    points = self.generate_cylinder(x, y, z)
                    all_points.extend(points)

            # Publikasikan semua titik sebagai PointCloud2
            self.publish_pointcloud(all_points)

        except Exception as e:
            self.get_logger().error(f"Error processing markers: {e}")

    def generate_cylinder(self, x, y, z, radius=0.4, height=1.0, resolution=0.02):
        """Menghasilkan titik-titik 3D dalam bentuk silinder"""
        # Menghitung titik-titik silinder menggunakan Numpy
        angles = np.arange(0, 2 * np.pi, resolution / radius)
        heights = np.arange(-height / 2, height / 2, resolution)
        angles_grid, heights_grid = np.meshgrid(angles, heights)

        px = x + radius * np.cos(angles_grid)
        py = y + radius * np.sin(angles_grid)
        pz = z + heights_grid

        # Flatten array menjadi daftar koordinat 3D
        points = np.stack((px.flatten(), py.flatten(), pz.flatten()), axis=-1)
        return points

    def publish_pointcloud(self, points):
        """Mempublikasikan PointCloud2 berdasarkan daftar titik 3D"""
        # Header untuk PointCloud2
        header = self.create_header("map")

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
        self.pointcloud_publisher.publish(pointcloud)

    def create_header(self, frame_id):
        """Membuat header standar untuk PointCloud2"""
        from std_msgs.msg import Header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        return header


def main(args=None):
    rclpy.init(args=args)
    node = MultiObjectPointCloudNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

