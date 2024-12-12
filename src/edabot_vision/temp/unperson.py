#sudah multi dan sudah decay (ngambil dari person)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import MarkerArray
import numpy as np
import time


class MultiObjectPointCloudNode(Node):
    def __init__(self):
        super().__init__('unperson_pointcloud')

        # Subscription ke topik /detected_visual
        self.marker_subscription = self.create_subscription(
            MarkerArray,
            '/detected_object/visual',
            self.marker_callback,
            10
        )

        # Publisher untuk PointCloud2
        self.pointcloud_publisher = self.create_publisher(PointCloud2, '/detected_object/unperson', 10)

        # Buffer untuk menyimpan titik-titik dan timestamp
        self.pointcloud_buffer = []
        self.pointcloud_lifetime = 2.2  # Waktu tinggal dalam detik

    def marker_callback(self, msg):
        try:
            current_time = time.time()
            all_points = []

            # Loop melalui semua marker
            for marker in msg.markers:
                # Filter hanya objek dengan label "person"
                if marker.text != "person":
                    x = marker.pose.position.x
                    y = marker.pose.position.y
                    z = marker.pose.position.z

                    # Generate silinder untuk objek "unperson"
                    points = self.generate_cylinder(x, y, z)
                    # Tambahkan titik dan waktu pembuatan ke buffer
                    self.pointcloud_buffer.append((current_time, points))

            # Bersihkan buffer dari titik yang sudah melewati waktu tinggal
            self.clean_pointcloud_buffer(current_time)

            # Gabungkan semua titik dari buffer
            for _, points in self.pointcloud_buffer:
                all_points.extend(points)

            # Publikasikan semua titik sebagai PointCloud2
            self.publish_pointcloud(all_points)

        except Exception as e:
            self.get_logger().error(f"Error processing markers: {e}")

    def generate_cylinder(self, x, y, z, radius=0.33, height=1.0, resolution=0.02):
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
