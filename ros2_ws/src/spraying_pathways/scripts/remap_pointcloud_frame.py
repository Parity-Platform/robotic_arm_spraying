#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import std_msgs.msg

class RotatePointCloud(Node):
    def __init__(self):
        super().__init__('rotate_pointcloud_node')

        # Subscribe to the raw point cloud topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/wrist2_cam/wrist2_depth_cam/points',
            self.listener_callback,
            10)

        # Publisher for the rotated point cloud
        self.publisher = self.create_publisher(PointCloud2, '/wrist2_cam/points_rotated', 10)

        # Define a 90-degree rotation matrix around Y-axis
        angle_rad = np.pi / 2
        self.rotation_matrix = np.array([
            [np.cos(angle_rad), 0, np.sin(angle_rad)],
            [0, 1, 0],
            [-np.sin(angle_rad), 0, np.cos(angle_rad)]
        ])

    def listener_callback(self, msg):
        # Extract points
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        rotated_points = []

        for pt in points:
            vec = np.array([pt[0], pt[1], pt[2]])
            rotated = self.rotation_matrix @ vec
            intensity = 128.0  # mid-gray intensity
            rotated_points.append((rotated[0], rotated[1], rotated[2], intensity))

        # Create header
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'wrist_2_camera_link'

        # Define fields: x, y, z, intensity
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        # Create and publish the new point cloud
        cloud = pc2.create_cloud(header, fields, rotated_points)
        self.publisher.publish(cloud)

def main(args=None):
    rclpy.init(args=args)
    node = RotatePointCloud()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()