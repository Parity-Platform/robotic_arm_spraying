#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np
import math

class RotatedPointCloudPublisher(Node):
    
    def __init__(self):
        super().__init__('depth_dip_detector')

        # Subscribe to input point cloud
        self.subscription = self.create_subscription(
            PointCloud2,
            '/transformed_points',
            self.pointcloud_callback,
            10)

        # Publisher for the final problematic points
        self.publisher_problematic_points = self.create_publisher(
            PointCloud2,
            '/problematic_points',
            10)


    def pointcloud_callback(self, msg):
        # Read point cloud
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        problematic_points = []
        standard_h = 0.015

        z_threshold = 0.05 + standard_h
        tolerance = 0.001
        for pt in points:
            if pt[2] < (z_threshold - tolerance):
                problematic_points.append(pt)

        header_final = msg.header
        header_final.frame_id = 'base_link'
        final_msg = pc2.create_cloud_xyz32(header_final, problematic_points)
        self.publisher_problematic_points.publish(final_msg)

        
def main(args=None):
    rclpy.init(args=args)
    node = RotatedPointCloudPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()