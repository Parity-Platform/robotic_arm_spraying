#!/usr/bin/env python3

import os
import sys
import xml.etree.ElementTree as ET
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker, MarkerArray
from tf_transformations import quaternion_from_euler, euler_matrix


def parse_pose(pose_text):
    values = list(map(float, pose_text.strip().split()))
    pos = values[0:3]
    rpy = values[3:6] if len(values) == 6 else [0.0, 0.0, 0.0]
    return pos, rpy


def rotate_and_translate(world_pos, world_rpy, local_pos):
    rot = euler_matrix(*world_rpy)[:3, :3]
    rotated = rot @ np.array(local_pos)
    translated = np.array(world_pos) + rotated
    return translated.tolist()


def combine_pose(world_pos, world_rpy, local_pos, local_rpy):
    position = rotate_and_translate(world_pos, world_rpy, local_pos)
    total_rpy = [w + l for w, l in zip(world_rpy, local_rpy)]
    quat = quaternion_from_euler(*total_rpy)
    return position, quat


class CombinedProcessor(Node):
    def __init__(self, world_file):
        super().__init__('combined_known_unknown_processor')
        self.get_logger().info("Initialized combined known/unknown object processor")
        self.position_offset = np.array([-0.25, 0.0, -0.715])
        self.known_objects = []
        self.subscription_pc = self.create_subscription(
            PointCloud2, '/final_points', self.pointcloud_callback, 10)

        self.known_pub = self.create_publisher(MarkerArray, "/known_objects_markers", 10)
        self.unknown_pub = self.create_publisher(PointCloud2, "/unknown_points", 10)

        self.parse_and_store_known_objects(world_file)

        # Publish known markers repeatedly every 2 seconds
        self.create_timer(2.0, self.publish_known_markers)

    def parse_and_store_known_objects(self, world_file):
        try:
            tree = ET.parse(world_file)
        except Exception as e:
            self.get_logger().error(f"Failed to parse world file: {e}")
            return

        root = tree.getroot()
        includes = root.findall(".//include")
        models = []

        for inc in includes:
            uri = inc.find("uri")
            pose_tag = inc.find("pose")
            if uri is None or not uri.text.startswith("file://"):
                continue
            path = uri.text.replace("file://", "")
            model_path = os.path.join(path, "model.sdf") if os.path.isdir(path) else path
            if not os.path.exists(model_path):
                self.get_logger().warn(f"Model not found: {model_path}")
                continue
            pos, rpy = parse_pose(pose_tag.text) if pose_tag is not None else ([0, 0, 0], [0, 0, 0])
            models.append({"model_path": model_path, "pose": pos, "rpy": rpy})

        for model in models:
            try:
                tree = ET.parse(model["model_path"])
                root = tree.getroot()
                model_elem = root.find(".//model")
                model_name = model_elem.get("name", "unknown")
            except Exception as e:
                self.get_logger().warn(f"Could not parse {model['model_path']}: {e}")
                continue

            for col in root.findall(".//collision"):
                name = col.get("name", "part")
                pose_tag = col.find("pose")
                local_pos, local_rpy = parse_pose(pose_tag.text) if pose_tag is not None else ([0, 0, 0], [0, 0, 0])
                global_pos, quat = combine_pose(model["pose"], model["rpy"], local_pos, local_rpy)

                geom = col.find("geometry")
                if geom is None:
                    continue

                if geom.find("box") is not None:
                    size = list(map(float, geom.find("box/size").text.strip().split()))
                    shape = "box"
                elif geom.find("cylinder") is not None:
                    radius = float(geom.find("cylinder/radius").text)
                    height = float(geom.find("cylinder/length").text)
                    size = [radius, height]
                    shape = "cylinder"
                else:
                    continue

                self.known_objects.append({
                    "id": f"{model_name}::{name}",
                    "shape": shape,
                    "size": size,
                    "position": global_pos,
                    "orientation": quat
                })
                self.get_logger().info(f"[Parsed] {model_name}::{name} as {shape} at {global_pos}")

        self.get_logger().info(f"✅ Stored {len(self.known_objects)} known collision objects.")

    def publish_known_markers(self):
        padding = 0.001  # same padding as filtering
        if not self.known_objects:
            self.get_logger().warn("No known objects to publish as markers.")
            return

        marker_array = MarkerArray()
        for i, obj in enumerate(self.known_objects):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "known_objs"
            marker.id = i
            marker.action = Marker.ADD

            # Apply translation offset here
            pos = np.array(obj["position"]) + self.position_offset
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = pos.tolist()

            qx, qy, qz, qw = obj["orientation"]
            marker.pose.orientation.x = qx
            marker.pose.orientation.y = qy
            marker.pose.orientation.z = qz
            marker.pose.orientation.w = qw

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.6

            if obj["shape"] == "box":
                marker.type = Marker.CUBE
                # Add padding on all sides
                marker.scale.x = obj["size"][0] + padding * 2
                marker.scale.y = obj["size"][1] + padding * 2
                marker.scale.z = obj["size"][2] + padding * 2
            elif obj["shape"] == "cylinder":
                marker.type = Marker.CYLINDER
                # Add padding to radius and height
                marker.scale.x = marker.scale.y = (obj["size"][0] + padding) * 2  # diameter + padding
                marker.scale.z = obj["size"][1] + padding * 2  # height + padding

            marker_array.markers.append(marker)

        self.known_pub.publish(marker_array)

    def quaternion_to_rotation_matrix(self, q):
        qx, qy, qz, qw = q
        # Υπολογισμός πίνακα περιστροφής από quaternion
        R = np.array([
            [1 - 2*qy*qy - 2*qz*qz,     2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw,         1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw,         2*qy*qz + 2*qx*qw,     1 - 2*qx*qx - 2*qy*qy]
        ])
        return R

    def remove_known(self, points):
        padding = 0.001
        mask = np.ones(len(points), dtype=bool)

        for obj in self.known_objects:
            pos = np.array(obj['position']) + self.position_offset
            q = obj['orientation']  # quaternion x,y,z,w

            # Δημιουργούμε πίνακα περιστροφής από quaternion
            R_mat = self.quaternion_to_rotation_matrix(q)

            # Αντίστροφη περιστροφή = μεταθετός πίνακας (orthonormal matrix)
            R_inv = R_mat.T

            if obj['shape'] == "box":
                dx, dy, dz = obj['size']
                dx += padding * 2
                dy += padding * 2
                dz += padding * 2
            elif obj['shape'] == "cylinder":
                r, h = obj['size']
                r += padding
                dx = dy = r * 2
                dz = h + padding * 2
            else:
                continue

            # Μετατοπίζουμε σημεία σχετικά με το κέντρο του αντικειμένου
            pts_local = points - pos

            # Περιστρέφουμε σημεία στον τοπικό χώρο του αντικειμένου
            pts_local_rot = pts_local @ R_inv

            # Έλεγχος αν το σημείο είναι εντός του axis-aligned box (με padding)
            in_box = (
                (pts_local_rot[:, 0] >= -dx / 2) & (pts_local_rot[:, 0] <= dx / 2) &
                (pts_local_rot[:, 1] >= -dy / 2) & (pts_local_rot[:, 1] <= dy / 2) &
                (pts_local_rot[:, 2] >= -dz / 2) & (pts_local_rot[:, 2] <= dz / 2)
            )

            mask &= ~in_box

        return points[mask]

    def pointcloud_callback(self, msg):
        raw_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if len(raw_points) == 0:
            self.get_logger().warn("Empty pointcloud, skipping.")
            return

        try:
            points = np.array([[p[0], p[1], p[2]] for p in raw_points], dtype=np.float32)
        except Exception as e:
            self.get_logger().error(f"Failed to convert pointcloud: {e}")
            return

        if points.ndim != 2 or points.shape[1] != 3:
            self.get_logger().warn("PointCloud format invalid or unexpected shape.")
            return

        self.get_logger().info(f"PointCloud received with {len(points)} points.")

        unknown_points = self.remove_known(points)
        self.get_logger().info(f"Found {len(unknown_points)} unknown points.")
        self.publish_unknown_pointcloud(unknown_points, msg.header)


    def publish_unknown_pointcloud(self, points, header):
        cloud = pc2.create_cloud_xyz32(header, points.tolist())
        self.unknown_pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("Usage: ros2 run spraying_pathways unknown_object_detector.py <path_to_world_file>")
        return
    world_file = sys.argv[1]
    print(f"Starting combined object processor with world file: {world_file}")
    node = CombinedProcessor(world_file)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
