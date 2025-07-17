#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion
from datetime import datetime
from pathlib import Path
import math
import csv
import time


class TrajectoryLogger(Node):
    def __init__(self):
        super().__init__('trajectory_logger')

        # Logging directory
        default_log_dir = str(Path.home() / 'ros2_ws/src/spraying_pathways/robot_logs')
        self.declare_parameter('log_dir', default_log_dir)
        log_dir_param = self.get_parameter('log_dir').get_parameter_value().string_value
        self.log_dir = Path(log_dir_param)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f'Logging trajectory data to: {self.log_dir}')

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Robot state
        self.joint_names = []
        self.joint_positions = {}
        self.joint_velocities = {}
        self.last_positions = {}

        # Logging state
        self.logging = False
        self.csv_file = None
        self.csv_writer = None

        # Motion tracking
        self.position_change_threshold = 1e-4  # Euclidean threshold
        self.motion_time_required = 0.5        # Seconds
        self.last_position_change_time = None

        # End-effector frame
        self.end_effector_frame = None
        self.detect_end_effector_frame()

        # Subscriptions
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)

        # Timer
        self.create_timer(0.25, self.log_data)  # 25 Hz

    def detect_end_effector_frame(self):
        self.get_logger().info('Detecting end-effector frame from TF...')
        time.sleep(2.0)  # Wait for TF to initialize
        try:
            frames_yaml = self.tf_buffer.all_frames_as_yaml()
            candidate_frames = []

            for line in frames_yaml.splitlines():
                if 'child_frame_id' in line:
                    frame = line.split(':')[-1].strip()
                    if frame not in ['base_link', 'world', 'odom', 'map'] and not frame.startswith('camera'):
                        candidate_frames.append(frame)

            if candidate_frames:
                self.end_effector_frame = max(candidate_frames, key=len)
                self.get_logger().info(f'Using end-effector frame: {self.end_effector_frame}')
            else:
                raise RuntimeError('No suitable end-effector frame found.')

        except Exception as e:
            self.get_logger().warn(f'EE frame detection failed, using default "tool0": {e}')
            self.end_effector_frame = 'tool0'

    def joint_state_cb(self, msg):
        self.joint_names = msg.name
        now = self.get_clock().now().nanoseconds * 1e-9

        current_positions = [msg.position[i] for i in range(len(msg.name))]
        prev_positions = [self.last_positions.get(name, pos) for name, pos in zip(msg.name, current_positions)]

        # Euclidean distance
        distance = math.sqrt(sum((curr - prev) ** 2 for curr, prev in zip(current_positions, prev_positions)))
        #self.get_logger().debug(f"[Motion Check] Δq (Euclidean) = {distance:.8f}")

        if distance > self.position_change_threshold:
            #self.get_logger().info(f"Motion detected (Δq = {distance:.6f}) → Timer reset")
            self.last_position_change_time = now
        #else:
            #self.get_logger().debug("No significant motion")

        # Update joint data
        for i, name in enumerate(msg.name):
            self.joint_positions[name] = msg.position[i]
            self.joint_velocities[name] = msg.velocity[i] if i < len(msg.velocity) else 0.0
            self.last_positions[name] = msg.position[i]

    def start_logging(self):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = self.log_dir / f'trajectory_log_{timestamp}.csv'
        self.csv_file = open(filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        joint_headers = []
        for name in self.joint_names:
            joint_headers.extend([f'{name}_pos', f'{name}_vel'])

        self.csv_writer.writerow(
            ['time'] + joint_headers + ['ee_x', 'ee_y', 'ee_z', 'ee_roll', 'ee_pitch', 'ee_yaw']
        )

        self.logging = True
        self.get_logger().info(f' Started logging to {filename}')

    def stop_logging(self):
        self.logging = False
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
            self.get_logger().info('Stopped logging and saved CSV.')

    def log_data(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        if self.last_position_change_time is None:
            return

        time_since_change = now - self.last_position_change_time
        self.get_logger().debug(f"[State] time_since_change = {time_since_change:.2f}s  logging={self.logging}")

        if not self.logging and time_since_change < self.motion_time_required:
            self.start_logging()

        if self.logging and time_since_change > self.motion_time_required:
            self.stop_logging()

        if not self.logging or not self.csv_writer:
            return

        joint_values = []
        for name in self.joint_names:
            pos = self.joint_positions.get(name, 0.0)
            vel = self.joint_velocities.get(name, 0.0)
            joint_values.extend([pos, vel])

        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                self.end_effector_frame,
                rclpy.time.Time()
            )
            t = transform.transform.translation
            r = transform.transform.rotation
            roll, pitch, yaw = euler_from_quaternion([r.x, r.y, r.z, r.w])
            ee_values = [t.x, t.y, t.z, roll, pitch, yaw]
        except Exception as e:
            self.get_logger().warn_once(f'EE transform not available: {e}')
            ee_values = [0.0] * 6

        self.csv_writer.writerow([now] + joint_values + ee_values)

    def destroy_node(self):
        if self.csv_file:
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.set_logger_level('trajectory_logger', rclpy.logging.LoggingSeverity.DEBUG)
    node = TrajectoryLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()