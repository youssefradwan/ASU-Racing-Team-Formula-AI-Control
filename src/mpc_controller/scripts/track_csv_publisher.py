#!/usr/bin/env python3
"""
track_csv_publisher.py
Reads a CSV file with x,y waypoints (one per row, no header) and publishes
a nav_msgs/Path on /path using TRANSIENT_LOCAL QoS so late subscribers
(like the MPC controller) always receive the full path on connect.

Usage (standalone):
    ros2 run mpc_controller track_csv_publisher --ros-args -p csv_path:=/abs/path/track.csv

Usage (via launch):
    ros2 launch mpc_controller track_publisher.launch.py
    ros2 launch mpc_controller track_publisher.launch.py csv_path:=/abs/path/track.csv
"""

import csv
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class TrackCsvPublisher(Node):
    def __init__(self):
        super().__init__('track_csv_publisher')

        # ── Parameter: path to CSV file ──────────────────────────────────
        self.declare_parameter('csv_path', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate_hz', 1.0)  # re-publish periodically
                                                         # so any late subscriber gets it

        csv_path    = self.get_parameter('csv_path').get_parameter_value().string_value
        frame_id    = self.get_parameter('frame_id').get_parameter_value().string_value
        pub_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        if not csv_path:
            self.get_logger().fatal(
                'Parameter "csv_path" is empty. '
                'Pass it with: --ros-args -p csv_path:=/path/to/track.csv')
            sys.exit(1)

        # ── Load waypoints from CSV ───────────────────────────────────────
        self.path_msg_ = self._load_csv(csv_path, frame_id)
        if self.path_msg_ is None:
            sys.exit(1)

        self.get_logger().info(
            f'Loaded {len(self.path_msg_.poses)} waypoints from {csv_path}')

        # ── Publisher: TRANSIENT_LOCAL so MPC receives path on connect ────
        # Must match MPC controller's subscriber QoS (transient_local, reliable)
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub_ = self.create_publisher(Path, '/path', qos)

        # Publish immediately once, then at publish_rate_hz
        self._publish()
        period = 1.0 / max(pub_rate_hz, 0.1)
        self.timer_ = self.create_timer(period, self._publish)

        self.get_logger().info(
            f'Publishing /path at {pub_rate_hz:.1f} Hz '
            f'(frame_id={frame_id}, {len(self.path_msg_.poses)} poses)')

    # ─────────────────────────────────────────────────────────────────────
    def _load_csv(self, csv_path: str, frame_id: str):
        """Parse the CSV and build a nav_msgs/Path message."""
        poses = []
        try:
            with open(csv_path, 'r') as f:
                reader = csv.reader(f)
                for line_no, row in enumerate(reader, start=1):
                    # Skip blank lines and comment lines
                    if not row or row[0].strip().startswith('#'):
                        continue
                    # Skip header row if it contains letters
                    if not row[0].strip().replace('.', '', 1).replace('-', '', 1).lstrip('-').replace('e', '', 1).replace('E', '', 1).replace('+', '', 1).isdigit():
                        self.get_logger().debug(f'Skipping header row {line_no}: {row}')
                        continue
                    if len(row) < 2:
                        self.get_logger().warn(
                            f'Line {line_no}: expected at least 2 columns, got {len(row)} — skipping')
                        continue
                    try:
                        x = float(row[0].strip())
                        y = float(row[1].strip())
                    except ValueError as e:
                        self.get_logger().warn(f'Line {line_no}: parse error ({e}) — skipping')
                        continue

                    ps = PoseStamped()
                    ps.header.frame_id = frame_id
                    ps.pose.position.x = x
                    ps.pose.position.y = y
                    ps.pose.position.z = 0.0
                    # Heading: will be filled after all points are loaded
                    poses.append(ps)

        except FileNotFoundError:
            self.get_logger().fatal(f'CSV file not found: {csv_path}')
            return None
        except Exception as e:
            self.get_logger().fatal(f'Error reading CSV: {e}')
            return None

        if len(poses) < 2:
            self.get_logger().fatal(
                f'CSV must contain at least 2 waypoints, got {len(poses)}')
            return None

        # Fill orientations: each pose faces the next waypoint
        for i in range(len(poses)):
            if i < len(poses) - 1:
                dx = poses[i + 1].pose.position.x - poses[i].pose.position.x
                dy = poses[i + 1].pose.position.y - poses[i].pose.position.y
            else:
                dx = poses[i].pose.position.x - poses[i - 1].pose.position.x
                dy = poses[i].pose.position.y - poses[i - 1].pose.position.y
            yaw = math.atan2(dy, dx)
            # Quaternion from yaw (roll=0, pitch=0)
            poses[i].pose.orientation.z = math.sin(yaw / 2.0)
            poses[i].pose.orientation.w = math.cos(yaw / 2.0)

        path = Path()
        path.header.frame_id = frame_id
        path.poses = poses
        return path

    # ─────────────────────────────────────────────────────────────────────
    def _publish(self):
        """Stamp and publish the path."""
        now = self.get_clock().now().to_msg()
        self.path_msg_.header.stamp = now
        for ps in self.path_msg_.poses:
            ps.header.stamp = now
        self.pub_.publish(self.path_msg_)


def main(args=None):
    rclpy.init(args=args)
    node = TrackCsvPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
