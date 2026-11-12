#!/usr/bin/env python3
"""
save_cones_to_csv.py
--------------------
Subscribes to /mpc/track_markers (visualization_msgs/MarkerArray), extracts
the left-cone (namespace "cones_left") and right-cone (namespace "cones_right")
SPHERE_LIST markers, and writes all cone positions to a CSV file.

Usage
-----
  ros2 run mpc_controller save_cones_to_csv            # saves to ~/cones.csv
  ros2 run mpc_controller save_cones_to_csv --ros-args \
      -p output_file:=/path/to/output.csv              # custom path
  ros2 run mpc_controller save_cones_to_csv --ros-args \
      -p once:=false                                   # keep updating on every message

CSV format
----------
  side, x, y, z
  left, 1.23, 4.56, 0.15
  right, ...
"""

import os
import csv
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import MarkerArray


class SaveConesToCsv(Node):
    def __init__(self):
        super().__init__('save_cones_to_csv')

        self.declare_parameter('output_file',
                               os.path.expanduser('~/cones.csv'))
        self.declare_parameter('once', True)   # save on first message then quit

        self._output_file = self.get_parameter('output_file').value
        self._once        = self.get_parameter('once').value
        self._saved       = False

        # Match the transient_local QoS used by the visualizer publisher
        qos = QoSProfile(depth=10,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self._sub = self.create_subscription(
            MarkerArray,
            '/mpc/track_markers',
            self._callback,
            qos)

        self.get_logger().info(
            f'Waiting for /mpc/track_markers … output → {self._output_file}')

    # -------------------------------------------------------------------------
    def _callback(self, msg: MarkerArray):
        if self._saved and self._once:
            return

        left_cones  = []
        right_cones = []

        for marker in msg.markers:
            if marker.ns == 'cones_left':
                for p in marker.points:
                    left_cones.append(('left', p.x, p.y, p.z))
            elif marker.ns == 'cones_right':
                for p in marker.points:
                    right_cones.append(('right', p.x, p.y, p.z))

        if not left_cones and not right_cones:
            self.get_logger().warn(
                'MarkerArray received but no cones_left / cones_right markers found.')
            return

        rows = left_cones + right_cones

        os.makedirs(os.path.dirname(os.path.abspath(self._output_file)),
                    exist_ok=True)

        with open(self._output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['side', 'x', 'y', 'z'])
            for row in rows:
                writer.writerow(row)

        self.get_logger().info(
            f'Saved {len(left_cones)} left + {len(right_cones)} right cones '
            f'to {self._output_file}')

        self._saved = True
        if self._once:
            raise SystemExit


# -----------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = SaveConesToCsv()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
