"""
mpc_controller.launch.py — Real car / IPG CarMaker launch

Launches ONLY the MPC controller node.
Expects external system (IPG CarMaker) to publish:
  - /carmaker/Odometry  (nav_msgs/Odometry)  — vehicle state  (configurable in nodes.json)
  - /path               (nav_msgs/Path)       — track centerline (configurable in nodes.json)

The controller publishes:
  - /ackr               (AckermannDriveStamped) — steering + speed commands
  - /mpc/predicted_path (nav_msgs/Path)          — MPC horizon for visualizer

All topic names, node name, control frequency, and CSV path are read at
runtime from  config/nodes.json  — edit that file, then rebuild.

Run:
  ros2 launch mpc_controller mpc_controller.launch.py
  ros2 launch mpc_controller mpc_controller.launch.py csv_enabled:=false
"""

import os
import json
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('mpc_controller')
    cfg_dir = os.path.join(pkg_dir, 'config')

    # ── Read node_name from nodes.json (for the ROS node name only) ──────
    node_name = 'mpc_controller'
    try:
        with open(os.path.join(cfg_dir, 'nodes.json')) as f:
            nodes_cfg = json.load(f)
        node_name = nodes_cfg.get('node_names', {}).get('mpc_controller', node_name)
    except Exception:
        pass

    ws_root = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.dirname(pkg_dir))))
    lap_dir = os.path.join(ws_root, 'src', 'MPC_CONTROLLERRRR', 'lap_tests')

    csv_arg = DeclareLaunchArgument(
        'csv_enabled', default_value='true',
        description='Enable lap CSV logging (true/false)')

    mpc_node = Node(
        package='mpc_controller',
        executable='mpc_controller_node',
        name=node_name,
        output='screen',
        parameters=[{
            # JSON file paths — all topic names and runtime params are inside these files
            'nodes_path':  os.path.join(cfg_dir, 'nodes.json'),
            'model_path':  os.path.join(cfg_dir, 'model.json'),
            'costs_path':  os.path.join(cfg_dir, 'cost.json'),
            'bounds_path': os.path.join(cfg_dir, 'bounds.json'),
            'norm_path':   os.path.join(cfg_dir, 'normalization.json'),
            # CSV logging
            'csv_enabled': LaunchConfiguration('csv_enabled'),
            'csv_lap_dir': lap_dir,
        }],
    )

    return LaunchDescription([
        csv_arg,
        mpc_node,
    ])
