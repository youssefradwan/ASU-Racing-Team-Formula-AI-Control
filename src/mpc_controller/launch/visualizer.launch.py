"""
visualizer.launch.py — Standalone RViz visualizer for IPG CarMaker

Launches the mpc_visualizer node + RViz.
Assumes the MPC controller is already running and publishing on the topics
defined in  config/nodes.json.

Run:
  ros2 launch mpc_controller visualizer.launch.py
"""

import os
import json
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('mpc_controller')
    cfg_dir = os.path.join(pkg_dir, 'config')

    # ── Read node name and model params from JSON (for launch-time use only) ─
    node_name   = 'mpc_visualizer'
    r_inner     = 1.5
    r_outer     = 1.5
    car_length  = 2.8
    car_width   = 1.4
    wheelbase   = 1.575
    cone_spacing = 5
    try:
        with open(os.path.join(cfg_dir, 'nodes.json')) as f:
            nodes_cfg = json.load(f)
        node_name    = nodes_cfg.get('node_names', {}).get('mpc_visualizer', node_name)
        cone_spacing = int(nodes_cfg.get('ros_params', {}).get('cone_spacing', cone_spacing))
    except Exception:
        pass
    try:
        with open(os.path.join(cfg_dir, 'model.json')) as f:
            m = json.load(f)
            r_inner    = float(m.get('r_inner',    r_inner))
            r_outer    = float(m.get('r_outer',    r_outer))
            car_length = float(m.get('car_length', car_length))
            car_width  = float(m.get('car_width',  car_width))
            wheelbase  = float(m.get('wheelbase',  wheelbase))
    except Exception:
        pass

    rviz_cfg = os.path.join(cfg_dir, 'mpc_test.rviz')

    viz_node = Node(
        package='mpc_controller',
        executable='mpc_visualizer',
        name=node_name,
        output='screen',
        parameters=[{
            # All topic names are read at runtime from nodes.json by the node itself
            'nodes_path':   os.path.join(cfg_dir, 'nodes.json'),
            # Vehicle geometry (loaded from model.json at launch time)
            'car_length':   car_length,
            'car_width':    car_width,
            'wheelbase':    wheelbase,
            'track_width':  (r_inner + r_outer) / 2.0,
            'r_inner':      r_inner,
            'r_outer':      r_outer,
            'cone_spacing': cone_spacing,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
        additional_env={'GTK_PATH': ''},
    )

    return LaunchDescription([
        viz_node,
        rviz_node,
    ])
