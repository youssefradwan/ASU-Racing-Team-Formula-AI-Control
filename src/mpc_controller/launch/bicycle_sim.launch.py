"""
bicycle_sim.launch.py — Standalone bicycle simulator

Launches ONLY the bicycle simulator node.
It will:
  • Load the track from the CSV file (track_csv parameter)
  • Publish the track on /path (transient_local, replays for late subscribers)
  • Publish vehicle odometry on /carmaker/Odometry
  • Subscribe to Ackermann commands on /ackr

All numeric parameters are loaded from JSON config files (no hardcoded values here).

Run:
  ros2 launch mpc_controller bicycle_sim.launch.py
  ros2 launch mpc_controller bicycle_sim.launch.py track_csv:=/path/to/track.csv
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

    # ── Load nodes.json: node name, topic names, sim_dt ─────────────────
    node_name  = 'bicycle_simulator'
    topic_ackr = '/ackr'
    topic_odom = '/carmaker/Odometry'
    topic_path = '/path'
    sim_dt     = 0.01
    try:
        with open(os.path.join(cfg_dir, 'nodes.json')) as f:
            nodes_cfg = json.load(f)
        node_name  = nodes_cfg.get('node_names', {}).get('bicycle_simulator', node_name)
        topics     = nodes_cfg.get('topics', {})
        topic_ackr = topics.get('ackermann_cmd', topic_ackr)
        topic_odom = topics.get('odometry',      topic_odom)
        topic_path = topics.get('reference_path', topic_path)
        sim_dt     = float(nodes_cfg.get('ros_params', {}).get('sim_dt', sim_dt))
    except Exception:
        pass

    # ── Load model.json: wheelbase ───────────────────────────────────────
    wheelbase = 1.575
    try:
        with open(os.path.join(cfg_dir, 'model.json')) as f:
            m = json.load(f)
            wheelbase = float(m.get('wheelbase', wheelbase))
    except Exception:
        pass

    # ── Load bounds.json: v_max, delta_max ───────────────────────────────
    v_max     = 15.0
    delta_max = 0.6109
    try:
        with open(os.path.join(cfg_dir, 'bounds.json')) as f:
            b = json.load(f)
            v_max     = float(b.get('v_max',     v_max))
            delta_max = float(b.get('delta_max', delta_max))
    except Exception:
        pass

    default_track = os.path.join(cfg_dir, 'track.csv')

    track_csv_arg = DeclareLaunchArgument(
        'track_csv', default_value=default_track,
        description='Path to track centerline CSV (x,y per row). '
                    'Leave empty to use generated oval.')
    initial_v_arg = DeclareLaunchArgument(
        'initial_v', default_value='2.0',
        description='Initial vehicle speed [m/s]')

    sim_node = Node(
        package='mpc_controller',
        executable='bicycle_simulator',
        name=node_name,
        output='screen',
        parameters=[{
            'wheelbase':      wheelbase,
            'sim_dt':         sim_dt,
            'v_max':          v_max,
            'delta_max':      delta_max,
            'track_csv_path': LaunchConfiguration('track_csv'),
            'initial_v':      LaunchConfiguration('initial_v'),
        }],
        remappings=[
            ('/ackr',              topic_ackr),
            ('/carmaker/Odometry', topic_odom),
            ('/path',              topic_path),
        ],
    )

    return LaunchDescription([
        track_csv_arg,
        initial_v_arg,
        sim_node,
    ])
