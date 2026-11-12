"""
track_publisher.launch.py
Publishes the track centerline from a CSV file to /path as nav_msgs/Path
with TRANSIENT_LOCAL QoS so the MPC controller node always receives the
path even if it starts after this publisher.

Default CSV: <workspace_root>/track.csv   (two-column x,y, no header)

Usage:
    ros2 launch mpc_controller track_publisher.launch.py
    ros2 launch mpc_controller track_publisher.launch.py csv_path:=/abs/path/to/track.csv
    ros2 launch mpc_controller track_publisher.launch.py csv_path:=/abs/path/to/track.csv publish_rate_hz:=0.5
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('mpc_controller')

    # Default: track.csv installed alongside config files
    default_csv = os.path.join(pkg_dir, 'config', 'track.csv')

    # ── Launch arguments ─────────────────────────────────────────────────
    csv_path_arg = DeclareLaunchArgument(
        'csv_path',
        default_value=default_csv,
        description='Absolute path to the track CSV file (x,y per row, no header)')

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='ROS frame_id for the published path (must match MPC visualizer)')

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate_hz',
        default_value='1.0',
        description='Rate at which to re-publish the path [Hz]. '
                    'TRANSIENT_LOCAL ensures late subscribers always get it '
                    'regardless of rate.')

    # ── Node ─────────────────────────────────────────────────────────────
    track_pub_node = Node(
        package='mpc_controller',
        executable='track_csv_publisher',
        name='track_csv_publisher',
        output='screen',
        parameters=[{
            'csv_path':        LaunchConfiguration('csv_path'),
            'frame_id':        LaunchConfiguration('frame_id'),
            'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
        }],
    )

    return LaunchDescription([
        csv_path_arg,
        frame_id_arg,
        publish_rate_arg,
        track_pub_node,
    ])
