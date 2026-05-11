import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("switch_udp_bridge")
    config_file = os.path.join(pkg_dir, "config", "udp_bridge.yaml")

    return LaunchDescription([
        Node(
            package="switch_udp_bridge",
            executable="udp_bridge_node",
            name="udp_bridge_node",
            output="screen",
            parameters=[config_file],
        )
    ])