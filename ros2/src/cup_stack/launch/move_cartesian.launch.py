"""Launch the move_cartesian node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="cup_stack",
                executable="move_cartesian",
                output="screen",
            ),
        ]
    )
