from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="data_collection",
                executable="data_collector",
                output="screen",
            )
        ]
    )
