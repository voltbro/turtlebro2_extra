from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package='turtlebro_actions',
                executable='move_server',
                name='move_server',
                output='log',
                respawn=True,
            ),
            Node(
                package='turtlebro_actions',
                executable='rotate_server',
                name='rotate_server',
                output='log',
                respawn=True,
            ),
        ]
    )
