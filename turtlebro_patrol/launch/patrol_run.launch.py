#!/usr/bin/env python3

# Copyright 2026 VoltBro
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
"""Launch turtlebro_patrol node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config_data_file = DeclareLaunchArgument(
        'config_data_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('turtlebro_patrol'), 'data', 'goals.toml']
        ),
        description='Path to TOML file with patrol points',
    )

    point_callback_service = DeclareLaunchArgument(
        'point_callback_service',
        default_value='',
        description='Optional service name for callback on each reached point',
    )

    patrol_node = Node(
        package='turtlebro_patrol',
        executable='patrol_node.py',
        name='turtlebro_patrol',
        output='screen',
        parameters=[
            {
                'config_data_file': LaunchConfiguration('config_data_file'),
                'point_callback_service': LaunchConfiguration('point_callback_service'),
            }
        ],
    )

    return LaunchDescription(
        [
            config_data_file,
            point_callback_service,
            patrol_node,
        ]
    )
