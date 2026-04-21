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
"""Launch patrol node with optional navigation stack startup."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    run_navigation = DeclareLaunchArgument(
        'run_navigation',
        default_value='false',
        description='Start navigation stack together with patrol node',
    )

    fake_move_base = DeclareLaunchArgument(
        'fake_move_base',
        default_value='false',
        description='Start fake_move_base from turtlebro_navigation package',
    )

    config_data_file = DeclareLaunchArgument(
        'config_data_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('turtlebro_patrol'), 'data', 'goals.toml']
        ),
        description='Path to TOML file with patrol points',
    )

    callback_service = DeclareLaunchArgument(
        'point_callback_service',
        default_value='',
        description='Optional service name to call on reached point',
    )

    real_navigation = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('turtlebro_navigation'),
                    'launch',
                    'turtlebro_slam_navigation.xml',
                ]
            )
        ),
        launch_arguments={'open_rviz': 'false'}.items(),
        condition=IfCondition(
            PythonExpression(
                [
                    LaunchConfiguration('run_navigation'),
                    " and not ",
                    LaunchConfiguration('fake_move_base'),
                ]
            )
        ),
    )

    fake_navigation = Node(
        package='turtlebro_navigation',
        executable='fake_move_base.py',
        name='fake_move_base',
        output='screen',
        condition=IfCondition(
            PythonExpression(
                [
                    LaunchConfiguration('run_navigation'),
                    " and ",
                    LaunchConfiguration('fake_move_base'),
                ]
            )
        ),
    )

    patrol = Node(
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
            run_navigation,
            fake_move_base,
            config_data_file,
            callback_service,
            real_navigation,
            fake_navigation,
            patrol,
        ]
    )
