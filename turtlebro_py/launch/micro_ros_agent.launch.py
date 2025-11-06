#!/usr/bin/env python3

# Copyright 2024 VoltBro
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

"""Launch micro-ROS agent for TurtleBro hardware."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    dev_arg = DeclareLaunchArgument(
        'dev',
        default_value='/dev/ttyACM0',
        description='Serial device where the TurtleBro micro-ROS board is connected',
    )

    baud_arg = DeclareLaunchArgument(
        'baud',
        default_value='115200',
        description='Serial baudrate for the micro-ROS transport',
    )

    transport_arg = DeclareLaunchArgument(
        'transport',
        default_value='serial',
        description='micro-ROS transport to use (serial, udp, etc.)',
    )

    agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=[
            LaunchConfiguration('transport'),
            '--dev', LaunchConfiguration('dev'),
            '-b', LaunchConfiguration('baud'),
        ],
        respawn=True,
    )

    return LaunchDescription([
        dev_arg,
        baud_arg,
        transport_arg,
        agent_node,
    ])
