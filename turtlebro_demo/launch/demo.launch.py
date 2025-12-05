#!/usr/bin/env python3
#
# Copyright 2025 VoltBro
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
"""Лаунч-файл для запуска демонстрационного узла TurtleBro."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    forward = DeclareLaunchArgument('forward_distance_m', default_value='0.5')
    backward = DeclareLaunchArgument('backward_distance_m', default_value='0.3')
    rotation = DeclareLaunchArgument('rotation_angle_deg', default_value='90.0')
    voice = DeclareLaunchArgument('voice', default_value='Vsevolod')
    record_duration = DeclareLaunchArgument('record_duration_s', default_value='3.0')
    distance_angle = DeclareLaunchArgument('distance_angle_deg', default_value='0')
    enable_audio = DeclareLaunchArgument('enable_audio', default_value='true')
    enable_photo = DeclareLaunchArgument('enable_photo', default_value='true')
    led_intro = DeclareLaunchArgument('led_intro_color', default_value='yellow')
    led_finish = DeclareLaunchArgument('led_finish_color', default_value='blue')

    node = Node(
        package='turtlebro_demo',
        executable='demo_node.py',
        name='turtlebro_demo',
        output='screen',
        parameters=[
            {
                'forward_distance_m': LaunchConfiguration('forward_distance_m'),
                'backward_distance_m': LaunchConfiguration('backward_distance_m'),
                'rotation_angle_deg': LaunchConfiguration('rotation_angle_deg'),
                'voice': LaunchConfiguration('voice'),
                'record_duration_s': LaunchConfiguration('record_duration_s'),
                'distance_angle_deg': LaunchConfiguration('distance_angle_deg'),
                'enable_audio': LaunchConfiguration('enable_audio'),
                'enable_photo': LaunchConfiguration('enable_photo'),
                'led_intro_color': LaunchConfiguration('led_intro_color'),
                'led_finish_color': LaunchConfiguration('led_finish_color'),
            }
        ],
    )

    return LaunchDescription(
        [
            forward,
            backward,
            rotation,
            voice,
            record_duration,
            distance_angle,
            enable_audio,
            enable_photo,
            led_intro,
            led_finish,
            node,
        ]
    )
