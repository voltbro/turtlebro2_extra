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
#
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package='turtlebro_actions',
                executable='move_server.py',
                name='move_server',
                output='log',
                respawn=True,
            ),
            Node(
                package='turtlebro_actions',
                executable='rotate_server.py',
                name='rotate_server',
                output='log',
                respawn=True,
            ),
            Node(
                package='turtlebro_actions',
                executable='photo_service.py',
                name='photo_service',
                output='log',
                respawn=True,
            ),
            Node(
                package='turtlebro_actions',
                executable='record_audio_service.py',
                name='record_audio_service',
                output='log',
                respawn=True,
            ),
            Node(
                package='turtlebro_actions',
                executable='play_audio_service.py',
                name='play_audio_service',
                output='log',
                respawn=True,
            ),
            Node(
                package='turtlebro_actions',
                executable='text_to_speech_server.py',
                name='text_to_speech_server',
                output='log',
                respawn=True,
            ),
        ]
    )
