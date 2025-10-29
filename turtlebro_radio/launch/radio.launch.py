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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    radio_device = DeclareLaunchArgument(
        'radio_device',
        default_value=(
            '/dev/serial/by-id/'
            'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
        ),
    )
    radio_baud = DeclareLaunchArgument('radio_baud', default_value='19200')
    linear_speed = DeclareLaunchArgument('linear_speed', default_value='0.22')
    angular_speed = DeclareLaunchArgument('angular_speed', default_value='1.0')

    move_server = Node(
        package='turtlebro_actions',
        executable='move_server',
        name='move_server',
        output='log',
        respawn=True,
    )

    rotate_server = Node(
        package='turtlebro_actions',
        executable='rotate_server',
        name='rotate_server',
        output='log',
        respawn=True,
    )

    radio_node = Node(
        package='turtlebro_radio',
        executable='radio_command',
        name='radio_command_node',
        output='screen',
        respawn=True,
        parameters=[
            {
                'port': LaunchConfiguration('radio_device'),
                'baud': ParameterValue(
                    LaunchConfiguration('radio_baud'), value_type=int
                ),
                'linear_speed': ParameterValue(
                    LaunchConfiguration('linear_speed'), value_type=float
                ),
                'angular_speed': ParameterValue(
                    LaunchConfiguration('angular_speed'), value_type=float
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            radio_device,
            radio_baud,
            linear_speed,
            angular_speed,
            move_server,
            rotate_server,
            radio_node,
        ]
    )
