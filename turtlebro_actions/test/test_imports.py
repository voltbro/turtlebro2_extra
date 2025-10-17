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
import importlib

MODULES = [
    'turtlebro_actions',
    'turtlebro_actions_nodes',
    'turtlebro_actions_nodes.commands_controller',
    'turtlebro_actions_nodes.move_client',
    'turtlebro_actions_nodes.move_server',
    'turtlebro_actions_nodes.photo_service',
    'turtlebro_actions_nodes.radio',
    'turtlebro_actions_nodes.rotate_client',
    'turtlebro_actions_nodes.rotate_server',
    'turtlebro_actions_nodes.servo_client',
    'turtlebro_actions_nodes.video_client',
]


def test_modules_can_be_imported():
    for module in MODULES:
        importlib.import_module(module)
