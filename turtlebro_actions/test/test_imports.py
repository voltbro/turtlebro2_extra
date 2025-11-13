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
from pathlib import Path
import sys

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

MODULES = [
    "turtlebro_actions",
    "turtlebro_actions.servers",
    "turtlebro_actions.examples",
    "turtlebro_actions.examples.move_client",
    "turtlebro_actions.examples.rotate_client",
    "turtlebro_actions.examples.servo_client",
    "turtlebro_actions.examples.video_client",
    "turtlebro_actions.servers.record_audio_service",
    "turtlebro_actions.servers.play_audio_service",
    "turtlebro_actions.servers.rotate_server",
    "turtlebro_actions.servers.move_server",
    "turtlebro_actions.servers.photo_service",
]


def test_modules_can_be_imported():
    try:
        importlib.import_module("turtlebro_interfaces.action")
    except ImportError as exc:
        pytest.skip(f"Пропуск проверки импорта: {exc}")
    try:
        importlib.import_module("rclpy")
    except (ImportError, OSError) as exc:
        pytest.skip(f"Пропуск проверки импорта: {exc}")
    for module in MODULES:
        importlib.import_module(module)
