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
"""Smoke and logic tests for turtlebro_patrol ROS 2 node."""

from __future__ import annotations

import importlib
from pathlib import Path
import sys

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))


def _require_runtime_deps() -> None:
    for module_name in (
        'rclpy',
        'action_msgs.msg',
        'geometry_msgs.msg',
        'tf_transformations',
    ):
        try:
            importlib.import_module(module_name)
        except (ImportError, OSError) as exc:
            pytest.skip(f'Пропуск проверки импорта: {exc}')


def test_patrol_node_module_can_be_imported() -> None:
    _require_runtime_deps()
    importlib.import_module('turtlebro_patrol_node.patrol_node')


def test_normalize_control_command_accepts_supported_values() -> None:
    _require_runtime_deps()
    module = importlib.import_module('turtlebro_patrol_node.patrol_node')

    assert module.normalize_control_command('start') == 'start'
    assert module.normalize_control_command('  RESUME  ') == 'resume'
    assert module.normalize_control_command('Home') == 'home'


def test_normalize_control_command_rejects_invalid_values() -> None:
    _require_runtime_deps()
    module = importlib.import_module('turtlebro_patrol_node.patrol_node')

    assert module.normalize_control_command('') is None
    assert module.normalize_control_command('dance') is None
    assert module.normalize_control_command('patrol') is None
