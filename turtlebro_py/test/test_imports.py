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

import pytest

MODULES = [
    'turtlebro_py',
    'turtlebro_py.turtlebro_py',
]


def test_modules_can_be_imported():
    try:
        importlib.import_module('rclpy')
    except (ImportError, OSError) as exc:
        pytest.skip(f'Пропуск проверки импорта: {exc}')
    for module in MODULES:
        importlib.import_module(module)
