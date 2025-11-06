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
"""Регрессионные тесты публичного API для turtlebro_py."""

from typing import Iterable

import pytest


def _require_turtlebro() -> type:
    try:
        import rclpy  # noqa: F401
        from turtlebro_py.turtlebro_py import TurtleBro
    except (ImportError, OSError) as exc:
        pytest.skip(f'Пропуск тестов API turtlebro_py: {exc}')
    return TurtleBro


def _require_turtlebro_nav() -> type:
    try:
        import rclpy  # noqa: F401
        from turtlebro_py import TurtleBroNav
    except (ImportError, OSError) as exc:
        pytest.skip(f'Пропуск тестов API turtlebro_py: {exc}')
    return TurtleBroNav


def _require_thermal_images() -> type:
    try:
        import rclpy  # noqa: F401
        from turtlebro_py import ThermalImages
    except (ImportError, OSError) as exc:
        pytest.skip(f'Пропуск тестов API turtlebro_py: {exc}')
    return ThermalImages


def _assert_has_members(cls: type, members: Iterable[str]) -> None:
    for name in members:
        assert hasattr(cls, name), (
            f"Ожидалось, что '{cls.__name__}' предоставляет атрибут '{name}'"
        )


def _assert_methods_are_callable(cls: type, methods: Iterable[str]) -> None:
    for name in methods:
        attr = getattr(cls, name)
        assert callable(attr), (
            f"Атрибут '{name}' класса {cls.__name__} должен быть вызываемым"
        )


def test_turtlebro_exposes_expected_methods() -> None:
    TurtleBro = _require_turtlebro()
    methods = (
        'forward',
        'backward',
        'left',
        'right',
        'goto',
        'call',
        'wait',
        'color',
        'backlight_all',
        'backlight_array',
        'save_photo',
        'get_photo',
        'record',
        'say',
        'play',
        'distance',
        'speed',
        'linear_speed',
        'angular_speed',
    )

    _assert_has_members(TurtleBro, methods)
    _assert_methods_are_callable(TurtleBro, methods)


def test_turtlebro_exposes_expected_properties() -> None:
    TurtleBro = _require_turtlebro()
    properties = (
        'coords',
    )

    _assert_has_members(TurtleBro, properties)
    for name in properties:
        attr = getattr(TurtleBro, name)
        assert isinstance(attr, property), f"'{name}' должен быть объявлен как property"


def test_thermal_images_exposes_expected_api() -> None:
    ThermalImages = _require_thermal_images()
    properties = ('thermo_pixels', 'pixels')
    methods = ('wait_for_frame',)

    _assert_has_members(ThermalImages, properties)
    for name in properties:
        attr = getattr(ThermalImages, name)
        assert isinstance(attr, property), f"'{name}' должен быть объявлен как property"

    _assert_has_members(ThermalImages, methods)
    _assert_methods_are_callable(ThermalImages, methods)


def test_turtlebro_nav_is_exposed_via_public_api() -> None:
    TurtleBro = _require_turtlebro()
    TurtleBroNav = _require_turtlebro_nav()

    assert issubclass(TurtleBroNav, TurtleBro)
    methods = (
        'goto',
        'say',
    )
    _assert_has_members(TurtleBroNav, methods)
    _assert_methods_are_callable(TurtleBroNav, methods)
