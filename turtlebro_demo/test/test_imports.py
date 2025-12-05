"""Smoke-тесты пакета turtlebro_demo."""

import importlib


def test_package_importable():
    assert importlib.import_module('turtlebro_demo') is not None


def test_node_importable():
    assert importlib.import_module('turtlebro_demo.demo_node') is not None
