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
"""Tests that TurtleBro and ThermalImages manage the shared ROS context."""

from __future__ import annotations

import importlib
import sys
import types
from types import SimpleNamespace

import pytest


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, message: str) -> None:
        self.messages.append(("info", message))

    def error(self, message: str) -> None:
        self.messages.append(("error", message))


class _Destroyable:
    def __init__(self):
        self.destroyed = False

    def destroy(self) -> None:
        self.destroyed = True


class FakePublisher(_Destroyable):
    def publish(self, _msg) -> None:  # pragma: no cover - nothing to report during tests
        pass


class FakeSubscription(_Destroyable):
    pass


class FakeServiceClient(_Destroyable):
    def __init__(self, name: str):
        super().__init__()
        self.name = name
        self.wait_calls = 0

    def wait_for_service(self, timeout_sec: float | None = None) -> bool:  # noqa: ARG002
        self.wait_calls += 1
        return True


class FakeActionClient(_Destroyable):
    def __init__(self, _node, _action_type, name: str):  # noqa: D401,ARG002
        super().__init__()
        self.name = name

    def wait_for_server(self, timeout_sec: float | None = None) -> bool:  # noqa: ARG002
        return True


class FakeExecutor:
    def __init__(self):
        self.nodes = []
        self.shutdown_called = False

    def add_node(self, node) -> None:
        self.nodes.append(node)

    def remove_node(self, node) -> None:
        if node in self.nodes:
            self.nodes.remove(node)

    def spin(self) -> None:  # pragma: no cover - executor is never actually spun
        pass

    def shutdown(self) -> None:
        self.shutdown_called = True


class FakeNode:
    def __init__(self, name: str = "fake"):
        self.name = name
        self._logger = FakeLogger()

    def create_subscription(self, *_args, **_kwargs):
        return FakeSubscription()

    def destroy_subscription(self, sub):
        if hasattr(sub, "destroy"):
            sub.destroy()

    def create_publisher(self, *_args, **_kwargs):
        return FakePublisher()

    def destroy_publisher(self, pub):
        if hasattr(pub, "destroy"):
            pub.destroy()

    def create_client(self, *_args, **_kwargs):
        return FakeServiceClient(_args[-1] if _args else "service")

    def destroy_client(self, client):
        if hasattr(client, "destroy"):
            client.destroy()

    def get_logger(self) -> FakeLogger:
        return self._logger

    def destroy_node(self):
        self._destroyed = True


class FakeFuture:
    def __init__(self):
        self._done = False
        self._result = None

    def done(self) -> bool:
        return self._done

    def set_result(self, result) -> None:
        self._result = result
        self._done = True

    def result(self):
        return self._result

    def exception(self):  # pragma: no cover - exceptions are not used in these tests
        return None

    def cancelled(self) -> bool:
        return False


class DummyUtility:
    def __init__(self, node, *, tts_action_client=None):  # noqa: ARG002
        self.node = node
        self.closed = False

    def close(self):
        self.closed = True

    def __getattr__(self, _name):
        def _noop(*_args, **_kwargs):  # noqa: D401
            return None

        return _noop


class FakeRclpy(types.ModuleType):
    def __init__(self):
        super().__init__("rclpy")
        self.__path__ = []  # allow importing submodules
        self.init_calls = 0
        self.shutdown_calls = 0
        self._initialized = False
        self.created_nodes: list[FakeNode] = []

    def init(self) -> None:
        self._initialized = True
        self.init_calls += 1

    def shutdown(self) -> None:
        if self._initialized:
            self.shutdown_calls += 1
        self._initialized = False

    def is_initialized(self) -> bool:
        return self._initialized

    def ok(self) -> bool:
        return True

    def create_node(self, name: str):
        node = FakeNode(name)
        self.created_nodes.append(node)
        return node

    def spin_until_future_complete(self, *_args, **_kwargs):  # noqa: ARG002
        pass


@pytest.fixture
def ros_test_env(monkeypatch):
    fake_rclpy = FakeRclpy()
    submodules = {
        "rclpy": fake_rclpy,
        "rclpy.action": types.SimpleNamespace(ActionClient=FakeActionClient),
        "rclpy.client": types.SimpleNamespace(Client=FakeServiceClient),
        "rclpy.executors": types.SimpleNamespace(MultiThreadedExecutor=FakeExecutor),
        "rclpy.node": types.SimpleNamespace(Node=FakeNode),
        "rclpy.task": types.SimpleNamespace(Future=FakeFuture),
    }
    for name, module in submodules.items():
        monkeypatch.setitem(sys.modules, name, module)

    for name in [key for key in sys.modules.keys() if key.startswith("turtlebro_py")]:
        sys.modules.pop(name)

    pkg = importlib.import_module("turtlebro_py")
    tb_module = importlib.import_module("turtlebro_py.turtlebro_py")
    thermal_module = importlib.import_module("turtlebro_py.thermal_images")

    monkeypatch.setattr(tb_module.TurtleBro, "wait_for_odom_to_start", lambda self: setattr(self, "odom_has_started", True))
    monkeypatch.setattr(tb_module, "Utility", DummyUtility)

    return SimpleNamespace(
        rclpy=fake_rclpy,
        TurtleBro=tb_module.TurtleBro,
        ThermalImages=thermal_module.ThermalImages,
        FakeNode=FakeNode,
    )


def test_turtlebro_instances_share_context(ros_test_env):
    rclpy_stub = ros_test_env.rclpy
    TurtleBro = ros_test_env.TurtleBro

    with TurtleBro(auto_spin_executor=False) as tb1:
        assert tb1._context_handle is not None
        assert rclpy_stub.init_calls == 1
        assert rclpy_stub.shutdown_calls == 0
        with TurtleBro(auto_spin_executor=False) as tb2:
            assert tb2._context_handle is not None
            assert rclpy_stub.init_calls == 1
            assert rclpy_stub.shutdown_calls == 0
        assert rclpy_stub.shutdown_calls == 0
    assert rclpy_stub.shutdown_calls == 1


def test_thermal_images_instances_share_context(ros_test_env):
    rclpy_stub = ros_test_env.rclpy
    ThermalImages = ros_test_env.ThermalImages

    with ThermalImages(auto_spin_executor=False) as ti1:
        assert ti1._context_handle is not None
        assert rclpy_stub.init_calls == 1
        with ThermalImages(auto_spin_executor=False) as ti2:
            assert ti2._context_handle is not None
            assert rclpy_stub.init_calls == 1
            assert rclpy_stub.shutdown_calls == 0
        assert rclpy_stub.shutdown_calls == 0
    assert rclpy_stub.shutdown_calls == 1


def test_mixed_instances_reference_count_context(ros_test_env):
    rclpy_stub = ros_test_env.rclpy
    TurtleBro = ros_test_env.TurtleBro
    ThermalImages = ros_test_env.ThermalImages

    with TurtleBro(auto_spin_executor=False):
        assert rclpy_stub.init_calls == 1
        with ThermalImages(auto_spin_executor=False):
            assert rclpy_stub.init_calls == 1
            assert rclpy_stub.shutdown_calls == 0
        assert rclpy_stub.shutdown_calls == 0
    assert rclpy_stub.shutdown_calls == 1


def test_turtlebro_with_external_node_does_not_touch_context(ros_test_env):
    rclpy_stub = ros_test_env.rclpy
    TurtleBro = ros_test_env.TurtleBro
    fake_node = ros_test_env.FakeNode()

    rclpy_stub.init()
    init_before = rclpy_stub.init_calls
    tb = TurtleBro(node=fake_node, auto_spin_executor=False)
    try:
        assert tb._context_handle is None
        assert rclpy_stub.init_calls == init_before
        assert rclpy_stub.shutdown_calls == 0
    finally:
        tb.close()
    assert rclpy_stub.shutdown_calls == 0


def test_thermal_images_with_external_node_does_not_touch_context(ros_test_env):
    rclpy_stub = ros_test_env.rclpy
    ThermalImages = ros_test_env.ThermalImages
    fake_node = ros_test_env.FakeNode()

    rclpy_stub.init()
    init_before = rclpy_stub.init_calls
    ti = ThermalImages(node=fake_node, auto_spin_executor=False)
    try:
        assert ti._context_handle is None
        assert rclpy_stub.init_calls == init_before
        assert rclpy_stub.shutdown_calls == 0
    finally:
        ti.close()
    assert rclpy_stub.shutdown_calls == 0
