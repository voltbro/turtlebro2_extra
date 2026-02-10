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
"""Вспомогательные функции для совместного владения глобальным контекстом rclpy."""

from __future__ import annotations

import atexit
import threading
import weakref
from typing import Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

# Jazzy не содержит rclpy.utilities.is_initialized; вводим совместимый шорткат
try:  # Foxy/Humble API
    from rclpy.utilities import is_initialized as _is_initialized
except ImportError:  # Jazzy: проверяем контекст вручную
    def _is_initialized() -> bool:
        try:
            return rclpy.get_default_context().ok()
        except Exception:
            return False


__all__ = ['acquire_ros_context', 'RosContextManaged']


class _RosContextHandle:
    """Счётчик ссылок, который завершает rclpy после освобождения последнего владельца."""

    def __init__(self, manager: '_RosContextManager') -> None:
        self._manager = manager
        self._released = False

    def release(self) -> None:
        if self._released:
            return
        self._released = True
        self._manager._release()


class _RosContextManager:
    """Централизованный менеджер инициализации/завершения глобального контекста rclpy."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._refcount = 0
        self._owns_context = False

    def acquire(self) -> _RosContextHandle:
        with self._lock:
            if not _is_initialized():
                rclpy.init()
                self._owns_context = True
            self._refcount += 1
        return _RosContextHandle(self)

    def _release(self) -> None:
        with self._lock:
            if self._refcount == 0:
                return
            self._refcount -= 1
            if self._owns_context and self._refcount == 0:
                if rclpy.ok():
                    rclpy.shutdown()
                self._owns_context = False


_GLOBAL_ROS_CONTEXT = _RosContextManager()


def acquire_ros_context() -> _RosContextHandle:
    """Получить доступ к общему контексту ROS 2, инициализируя его при необходимости."""
    return _GLOBAL_ROS_CONTEXT.acquire()


class RosContextManaged:
    """Базовый класс, который управляет жизненным циклом rclpy и связанного узла."""

    def __init__(
        self,
        *,
        node: Optional[Node] = None,
        executor: Optional[MultiThreadedExecutor] = None,
        node_name: str = 'ros_resource',
        auto_spin_executor: bool = True,
    ) -> None:
        self._external_node = node is not None
        self._external_executor = executor is not None
        self._node: Optional[Node] = node
        self._executor: Optional[MultiThreadedExecutor] = None
        self._spin_thread: Optional[threading.Thread] = None
        self._added_node_to_executor = False
        self._context_handle: Optional[_RosContextHandle] = None
        self._closed = False
        self._close_hook_registered = False

        if node is not None and not _is_initialized():
            raise RuntimeError(
                f'rclpy.init() must be called before passing an existing node to {self.__class__.__name__}'
            )

        if not self._external_node:
            self._context_handle = acquire_ros_context()
            self._node = rclpy.create_node(node_name)
        elif self._node is None:
            raise ValueError('External node instance must be provided when node parameter is used')

        if not self._external_node:
            self._executor = executor or MultiThreadedExecutor()
            if self._executor is not None:
                self._executor.add_node(self._node)
                self._added_node_to_executor = True
                if not self._external_executor and auto_spin_executor:
                    self._spin_thread = threading.Thread(target=self._spin_executor_worker, daemon=True)
                    self._spin_thread.start()
        else:
            self._executor = executor

        self._register_close_hook()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):  # noqa: D401
        self.close()

    def __del__(self):
        self.close()

    def close(self) -> None:
        """Закрыть ресурс и освободить контекст rclpy."""
        if getattr(self, '_closed', False):
            return
        self._closed = True
        try:
            self._on_close()
        finally:
            self._teardown_ros()

    def _on_close(self) -> None:  # pragma: no cover - по умолчанию делать нечего
        """Переопределите в наследниках для освобождения собственных ресурсов."""

    def _spin_executor_worker(self) -> None:
        executor = getattr(self, '_executor', None)
        if executor is None:
            return
        try:
            executor.spin()
        except Exception as exc:  # noqa: BLE001
            if self._is_context_shutdown_spin_error(exc):
                return
            raise

    @staticmethod
    def _is_context_shutdown_spin_error(exc: BaseException) -> bool:
        text = str(exc).lower()
        markers = (
            'context is not valid',
            'rcl_shutdown() was called',
            'rcl_shutdown was called',
            'failed to initialize wait set',
        )
        return any(marker in text for marker in markers)

    def _teardown_ros(self) -> None:
        executor = getattr(self, '_executor', None)
        added_node_to_executor = getattr(self, '_added_node_to_executor', False)
        node = getattr(self, '_node', None)
        external_executor = getattr(self, '_external_executor', False)
        spin_thread = getattr(self, '_spin_thread', None)
        external_node = getattr(self, '_external_node', False)
        context_handle = getattr(self, '_context_handle', None)

        if executor is not None and added_node_to_executor and node is not None:
            try:
                executor.remove_node(node)
            except Exception:
                pass

        if executor is not None and not external_executor:
            try:
                executor.shutdown()
            except Exception:
                pass

        if spin_thread is not None:
            spin_thread.join(timeout=1.0)

        if not external_node and node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass

        if context_handle is not None:
            context_handle.release()
            self._context_handle = None

    def _register_close_hook(self) -> None:
        if self._close_hook_registered:
            return

        self_ref = weakref.ref(self)

        def _close_instance(ref=self_ref):
            instance = ref()
            if instance is not None:
                instance.close()

        atexit.register(_close_instance)
        self._close_hook_registered = True
