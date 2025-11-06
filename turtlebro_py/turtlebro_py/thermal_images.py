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
"""ROS 2 helper for receiving thermal frames from TurtleBro."""

from __future__ import annotations

import threading
import time
from typing import Optional, Sequence

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class ThermalImages:
    """Подписка на поток тепловизора TurtleBro через /thermovisor."""

    def __init__(
        self,
        *,
        topic: str = '/thermovisor',
        queue_size: int = 10,
        wait_for_first_frame: bool = False,
        wait_timeout: Optional[float] = None,
    ) -> None:
        self._topic = topic
        self._owns_context = not rclpy.is_initialized()
        if self._owns_context:
            rclpy.init()

        self._node: Node = rclpy.create_node('thermal_images')
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        self._condition = threading.Condition()
        self._last_msg = Float32MultiArray()
        self._has_frame = False

        self._subscription = self._node.create_subscription(
            Float32MultiArray, topic, self._on_frame, queue_size
        )

        if wait_for_first_frame:
            self.wait_for_frame(timeout=wait_timeout)

    def __del__(self):
        self.shutdown()

    def shutdown(self) -> None:
        """Остановить поток исполнения и освободить ресурсы ROS."""
        executor = getattr(self, '_executor', None)
        node = getattr(self, '_node', None)
        thread = getattr(self, '_spin_thread', None)

        if executor is not None:
            executor.shutdown()
        if thread is not None:
            thread.join(timeout=1.0)
        if node is not None:
            node.destroy_node()
        if self._owns_context and rclpy.ok():
            rclpy.shutdown()

    def wait_for_frame(self, *, timeout: Optional[float] = None) -> Float32MultiArray:
        """
        Дождаться первого кадра тепловизора.

        :param timeout: максимум секунд ожидания; None означает бесконечное время.
        :raises TimeoutError: если кадр так и не пришёл.
        :return: последнее полученное сообщение.
        """
        deadline = None if timeout is None else time.monotonic() + timeout
        with self._condition:
            while not self._has_frame:
                remaining = None if deadline is None else max(0.0, deadline - time.monotonic())
                if deadline is not None and remaining <= 0:
                    raise TimeoutError(
                        f'На топик {self._topic} не поступило сообщение за {timeout} с'
                    )
                self._condition.wait(timeout=remaining)
            return self._last_msg

    @property
    def thermo_pixels(self) -> Sequence[float]:
        """Последний кадр тепловизора, совместимый с предыдущим API."""
        return self._last_msg.data

    @property
    def pixels(self) -> Sequence[float]:
        """Синоним thermo_pixels для нового кода."""
        return self._last_msg.data

    def _on_frame(self, msg: Float32MultiArray) -> None:
        with self._condition:
            self._last_msg = msg
            self._has_frame = True
            self._condition.notify_all()

