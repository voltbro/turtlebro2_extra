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
import sys
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import CompressedImage


class VideoClient:
    """Блокирующий помощник для получения одного кадра сжатого изображения."""

    def __init__(self, node: Node, topic: str = '/front_camera/compressed') -> None:
        self._node = node
        self._topic = topic
        self._node.get_logger().info(f'Видеоклиент готов на топике {topic}')

    def getImage(self, timeout: Optional[float] = None) -> bytes:
        future: Future = Future()

        def _callback(msg: CompressedImage) -> None:
            if not future.done():
                future.set_result(msg.data)

        subscription = self._node.create_subscription(
            CompressedImage,
            self._topic,
            _callback,
            10,
        )
        try:
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout)
        finally:
            self._node.destroy_subscription(subscription)

        if future.done():
            return future.result()
        raise TimeoutError(f'На топик {self._topic} не поступило изображение')


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = Node('video_client_node')
    try:
        client = VideoClient(node)
        data = client.getImage(timeout=5.0)
        node.get_logger().info(f'Получено сжатое изображение размером {len(data)} байт')
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f'Сбой видеоклиента: {exc}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
