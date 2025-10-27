#!/usr/bin/env python3

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
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16


class ServoClient:
    """Обертка над топиком std_msgs/UInt16 для управления сервоприводом."""

    def __init__(self, node: Node, servo: int) -> None:
        self._node = node
        self.servo = servo
        topic = f'/servo{servo}'
        self.pub = node.create_publisher(UInt16, topic, 10)
        self._node.get_logger().info(f'Клиент сервопривода готов на топике {topic}')

    def move_await(self, angle: int) -> None:
        self._node.get_logger().info(
            f'Перемещение сервопривода {self.servo} к {angle} град. (ожидание подписчика)'
        )
        while rclpy.ok() and self.pub.get_subscription_count() < 1:
            time.sleep(0.2)
        self.move(angle)

    def move(self, angle: int) -> None:
        self._node.get_logger().info(
            f'Перемещение сервопривода {self.servo} к {angle} град.'
        )
        msg = UInt16()
        msg.data = int(angle)
        self.pub.publish(msg)
        time.sleep(1.0)

    def shutdown(self) -> None:
        self._node.get_logger().info(f'Клиент сервопривода {self.servo} завершает работу')
        self._node.destroy_publisher(self.pub)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = Node('servo_client_node')
    try:
        servo_num = 44
        if len(sys.argv) > 1:
            servo_num = int(sys.argv[1])
        servo_client = ServoClient(node, servo_num)
        servo_client.move_await(90)
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f'Сбой клиента сервопривода: {exc}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
