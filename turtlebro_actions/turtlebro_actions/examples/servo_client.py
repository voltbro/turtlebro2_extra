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
from std_msgs.msg import Int16MultiArray

SERVO_COMMAND_TOPIC = '/arduino/servos'
SERVO_ANGLE_MIN = 0
SERVO_ANGLE_MAX = 180
SERVO_ANGLE_DEFAULT = 90


class ServoClient:
    """Обертка над топиком std_msgs/Int16MultiArray для управления сервоприводом."""

    def __init__(self, node: Node, servo: int) -> None:
        self._node = node
        self.servo = servo
        self.pub = node.create_publisher(Int16MultiArray, SERVO_COMMAND_TOPIC, 10)
        self._node.get_logger().info(
            f'Клиент сервопривода готов на топике {SERVO_COMMAND_TOPIC} (D{self.servo})'
        )

    def move_await(self, angle: int) -> None:
        self._node.get_logger().info(
            f'Угол {angle}° для сервопривода D{self.servo} (ожидание подписчика)'
        )
        while rclpy.ok() and self.pub.get_subscription_count() < 1:
            time.sleep(0.2)
        self.move(angle)

    def move(self, angle: int) -> None:
        clamped = self._clamp_angle(angle)
        self._node.get_logger().info(f'Перемещение сервопривода D{self.servo} к {clamped}°')
        msg = Int16MultiArray()
        msg.data = [int(self.servo), clamped]
        self.pub.publish(msg)
        time.sleep(1.0)

    def shutdown(self) -> None:
        self._node.get_logger().info(f'Клиент сервопривода {self.servo} завершает работу')
        self._node.destroy_publisher(self.pub)

    @staticmethod
    def _clamp_angle(angle: int) -> int:
        return max(SERVO_ANGLE_MIN, min(SERVO_ANGLE_MAX, int(angle)))


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = Node('servo_client_node')
    try:
        servo_num = 2
        if len(sys.argv) > 1:
            servo_num = int(sys.argv[1])
        angle = SERVO_ANGLE_DEFAULT
        if len(sys.argv) > 2:
            angle = int(sys.argv[2])
        servo_client = ServoClient(node, servo_num)
        servo_client.move_await(angle)
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f'Сбой клиента сервопривода: {exc}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
